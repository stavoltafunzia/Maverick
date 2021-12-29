#include "MaverickCInterface.h"
#include "MaverickCore/OcpSolverImpl.hh"
#include "MaverickGC/GenericContainerLuaInterface.hh"
#include "MaverickCore/MaverickPrivateDefinitions.hh"
#include "MaverickSplines/Splines.hh"
#include "MaverickCore/MaverickFunctions.hh"
#include <iostream>

using namespace Maverick;
using namespace MaverickUtils;
using namespace std;

namespace Maverick {
  
  static std::map<MaverickOcp *, void *> map_open_lib;

  template<typename T>
  static maverick_int getNewIdForMap(std::map<int, T> const &mappa) {
    if (mappa.empty()) return 1;

    maverick_int id = 0;

    maverick_int const last_id = mappa.rbegin()->first;

    for (maverick_int i = last_id; i < std::numeric_limits<int>::max(); i++) {
      if (mappa.find(i) == mappa.end()) {
        id = i;
        break;
      }
    }

    if (id != 0) return id;

    for (maverick_int i = std::numeric_limits<int>::min(); i < last_id; i++) {
      if (mappa.find(i) == mappa.end()) {
        id = i;
        break;
      }
    }

    return id;
  }
}

maverick_int getMaverickCInterfaceVersion() {
  return 1;
}

bool isMaverickCInterfaceCompatibleWithVersion(maverick_int ver) {
  if (ver == 1)
    return true;
  return false;
}

void * getMaverickOcpFromLib(char const lib_name[], char err_msg_buf[], size_t buf_len) {
  MaverickOcp *p_maverick_ocp = nullptr;

  void *lib_handle = nullptr;
  lib_handle = dlopen(lib_name, RTLD_LAZY);

  if (lib_handle == nullptr) {
    strncpy(err_msg_buf, dlerror(), buf_len);
    return nullptr;
  }

  typedef MaverickOcp *(*GetMaverickOcp)();

  GetMaverickOcp p_get_maverick_ocp = nullptr;

  p_get_maverick_ocp = (GetMaverickOcp) dlsym(lib_handle, "getMaverickOcpPointer");

  // if p_get_maverick_ocp exist
  if (p_get_maverick_ocp != nullptr) {
    try {
      p_maverick_ocp = p_get_maverick_ocp();
      map_open_lib.insert(std::pair<MaverickOcp *, void *>(p_maverick_ocp, lib_handle));
    } catch (std::exception &exc) {
      strncpy(err_msg_buf, exc.what(), buf_len);
      return nullptr;
    }
  } else {
    strncpy(err_msg_buf, "The shared library does not have the symbol 'getMaverickOcpPointer'", buf_len);
    return nullptr;
  }
  return p_maverick_ocp;
}

void deleteMaverickOcp(void *const p_maverick_ocp) {
  MaverickOcp *ocp = (MaverickOcp *) p_maverick_ocp;
  delete ocp;

  // look for opened shared library with that object
  try {
    std::map<MaverickOcp *, void *>::iterator it = map_open_lib.find(ocp);
    if (it != map_open_lib.end()) { // if the pointer is found
      void *p_shared_lib = it->second;
      map_open_lib.erase(it);
      bool should_close_lib = true;
      for (std::map<MaverickOcp *, void *>::iterator xx = map_open_lib.begin();
           xx != map_open_lib.end(); xx++) { // check if we should close the library
        if (xx->second == p_shared_lib) {
          should_close_lib = false;
          break;
        }
      }
      if (should_close_lib && (p_shared_lib != nullptr)) {
        dlclose(p_shared_lib);
      }
    }
  } catch (...) {
    cout << "ERROR: Internal error in the Maverick C interface. Kill the application." << endl;
  }
}


//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// MAVERICK OCP

void *getMaverickOcpSolver(void *const p_maverick_ocp, char err_msg_buf[], size_t buf_len) {
  // check for NULL pointer
  if (p_maverick_ocp == NULL) {
    strncpy(err_msg_buf, "p_maverick_ocp is a nullptr", buf_len);
    return nullptr;
  }

  //create the solver
  OcpSolverImpl *solver = nullptr;
  try {
    MaverickOcp *ocp = (MaverickOcp *) p_maverick_ocp;
    solver = new OcpSolverImpl(*ocp);
  } catch (std::exception & exc) {
    strncpy(err_msg_buf, exc.what(), buf_len);
  }

  return (void *) solver;
}

void deleteMaverickOcpSolver(void *solver) {
  OcpSolverImpl *_solver = (OcpSolverImpl *) solver;
  delete _solver;
}

maverick_int solveMaverick(void *p_maverick_solver, void *p_input_generic_container, maverick_int sol_type,
                  void *p_output_generic_container, char err_msg_buf[], size_t buf_len) {

  if (p_maverick_solver == nullptr) {
    strncpy(err_msg_buf, "p_maverick_solver is a nullptr", buf_len);
    return 1;
  }

  if (p_input_generic_container == nullptr) {
    strncpy(err_msg_buf, "p_input_generic_container is a nullptr", buf_len);
    return 1;
  }

  if (p_output_generic_container == nullptr) {
    strncpy(err_msg_buf, "p_output_generic_container is a nullptr", buf_len);
    return 1;
  }

  // solve
  try {
    auto out = ((MaverickOcpSolver *) p_maverick_solver)->solve(*((GC::GenericContainer *) p_input_generic_container));
    ((GC::GenericContainer *) p_output_generic_container)->clear();
    if (sol_type == 0)
      out.writeContentToGC(*((GC::GenericContainer *) p_output_generic_container), nullptr);
    else
      out.writeContentToGC(*((GC::GenericContainer *) p_output_generic_container),
                           &(((MaverickOcpSolver *) p_maverick_solver)->getOcpProblem()));
  } catch (std::exception &exc) {
    std::strncpy(err_msg_buf, exc.what(), buf_len);
    return 1;
  }
  return 0;
}

//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// GENERIC CONTAINER

maverick_int fillGenericContainerFromLua(void const * p_generic_container,
                                         char const lua_file[], char const lua_global_var[],
                                         char err_msg_buf[], size_t buf_len) {

  if (p_generic_container == nullptr) {
    strncpy(err_msg_buf, "p_generic_container is nullptr", buf_len);
    return 1;
  }

  try {
    GC::LuaInterpreter lua;
    lua.do_file(lua_file);
    lua.global_to_GC(lua_global_var, *((GC::GenericContainer *) p_generic_container));
  } catch (std::exception &exc) {
    strncpy(err_msg_buf, exc.what(), buf_len);
    return 1;
  }

  return 0;
}

// SPLINES

void * create1DSpline(char const spline_type[], size_t n, maverick_real const * x, maverick_real const * y,
  bool check_range, char error_msg[], size_t msg_len) {

  Splines::Spline * spline = nullptr;

  try {
    string spline_type_str(spline_type);
    if (compareStringIgnoreCase(spline_type_str, string(SPLINE_BESSEL))) spline = new Splines::BesselSpline();
    else if (compareStringIgnoreCase(spline_type_str, SPLINE_AKIMA)) spline = new Splines::AkimaSpline();
    else if (compareStringIgnoreCase(spline_type_str, SPLINE_CUBIC)) spline = new Splines::CubicSpline();
    else if (compareStringIgnoreCase(spline_type_str, SPLINE_QUINTIC)) spline = new Splines::QuinticSpline();
    else if (compareStringIgnoreCase(spline_type_str, SPLINE_CONSTANTS)) spline = new Splines::ConstantSpline();
    else if (compareStringIgnoreCase(spline_type_str, SPLINE_LINEAR)) spline = new Splines::LinearSpline();
    else if (compareStringIgnoreCase(spline_type_str, SPLINE_PCHIP)) spline = new Splines::PchipSpline();
    else {
      std::string mess = "unable to initialize spline. Requested spline of type " + spline_type_str +
          " which is not available. Available types are: " + SPLINE_AKIMA + ", "
          + SPLINE_BESSEL + ", " + SPLINE_CONSTANTS + ", " + SPLINE_CUBIC + ", " + SPLINE_PCHIP +
          ", " + SPLINE_QUINTIC + ".\n";
      strncpy(error_msg, mess.c_str(), msg_len);
      return nullptr;
    }
    spline->reserve(n);

    for (size_t i = 0; i < n; ++i) // add provided points
      spline->pushBack(x[i], y[i]);

    spline->build();

    spline->setCheckRange(check_range);

  } catch (std::exception & exc) {
    strncpy(error_msg, exc.what(), msg_len);
    return nullptr;
  }

  return (void*) spline;
}

void delete1DSpline(void * p_spline) {
  auto spline = (Splines::Spline *) p_spline;
  delete spline;
}

maverick_int eval1DSpline(void * spline_ptr, maverick_real x, maverick_real * out, char error_msg[], size_t err_msg_len) {
  try {
    Splines::Spline * spline = (Splines::Spline *) spline_ptr;
    *out = spline->eval(x);
  } catch (std::exception & exc) {
    strncpy(error_msg, exc.what(), err_msg_len);
    return 1;
  }
  return 0;
}

maverick_int eval1DSplineD(void * spline_ptr, maverick_real x, maverick_real * out, char error_msg[], size_t err_msg_len) {
  try {
    Splines::Spline * spline = (Splines::Spline *) spline_ptr;
    *out = spline->eval_D(x);
  } catch (std::exception & exc) {
    strncpy(error_msg, exc.what(), err_msg_len);
    return 1;
  }
  return 0;
}

maverick_int eval1DSplineDD(void * spline_ptr, maverick_real x, maverick_real * out, char error_msg[], size_t err_msg_len) {
  try {
    Splines::Spline * spline = (Splines::Spline *) spline_ptr;
    *out = spline->eval_DD(x);
  } catch (std::exception & exc) {
    strncpy(error_msg, exc.what(), err_msg_len);
    return 1;
  }
  return 0;
}

maverick_int eval1DSplineVec(void * spline_ptr, size_t n, maverick_real const * x, maverick_real * out, char error_msg[], size_t err_msg_len) {
  try {
    Splines::Spline * spline = (Splines::Spline *) spline_ptr;
    for (size_t i=0; i<n; i++)
      out[i] = spline->eval(x[i]);
  } catch (std::exception & exc) {
    strncpy(error_msg, exc.what(), err_msg_len);
    return 1;
  }
  return 0;
}

maverick_int eval1DSplineDVec(void * spline_ptr, size_t n, maverick_real const * x, maverick_real * out, char error_msg[], size_t err_msg_len) {
  try {
    Splines::Spline * spline = (Splines::Spline *) spline_ptr;
    for (size_t i=0; i<n; i++)
      out[i] = spline->eval_D(x[i]);
  } catch (std::exception & exc) {
    strncpy(error_msg, exc.what(), err_msg_len);
    return 1;
  }
  return 0;
}

maverick_int eval1DSplineDDVec(void * spline_ptr, size_t n, maverick_real const * x, maverick_real * out, char error_msg[], size_t err_msg_len) {
  try {
    Splines::Spline * spline = (Splines::Spline *) spline_ptr;
    for (size_t i=0; i<n; i++)
      out[i] = spline->eval_DD(x[i]);
  } catch (std::exception & exc) {
    strncpy(error_msg, exc.what(), err_msg_len);
    return 1;
  }
  return 0;
}

size_t getSpline1DSize(void * p_spline, char error_msg[], size_t err_msg_len) {
  auto spline = (Splines::Spline *) p_spline;
  try {
    return spline->numPoints();
  } catch (std::exception & exc) {
    strncpy(error_msg, exc.what(), err_msg_len);
  }
  return 0;
}

maverick_int getSpline1DPoints(void * p_spline, size_t n, maverick_real * x, maverick_real * y, char error_msg[], size_t err_msg_len) {
  auto spline = (Splines::Spline *) p_spline;
  try {
    if (n != spline->numPoints()) {
      strncpy(error_msg, "Wrong size", err_msg_len);
      return 1;
    }
    for (size_t i=0; i<n; i++) {
      x[i] = spline->xNode(i);
      y[i] = spline->yNode(i);
    }
  } catch (std::exception & exc) {
    strncpy(error_msg, exc.what(), err_msg_len);
    return 1;
  }
  return 0;
}

void *create2DSpline(char const spline_type[], size_t nx, maverick_real const * x, size_t ny, maverick_real const * y,
                     maverick_real const * z, bool check_range, bool fortran_order, bool transposed, char error_msg[], size_t msg_len) {

  Splines::SplineSurf * spline = nullptr;

  try {
    string spline_type_str(spline_type);
    if (compareStringIgnoreCase(spline_type_str, string(SPLINE_BILINEAR))) spline = new Splines::BilinearSpline();
    else if (compareStringIgnoreCase(spline_type_str, SPLINE_BICUBIC)) spline = new Splines::BiCubicSpline();
    else if (compareStringIgnoreCase(spline_type_str, SPLINE_BIQUINTIC)) spline = new Splines::BiQuinticSpline();
    else {
      std::string mess = "unable to initialize spline. Requested spline of type " + spline_type_str +
                         " which is not available. Available types are: " + SPLINE_BILINEAR + ", "
                         + SPLINE_BICUBIC + ", " + SPLINE_BIQUINTIC + ".\n";
      strncpy(error_msg, mess.c_str(), msg_len);
      return nullptr;
    }

    auto x_vec = vec_1d_real(nx);
    auto y_vec = vec_1d_real(ny);
    auto z_vec = vec_1d_real(nx * ny);
    spline->build(x, 1, y, 1, z, 1, nx, ny, fortran_order, transposed);
    spline->setCheckRange(check_range);

  } catch (std::exception & exc) {
    strncpy(error_msg, exc.what(), msg_len);
    return nullptr;
  }

  return (void*) spline;
}

maverick_int eval2DSpline(void * spline_ptr, maverick_real x, maverick_real y, maverick_real * out, char error_msg[], size_t err_msg_len) {
  try {
    Splines::SplineSurf * spline = (Splines::SplineSurf *) spline_ptr;
    *out = spline->eval(x, y);
  } catch (std::exception & exc) {
    strncpy(error_msg, exc.what(), err_msg_len);
    return 1;
  }
  return 0;
}

maverick_int eval2DSplineD1(void * spline_ptr, maverick_real x, maverick_real y, maverick_real * out, char error_msg[], size_t err_msg_len) {
  try {
    Splines::SplineSurf * spline = (Splines::SplineSurf *) spline_ptr;
    *out = spline->eval_D_1(x, y);
  } catch (std::exception & exc) {
    strncpy(error_msg, exc.what(), err_msg_len);
    return 1;
  }
  return 0;
}

maverick_int eval2DSplineD2(void * spline_ptr, maverick_real x, maverick_real y, maverick_real * out, char error_msg[], size_t err_msg_len) {
  try {
    Splines::SplineSurf * spline = (Splines::SplineSurf *) spline_ptr;
    *out = spline->eval_D_2(x, y);
  } catch (std::exception & exc) {
    strncpy(error_msg, exc.what(), err_msg_len);
    return 1;
  }
  return 0;
}

maverick_int eval2DSplineD11(void * spline_ptr, maverick_real x, maverick_real y, maverick_real * out, char error_msg[], size_t err_msg_len) {
  try {
    Splines::SplineSurf * spline = (Splines::SplineSurf *) spline_ptr;
    *out = spline->eval_D_1_1(x, y);
  } catch (std::exception & exc) {
    strncpy(error_msg, exc.what(), err_msg_len);
    return 1;
  }
  return 0;
}

maverick_int eval2DSplineD22(void * spline_ptr, maverick_real x, maverick_real y, maverick_real * out, char error_msg[], size_t err_msg_len) {
  try {
    Splines::SplineSurf * spline = (Splines::SplineSurf *) spline_ptr;
    *out = spline->eval_D_2_2(x, y);
  } catch (std::exception & exc) {
    strncpy(error_msg, exc.what(), err_msg_len);
    return 1;
  }
  return 0;
}

maverick_int eval2DSplineD12(void * spline_ptr, maverick_real x, maverick_real y, maverick_real * out, char error_msg[], size_t err_msg_len) {
  try {
    Splines::SplineSurf * spline = (Splines::SplineSurf *) spline_ptr;
    *out = spline->eval_D_1_2(x, y);
  } catch (std::exception & exc) {
    strncpy(error_msg, exc.what(), err_msg_len);
    return 1;
  }
  return 0;
}

maverick_int eval2DSplineVec(void * spline_ptr, size_t n, maverick_real const * x, maverick_real const * y, maverick_real * out, char error_msg[], size_t err_msg_len) {
  try {
    Splines::SplineSurf * spline = (Splines::SplineSurf *) spline_ptr;
    for (size_t i=0; i<n; i++)
      out[i] = spline->eval(x[i], y[i]);
  } catch (std::exception & exc) {
    strncpy(error_msg, exc.what(), err_msg_len);
    return 1;
  }
  return 0;
}

maverick_int eval2DSplineD1Vec(void * spline_ptr, size_t n, maverick_real const * x, maverick_real const * y, maverick_real * out, char error_msg[], size_t err_msg_len) {
  try {
    Splines::SplineSurf * spline = (Splines::SplineSurf *) spline_ptr;
    for (size_t i=0; i<n; i++)
      out[i] = spline->eval_D_1(x[i], y[i]);
  } catch (std::exception & exc) {
    strncpy(error_msg, exc.what(), err_msg_len);
    return 1;
  }
  return 0;
}

maverick_int eval2DSplineD2Vec(void * spline_ptr, size_t n, maverick_real const * x, maverick_real const * y, maverick_real * out, char error_msg[], size_t err_msg_len) {
  try {
    Splines::SplineSurf * spline = (Splines::SplineSurf *) spline_ptr;
    for (size_t i=0; i<n; i++)
      out[i] = spline->eval_D_2(x[i], y[i]);
  } catch (std::exception & exc) {
    strncpy(error_msg, exc.what(), err_msg_len);
    return 1;
  }
  return 0;
}

maverick_int eval2DSplineD11Vec(void * spline_ptr, size_t n, maverick_real const * x, maverick_real const * y, maverick_real * out, char error_msg[], size_t err_msg_len) {
  try {
    Splines::SplineSurf * spline = (Splines::SplineSurf *) spline_ptr;
    for (size_t i=0; i<n; i++)
      out[i] = spline->eval_D_1_1(x[i], y[i]);
  } catch (std::exception & exc) {
    strncpy(error_msg, exc.what(), err_msg_len);
    return 1;
  }
  return 0;
}

maverick_int eval2DSplineD22Vec(void * spline_ptr, size_t n, maverick_real const * x, maverick_real const * y, maverick_real * out, char error_msg[], size_t err_msg_len) {
  try {
    Splines::SplineSurf * spline = (Splines::SplineSurf *) spline_ptr;
    for (size_t i=0; i<n; i++)
      out[i] = spline->eval_D_2_2(x[i], y[i]);
  } catch (std::exception & exc) {
    strncpy(error_msg, exc.what(), err_msg_len);
    return 1;
  }
  return 0;
}

maverick_int eval2DSplineD12Vec(void * spline_ptr, size_t n, maverick_real const * x, maverick_real const * y, maverick_real * out, char error_msg[], size_t err_msg_len) {
  try {
    Splines::SplineSurf * spline = (Splines::SplineSurf *) spline_ptr;
    for (size_t i=0; i<n; i++)
      out[i] = spline->eval_D_1_2(x[i], y[i]);
  } catch (std::exception & exc) {
    strncpy(error_msg, exc.what(), err_msg_len);
    return 1;
  }
  return 0;
}

void delete2DSpline(void * p_spline) {
  auto spline = (Splines::SplineSurf *) p_spline;
  delete spline;
}

size_t getSpline2DSizeX(void * p_spline, char error_msg[], size_t err_msg_len) {
  auto spline = (Splines::SplineSurf *) p_spline;
  try {
    return spline->numPointX();
  } catch (std::exception & exc) {
    strncpy(error_msg, exc.what(), err_msg_len);
  }
  return 0;
}

size_t getSpline2DSizeY(void * p_spline, char error_msg[], size_t err_msg_len) {
  auto spline = (Splines::SplineSurf *) p_spline;
  try {
    return spline->numPointY();
  } catch (std::exception & exc) {
    strncpy(error_msg, exc.what(), err_msg_len);
  }
  return 0;
}

maverick_int getSpline2DPoints(void * p_spline, size_t nx, maverick_real * x, size_t ny, maverick_real * y, maverick_real * z, char error_msg[], size_t err_msg_len) {
  auto spline = (Splines::SplineSurf *) p_spline;
  try {
    if (nx != spline->numPointX()) {
      strncpy(error_msg, "Wrong size X", err_msg_len);
      return 1;
    }
    if (ny != spline->numPointY()) {
      strncpy(error_msg, "Wrong size Y", err_msg_len);
      return 1;
    }
    for (size_t i=0; i<nx; i++)
      x[i] = spline->xNode(i);
    for (size_t i=0; i<ny; i++)
      y[i] = spline->yNode(i);
    for (size_t i=0; i<nx; i++)
      for (size_t j=0; j<ny; j++)
        z[i * ny + j] = spline->zNode(i, j);
  } catch (std::exception & exc) {
    strncpy(error_msg, exc.what(), err_msg_len);
    return 1;
  }
  return 0;
}