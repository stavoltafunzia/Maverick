#include "MaverickCInterface.h"
#include "MaverickCore/OcpSolverImpl.hh"
#include "MaverickGC/GenericContainerLuaInterface.hh"
#include "MaverickCore/MaverickPrivateDefinitions.hh"

using namespace Maverick;
using namespace std;

static std::map<int, OcpSolverImpl *> map_solver;
static std::map<int, GC::GenericContainer *> map_gc;
static std::map<int, MaverickOcp *> map_ocp;
static std::map<MaverickOcp *, void *> map_open_lib;

template<typename T>
static int getNewIdForMap(std::map<int, T> const &mappa) {
  if (mappa.empty()) return 1;

  int id = 0;

  int const last_id = mappa.rbegin()->first;

  for (int i = last_id; i < std::numeric_limits<int>::max(); i++) {
    if (mappa.find(i) == mappa.end()) {
      id = i;
      break;
    }
  }

  if (id != 0) return id;

  for (int i = std::numeric_limits<int>::min(); i < last_id; i++) {
    if (mappa.find(i) == mappa.end()) {
      id = i;
      break;
    }
  }

  return id;
}

//#ifdef __cplusplus
//extern "C" {
//#endif

/* create a MaverickOcp object loaded from the shared library specified. The object is kept inernally and
 an ID is returned for further use.
 Error codes are:  0  = succsefull
 1  = shared library not found
 2  = shared library do not expose the method 'void * getMaverickOcpPointer()'
 -1  = error in creating the object (e.g. not enough available) memory
 */
int loadMaverickOcpFromLib(char const lib_name[], int *const p_err_code,
                           char **error) {
  MaverickOcp *ocp = (MaverickOcp *) getMaverickOcpFromLib(lib_name, p_err_code, error);
  if (*p_err_code != 0)
    return 0;

  int id = getNewIdForMap(map_ocp);
  if (id == 0) {
    *p_err_code = -1;
    int tmp;
    deleteMaverickOcp(ocp, &tmp);
    return 0;
  }
  *p_err_code = -1;
  map_ocp.insert(std::pair<int, MaverickOcp *>(id, ocp));
  *p_err_code = 0;
  return id;
}

/* delete a MaverickOcp object which has the provided ID
 Error codes are: 0 = succsefull
 1 = ID not found (object non existing)
 */
void unloadMaverickOcp(int const maverick_ocp_id, int *const p_err_code) {
  std::map<int, MaverickOcp *>::iterator it = map_ocp.find(maverick_ocp_id);

  *p_err_code = 0;
  if (it == map_ocp.end()) {
    *p_err_code = 1;
    return;
  }

  deleteMaverickOcp((void *) it->second, p_err_code);
  if (*p_err_code == 0)
    map_ocp.erase(it);
}


/* create a MaverickOcp object loaded from the shared library specified. The object pointer is returned and
 the management is left to the user
 Error codes are: 0 = succsefull
 1 = shared library not found
 2 = shared library do not expose the method 'void * getMaverickOcpPointer()'
 -1 = error in creating the object (e.g. not enough available) memory
 */
void *getMaverickOcpFromLib(char const lib_name[], int *const p_err_code,
                            char **error) {
  MaverickOcp *p_maverick_ocp = nullptr;

  void *lib_handle = nullptr;
  lib_handle = dlopen(lib_name, RTLD_LAZY);

  if (lib_handle == nullptr) {
    *p_err_code = 1;
    *error = dlerror();
    return nullptr;
  }

  typedef MaverickOcp *(*GetMaverickOcp)();

  GetMaverickOcp p_get_maverick_ocp = nullptr;

  p_get_maverick_ocp = (GetMaverickOcp) dlsym(lib_handle, "getMaverickOcpPointer");

  // if sl_handle exist
  if (p_get_maverick_ocp != nullptr) {
    try {
      p_maverick_ocp = p_get_maverick_ocp();
    } catch (...) {
      *p_err_code = -1;
      return nullptr;
    }
  } else {
    *p_err_code = 2;
    return nullptr;
  }

  map_open_lib.insert(std::pair<MaverickOcp *, void *>(p_maverick_ocp, lib_handle));
  return p_maverick_ocp;
}

/* delete the MaverickOcp object pointed by the pointer
 Error codes are: 0 = succsefull
 1 = null pointer
 */
void deleteMaverickOcp(void *const p_maverick_ocp, int *const p_err_code) {
  MaverickOcp *ocp = (MaverickOcp *) p_maverick_ocp;
  if (ocp == NULL) {
    *p_err_code = 1;
    return;
  }
  delete ocp;
  *p_err_code = 0;

  // look for opened shared library with that object
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
}


//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// MAVERICK OCP

/* create a MaverickOcpSolver object binded to the MaverickOcp object with the provided ID. Only one solver
 for each MaverickOcp object can exists. The ID to use for furhter usage of the solver is the same of
 the MaverickOcp object.
 Error codes are: 0 = succsefull
 1 = a Solver for the provided MaverickOcp ID already exists
 2 = MaverickOcp object not found for the given ID
 -1 = error in creating the object (e.g. not enough available) memory
 */
int loadMaverickOcpSolver(int maverick_ocp_id, int *const p_err_code) {
  // check if the MaverickOcp exists
  std::map<int, MaverickOcp *>::iterator it = map_ocp.find(maverick_ocp_id);

  if (it == map_ocp.end()) {
    *p_err_code = 2;
    return 0;
  }

  // get new ID for the ocp solver
  int id = getNewIdForMap(map_solver);
  if (id == 0) {
    *p_err_code = -1;
    return 0;
  }

  // check for duplicate solver
  for (std::map<int, OcpSolverImpl *>::iterator its = map_solver.begin(); its != map_solver.end(); its++) {
    MaverickOcp const *tmp = &((its->second)->getOcpProblem());
    if (tmp == it->second) {
      *p_err_code = 1;
      return 0;
    }
  }

  // now proceed
  *p_err_code = -1;
  OcpSolverImpl *solver = (OcpSolverImpl *) getMaverickOcpSolver(it->second, p_err_code);
  if (*p_err_code != 0)
    return 0;

  *p_err_code = -1;
  map_solver.insert(std::pair<int, OcpSolverImpl *>(id, solver));
  *p_err_code = 0;

  return id;
}

/* delete the MaverickOcpSolver object with the provided ID.
 Error codes are: 0 = succsefull
 1 = ID not found (object non existing)
 */
void unloadMaverickOcpSolver(int maverick_ocp_id, int *const p_err_code) {
  std::map<int, OcpSolverImpl *>::iterator it = map_solver.find(maverick_ocp_id);

  if (it == map_solver.end()) {
    *p_err_code = 1;
    return;
  }

  delete it->second;
  map_solver.erase(it);
  *p_err_code = 0;
}

/* create a MaverickOcpSolver object binded to the MaverickOcp object provided.
 The object pointer is returned the management is left to the user.
 Error codes are: 0 = succsefull
 1 = null MaverickOcp pointer
 -1 = error in creating the object (e.g. not enough available) memory
 */
void *getMaverickOcpSolver(void *const p_maverick_ocp, int *const p_err_code) {
  // check for NULL pointer
  if (p_maverick_ocp == NULL) {
    *p_err_code = 1;
    return NULL;
  }

  //create the solver
  *p_err_code = -1;
  MaverickOcp *ocp = (MaverickOcp *) p_maverick_ocp;
  OcpSolverImpl *solver = new OcpSolverImpl(*ocp);
  *p_err_code = 0;
  return (void *) solver;
}

/* delete the MaverickOcpSolver object with the provided ID.
 Error codes are: 0 = succsefull
 1 = null pointer
 */
void deleteMaverickOcpSolver(void *solver, int *const p_err_code) {
  OcpSolverImpl *_solver = (OcpSolverImpl *) solver;
  if (_solver == NULL) {
    *p_err_code = 1;
    return;
  }
  delete _solver;
  *p_err_code = 0;
}

/* Search the maverick solver with the ID specified and calls the 'solve' method passing the GenericContainer
 with the provided id. The solution is saved in the output generic container passed through pointer
 Error codes are: 0 = succsefull
 1 = Solver ID not found
 2 = input GenericContainer ID not found
 3 = Error in computing the solution
 4 = output GenericContainer pointer null
 5 = error in writing the solution to the output generic container
 */
void
solveFromInternalGC(int maverick_id, int input_generic_container_id, int sol_type, void *p_output_generic_container,
                    int *const p_err_code,
                    char error[], size_t error_length) {

  // check that the generic container exists
  std::map<int, GC::GenericContainer *>::iterator it = map_gc.find(input_generic_container_id);
  if (it == map_gc.end()) {
    *p_err_code = 2;
    return;
  }

  solveFromExternalGC(maverick_id, it->second, sol_type, p_output_generic_container,
                      p_err_code,
                      error, error_length);
}

/* Search the maverick solver with the ID specified and calls the 'solve' method passing the GenericContainer
 passed thorugh pointer. The solution is saved in the output generic container passed through pointer.
 Error codes are: 0 = succsefull
 1 = Solver ID not found
 2 = input GenericContainer pointer null
 3 = Error in computing the solution
 4 = output GenericContainer pointer null
 5 = error in writing the solution to the output generic container
 */
void
solveFromExternalGC(int maverick_id, void *p_input_generic_container, int sol_type, void *p_output_generic_container,
                    int *const p_err_code,
                    char error[], size_t error_length) {

  // check if solver exists
  std::map<int, OcpSolverImpl *>::iterator it = map_solver.find(maverick_id);
  if (it == map_solver.end()) {
    *p_err_code = 1;
    return;
  }

  // if exists then solve
  solveFromExternalSolver(it->second, p_input_generic_container, sol_type, p_output_generic_container,
                          p_err_code,
                          error, error_length);
}

/* call the 'solve' method to the solver passed through pointer and pass as argument the GenericContainer
 passed thorugh pointer. The solution is saved in the output generic container passed through pointer.
 Error codes are: 0 = succsefull
 1 = Solver pointer null
 2 = input GenericContainer pointer null
 3 = Error in computing the solution
 4 = output GenericContainer pointer null
 5 = error in writing the solution to the output generic container
 */
void solveFromExternalSolver(void *p_maverick_solver, void *p_input_generic_container, int sol_type,
                             void *p_output_generic_container,
                             int *const p_err_code,
                             char error[], size_t error_length) {
  if (p_maverick_solver == NULL) {
    *p_err_code = 1;
    return;
  }

  if (p_input_generic_container == NULL) {
    *p_err_code = 2;
    return;
  }

  if (p_output_generic_container == NULL) {
    *p_err_code = 4;
    return;
  }

  OcpSolverOutput out;
  // solve
  try {
    out = ((MaverickOcpSolver *) p_maverick_solver)->solve(*((GC::GenericContainer *) p_input_generic_container));
  } catch (std::exception &exc) {
    std::string mess = exc.what();
    std::strcpy(error, mess.substr(0, error_length - 1).c_str());
    *p_err_code = 3;
    return;
  }

  ((GC::GenericContainer *) p_output_generic_container)->clear();

  if (sol_type == 0)
    out.writeContentToGC(*((GC::GenericContainer *) p_output_generic_container), nullptr);
  else
    out.writeContentToGC(*((GC::GenericContainer *) p_output_generic_container),
                         &(((MaverickOcpSolver *) p_maverick_solver)->getOcpProblem()));
}





//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// GENERIC CONTAINER

/* create a GenericContainer object and inflate it with the specified global variable
 form the specified lua file. The object is kept inernally and an ID is returned for further use.
 Error codes are:  0  = succsefull
 1  = error inflating lua
 2  = error converting lua to gc
 -1  = error in creating the object (e.g. not enough available) memory
 */
int loadGenericContainerFromLua(char const lua_name[], char const lua_global_var[],
                                int print_content,
                                int *const p_err_code,
                                char error[], size_t error_length) {
  GC::GenericContainer *gc = (GC::GenericContainer *) getGenericContainerFromLua(lua_name, lua_global_var,
                                                                                 p_err_code,
                                                                                 error, error_length);
  if (*p_err_code != 0)
    return 0;

  int id = getNewIdForMap(map_gc);
  if (id == 0) {
    *p_err_code = -1;
    return 0;
  }

  *p_err_code = -1;
  map_gc.insert(std::pair<int, GC::GenericContainer *>(id, gc));
  *p_err_code = 0;

  if (print_content)
    gc->print(std::cout);

  return id;
}

/* delete a GenericContainer object which has the provided ID.
 Error codes are: 0 = succsefull
 1 = ID not found (object non existing)
 */
void unloadGenericContainer(int const generic_container_id, int *const p_err_code) {
  std::map<int, GC::GenericContainer *>::iterator it = map_gc.find(generic_container_id);

  if (it == map_gc.end()) {
    *p_err_code = 1;
    return;
  }

  delete it->second;
  map_gc.erase(it);
  *p_err_code = 0;
}

/* create a GenericContainer object and inflate it with the specified global variable
 form the specified lua file. The object pointer is returned and the management is left
 to the user
 Error codes are: 0 = succsefull
 1  = error inflating lua
 2  = error converting lua to gc
 -1  = error in creating the object (e.g. not enough available) memory
 */
void *getGenericContainerFromLua(char const lua_name[], char const lua_global_var[],
                                 int *const p_err_code,
                                 char error[], size_t error_length) {

  *p_err_code = -1;
  GC::GenericContainer *gc = nullptr;
  gc = new GC::GenericContainer();
  if (gc == nullptr)
    return NULL;

  fillGenericContainerFromLua((void *) gc, lua_name, lua_global_var,
                              p_err_code,
                              error, error_length);

  if (*p_err_code != 0)
    return NULL;

  return (void *) gc;
}

/* delete the GenericContainer object pointed by the pointer
 Error codes are: 0 = succsefull
 1 = null pointer
 */
void deleteGenericContainer(void *const p_generic_container, int *const p_err_code) {
  GC::GenericContainer *gc = (GC::GenericContainer *) p_generic_container;
  if (gc == NULL) {
    *p_err_code = 1;
    return;
  }
  delete gc;
  *p_err_code = 0;
}

/* fill a generic container reading data from a lua file.
Error codes are: 0 = succsefull
                 1  = error inflating lua
                 2  = error converting lua to gc
                 3  = null generic container pointer
*/
void fillGenericContainerFromLua(void *const p_generic_container,
                                 char const lua_file[], char const lua_global_var[],
                                 int *const p_err_code,
                                 char error[], size_t error_length) {

  if (p_generic_container == NULL) {
    *p_err_code = 3;
    return;
  }

  GC::LuaInterpreter lua;
  try {
    lua.do_file(lua_file);
  } catch (std::exception &exc) {
    std::string mess = exc.what();
    std::strcpy(error, mess.substr(0, error_length - 1).c_str());
    *p_err_code = 1;
    return;
  }

  try {
    lua.global_to_GC(lua_global_var, *((GC::GenericContainer *) p_generic_container));
  } catch (std::exception &exc) {
    std::string mess = exc.what();
    std::strcpy(error, mess.substr(0, error_length - 1).c_str());
    *p_err_code = 2;
    return;
  }

  *p_err_code = 0;
}



//#ifdef __cplusplus
//}
//#endif
