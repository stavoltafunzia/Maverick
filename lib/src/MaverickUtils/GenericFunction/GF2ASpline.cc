#include "GF2ASpline.hh"
#include "MaverickCore/MaverickPrivateDefinitions.hh"
#include "MaverickCore/MaverickFunctions.hh"

using namespace MaverickUtils;
using namespace std;
using namespace SplinesLoad;

GF2ASpline::GF2ASpline() {
  setModelType();
}

GF2ASpline::~GF2ASpline() {
  if (_p_spline != nullptr)
    delete _p_spline;
}

void GF2ASpline::setup(GC::GenericContainer const &gc) {

  GenericFunction2ABase::setup(gc);

  string requested_type = findStringParamWithPrefixAndName("", "spline_type", gc);

  //check fortran order
  string yesno = "no";
  if (gc.get_if_exists("fortran_order", yesno)) {
    if ((yesno.compare("yes") == 0) || (yesno.compare("Yes") == 0) || (yesno.compare("YES") == 0))
      _fortran_order = true;
  } else {
    gc.get_if_exists("fortran_order", _fortran_order);
  }

  //check transposed
  yesno = "no";
  if (gc.get_if_exists("transposed", yesno)) {
    if ((yesno.compare("yes") == 0) || (yesno.compare("Yes") == 0) || (yesno.compare("YES") == 0))
      _transposed = true;
  } else {
    gc.get_if_exists("transposed", _transposed);
  }

  vec_1d_real const x1_data = findVectorParamWithPrefixAndName("", "x1", gc);
  vec_1d_real const x2_data = findVectorParamWithPrefixAndName("", "x2", gc);
  vec_1d_real const y_data = findVectorParamWithPrefixAndName("", "y", gc);

  //check for rage_check
  gc.get_if_exists("check_range", _check_range);

  setup(requested_type, x1_data, x2_data, y_data, _check_range, _fortran_order, _transposed);

}

void GF2ASpline::setup(string const &spine_type, vec_1d_real const &x1, vec_1d_real const &x2, vec_1d_real const &y,
                       bool check_range, bool fortran_order, bool transposed) {
  if (_p_spline != nullptr)
    delete _p_spline;

  if (compareStringIgnoreCase(spine_type, SPLINE_BILINEAR)) _p_spline = new BilinearSpline();
  else if (compareStringIgnoreCase(spine_type, SPLINE_BICUBIC)) _p_spline = new BiCubicSpline();
  else if (compareStringIgnoreCase(spine_type, SPLINE_BIQUINTIC)) _p_spline = new BiQuinticSpline();
  else {
    std::string error_mess = "unable to initialize spline. Requested spline of type " + spine_type +
                             " which is not available. Available types are: " + SPLINE_BILINEAR + ", "
                             + SPLINE_BICUBIC + ", " + SPLINE_BIQUINTIC + ".\n";
    std::runtime_error exc(error_mess);
    throw exc;
  }
  _spline_type = spine_type;

  auto nx1 = x1.size();
  auto nx2 = x2.size();
  auto ny = y.size();

  MAVERICK_ASSERT(nx1 * nx2 == ny, "GF2ASpline::setup: y vector size does not equal size(x1) times size(x2)\n")
  _fortran_order = fortran_order;
  _transposed = transposed;
  _check_range = check_range;

  _p_spline->clear();
  _p_spline->build(x1, x2, y, _fortran_order, _transposed);
  _p_spline->setCheckRange(_check_range);
}

void GF2ASpline::printParametersInfo(std::ostream &out) const {
  if (_p_spline != nullptr) {
    out << "Spline type: " << _spline_type;

    out << ", fortran_order: ";
    if (_fortran_order)
      out << "yes";
    else
      out << "no";
    out << ", ";

    out << ", transposed: ";
    if (_transposed)
      out << "yes";
    else
      out << "no";
    out << ", ";

    out << ", check_range: ";
    if (_check_range)
      out << "yes";
    else
      out << "no";
    out << ", ";

    GenericFunction2ABase::printParametersInfo(out);

  } else {
    out << "Spline not initialized yet.\n";
  }
}

void GF2ASpline::setModelType() {
  _model = "Function 2 arg spline";
}
