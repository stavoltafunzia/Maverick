#include "RegAbsValSqrt.hh"

using namespace MaverickUtils;

RegAbsValSqrt::RegAbsValSqrt() {
  setModelType();
}

RegAbsValSqrt::~RegAbsValSqrt() {}

void RegAbsValSqrt::setup(GC::GenericContainer const &gc) {
  GenericFunction1ABase::setup(gc);
  _h = findValueParamWithPrefixAndName("", "epsilon", gc);
}

void RegAbsValSqrt::printParametersInfo(std::ostream &out) const {
  out << "epsilon: " << _h;
}

void RegAbsValSqrt::setModelType() {
  _model = "Function 1 arg regularized absolute value by sqrt";
}

real RegAbsValSqrt::unscaledFuncEval(real const x1) const {
  return sqrt(x1 * x1 + _h * _h);
}

real RegAbsValSqrt::unscaledFuncEval_D_1(real const x1) const {
  real sqr = sqrt(x1 * x1 + _h * _h);
  return x1 / sqr;
}

real RegAbsValSqrt::unscaledFuncEval_D_1_1(real const x1) const {
  real h_2 = _h * _h;
  real sqr_inv = 1.0 / pow(x1 * x1 + h_2, 1.5);
  return h_2 * sqr_inv;
}

real RegAbsValSqrt::unscaledFuncEval_D_1_1_1(real const x1) const {
  real h_2 = _h * _h;
  real sqr_inv = 1.0 / pow(x1 * x1 + h_2, 2.5);
  return -3.0 * x1 * h_2 * sqr_inv;
}
