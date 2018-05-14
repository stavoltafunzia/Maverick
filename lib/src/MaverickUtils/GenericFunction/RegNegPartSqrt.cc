#include "RegNegPartSqrt.hh"

using namespace MaverickUtils;

RegNegPartSqrt::RegNegPartSqrt() {
  setModelType();
}

RegNegPartSqrt::~RegNegPartSqrt() {}

void RegNegPartSqrt::setup(GC::GenericContainer const &gc) {
  GenericFunction1ABase::setup(gc);
  _h = findValueParamWithPrefixAndName("", "epsilon", gc);
}

void RegNegPartSqrt::printParametersInfo(std::ostream &out) const {
  out << "epsilon: " << _h;
}

void RegNegPartSqrt::setModelType() {
  _model = "Function 1 arg regularized negative part by sqrt";
}

real RegNegPartSqrt::unscaledFuncEval(real const x1) const {
  return 0.5 * (x1 - sqrt(x1 * x1 + _h * _h));
}

real RegNegPartSqrt::unscaledFuncEval_D_1(real const x1) const {
  real sqr = sqrt(x1 * x1 + _h * _h);
  return 0.5 * (1 - x1 / sqr);
}

real RegNegPartSqrt::unscaledFuncEval_D_1_1(real const x1) const {
  real h_2 = _h * _h;
  real sqr_inv = 1.0 / pow(x1 * x1 + h_2, 1.5);
  return -0.5 * h_2 * sqr_inv;
}

real RegNegPartSqrt::unscaledFuncEval_D_1_1_1(real const x1) const {
  real h_2 = _h * _h;
  real sqr_inv = 1.0 / pow(x1 * x1 + h_2, 2.5);
  return 1.5 * x1 * h_2 * sqr_inv;
}
