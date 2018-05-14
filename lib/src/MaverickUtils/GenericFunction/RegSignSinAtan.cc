#include "RegSignSinAtan.hh"

using namespace MaverickUtils;

RegSignSinAtan::RegSignSinAtan() {
  setModelType();
}

RegSignSinAtan::~RegSignSinAtan() {}

void RegSignSinAtan::setup(GC::GenericContainer const &gc) {
  sharpness = 0;
  GenericFunction1ABase::setup(gc);

  sharpness = findValueParamWithPrefixAndName("", "sharpness", gc);

}

void RegSignSinAtan::printParametersInfo(std::ostream &out) const {
  out << "sharpness: " << sharpness;
}

void RegSignSinAtan::setModelType() {
  _model = "Function 1 arg regularized sign by sin arctan";
}

real RegSignSinAtan::unscaledFuncEval(real const x1) const {
  real t4;
  real t5;
  real t8;
  real t14;
  t4 = (sharpness * sharpness);
  t5 = (x1 * x1);
  t8 = sqrt(t4 * t5 + 1);
  t14 = -1.0 + 2.0 * (0.1e1 + sharpness * x1 / t8) / 0.2e1;
  return t14;
}

real RegSignSinAtan::unscaledFuncEval_D_1(real const x1) const {
  real t2;
  real t4;
  real t6;
  real t7;
  real t17;
  t2 = sharpness * sharpness;
  t4 = pow(x1, 0.2e1);
  t6 = t2 * t4 + 0.1e1;
  t7 = sqrt(t6);
  t17 = (2.0) * (sharpness / t7 - t2 * sharpness * t4 / t7 / t6) / 0.2e1;
  return t17;
}

real RegSignSinAtan::unscaledFuncEval_D_1_1(real const x1) const {
  real t2;
  real t5;
  real t7;
  real t8;
  real t13;
  real t17;
  real t23;
  t2 = (sharpness * sharpness);
  t5 = (x1 * x1);
  t7 = t2 * t5 + 1;
  t8 = sqrt(t7);
  t13 = t2 * t2;
  t17 = t7 * t7;
  t23 = 0.3e1 / 0.2e1 * (2.0) * (-t2 * sharpness / t8 / t7 * x1 + t13 * sharpness * x1 * t5 / t8 / t17);
  return t23;
}

real RegSignSinAtan::unscaledFuncEval_D_1_1_1(real const x1) const {
  real t2;
  real t3;
  real t6;
  real t8;
  real t9;
  real t10;
  real t16;
  real t22;
  real t31;
  t2 = sharpness * sharpness;
  t3 = t2 * t2;
  t6 = pow(x1, 0.2e1);
  t8 = t2 * t6 + 0.1e1;
  t9 = t8 * t8;
  t10 = sqrt(t8);
  t16 = t2 * sharpness;
  t22 = t6 * t6;
  t31 = (2.0) *
        (0.18e2 * t3 * sharpness / t10 / t9 * t6 - 0.3e1 * t16 / t10 / t8 - 0.15e2 * t3 * t16 * t22 / t10 / t9 / t8) /
        0.2e1;
  return t31;
}
