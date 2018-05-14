#include "GF1ASinAtan.hh"

using namespace MaverickUtils;

GF1ASinAtan::GF1ASinAtan() {
  setModelType();
}

GF1ASinAtan::~GF1ASinAtan() {}

void GF1ASinAtan::setup(GC::GenericContainer const &gc) {
  left = 0;
  right = 0;
  sharpness = 0;
  x0 = 0;
  GenericFunction1ABase::setup(gc);

  try { x0 = findValueParamWithPrefixAndName("", "x0", gc); } catch (...) {}

  left = findValueParamWithPrefixAndName("", "left", gc);
  right = findValueParamWithPrefixAndName("", "right", gc);
  sharpness = findValueParamWithPrefixAndName("", "sharpness", gc);

}

void GF1ASinAtan::printParametersInfo(std::ostream &out) const {
  out << "left: " << left << "\tright: " << right << "\tsharpness: " << sharpness << "\tx0: " << x0;
}

void GF1ASinAtan::setModelType() {
  _model = "Function 1 arg sin arctan";
}

real GF1ASinAtan::unscaledFuncEval(real const x1) const {
  real t2;
  real t4;
  real t5;
  real t8;
  real t14;
  t2 = x1 - x0;
  t4 = (sharpness * sharpness);
  t5 = (t2 * t2);
  t8 = sqrt(t4 * t5 + 1);
  t14 = left + (right - left) * (0.1e1 + sharpness * t2 / t8) / 0.2e1;
  return t14;
}

real GF1ASinAtan::unscaledFuncEval_D_1(real const x1) const {
  real t2;
  real t4;
  real t6;
  real t7;
  real t17;
  t2 = sharpness * sharpness;
  t4 = pow(x1 - x0, 0.2e1);
  t6 = t2 * t4 + 0.1e1;
  t7 = sqrt(t6);
  t17 = (right - left) * (sharpness / t7 - t2 * sharpness * t4 / t7 / t6) / 0.2e1;
  return t17;
}

real GF1ASinAtan::unscaledFuncEval_D_1_1(real const x1) const {
  real t2;
  real t4;
  real t5;
  real t7;
  real t8;
  real t13;
  real t17;
  real t23;
  t2 = (sharpness * sharpness);
  t4 = x1 - x0;
  t5 = (t4 * t4);
  t7 = t2 * t5 + 1;
  t8 = sqrt(t7);
  t13 = t2 * t2;
  t17 = t7 * t7;
  t23 = 0.3e1 / 0.2e1 * (right - left) * (-t2 * sharpness / t8 / t7 * t4 + t13 * sharpness * t4 * t5 / t8 / t17);
  return t23;
}

real GF1ASinAtan::unscaledFuncEval_D_1_1_1(real const x1) const {
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
  t6 = pow(x1 - x0, 0.2e1);
  t8 = t2 * t6 + 0.1e1;
  t9 = t8 * t8;
  t10 = sqrt(t8);
  t16 = t2 * sharpness;
  t22 = t6 * t6;
  t31 = (right - left) *
        (0.18e2 * t3 * sharpness / t10 / t9 * t6 - 0.3e1 * t16 / t10 / t8 - 0.15e2 * t3 * t16 * t22 / t10 / t9 / t8) /
        0.2e1;
  return t31;
}
