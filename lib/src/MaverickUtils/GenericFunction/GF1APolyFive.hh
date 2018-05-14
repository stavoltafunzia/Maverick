/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef GENERIC_FUNCTION_1ARG_POLY_FIVE_HH
#define GENERIC_FUNCTION_1ARG_POLY_FIVE_HH

#include "GenericFunction1ABase.hh"

namespace MaverickUtils {

  class GF1APolyFive : public GenericFunction1ABase {

  public:

    GF1APolyFive();

    ~GF1APolyFive();

    void setup(GC::GenericContainer const &gc);

    void setA0(real const aa0) { a0 = aa0; }

    void setA1(real const aa1) { a1 = aa1; }

    void setA2(real const aa2) { a2 = aa2; }

    void setA3(real const aa3) { a3 = aa3; }

    void setA4(real const aa4) { a4 = aa4; }

    void setA5(real const aa5) { a5 = aa5; }

    void printParametersInfo(std::ostream &out) const;

    real unscaledFuncEval(real const x1) const;

    real unscaledFuncEval_D_1(real const x1) const;

    real unscaledFuncEval_D_1_1(real const x1) const;

    real unscaledFuncEval_D_1_1_1(real const x1) const;

  protected:

    real a0, a1, a2, a3, a4, a5;

    void setModelType();

  };
}

#endif
