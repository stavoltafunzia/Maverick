/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef GENERIC_FUNCTION_2ARG_POLY_THREE_HH
#define GENERIC_FUNCTION_2ARG_POLY_THREE_HH

#include "GenericFunction2ABase.hh"

namespace MaverickUtils {

  class GF2APolyThree : public GenericFunction2ABase {

  public:

    GF2APolyThree();

    ~GF2APolyThree();

    void setup(GC::GenericContainer const &gc);

    void printParametersInfo(std::ostream &out) const;

    real unscaledFuncEval(real const x1, real const x2) const;

    real unscaledFuncEval_D_1(real const x1, real const x2) const;

    real unscaledFuncEval_D_2(real const x1, real const x2) const;

    real unscaledFuncEval_D_1_1(real const x1, real const x2) const;

    real unscaledFuncEval_D_1_2(real const x1, real const x2) const;

    real unscaledFuncEval_D_2_2(real const x1, real const x2) const;

  private:

    real a0,
        a10, a01,
        a20, a11, a02,
        a30, a21, a12, a03;

    void setModelType();
  };
}

#endif
