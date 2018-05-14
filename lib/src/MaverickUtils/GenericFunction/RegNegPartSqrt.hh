/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef REG_NEG_PART_SQRT_HH
#define REG_NEG_PART_SQRT_HH

#include "GF1ASinAtan.hh"
#include "GenericFunction1ABase.hh"

namespace MaverickUtils {

  class RegNegPartSqrt : public GenericFunction1ABase {

  public:

    RegNegPartSqrt();

    ~RegNegPartSqrt();

    void setup(GC::GenericContainer const &gc);

    void printParametersInfo(std::ostream &out) const;

    real unscaledFuncEval(real const x1) const;

    real unscaledFuncEval_D_1(real const x1) const;

    real unscaledFuncEval_D_1_1(real const x1) const;

    real unscaledFuncEval_D_1_1_1(real const x1) const;

  protected:

    real _h = 0;

    void setModelType();

  };
}

#endif
