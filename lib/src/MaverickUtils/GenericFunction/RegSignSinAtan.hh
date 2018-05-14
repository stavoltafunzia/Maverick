/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef REG_SIGN_SIN_ATAN_HH
#define REG_SIGN_SIN_ATAN_HH

#include "GenericFunction1ABase.hh"

namespace MaverickUtils {

  class RegSignSinAtan : public GenericFunction1ABase {

  public:

    RegSignSinAtan();

    ~RegSignSinAtan();

    void setup(GC::GenericContainer const &gc);

    void printParametersInfo(std::ostream &out) const;

    real unscaledFuncEval(real const x1) const;

    real unscaledFuncEval_D_1(real const x1) const;

    real unscaledFuncEval_D_1_1(real const x1) const;

    real unscaledFuncEval_D_1_1_1(real const x1) const;

  protected:

    real sharpness = 0;

    void setModelType();

  };
}

#endif
