/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef GENERIC_FUNCTION_2A_BASE_HH
#define GENERIC_FUNCTION_2A_BASE_HH

#include "GenericFunction2AInterface.hh"
#include "../ComponentBase/ComponentBase.hh"

namespace MaverickUtils {

  class GenericFunction2ABase : public GenericFunction2AInterface, public ComponentBase {

  public:

    virtual ~GenericFunction2ABase() {}

    virtual void setup(GC::GenericContainer const &gc);

    virtual void printParametersInfo(std::ostream &out) const;

    inline real funcEval(real const x1, real const x2) const {
      return _y_scale * unscaledFuncEval(x1 * _x1_scale_inv, x2 * _x2_scale_inv);
    }

    inline real funcEval_D_1(real const x1, real const x2) const {
      return _y_scale * _x1_scale_inv * unscaledFuncEval_D_1(x1 * _x1_scale_inv, x2 * _x2_scale_inv);
    }

    inline real funcEval_D_2(real const x1, real const x2) const {
      return _y_scale * _x2_scale_inv * unscaledFuncEval_D_2(x1 * _x1_scale_inv, x2 * _x2_scale_inv);
    }

    inline real funcEval_D_1_1(real const x1, real const x2) const {
      return _y_scale * _x1_scale_inv * _x1_scale_inv * unscaledFuncEval_D_1_1(x1 * _x1_scale_inv, x2 * _x2_scale_inv);
    }

    inline real funcEval_D_1_2(real const x1, real const x2) const {
      return _y_scale * _x1_scale_inv * _x2_scale_inv * unscaledFuncEval_D_1_2(x1 * _x1_scale_inv, x2 * _x2_scale_inv);
    }

    inline real funcEval_D_2_2(real const x1, real const x2) const {
      return _y_scale * _x2_scale_inv * _x2_scale_inv * unscaledFuncEval_D_2_2(x1 * _x1_scale_inv, x2 * _x2_scale_inv);
    }

    virtual real unscaledFuncEval(real const x1, real const x2) const = 0;

    virtual real unscaledFuncEval_D_1(real const x1, real const x2) const = 0;

    virtual real unscaledFuncEval_D_2(real const x1, real const x2) const = 0;

    virtual real unscaledFuncEval_D_1_1(real const x1, real const x2) const = 0;

    virtual real unscaledFuncEval_D_1_2(real const x1, real const x2) const = 0;

    virtual real unscaledFuncEval_D_2_2(real const x1, real const x2) const = 0;


    inline virtual std::string getComponentType() const { return ComponentBase::getComponentType(); }

    virtual void printInfo(std::ostream &out, InfoLevel info_level) const {
      ComponentBase::printInfo(out, info_level);
    };

  protected:

    real _x1_scale_inv = 1;
    real _x2_scale_inv = 1;
    real _y_scale = 1;

  };
}

#endif
