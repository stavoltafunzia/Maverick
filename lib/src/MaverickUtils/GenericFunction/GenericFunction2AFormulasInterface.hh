/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef GENERIC_FUNCTION_2A_FORMULAS_INTERFACE_HH
#define GENERIC_FUNCTION_2A_FORMULAS_INTERFACE_HH

#include "../ComponentBase/ComponentDefinitions.hh"

namespace MaverickUtils {

  class GenericFunction2AFormulasInterface {

  public:

    virtual ~GenericFunction2AFormulasInterface() {};

    virtual real funcEval(real const x1, real const x2) const = 0;

    virtual real funcEval_D_1(real const x1, real const x2) const = 0;

    virtual real funcEval_D_2(real const x1, real const x2) const = 0;

    virtual real funcEval_D_1_1(real const x1, real const x2) const = 0;

    virtual real funcEval_D_1_2(real const x1, real const x2) const = 0;

    virtual real funcEval_D_2_2(real const x1, real const x2) const = 0;
  };
}

#endif
