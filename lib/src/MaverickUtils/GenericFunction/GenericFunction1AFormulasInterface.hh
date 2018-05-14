/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef GENERIC_FUNCTION_1A_FORMULAS_INTERFACE_HH
#define GENERIC_FUNCTION_1A_FORMULAS_INTERFACE_HH

#include "../ComponentBase/ComponentDefinitions.hh"

namespace MaverickUtils {

  class GenericFunction1AFormulasInterface {

  public:

    virtual ~GenericFunction1AFormulasInterface() {};

    virtual real funcEval(real const x1) const = 0;

    virtual real funcEval_D_1(real const x1) const = 0;

    virtual real funcEval_D_1_1(real const x1) const = 0;

    virtual real funcEval_D_1_1_1(real const x1) const = 0;
  };
}

#endif
