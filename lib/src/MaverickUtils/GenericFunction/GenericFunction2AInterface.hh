/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef MDRG_GENERIC_FUNCTION_2A_INTERFACE_HH
#define MDRG_GENERIC_FUNCTION_2A_INTERFACE_HH

#include "GenericFunction2AFormulasInterface.hh"
#include "../ComponentBase/ComponentInterface.hh"

namespace MaverickUtils {
  class GenericFunction2AInterface : public GenericFunction2AFormulasInterface, public ComponentInterface {

  public:

    virtual ~GenericFunction2AInterface() {};

  };
}

#endif
