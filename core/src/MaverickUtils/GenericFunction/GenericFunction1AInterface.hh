/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef MDRG_GENERIC_FUNCTION_INTERFACE_HH
#define MDRG_GENERIC_FUNCTION_INTERFACE_HH

#include "GenericFunction1AFormulasInterface.hh"
#include "../ComponentBase/ComponentInterface.hh"

namespace MaverickUtils {
	class GenericFunction1AInterface : public GenericFunction1AFormulasInterface, public ComponentInterface {

	public:

			virtual ~GenericFunction1AInterface() {};

	};
}

#endif
