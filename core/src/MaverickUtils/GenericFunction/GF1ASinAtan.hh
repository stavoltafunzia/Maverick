/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef GENERIC_FUNCTION_1_SINATAN_HH
#define GENERIC_FUNCTION_1_SINATAN_HH

#include "GenericFunction1ABase.hh"

namespace MaverickUtils {

	class GF1ASinAtan : public GenericFunction1ABase {

    public:

		GF1ASinAtan();

		~GF1ASinAtan();

        void setup( GC::GenericContainer const & gc );

        void printParametersInfo( std::ostream & out ) const;

        real unscaledFuncEval(real const x1) const;
        real unscaledFuncEval_D_1(real const x1) const;
        real unscaledFuncEval_D_1_1(real const x1) const;
        real unscaledFuncEval_D_1_1_1(real const x1) const;

    protected:

		real left = 0;
        real right = 0;
        real sharpness = 0;
        real x0 = 0;

		void setModelType();

	};
}

#endif
