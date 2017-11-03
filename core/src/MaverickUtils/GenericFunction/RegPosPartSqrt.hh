/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef REG_POS_PART_SQRT_HH
#define REG_POS_PART_SQRT_HH

#include "GF1ASinAtan.hh"
#include "GenericFunction1ABase.hh"

namespace MaverickUtils {

	class RegPosPartSqrt : public GenericFunction1ABase {

    public:

		RegPosPartSqrt();

		~RegPosPartSqrt();

        void setup( GC::GenericContainer const & gc );

        void printParametersInfo( std::ostream & out ) const;

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
