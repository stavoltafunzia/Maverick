/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef GENERIC_FUNCTION_1A_SPLINE_HH
#define GENERIC_FUNCTION_1A_SPLINE_HH

#include "GenericFunction1ABase.hh"
#include "MaverickSplines/Splines.hh"

namespace MaverickUtils {

	class GF1ASpline : public GenericFunction1ABase {

	public:
        
        enum ExtendRange {
            none = 0,
            keep_value = 1,
            keep_derivative = 2
        };

		GF1ASpline();

		~GF1ASpline();

        void setup( GC::GenericContainer const & gc );

        void setup( std::string const & spline_type, vec_1d_real const & x, vec_1d_real const & y, ExtendRange extend_range );

        void printParametersInfo( std::ostream & out ) const;

        inline real unscaledFuncEval(real const x1) const { return _p_spline->eval(x1); }
        inline real unscaledFuncEval_D_1(real const x1) const { return _p_spline->D(x1); }
        inline real unscaledFuncEval_D_1_1(real const x1) const { return _p_spline->DD(x1); }
        inline real unscaledFuncEval_D_1_1_1(real const x1) const { return _p_spline->DDD(x1); }

        void setCheckRange(bool check);

        void clear();

    private:

        Splines::Spline * _p_spline = nullptr;

        bool _check_range = false;

        bool _has_done_base_setup = false;
        
        ExtendRange _extend_range = keep_value;

        std::string _spline_type = "";

        //void buildSpline(GenericContainer const &gc);

        void setModelType();

    };
}

#endif
