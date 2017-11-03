/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef GENERIC_FUNCTION_2A_SPLINE_HH
#define GENERIC_FUNCTION_2A_SPLINE_HH

#include "GenericFunction2ABase.hh"
#include "MaverickSplines/Splines.hh"

namespace MaverickUtils {

	class GF2ASpline : public GenericFunction2ABase {

	public:

		GF2ASpline();

		~GF2ASpline();

        void setup( GC::GenericContainer const & gc );

        void setup( std::string const & spine_type, vec_1d_real const & x1, vec_1d_real const & x2, vec_1d_real const & y );

        void printParametersInfo( std::ostream & out ) const;

        inline real unscaledFuncEval(real const x1, real const x2) const { return       _p_spline->eval(x1, x2); }
        inline real unscaledFuncEval_D_1(real const x1, real const x2) const { return   _p_spline->Dx  (x1, x2); }
        inline real unscaledFuncEval_D_2(real const x1, real const x2) const { return   _p_spline->Dy  (x1, x2); }
        inline real unscaledFuncEval_D_1_1(real const x1, real const x2) const { return _p_spline->Dxx (x1, x2); }
        inline real unscaledFuncEval_D_1_2(real const x1, real const x2) const { return _p_spline->Dxy (x1, x2); }
        inline real unscaledFuncEval_D_2_2(real const x1, real const x2) const { return _p_spline->Dyy (x1, x2); }

    private:

        Splines::SplineSurf * _p_spline = nullptr;

        std::string _spline_type = "";

        bool _fortran_storage = false;
        bool _transposed      = false;
        bool _check_range     = false;

        //void buildSpline(GenericContainer const &gc);

        void setModelType();

    };
}

#endif
