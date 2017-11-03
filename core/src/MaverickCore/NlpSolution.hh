/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef NLP_SOLUTION_HH
#define NLP_SOLUTION_HH

#include "Nlp.hh"
#include "MaverickDefinitions.hh"

namespace Maverick {

    class NlpSolution : public Nlp {

    public:

        NlpSolution();

        NlpSolution( NlpSolution const & nlp_solution );

        NlpSolution( Nlp const & nlp, integer const solver_return_status );

        void setSolution( size const n_y, real const y[], real const upper_bounds_multipliers[], real const lower_bounds_multipliers[],
                          size const n_c, real const constraints[], real const constraints_multiplier[],
//                          real const lagrange_mayer_percentage,
                          integer const solver_return_status);

        void setSolverReturnStatus( integer const status );

        integer getSolverReturnStatus() const;

        void clear();

    protected:

        integer _solver_return_status;

    };
}

#endif
