/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef MAVERICK_MIDPOINT_OCP_SOLUTION_HH
#define MAVERICK_MIDPOINT_OCP_SOLUTION_HH

#include "MaverickCore/OcpSolution.hh"
#include "MaverickCore/OcpGuess.hh"
#include "MaverickCore/Midpoint/MidpointOcpSolutionSinglePhase.hh"
#include <iostream>

namespace Maverick {

    class MidpointOcpSolution : public OcpSolution {

    public:

        MidpointOcpSolution();
      
        static std::unique_ptr<MidpointOcpSolution> getFromGuessTablesForOcpProblem(std::vector<real_table> const & guess_table, MaverickOcp const & ocp_problem, std::vector<std::vector<std::string>> & found_variables);

        // copy constructor
        MidpointOcpSolution( MidpointOcpSolution const & ocp_solution );

        // destructor
        ~MidpointOcpSolution() {}

        // OcpGuess interface
        void evalAtMesh(integer const i_phase,
                        real    const zeta,

                        integer const num_states_controls,  real * states_controls,       real *states_controls_upper_bounds_mult, real * states_controls_lower_bounds_mult,
                        integer const num_alg_states_controls,  real * algebraic_states_controls,       real * algebraic_states_controls_upper_bounds_mult,  real * algebraic_states_controls_lower_bounds_mult,
                        integer const num_fo_eqns,          real * fo_eqns_mult,
                        integer const num_point_constr,     real * point_constr_mult,
                        integer const num_path_constr,      real * path_constr_mult
                        ) const;

        void eval(integer const i_phase,
                  real const initial_zeta, real const final_zeta,

                  integer const num_parameters,          real * parameters,               real * params_upper_bounds_mult,    real * params_lower_bounds_mult,
                  integer const num_boundary_conditions, real * boundary_conditions_mult,
                  integer const num_int_constr,          real * int_constr_mult
                  ) const;

        // OcpSolution interface

        virtual DiscretisationType discretisationType() const { return DiscretisationType::midpoint; }

        void evalAtMesh(integer const i_phase,
                        real      zeta,

                        integer const num_states_controls,  real * states_controls,       real * states_controls_upper_bounds_mult,  real * states_controls_lower_bounds_mult,
                        integer const num_alg_states_controls,  real * algebraic_states_controls,       real * algebraic_states_controls_upper_bounds_mult,  real * algebraic_states_controls_lower_bounds_mult,
                        integer const num_fo_eqns,          real * fo_eqns,               real * fo_eqns_mult,
                        integer const num_point_constr,     real * states_constr,         real * point_constr_mult,
                        integer const num_path_constr,      real * path_constr,           real * path_constr_mult,
                        integer const num_int_constr,       real * int_constr,

                        integer const num_post_proc,        real * post_proc,
                        integer const num_diff_post_proc,   real * diff_post_proc,
                        integer const num_int_post_proc,    real * int_post_proc
                        ) const;

        void eval(integer const i_phase,
                  real const initial_zeta, real const final_zeta,

                  integer const num_parameters,             real * parameters,            real * params_upper_bounds_mult,   real * params_lower_bounds_mult,
                  integer const num_boundary_conditions,    real * boundary_conditions,   real * boundary_conditions_mult,
                  integer const num_int_constr,             real * int_constr_at_end,     real * int_constr_mult,
                  integer const num_int_post_proc,          real * int_post_proc_at_end
                  ) const;

        // get number of phases
        integer getNumberOfPhases() const;

        // get the overall target
        real getTarget() const;

        void clear();

        // output

        /* write content to GenericContainer
          If the MaverickOcp pointer is null, states, controls, equations, etc will be saved
          in vectors (i.e. state vector at i-th element has state[i]).
          If the MaverickOcp pointer is NOT null, states, controls, equations, etc will be saved
          with their specific names read from the MaverickOcp object. */
        void writeContentToGC(GC::GenericContainer & out_gc, MaverickOcp const * const p_ocp) const;

        void writeAllMeshVarsToStream( std::ostream & out ) const ;

        void writeOnePhaseMeshVarsToStream( integer const i_phase, std::ostream & out ) const;

        // get a copy
        virtual std::unique_ptr<OcpSolution> copy() const;

        // opertators

        OcpSolutionSinglePhase const & operator[](integer const i_phase) const;

        // additional methods

        // set a specific solution for a phase
        void setSolutionAtPhase( integer const i_phase, MidpointOcpSolutionSinglePhase const & ocp_solution );

        // opertators

        MidpointOcpSolutionSinglePhase const & operator()(integer const i_phase) const;

        MidpointOcpSolutionSinglePhase & operator()(integer const i_phase);

        MidpointOcpSolution & operator=(MidpointOcpSolution const & ocp_solution);

        MidpointOcpSolution & operator<<(MidpointOcpSolutionSinglePhase const & ocp_solution);

    protected:

        std::vector< MidpointOcpSolutionSinglePhase > _solutions;

    };
}

#endif
