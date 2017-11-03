/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef MAVERICK_OCP_SOLUTION_SINGLE_PHASE_HH
#define MAVERICK_OCP_SOLUTION_SINGLE_PHASE_HH

#include "MaverickCore/MaverickDefinitions.hh"
#include "MaverickGC/GenericContainer.hh"
#include <memory>

namespace Maverick {

    class MaverickOcp;

    class OcpSolutionSinglePhase {

    public:

        // destructor
        virtual ~OcpSolutionSinglePhase();

        // full evaluation of the phase solution
        virtual void evalAtMesh(real      zeta,

                        integer const num_states_controls,  real * states_controls,        real * states_controls_upper_bounds_mult,  real * states_controls_lower_bounds_mult,
                        integer const num_alg_states_controls,  real * algebraic_states_controls,       real * algebraic_states_controls_upper_bounds_mult,  real * algebraic_states_controls_lower_bounds_mult,
                        integer const num_fo_eqns,          real * fo_eqns,                real * fo_eqns_mult,
                        integer const num_point_constr,     real * states_constr,          real * point_constr_mult,
                        integer const num_path_constr,      real * path_constr,            real * path_constr_mult,

                        integer const num_int_constr,       real * int_constr,
                        integer const num_post_proc,        real * post_proc,
                        integer const num_diff_post_proc,   real * diff_post_proc,
                        integer const num_int_post_proc,    real * int_post_proc
                        ) const = 0;

        virtual void eval(real const initial_zeta, real const final_zeta,

                  integer const num_parameters,             real * parameters,             real * params_upper_bounds_mult,   real * params_lower_bounds_mult,
                  integer const num_boundary_conditions,    real * boundary_conditions,    real * boundary_conditions_mult,
                  integer const num_int_constr,             real * int_constr_at_end,      real * int_constr_mult,
                  integer const num_int_post_proc,          real * int_post_proc_at_end
                  ) const = 0;

        // getter
        virtual real getTarget() const = 0;

        virtual vec_1d_real const & getDiscretisationPoints() const = 0;

        // get the solution evaluated on all the discretisation points
        virtual vec_1d_real const & getCumulativeTarget() const = 0;
        virtual vec_1d_real const & getIntegrandTarget() const = 0;
        virtual vec_2d_real const & getStatesControls() const = 0;
        virtual vec_2d_real const & getAlgebraicStatesControls() const = 0;
        virtual vec_2d_real const & getPointConstraints() const = 0;
        virtual vec_2d_real const & getPathConstraints() const = 0;
        virtual vec_2d_real const & getIntegralConstraints() const = 0;
        virtual vec_2d_real const & getFoEquations() const = 0;
        virtual vec_1d_real const & getParameters() const = 0;
        virtual vec_1d_real const & getBoundaryConditions() const = 0;
        virtual vec_2d_real const & getPostProcessing() const = 0;
        virtual vec_2d_real const & getDifferentialPostProcessing() const = 0;
        virtual vec_2d_real const & getIntegralPostProcessing() const = 0;

        virtual vec_2d_real const & getStatesControlsUpperBoundsMultipliers() const = 0;
        virtual vec_2d_real const & getStatesControlsLowerBoundsMultipliers() const = 0;
        virtual vec_2d_real const & getAlgebraicStatesControlsUpperBoundsMultipliers() const = 0;
        virtual vec_2d_real const & getAlgebraicStatesControlsLowerBoundsMultipliers() const = 0;
        virtual vec_1d_real const & getParametersUpperBoundsMultipliers() const = 0;
        virtual vec_1d_real const & getParametersLowerBoundsMultipliers() const = 0;
        virtual vec_2d_real const & getPointConstraintsMultipliers() const = 0;
        virtual vec_2d_real const & getPathConstraintsMultipliers() const = 0;
        virtual vec_1d_real const & getIntConstraintsMultipliers() const = 0;
        virtual vec_2d_real const & getFoEqnsMultipliers() const = 0;
        virtual vec_1d_real const & getBoundaryConditionsMultipliers() const = 0;

        // write solution to output
        /* write content to GenericContainer
          If the MaverickOcp pointer is null, states, controls, equations, etc will be saved
          in vectors (i.e. state vector at i-th element has state[i]).
          If the MaverickOcp pointer is NOT null, states, controls, equations, etc will be saved
          with their specific names read from the MaverickOcp object. */
        virtual void writeContentToGC(GC::GenericContainer & out_gc, MaverickOcp const * const p_ocp, integer const i_phase) const = 0;

        virtual void writeMeshVarsToStream(std::ostream & body, std::ostream & header) const = 0;

        virtual void writeMeshVarsToStream(std::ostream & body, std::ostream & header,
                                   bool const add_phase_index, integer const i_phase) const = 0;

        // clear the solution
        virtual void clear() = 0;

        // get a copy
        virtual std::unique_ptr<OcpSolutionSinglePhase> copy() const = 0;

    };
}

#endif
