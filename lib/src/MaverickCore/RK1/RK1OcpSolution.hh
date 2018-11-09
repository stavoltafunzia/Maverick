/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef MAVERICK_RK1_OCP_SOLUTION_HH
#define MAVERICK_RK1_OCP_SOLUTION_HH

#include "MaverickCore/OcpSolution.hh"
#include "MaverickCore/OcpGuess.hh"
#include "MaverickCore/RK1/RK1OcpSolutionSinglePhase.hh"
#include <iostream>

namespace Maverick {

  class RK1OcpSolution : public OcpSolution {

  public:

    RK1OcpSolution();

    static std::unique_ptr<RK1OcpSolution>
    getFromGuessTablesForOcpProblem(std::vector<real_table> const &guess_table, MaverickOcp const &ocp_problem,
                                    std::vector<std::vector<std::string>> &found_variables);

    // copy constructor
    RK1OcpSolution(RK1OcpSolution const &ocp_solution);

    // destructor
    ~RK1OcpSolution() {}

    // OcpGuess interface
    void evalAtMesh(integer const i_phase,
                    real const zeta,

                    integer const num_states_controls, real *states_controls, real *states_controls_upper_bounds_mult,
                    real *states_controls_lower_bounds_mult,
                    integer const num_alg_states_controls, real *algebraic_states_controls,
                    real *algebraic_states_controls_upper_bounds_mult,
                    real *algebraic_states_controls_lower_bounds_mult,
                    integer const num_fo_eqns, real *fo_eqns_mult,
                    integer const num_point_constr, real *point_constr_mult,
                    integer const num_path_constr, real *path_constr_mult
    ) const;

    void eval(integer const i_phase,
              real const initial_zeta, real const final_zeta,

              integer const num_parameters, real *parameters, real *params_upper_bounds_mult,
              real *params_lower_bounds_mult,
              integer const num_boundary_conditions, real *boundary_conditions_mult,
              integer const num_int_constr, real *int_constr_mult
    ) const;

    // OcpSolution interface

    virtual Mesh::DiscretisationType discretisationType() const { return Mesh::DiscretisationType::runge_kutta_1; }

    void evalAtMesh(integer const i_phase,
                    real zeta,

                    integer const num_states_controls, real *states_controls, real *states_controls_upper_bounds_mult,
                    real *states_controls_lower_bounds_mult,
                    integer const num_alg_states_controls, real *algebraic_states_controls,
                    real *algebraic_states_controls_upper_bounds_mult,
                    real *algebraic_states_controls_lower_bounds_mult,
                    integer const num_fo_eqns, real *fo_eqns, real *fo_eqns_mult,
                    integer const num_point_constr, real *states_constr, real *point_constr_mult,
                    integer const num_path_constr, real *path_constr, real *path_constr_mult,
                    integer const num_int_constr, real *int_constr,

                    integer const num_post_proc, real *post_proc,
                    integer const num_diff_post_proc, real *diff_post_proc,
                    integer const num_int_post_proc, real *int_post_proc
    ) const;

    void eval(integer const i_phase,
              real const initial_zeta, real const final_zeta,

              integer const num_parameters, real *parameters, real *params_upper_bounds_mult,
              real *params_lower_bounds_mult,
              integer const num_boundary_conditions, real *boundary_conditions, real *boundary_conditions_mult,
              integer const num_int_constr, real *int_constr_at_end, real *int_constr_mult,
              integer const num_int_post_proc, real *int_post_proc_at_end
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
    void writeContentToGC(GC::GenericContainer &out_gc, MaverickOcp const *const p_ocp) const;

    void writeAllMeshVarsToStream(std::ostream &out) const;

    void writeOnePhaseMeshVarsToStream(integer const i_phase, std::ostream &out) const;

    // get a copy
    virtual std::unique_ptr<OcpSolution> copy() const;

    // opertators

    OcpSolutionSinglePhase const &operator[](integer const i_phase) const;

    // additional methods

    // set a specific solution for a phase
    void setSolutionAtPhase(integer const i_phase, RK1OcpSolutionSinglePhase const &ocp_solution);

    // opertators

    RK1OcpSolutionSinglePhase const &operator()(integer const i_phase) const;

    RK1OcpSolutionSinglePhase &operator()(integer const i_phase);

    RK1OcpSolution &operator=(RK1OcpSolution const &ocp_solution);

    RK1OcpSolution &operator<<(RK1OcpSolutionSinglePhase const &ocp_solution);

  protected:

    std::vector<RK1OcpSolutionSinglePhase> _solutions;

  };
}

#endif
