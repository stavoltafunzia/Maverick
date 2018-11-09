/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef MAVERICK_RK1_OCP_SOLUTION_SINGLE_PHASE_HH
#define MAVERICK_RK1_OCP_SOLUTION_SINGLE_PHASE_HH

#include "MaverickCore/OcpSolutionSinglePhase.hh"
#include "MaverickCore/MaverickOcp.hh"
#include "MaverickUtils/GenericFunction/GenericFunction1AInterface.hh"
#include <vector>

namespace Maverick {

  class RK1OcpSolutionSinglePhase : public OcpSolutionSinglePhase {

  public:

    RK1OcpSolutionSinglePhase();

    static std::unique_ptr<RK1OcpSolutionSinglePhase>
    convertFromRealTable(real_table const &table, MaverickOcp const &ocp_problem, integer const i_phase,
                         std::vector<std::string> &found_variables);

    // copy constructor
    RK1OcpSolutionSinglePhase(RK1OcpSolutionSinglePhase const &ocp_solution);

    // destructor
    ~RK1OcpSolutionSinglePhase();

    // full evaluation of the phase solution
    void evalAtMesh(real zeta,

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

    void eval(real const initial_zeta, real const final_zeta,

              integer const num_parameters, real *parameters, real *params_upper_bounds_mult,
              real *params_lower_bounds_mult,
              integer const num_boundary_conditions, real *boundary_conditions, real *boundary_conditions_mult,
              integer const num_int_constr, real *int_constr_at_end, real *int_constr_mult,
              integer const num_int_post_proc, real *int_post_proc_at_end
    ) const;

    // getter
    real getTarget() const;

    vec_1d_real const &getDiscretisationPoints() const;

    vec_1d_real const &getCenterZeta() const;

    vec_1d_real const &getCumulativeTarget() const;

    vec_1d_real const &getIntegrandTarget() const;

    vec_2d_real const &getStatesControls() const;

    vec_2d_real const &getAlgebraicStatesControls() const;

    vec_2d_real const &getPointConstraints() const;

    vec_2d_real const &getPathConstraints() const;

    vec_2d_real const &getIntegralConstraints() const;

    vec_2d_real const &getFoEquations() const;

    vec_1d_real const &getParameters() const;

    vec_1d_real const &getBoundaryConditions() const;

    vec_2d_real const &getPostProcessing() const;

    vec_2d_real const &getDifferentialPostProcessing() const;

    vec_2d_real const &getIntegralPostProcessing() const;

    vec_2d_real const &getStatesControlsUpperBoundsMultipliers() const;

    vec_2d_real const &getStatesControlsLowerBoundsMultipliers() const;

    vec_2d_real const &getAlgebraicStatesControlsUpperBoundsMultipliers() const;

    vec_2d_real const &getAlgebraicStatesControlsLowerBoundsMultipliers() const;

    vec_1d_real const &getParametersUpperBoundsMultipliers() const;

    vec_1d_real const &getParametersLowerBoundsMultipliers() const;

    vec_2d_real const &getPointConstraintsMultipliers() const;

    vec_2d_real const &getPathConstraintsMultipliers() const;

    vec_1d_real const &getIntConstraintsMultipliers() const;

    vec_2d_real const &getFoEqnsMultipliers() const;

    vec_1d_real const &getBoundaryConditionsMultipliers() const;

    // write solution to output
    /* write content to GenericContainer
      If the MaverickOcp pointer is null, states, controls, equations, etc will be saved
      in vectors (i.e. state vector at i-th element has state[i]).
      If the MaverickOcp pointer is NOT null, states, controls, equations, etc will be saved
      with their specific names read from the MaverickOcp object. */
    void writeContentToGC(GC::GenericContainer &out_gc, MaverickOcp const *const p_ocp, integer const i_phase) const;

    void writeMeshVarsToStream(std::ostream &body, std::ostream &header) const;

    void writeMeshVarsToStream(std::ostream &body, std::ostream &header,
                               bool const add_phase_index, integer const i_phase) const;

    // clear the solution
    void clear();

    // get a copy
    virtual std::unique_ptr<OcpSolutionSinglePhase> copy() const;

    // additional methods

    //setter

    // set solution with zero multipliers
    void setSolution(real const alpha,
                     real const target,
                     vec_1d_real const &zeta,
                     vec_1d_real const &cumulative_target,
                     vec_1d_real const &integrand_target,
                     vec_2d_real const &states_controls,
                     vec_2d_real const &algebraic_states_controls,
                     vec_1d_real const &params,
                     vec_2d_real const &point_constr,
                     vec_2d_real const &path_constr,
                     vec_2d_real const &int_constr,
                     vec_2d_real const &fo_eqns,
                     vec_1d_real const &boundary_conditions,
                     vec_2d_real const &post_processing,
                     vec_2d_real const &differential_post_processing,
                     vec_2d_real const &integral_post_processing);

    // set solution with multipliers
    void setSolution(real const alpha,
                     real const target,
                     vec_1d_real const &zeta,
                     vec_1d_real const &cumulative_target,
                     vec_1d_real const &integrand_target,
                     vec_2d_real const &states_controls,
                     vec_2d_real const &algebraic_states_controls,
                     vec_1d_real const &params,
                     vec_2d_real const &point_constr,
                     vec_2d_real const &path_constr,
                     vec_2d_real const &int_constr,
                     vec_2d_real const &fo_eqns,
                     vec_1d_real const &boundary_conditions,
                     vec_2d_real const &post_processing,
                     vec_2d_real const &differential_post_processing,
                     vec_2d_real const &integral_post_processing,
                     vec_2d_real const &states_controls_upper_bounds_multipliers,
                     vec_2d_real const &states_controls_lower_bounds_multipliers,
                     vec_2d_real const &algebraic_states_controls_upper_bounds_multipliers,
                     vec_2d_real const &algebraic_states_controls_lower_bounds_multipliers,
                     vec_1d_real const &parameters_upper_bounds_multipliers,
                     vec_1d_real const &parameters_lower_bounds_multipliers,
                     vec_2d_real const &point_constraints_multipliers,
                     vec_2d_real const &path_constr_multipliers,
                     vec_1d_real const &int_constr_multipliers,
                     vec_2d_real const &fo_eqns_multipliers,
                     vec_1d_real const &bcs_multipliers);

    // assignement operator
    RK1OcpSolutionSinglePhase &operator=(const RK1OcpSolutionSinglePhase &ocp_solution);

  protected:
    
    real _alpha = 0;

    real _target = 0;

    vec_1d_real _zeta = {};

    vec_1d_real _cumulative_target = {};
    vec_1d_real _integrand_target = {};
    vec_2d_real _states_controls = {};
    vec_2d_real _algebraic_states_controls = {};
    vec_2d_real _point_constraints = {};
    vec_2d_real _path_constr = {};
    vec_2d_real _int_constr = {};
    vec_2d_real _fo_eqns = {};
    vec_1d_real _parameters = {};
    vec_1d_real _boundary_conditions = {};
    vec_2d_real _post_processing = {};
    vec_2d_real _differential_post_processing = {};
    vec_2d_real _integral_post_processing = {};

    vec_2d_real _states_controls_upper_bounds_multipliers = {};
    vec_2d_real _states_controls_lower_bounds_multipliers = {};
    vec_2d_real _algebraic_states_controls_upper_bounds_multipliers = {};
    vec_2d_real _algebraic_states_controls_lower_bounds_multipliers = {};
    vec_1d_real _parameters_upper_bounds_multipliers = {};
    vec_1d_real _parameters_lower_bounds_multipliers = {};
    vec_2d_real _point_constraints_multipliers = {};
    vec_2d_real _path_constr_multipliers = {};
    vec_1d_real _int_constr_multipliers = {};
    vec_2d_real _fo_eqns_multipliers = {};
    vec_1d_real _bcs_multipliers = {};

    MaverickUtils::GenericFunction1AInterface *_gf1a_cumulative_target = nullptr;
    MaverickUtils::GenericFunction1AInterface *_gf1a_integrand_target = nullptr;
    std::vector<MaverickUtils::GenericFunction1AInterface *> _gf1a_states_controls = {};
    std::vector<MaverickUtils::GenericFunction1AInterface *> _gf1a_algebraic_states_controls = {};
    std::vector<MaverickUtils::GenericFunction1AInterface *> _gf1a_states_controls_upper_bounds_multipliers = {};
    std::vector<MaverickUtils::GenericFunction1AInterface *> _gf1a_states_controls_lower_bounds_multipliers = {};
    std::vector<MaverickUtils::GenericFunction1AInterface *> _gf1a_algebraic_states_controls_upper_bounds_multipliers = {};
    std::vector<MaverickUtils::GenericFunction1AInterface *> _gf1a_algebraic_states_controls_lower_bounds_multipliers = {};
    std::vector<MaverickUtils::GenericFunction1AInterface *> _gf1a_point_constraints = {};
    std::vector<MaverickUtils::GenericFunction1AInterface *> _gf1a_point_constraints_multipliers = {};
    std::vector<MaverickUtils::GenericFunction1AInterface *> _gf1a_path_constr = {};
    std::vector<MaverickUtils::GenericFunction1AInterface *> _gf1a_path_constr_multipliers = {};
    std::vector<MaverickUtils::GenericFunction1AInterface *> _gf1a_int_constr = {};
    std::vector<MaverickUtils::GenericFunction1AInterface *> _gf1a_fo_eqns = {};
    std::vector<MaverickUtils::GenericFunction1AInterface *> _gf1a_fo_eqns_multipliers = {};
    std::vector<MaverickUtils::GenericFunction1AInterface *> _gf1a_post_processing = {};
    std::vector<MaverickUtils::GenericFunction1AInterface *> _gf1a_differential_post_processing = {};
    std::vector<MaverickUtils::GenericFunction1AInterface *> _gf1a_integral_post_processing = {};

    // check the domensions of the solution vector
    bool areVectorsConsistent() const;

    void clearPointers();

    void buildGf1a();

  };
}

#endif
