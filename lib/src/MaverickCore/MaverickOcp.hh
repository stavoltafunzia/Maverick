/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef MAVERICK_OCP_HH
#define MAVERICK_OCP_HH

#include <string>
#include <vector>
#include <iostream>
#include "MaverickGC/GenericContainer.hh"
#include "MaverickCore/OcpGuess.hh"
#include "MaverickCore/OcpScaling.hh"

namespace Maverick {

  class OcpSolverOutput;

  class OcpSolution;

  class MaverickOcp : public OcpGuess {

  private:

    bool _has_setup_ocp = false;

    bool _has_setup_scaling = false;

  protected:

    integer _num_p; //!< number of phases
    std::vector<integer> _dim_x; //!< number of state arrays
    std::vector<integer> _dim_u; //!< number of control arrays
    std::vector<integer> _dim_ax; //!< number of algebraic state arrays
    std::vector<integer> _dim_au; //!< number of algebraic control arrays
    std::vector<integer> _dim_p; //!< number of parameters arrays
    std::vector<integer> _dim_poc; //!< number of point constraint arrays
    std::vector<integer> _dim_pc; //!< number of path constraints
    std::vector<integer> _dim_ic; //!< number of integral constraints
    std::vector<integer> _dim_bc; //!< number of boundary condition arrays
    std::vector<integer> _dim_ec; //!< number of event constraint arrays

    std::vector<std::vector<std::string>> _states_names;
    std::vector<std::vector<std::string>> _controls_names;
    std::vector<std::vector<std::string>> _algebraic_states_names;
    std::vector<std::vector<std::string>> _algebraic_controls_names;
    std::vector<std::vector<std::string>> _parameters_names;
    std::vector<std::vector<std::string>> _point_constraints_names;
    std::vector<std::vector<std::string>> _path_constraints_names;
    std::vector<std::vector<std::string>> _integral_constraints_names;
    std::vector<std::vector<std::string>> _boundary_conditions_names;
    std::vector<std::vector<std::string>> _event_constraints_names;

    std::string const _problem_name;

    // scaling
    Maverick::OcpScaling _scaling;

    // problem parameters
    real *_model_params = nullptr;
    std::vector<std::string> _model_params_names = {};

    // setup parameters and other stuff
    virtual void derivedSetup(GC::GenericContainer const &gc) = 0;

    // print additional info
    virtual void printDerivedInfo(std::ostream &out, InfoLevel info_level) const = 0;

  public:

    MaverickOcp();

    virtual ~MaverickOcp();

    void setupOcp(GC::GenericContainer const &gc);

    void setupScaling(GC::GenericContainer const &gc_scaling, vec_2d_real const &discretisation_points);

    OcpScaling const &getScaling() const;

    void printInfo(std::ostream &out, InfoLevel info_level) const;

    bool hasSetup() const { return _has_setup_ocp; }

    std::string getName() const { return _problem_name; }

    integer numberOfPhases() const { return _num_p; }

    integer numberOfStates(integer const i_phase) const { return _dim_x[i_phase]; }

    integer numberOfControls(integer const i_phase) const { return _dim_u[i_phase]; }

    integer numberOfAlgebraicStates(integer const i_phase) const { return _dim_ax[i_phase]; }

    integer numberOfAlgebraicControls(integer const i_phase) const { return _dim_au[i_phase]; }

    integer numberOfParameters(integer const i_phase) const { return _dim_p[i_phase]; }

    integer numberOfFirstOrderEquations(integer const i_phase) const { return _dim_x[i_phase] + _dim_ax[i_phase]; }

    integer numberOfPointConstraints(integer const i_phase) const { return _dim_poc[i_phase]; }

    integer numberOfPathConstraints(integer const i_phase) const { return _dim_pc[i_phase]; }

    integer numberOfIntConstraints(integer const i_phase) const { return _dim_ic[i_phase]; }

    integer numberOfBoundaryConditions(integer const i_phase) const { return _dim_bc[i_phase]; }

    integer numberOfEventConstraints(integer const i_phase) const { return _dim_ec[i_phase]; }

    std::string stateName(integer const i_phase, integer const i) const;

    std::string controlName(integer const i_phase, integer const i) const;

    std::string algebraicStateName(integer const i_phase, integer const i) const;

    std::string algebraicControlName(integer const i_phase, integer const i) const;

    std::string parameterName(integer const i_phase, integer const i) const;

    std::string pointConstraintName(integer const i_phase, integer const i) const;

    std::string pathConstraintName(integer const i_phase, integer const i) const;

    std::string intConstraintName(integer const i_phase, integer const i) const;

    std::string boundaryConditionName(integer const i_phase, integer const i) const;

    std::string eventConstraintName(integer const i_phase, integer const i) const;

    // write solution to file
    virtual void writeSolutionToFile(OcpSolution const &solution, std::string const &filename) const;

    // rename generic states and control names with correct names
    virtual void
    translateSolutionHeader(std::stringstream const &input_header, integer const i_phase, std::ostream &output) const;

    // mesh history
    virtual void writeMeshHistoryToFile(OcpSolverOutput const &solver_output, std::string const &filename) const;

    //   +-------------------------------------+
    //   |  _                           _      |
    //   | | |__   ___  _   _ _ __   __| |___  |
    //   | | '_ \ / _ \| | | | '_ \ / _` / __| |
    //   | | |_) | (_) | |_| | | | | (_| \__ \ |
    //   | |_.__/ \___/ \__,_|_| |_|\__,_|___/ |
    //   |                                     |
    //   +-------------------------------------+

    virtual integer getStatesControlsBounds(integer const i_phase,
                                            real const __zeta,
                                            real lower[],
                                            real upper[]) const = 0;

    virtual integer getAlgebraicStatesControlsBounds(integer const i_phase,
                                                     real const __zeta,
                                                     real lower[],
                                                     real upper[]) const = 0;

    virtual integer getParametersBounds(integer const i_phase,
                                        real lower[],
                                        real upper[]) const = 0;

    virtual integer getPointConstraintsBounds(integer const i_phase,
                                              real const __zeta,
                                              real lower[],
                                              real upper[]) const = 0;

    virtual integer getPathConstraintsBounds(integer const i_phase,
                                             real const __zeta,
                                             real lower[],
                                             real upper[]) const = 0;

    virtual integer getIntConstraintsBounds(integer const i_phase,
                                            real const __zeta_i,
                                            real const __zeta_f,
                                            real lower[],
                                            real upper[]) const = 0;

    virtual integer getBoundaryConditionsBounds(integer const i_phase,
                                                real const __zeta_i,
                                                real const __zeta_f,
                                                real lower[],
                                                real upper[]) const = 0;

    virtual integer getEventConstraintsBounds(integer const i_phase,
                                              real const __zeta_l,
                                              real const __zeta_r,
                                              real lower[],
                                              real upper[]) const = 0;

    // +----------------------------+
    // |                            |
    // |   __ _ _   _  ___ ___ ___  |
    // |  / _` | | | |/ _ Y __/ __| |
    // | | (_| | |_| |  __|__ \__ \ |
    // |  \__, |\__,_|\___|___/___/ |
    // |  |___/                     |
    // +----------------------------+

    // override this method if you inted to provide a guess for the OCP problem. By default return zero guess
    virtual void evalAtMesh(integer const i_phase,
                            real const zeta,

                            integer const num_states_controls, real *states_controls,
                            real *states_controls_upper_bounds_mult, real *states_controls_lower_bounds_mult,
                            integer const num_alg_states_controls, real *algebraic_states_controls,
                            real *algebraic_states_controls_upper_bounds_mult,
                            real *algebraic_states_controls_lower_bounds_mult,
                            integer const num_fo_eqns, real *fo_eqns_mult,
                            integer const num_point_constr, real *point_constr_mult,
                            integer const num_path_constr, real *path_constr_mult
    ) const;

    virtual void eval(integer const i_phase,
                      real const initial_zeta, real const final_zeta,

                      integer const num_parameters, real *parameters, real *params_upper_bounds_mult,
                      real *params_lower_bounds_mult,
                      integer const num_boundary_conditions, real *boundary_conditions_mult,
                      integer const num_int_constr, real *int_constr_mult
    ) const;

    //   +-----------------------------------+
    //   |                                   |
    //   |  _ __ ___   __ _ _   _  ___ _ __  |
    //   | | '_ ` _ \ / _` | | | |/ _ \ '__| |
    //   | | | | | | | (_| | |_| |  __/ |    |
    //   | |_| |_| |_|\__,_|\__, |\___|_|    |
    //   |                  |___/            |
    //   +-----------------------------------+

    virtual integer mayer(integer const i_phase,
                          real const initial_state_control[],
                          real const final_state_control[],
                          real const parameters[],
                          real &value) const = 0;

    virtual integer mayerJac(integer const i_phase,
                             real const initial_state_control[],
                             real const final_state_control[],
                             real const parameters[],
                             real jac_xu_init[],
                             real jac_xu_fin[],
                             real jac_p[]) const = 0;

    virtual integer mayerJacXuInitNnz(integer const i_phase) const = 0;

    virtual void mayerJacXuInitPattern(integer const i_phase, integer cols[]) const = 0;

    virtual integer mayerJacXuFinNnz(integer const i_phase) const = 0;

    virtual void mayerJacXuFinPattern(integer const i_phase, integer cols[]) const = 0;

    virtual integer mayerJacPNnz(integer const i_phase) const = 0;

    virtual void mayerJacPPattern(integer const i_phase, integer cols[]) const = 0;

    virtual integer mayerHess(integer const i_phase,
                              real const initial_state_control[],
                              real const final_state_control[],
                              real const parameters[],
                              real lambda_0,
                              real hess_xu_init_xu_init[],
                              real hess_xu_init_xu_fin[],
                              real hess_xu_init_p[],
                              real hess_xu_fin_xu_fin[],
                              real hess_xu_fin_p[],
                              real hess_p_p[]) const = 0;

    virtual integer mayerHessXuInitXuInitNnz(integer const i_phase) const = 0;

    virtual void mayerHessXuInitXuInitPattern(integer const i_phase,
                                              integer rows[],
                                              integer cols[]) const = 0;

    virtual integer mayerHessXuInitXuFinNnz(integer const i_phase) const = 0;

    virtual void mayerHessXuInitXuFinPattern(integer const i_phase,
                                             integer rows[],
                                             integer cols[]) const = 0;

    virtual integer mayerHessXuInitPNnz(integer const i_phase) const = 0;

    virtual void mayerHessXuInitPPattern(integer const i_phase,
                                         integer rows[],
                                         integer cols[]) const = 0;

    virtual integer mayerHessXuFinXuFinNnz(integer const i_phase) const = 0;

    virtual void mayerHessXuFinXuFinPattern(integer const i_phase,
                                            integer rows[],
                                            integer cols[]) const = 0;

    virtual integer mayerHessXuFinPNnz(integer const i_phase) const = 0;

    virtual void mayerHessXuFinPPattern(integer const i_phase,
                                        integer rows[],
                                        integer cols[]) const = 0;

    virtual integer mayerHessPPNnz(integer const i_phase) const = 0;

    virtual void mayerHessPPPattern(integer const i_phase,
                                    integer rows[],
                                    integer cols[]) const = 0;

    //   +--------------------------------------------+
    //   |  _                                         |
    //   | | | __ _  __ _ _ __ __ _ _ __   __ _  ___  |
    //   | | |/ _` |/ _` | '__/ _` | '_ \ / _` |/ _ \ |
    //   | | | (_| | (_| | | | (_| | | | | (_| |  __/ |
    //   | |_|\__,_|\__, |_|  \__,_|_| |_|\__, |\___| |
    //   |          |___/                 |___/       |
    //   +--------------------------------------------+

    virtual integer lagrange(integer const i_phase,
                             real const state_control[],
                             real const state_control_derivative[],
                             real const algebraic_state_control[],
                             real const parameters[],
                             real __zeta,
                             real &value) const = 0;

    virtual integer lagrangeJac(integer const i_phase,
                                real const state_control[],
                                real const state_control_derivative[],
                                real const algebraic_state_control[],
                                real const parameters[],
                                real __zeta,
                                real jac_xu[],
                                real jac_dxu[],
                                real jac_axu[],
                                real jac_p[]) const = 0;

    virtual integer lagrangeJacXuNnz(integer const i_phase) const = 0;

    virtual void lagrangeJacXuPattern(integer const i_phase, integer cols[]) const = 0;

    virtual integer lagrangeJacDxuNnz(integer const i_phase) const = 0;

    virtual void lagrangeJacDxuPattern(integer const i_phase, integer cols[]) const = 0;

    virtual integer lagrangeJacAxuNnz(integer const i_phase) const = 0;

    virtual void lagrangeJacAxuPattern(integer const i_phase, integer cols[]) const = 0;

    virtual integer lagrangeJacPNnz(integer const i_phase) const = 0;

    virtual void lagrangeJacPPattern(integer const i_phase, integer cols[]) const = 0;

    virtual integer lagrangeHess(integer const i_phase,
                                 real const state_control[],
                                 real const state_control_derivative[],
                                 real const algebraic_state_control[],
                                 real const parameters[],
                                 real __zeta,
                                 real lambda_0,
                                 real hess_xu_xu[],
                                 real hess_xu_dxu[],
                                 real hess_xu_axu[],
                                 real hess_xu_p[],
                                 real hess_dxu_dxu[],
                                 real hess_dxu_axu[],
                                 real hess_dxu_p[],
                                 real hess_axu_axu[],
                                 real hess_axu_p[],
                                 real hess_p_p[]) const = 0;

    virtual integer lagrangeHessXuXuNnz(integer const i_phase) const = 0;

    virtual void lagrangeHessXuXuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer lagrangeHessXuDxuNnz(integer const i_phase) const = 0;

    virtual void lagrangeHessXuDxuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer lagrangeHessXuAxuNnz(integer const i_phase) const = 0;

    virtual void lagrangeHessXuAxuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer lagrangeHessXuPNnz(integer const i_phase) const = 0;

    virtual void lagrangeHessXuPPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer lagrangeHessDxuDxuNnz(integer const i_phase) const = 0;

    virtual void lagrangeHessDxuDxuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer lagrangeHessDxuAxuNnz(integer const i_phase) const = 0;

    virtual void lagrangeHessDxuAxuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer lagrangeHessDxuPNnz(integer const i_phase) const = 0;

    virtual void lagrangeHessDxuPPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer lagrangeHessAxuAxuNnz(integer const i_phase) const = 0;

    virtual void lagrangeHessAxuAxuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer lagrangeHessAxuPNnz(integer const i_phase) const = 0;

    virtual void lagrangeHessAxuPPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer lagrangeHessPPNnz(integer const i_phase) const = 0;

    virtual void lagrangeHessPPPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    //   +-------------------------------------------------------------+
    //   |   __                                 _   _                  |
    //   |  / _| ___      ___  __ _ _   _  __ _| |_(_) ___  _ __  ___  |
    //   | | |_ / _ \    / _ \/ _` | | | |/ _` | __| |/ _ \| '_ \/ __| |
    //   | |  _| (_) |  |  __/ (_| | |_| | (_| | |_| | (_) | | | \__ \ |
    //   | |_|(_)___(_)  \___|\__, |\__,_|\__,_|\__|_|\___/|_| |_|___/ |
    //   |                       |_|                                   |
    //   +-------------------------------------------------------------+

    virtual integer foEqns(integer const i_phase,
                           real const state_control[],
                           real const state_control_derivative[],
                           real const algebraic_state_control[],
                           real const parameters[],
                           real __zeta,
                           real values[]) const = 0;

    virtual integer foEqnsJac(integer const i_phase,
                              real const state_control[],
                              real const state_control_derivative[],
                              real const algebraic_state_control[],
                              real const parameters[],
                              real __zeta,
                              real j_xu[],
                              real j_dxu[],
                              real j_axu[],
                              real j_p[]) const = 0;

    virtual integer foEqnsJacXuNnz(integer const i_phase) const = 0;

    virtual void foEqnsJacXuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer foEqnsJacDxuNnz(integer const i_phase) const = 0;

    virtual void foEqnsJacDxuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer foEqnsJacAxuNnz(integer const i_phase) const = 0;

    virtual void foEqnsJacAxuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer foEqnsJacPNnz(integer const i_phase) const = 0;

    virtual void foEqnsJacPPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer foEqnsHess(integer const i_phase,
                               real const state_control[],
                               real const state_control_derivative[],
                               real const algebraic_state_control[],
                               real const parameters[],
                               real __zeta,
                               real const lambda[],
                               real hess_xu_xu[],
                               real hess_xu_dxu[],
                               real hess_xu_axu[],
                               real hess_xu_p[],
                               real hess_dxu_dxu[],
                               real hess_dxu_axu[],
                               real hess_dxu_p[],
                               real hess_axu_axu[],
                               real hess_axu_p[],
                               real hess_p_p[]) const = 0;

    virtual integer foEqnsHessXuXuNnz(integer const i_phase) const = 0;

    virtual void foEqnsHessXuXuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer foEqnsHessXuDxuNnz(integer const i_phase) const = 0;

    virtual void foEqnsHessXuDxuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer foEqnsHessXuAxuNnz(integer const i_phase) const = 0;

    virtual void foEqnsHessXuAxuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer foEqnsHessXuPNnz(integer const i_phase) const = 0;

    virtual void foEqnsHessXuPPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer foEqnsHessDxuDxuNnz(integer const i_phase) const = 0;

    virtual void foEqnsHessDxuDxuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer foEqnsHessDxuAxuNnz(integer const i_phase) const = 0;

    virtual void foEqnsHessDxuAxuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer foEqnsHessDxuPNnz(integer const i_phase) const = 0;

    virtual void foEqnsHessDxuPPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer foEqnsHessAxuAxuNnz(integer const i_phase) const = 0;

    virtual void foEqnsHessAxuAxuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer foEqnsHessAxuPNnz(integer const i_phase) const = 0;

    virtual void foEqnsHessAxuPPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer foEqnsHessPPNnz(integer const i_phase) const = 0;

    virtual void foEqnsHessPPPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    // +-----------------------------------------------------------------------+
    // |      _ _  __  __                       _             _       _        |
    // |   __| (_)/ _|/ _|   ___ ___  _ __  ___| |_ _ __ __ _(_)_ __ | |_ ___  |
    // |  / _` | | |_| |_   / __/ _ \| '_ \/ __| __| '__/ _` | | '_ \| __/ __| |
    // | | (_| | |  _|  _| | (_| (_) | | | \__ \ |_| | | (_| | | | | | |_\__ \ |
    // |  \__,_|_|_| |_|    \___\___/|_| |_|___/\__|_|  \__,_|_|_| |_|\__|___/ |
    // |                                                                       |
    // +-----------------------------------------------------------------------+


    virtual integer pathConstraints(integer const i_phase,
                                    real const state_control[],
                                    real const state_control_derivative[],
                                    real const algebraic_state_control[],
                                    real const parameters[],
                                    real __zeta,
                                    real values[]) const = 0;

    virtual integer pathConstraintsJac(integer const i_phase,
                                       real const state_control[],
                                       real const state_control_derivative[],
                                       real const algebraic_state_control[],
                                       real const parameters[],
                                       real __zeta,
                                       real j_xu[],
                                       real j_dxu[],
                                       real j_axu[],
                                       real j_p[]) const = 0;

    virtual integer pathConstraintsJacXuNnz(integer const i_phase) const = 0;

    virtual void pathConstraintsJacXuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer pathConstraintsJacDxuNnz(integer const i_phase) const = 0;

    virtual void pathConstraintsJacDxuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer pathConstraintsJacAxuNnz(integer const i_phase) const = 0;

    virtual void pathConstraintsJacAxuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer pathConstraintsJacPNnz(integer const i_phase) const = 0;

    virtual void pathConstraintsJacPPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer pathConstraintsHess(integer const i_phase,
                                        real const state_control[],
                                        real const state_control_derivative[],
                                        real const algebraic_state_control[],
                                        real const parameters[],
                                        real __zeta,
                                        real const lambda[],
                                        real hess_xu_xu[],
                                        real hess_xu_dxu[],
                                        real hess_xu_axu[],
                                        real hess_xu_p[],
                                        real hess_dxu_dxu[],
                                        real hess_dxu_axu[],
                                        real hess_dxu_p[],
                                        real hess_axu_axu[],
                                        real hess_axu_p[],
                                        real hess_p_p[]) const = 0;

    virtual integer pathConstraintsHessXuXuNnz(integer const i_phase) const = 0;

    virtual void pathConstraintsHessXuXuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer pathConstraintsHessXuDxuNnz(integer const i_phase) const = 0;

    virtual void pathConstraintsHessXuDxuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer pathConstraintsHessXuAxuNnz(integer const i_phase) const = 0;

    virtual void pathConstraintsHessXuAxuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer pathConstraintsHessXuPNnz(integer const i_phase) const = 0;

    virtual void pathConstraintsHessXuPPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer pathConstraintsHessDxuDxuNnz(integer const i_phase) const = 0;

    virtual void pathConstraintsHessDxuDxuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer pathConstraintsHessDxuAxuNnz(integer const i_phase) const = 0;

    virtual void pathConstraintsHessDxuAxuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer pathConstraintsHessDxuPNnz(integer const i_phase) const = 0;

    virtual void pathConstraintsHessDxuPPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer pathConstraintsHessAxuAxuNnz(integer const i_phase) const = 0;

    virtual void pathConstraintsHessAxuAxuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer pathConstraintsHessAxuPNnz(integer const i_phase) const = 0;

    virtual void pathConstraintsHessAxuPPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer pathConstraintsHessPPNnz(integer const i_phase) const = 0;

    virtual void pathConstraintsHessPPPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    // +----------------------------------------------------------------------------+
    // |      _        _                             _             _       _        |
    // |  ___| |_ __ _| |_ ___    ___ ___  _ __  ___| |_ _ __ __ _(_)_ __ | |_ ___  |
    // | / __| __/ _` | __/ _ \  / __/ _ \| '_ \/ __| __| '__/ _` | | '_ \| __/ __| |
    // | \__ \ || (_| | ||  __/ | (_| (_) | | | \__ \ |_| | | (_| | | | | | |_\__ \ |
    // | |___/\__\__,_|\__\___|  \___\___/|_| |_|___/\__|_|  \__,_|_|_| |_|\__|___/ |
    // |                                                                            |
    // +----------------------------------------------------------------------------+


    virtual integer pointConstraints(integer const i_phase,
                                     real const state_control[],
                                     real const parameters[],
                                     real __zeta,
                                     real values[]) const = 0;

    virtual integer pointConstraintsJac(integer const i_phase,
                                        real const state_control[],
                                        real const parameters[],
                                        real __zeta,
                                        real j_xu[],
                                        real j_p[]) const = 0;

    virtual integer pointConstraintsJacXuNnz(integer const i_phase) const = 0;

    virtual void pointConstraintsJacXuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer pointConstraintsJacPNnz(integer const i_phase) const = 0;

    virtual void pointConstraintsJacPPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer pointConstraintsHess(integer const i_phase,
                                         real const state_control[],
                                         real const parameters[],
                                         real __zeta,
                                         real const lambda[],
                                         real hess_xu_xu[],
                                         real hess_xu_p[],
                                         real hess_p_p[]) const = 0;

    virtual integer pointConstraintsHessXuXuNnz(integer const i_phase) const = 0;

    virtual void pointConstraintsHessXuXuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer pointConstraintsHessPPNnz(integer const i_phase) const = 0;

    virtual void pointConstraintsHessPPPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer pointConstraintsHessXuPNnz(integer const i_phase) const = 0;

    virtual void pointConstraintsHessXuPPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    // +-----------------------------------------------------------------------------------------+
    // |  _       _                       _                       _             _       _        |
    // | (_)_ __ | |_ ___  __ _ _ __ __ _| |   ___ ___  _ __  ___| |_ _ __ __ _(_)_ __ | |_ ___  |
    // | | | '_ \| __/ _ \/ _` | '__/ _` | |  / __/ _ \| '_ \/ __| __| '__/ _` | | '_ \| __/ __| |
    // | | | | | | ||  __/ (_| | | | (_| | | | (_| (_) | | | \__ \ |_| | | (_| | | | | | |_\__ \ |
    // | |_|_| |_|\__\___|\__, |_|  \__,_|_|  \___\___/|_| |_|___/\__|_|  \__,_|_|_| |_|\__|___/ |
    // |                  |___/                                                                  |
    // +-----------------------------------------------------------------------------------------+


    virtual integer intConstraints(integer const i_phase,
                                   real const state_control[],
                                   real const state_control_derivative[],
                                   real const algebraic_state_control[],
                                   real const parameters[],
                                   real __zeta,
                                   real values[]) const = 0;

    virtual integer intConstraintsJac(integer const i_phase,
                                      real const state_control[],
                                      real const state_control_derivative[],
                                      real const algebraic_state_control[],
                                      real const parameters[],
                                      real __zeta,
                                      real j_xu[],
                                      real j_dxu[],
                                      real j_axu[],
                                      real j_p[]) const = 0;

    virtual integer intConstraintsJacXuNnz(integer const i_phase) const = 0;

    virtual void intConstraintsJacXuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer intConstraintsJacDxuNnz(integer const i_phase) const = 0;

    virtual void intConstraintsJacDxuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer intConstraintsJacAxuNnz(integer const i_phase) const = 0;

    virtual void intConstraintsJacAxuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer intConstraintsJacPNnz(integer const i_phase) const = 0;

    virtual void intConstraintsJacPPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer intConstraintsHess(integer const i_phase,
                                       real const state_control[],
                                       real const state_control_derivative[],
                                       real const algebraic_state_control[],
                                       real const parameters[],
                                       real __zeta,
                                       real const lambda[],
                                       real hess_xu_xu[],
                                       real hess_xu_dxu[],
                                       real hess_xu_axu[],
                                       real hess_xu_p[],
                                       real hess_dxu_dxu[],
                                       real hess_dxu_axu[],
                                       real hess_dxu_p[],
                                       real hess_axu_axu[],
                                       real hess_axu_p[],
                                       real hess_p_p[]) const = 0;

    virtual integer intConstraintsHessXuXuNnz(integer const i_phase) const = 0;

    virtual void intConstraintsHessXuXuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer intConstraintsHessXuDxuNnz(integer const i_phase) const = 0;

    virtual void intConstraintsHessXuDxuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer intConstraintsHessXuAxuNnz(integer const i_phase) const = 0;

    virtual void intConstraintsHessXuAxuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer intConstraintsHessXuPNnz(integer const i_phase) const = 0;

    virtual void intConstraintsHessXuPPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer intConstraintsHessDxuDxuNnz(integer const i_phase) const = 0;

    virtual void intConstraintsHessDxuDxuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer intConstraintsHessDxuAxuNnz(integer const i_phase) const = 0;

    virtual void intConstraintsHessDxuAxuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer intConstraintsHessDxuPNnz(integer const i_phase) const = 0;

    virtual void intConstraintsHessDxuPPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer intConstraintsHessAxuAxuNnz(integer const i_phase) const = 0;

    virtual void intConstraintsHessAxuAxuPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer intConstraintsHessAxuPNnz(integer const i_phase) const = 0;

    virtual void intConstraintsHessAxuPPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    virtual integer intConstraintsHessPPNnz(integer const i_phase) const = 0;

    virtual void intConstraintsHessPPPattern(integer const i_phase, integer rows[], integer cols[]) const = 0;

    //   +--------------------------------------------------------------------------------------------------+
    //   |  _                           _                                        _ _ _   _                  |
    //   | | |__   ___  _   _ _ __   __| | __ _ _ __ _   _    ___ ___  _ __   __| (_) |_(_) ___  _ __  ___  |
    //   | | '_ \ / _ \| | | | '_ \ / _` |/ _` | '__| | | |  / __/ _ \| '_ \ / _` | | __| |/ _ \| '_ \/ __| |
    //   | | |_) | (_) | |_| | | | | (_| | (_| | |  | |_| | | (_| (_) | | | | (_| | | |_| | (_) | | | \__ \ |
    //   | |_.__/ \___/ \__,_|_| |_|\__,_|\__,_|_|   \__, |  \___\___/|_| |_|\__,_|_|\__|_|\___/|_| |_|___/ |
    //   |                                           |___/                                                  |
    //   +--------------------------------------------------------------------------------------------------+

    virtual integer boundaryConditions(integer const i_phase,
                                       real const initial_state_control[],
                                       real const final_state_control[],
                                       real const parameters[],
                                       real __zeta_i,
                                       real __zeta_f,
                                       real values[]) const = 0;

    virtual integer boundaryConditionsJac(integer const i_phase,
                                          real const initial_state_control[],
                                          real const final_state_control[],
                                          real const parameters[],
                                          real __zeta_i,
                                          real __zeta_f,
                                          real jac_xu_init[],
                                          real jac_xu_fin[],
                                          real jac_p[]) const = 0;

    virtual integer boundaryConditionsJacXuInitNnz(integer const i_phase) const = 0;

    virtual void boundaryConditionsJacXuInitPattern(integer const i_phase,
                                                    integer rows[],
                                                    integer cols[]) const = 0;

    virtual integer boundaryConditionsJacXuFinNnz(integer const i_phase) const = 0;

    virtual void boundaryConditionsJacXuFinPattern(integer const i_phase,
                                                   integer rows[],
                                                   integer cols[]) const = 0;

    virtual integer boundaryConditionsJacPNnz(integer const i_phase) const = 0;

    virtual void boundaryConditionsJacPPattern(integer const i_phase,
                                               integer rows[],
                                               integer cols[]) const = 0;

    virtual integer boundaryConditionsHess(integer const i_phase,
                                           real const initial_state_control[],
                                           real const final_state_control[],
                                           real const parameters[],
                                           real __zeta_i,
                                           real __zeta_f,
                                           real const lambda[],
                                           real hess_xu_init_xu_init[],
                                           real hess_xu_init_xu_fin[],
                                           real hess_xu_init_p[],
                                           real hess_xu_fin_xu_fin[],
                                           real hess_xu_fin_p[],
                                           real hess_p_p[]) const = 0;

    virtual integer boundaryConditionsHessXuInitXuInitNnz(integer const i_phase) const = 0;

    virtual void boundaryConditionsHessXuInitXuInitPattern(integer const i_phase,
                                                           integer rows[],
                                                           integer cols[]) const = 0;

    virtual integer boundaryConditionsHessXuInitXuFinNnz(integer const i_phase) const = 0;

    virtual void boundaryConditionsHessXuInitXuFinPattern(integer const i_phase,
                                                          integer rows[],
                                                          integer cols[]) const = 0;

    virtual integer boundaryConditionsHessXuInitPNnz(integer const i_phase) const = 0;

    virtual void boundaryConditionsHessXuInitPPattern(integer const i_phase,
                                                      integer rows[],
                                                      integer cols[]) const = 0;

    virtual integer boundaryConditionsHessXuFinXuFinNnz(integer const i_phase) const = 0;

    virtual void boundaryConditionsHessXuFinXuFinPattern(integer const i_phase,
                                                         integer rows[],
                                                         integer cols[]) const = 0;

    virtual integer boundaryConditionsHessXuFinPNnz(integer const i_phase) const = 0;

    virtual void boundaryConditionsHessXuFinPPattern(integer const i_phase,
                                                     integer rows[],
                                                     integer cols[]) const = 0;

    virtual integer boundaryConditionsHessPPNnz(integer const i_phase) const = 0;

    virtual void boundaryConditionsHessPPPattern(integer const i_phase,
                                                 integer rows[],
                                                 integer cols[]) const = 0;

    // +--------------------------------------------------------------------------------+
    // |                       _                         _             _       _        |
    // |   _____   _____ _ __ | |_    ___ ___  _ __  ___| |_ _ __ __ _(_)_ __ | |_ ___  |
    // |  / _ \ \ / / _ \ '_ \| __|  / __/ _ \| '_ \/ __| __| '__/ _` | | '_ \| __/ __| |
    // | |  __/\ V /  __/ | | | |_  | (_| (_) | | | \__ \ |_| | | (_| | | | | | |_\__ \ |
    // |  \___| \_/ \___|_| |_|\__|  \___\___/|_| |_|___/\__|_|  \__,_|_|_| |_|\__|___/ |
    // |                                                                                |
    // +--------------------------------------------------------------------------------+


    virtual integer eventConstraints(integer const i_phase,
                                     real const left_state_control[],
                                     real const right_state_control[],
                                     real const parameters[],
                                     real const __zeta_l,
                                     real const __zeta_r,
                                     real values[]) const = 0;

    virtual integer eventConstraintsJac(integer const i_phase,
                                        real const left_state_control[],
                                        real const right_state_control[],
                                        real const parameters[],
                                        real const __zeta_l,
                                        real const __zeta_r,
                                        real jac_xu_init[],
                                        real jac_xu_fin[],
                                        real jac_p[]) const = 0;

    virtual integer eventConstraintsJacXuInitNnz(integer const i_phase) const = 0;

    virtual void eventConstraintsJacXuInitPattern(integer const i_phase,
                                                  integer rows[],
                                                  integer cols[]) const = 0;

    virtual integer eventConstraintsJacXuFinNnz(integer const i_phase) const = 0;

    virtual void eventConstraintsJacXuFinPattern(integer const i_phase,
                                                 integer rows[],
                                                 integer cols[]) const = 0;

    virtual integer eventConstraintsJacPNnz(integer const i_phase) const = 0;

    virtual void eventConstraintsJacPPattern(integer const i_phase,
                                             integer rows[],
                                             integer cols[]) const = 0;

    virtual integer eventConstraintsHess(integer const i_phase,
                                         real const left_state_control[],
                                         real const right_state_control[],
                                         real const parameters[],
                                         real const __zeta_l,
                                         real const __zeta_r,
                                         real const lambda[],
                                         real hess_xu_init_xu_init[],
                                         real hess_xu_init_xu_fin[],
                                         real hess_xu_init_p[],
                                         real hess_xu_fin_xu_fin[],
                                         real hess_xu_fin_p[],
                                         real hess_p_p[]) const = 0;

    virtual integer eventConstraintsHessXuInitXuInitNnz(integer const i_phase) const = 0;

    virtual void eventConstraintsHessXuInitXuInitPattern(integer const i_phase,
                                                         integer rows[],
                                                         integer cols[]) const = 0;

    virtual integer eventConstraintsHessXuInitXuFinNnz(integer const i_phase) const = 0;

    virtual void eventConstraintsHessXuInitXuFinPattern(integer const i_phase,
                                                        integer rows[],
                                                        integer cols[]) const = 0;

    virtual integer eventConstraintsHessXuInitPNnz(integer const i_phase) const = 0;

    virtual void eventConstraintsHessXuInitPPattern(integer const i_phase,
                                                    integer rows[],
                                                    integer cols[]) const = 0;

    virtual integer eventConstraintsHessXuFinXuFinNnz(integer const i_phase) const = 0;

    virtual void eventConstraintsHessXuFinXuFinPattern(integer const i_phase,
                                                       integer rows[],
                                                       integer cols[]) const = 0;

    virtual integer eventConstraintsHessXuFinPNnz(integer const i_phase) const = 0;

    virtual void eventConstraintsHessXuFinPPattern(integer const i_phase,
                                                   integer rows[],
                                                   integer cols[]) const = 0;

    virtual integer eventConstraintsHessPPNnz(integer const i_phase) const = 0;

    virtual void eventConstraintsHessPPPattern(integer const i_phase,
                                               integer rows[],
                                               integer cols[]) const = 0;

    // +------------------------------------------------------------------------+
    // |                  _                                      _              |
    // |  _ __   ___  ___| |_   _ __  _ __ ___   ___ ___ ___ ___(_)_ __   __ _  |
    // | | '_ \ / _ \/ __| __| | '_ \| '__/ _ \ / __/ _ Y __/ __| | '_ \ / _` | |
    // | | |_) | (_) \__ \ |_  | |_) | | | (_) | (_|  __|__ \__ \ | | | | (_| | |
    // | | .__/ \___/|___/\__| | .__/|_|  \___/ \___\___|___/___/_|_| |_|\__, | |
    // | |_|                   |_|                                       |___/  |
    // +------------------------------------------------------------------------+

    virtual integer numberOfPostProcessing(integer const i_phase) const { return 0; }

    virtual integer numberOfDifferentialPostProcessing(integer const i_phase) const { return 0; }

    virtual integer numberOfIntegralPostProcessing(integer const i_phase) const { return 0; }

    virtual std::string postProcessingName(integer const i_phase, integer const i) const { return ""; }

    virtual std::string differentialPostProcessingName(integer const i_phase, integer const i) const { return ""; }

    virtual std::string integralPostProcessingName(integer const i_phase, integer const i) const { return ""; }

    virtual void postProcessing(integer const i_phase,
                                real const state_control[],
                                real const parameters[],
                                real __zeta,
                                real values[]) const {};

    virtual void differentialPostProcessing(integer const i_phase,
                                            real const state_control[],
                                            real const state_control_derivative[],
                                            real const algebraic_state_control[],
                                            real const parameters[],
                                            real __zeta,
                                            real values[]) const {};

    virtual void integralPostProcessing(integer const i_phase,
                                        real const state_control[],
                                        real const state_control_derivative[],
                                        real const algebraic_state_control[],
                                        real const parameters[],
                                        real __zeta,
                                        real values[]) const {};

  };
}

#endif
