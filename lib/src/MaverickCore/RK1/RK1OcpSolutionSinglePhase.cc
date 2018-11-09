#include "RK1OcpSolutionSinglePhase.hh"
#include "MaverickCore/MaverickFunctions.hh"
#include "MaverickCore/MaverickSingleton.hh"
#include "MaverickUtils/GenericFunction/GF1ASpline.hh"
#include "MaverickUtils/GenericFunction/GF1APolyFive.hh"
#include "MaverickCore/MaverickPrivateDefinitions.hh"
#include <iomanip>
#include <tuple>

#define GF1A_TYPE MaverickUtils::GF1ASpline

#define SPLINE_TYPE "Akima"
//#define SPLINE_TYPE "Linear"

//#define SPLINE_CHECK_RANGE true
#define SPLINE_CHECK_RANGE false

#define SPLINE_EXTEND_RANGE MaverickUtils::GF1ASpline::ExtendRange::keep_derivative

using namespace std;

namespace Maverick {

  RK1OcpSolutionSinglePhase::RK1OcpSolutionSinglePhase() {}

  RK1OcpSolutionSinglePhase::RK1OcpSolutionSinglePhase(RK1OcpSolutionSinglePhase const &ocp_solution) {
    setSolution(ocp_solution._alpha,
                ocp_solution.getTarget(),
                ocp_solution.getDiscretisationPoints(),
                ocp_solution.getCumulativeTarget(),
                ocp_solution.getIntegrandTarget(),
                ocp_solution.getStatesControls(),
                ocp_solution.getAlgebraicStatesControls(),
                ocp_solution.getParameters(),
                ocp_solution.getPointConstraints(),
                ocp_solution.getPathConstraints(),
                ocp_solution.getIntegralConstraints(),
                ocp_solution.getFoEquations(),
                ocp_solution.getBoundaryConditions(),
                ocp_solution.getPostProcessing(),
                ocp_solution.getDifferentialPostProcessing(),
                ocp_solution.getIntegralPostProcessing(),
                ocp_solution.getStatesControlsUpperBoundsMultipliers(),
                ocp_solution.getStatesControlsLowerBoundsMultipliers(),
                ocp_solution.getAlgebraicStatesControlsUpperBoundsMultipliers(),
                ocp_solution.getAlgebraicStatesControlsLowerBoundsMultipliers(),
                ocp_solution.getParametersUpperBoundsMultipliers(),
                ocp_solution.getParametersLowerBoundsMultipliers(),
                ocp_solution.getPointConstraintsMultipliers(),
                ocp_solution.getPathConstraintsMultipliers(),
                ocp_solution.getIntConstraintsMultipliers(),
                ocp_solution.getFoEqnsMultipliers(),
                ocp_solution.getBoundaryConditionsMultipliers());
  }

  RK1OcpSolutionSinglePhase &
  RK1OcpSolutionSinglePhase::operator=(const RK1OcpSolutionSinglePhase &ocp_solution) {
    setSolution(ocp_solution._alpha,
                ocp_solution.getTarget(),
                ocp_solution.getDiscretisationPoints(),
                ocp_solution.getCumulativeTarget(),
                ocp_solution.getIntegrandTarget(),
                ocp_solution.getStatesControls(),
                ocp_solution.getAlgebraicStatesControls(),
                ocp_solution.getParameters(),
                ocp_solution.getPointConstraints(),
                ocp_solution.getPathConstraints(),
                ocp_solution.getIntegralConstraints(),
                ocp_solution.getFoEquations(),
                ocp_solution.getBoundaryConditions(),
                ocp_solution.getPostProcessing(),
                ocp_solution.getDifferentialPostProcessing(),
                ocp_solution.getIntegralPostProcessing(),
                ocp_solution.getStatesControlsUpperBoundsMultipliers(),
                ocp_solution.getStatesControlsLowerBoundsMultipliers(),
                ocp_solution.getAlgebraicStatesControlsUpperBoundsMultipliers(),
                ocp_solution.getAlgebraicStatesControlsLowerBoundsMultipliers(),
                ocp_solution.getParametersUpperBoundsMultipliers(),
                ocp_solution.getParametersLowerBoundsMultipliers(),
                ocp_solution.getPointConstraintsMultipliers(),
                ocp_solution.getPathConstraintsMultipliers(),
                ocp_solution.getIntConstraintsMultipliers(),
                ocp_solution.getFoEqnsMultipliers(),
                ocp_solution.getBoundaryConditionsMultipliers());
    return *this;
  }


  RK1OcpSolutionSinglePhase::~RK1OcpSolutionSinglePhase() {
    clearPointers();
  }

  void RK1OcpSolutionSinglePhase::clearPointers() {
    if (_gf1a_cumulative_target != nullptr)
      delete _gf1a_cumulative_target;
    _gf1a_cumulative_target = nullptr;

    if (_gf1a_integrand_target != nullptr)
      delete _gf1a_integrand_target;
    _gf1a_integrand_target = nullptr;

    auto all_spline_vecs = {
      &_gf1a_states_controls, &_gf1a_states_controls_upper_bounds_multipliers, &_gf1a_states_controls_lower_bounds_multipliers,
      &_gf1a_algebraic_states_controls, &_gf1a_algebraic_states_controls_upper_bounds_multipliers, &_gf1a_algebraic_states_controls_lower_bounds_multipliers,
      &_gf1a_point_constraints, &_gf1a_point_constraints_multipliers,
      &_gf1a_path_constr, &_gf1a_path_constr_multipliers,
      &_gf1a_int_constr,
      &_gf1a_fo_eqns, &_gf1a_fo_eqns_multipliers,
      &_gf1a_post_processing, &_gf1a_differential_post_processing, &_gf1a_integral_post_processing
    };

    for (auto spline_vec : all_spline_vecs)
      for (auto & tmp_gf1a : *spline_vec) {
        if (tmp_gf1a != nullptr)
          delete tmp_gf1a;
        tmp_gf1a = nullptr;
      }

  }

  // get a copy
  unique_ptr<OcpSolutionSinglePhase> RK1OcpSolutionSinglePhase::copy() const {
    return std::unique_ptr<OcpSolutionSinglePhase>(new RK1OcpSolutionSinglePhase(*this));
  }

  //setter

  void RK1OcpSolutionSinglePhase::setSolution(real const alpha,
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
                                                   vec_2d_real const &integral_post_processing) {

    vec_1d_real vec_1d_points_zeros(zeta.size(), 0);
    vec_1d_real vec_1d_intervals_zeros(zeta.size() - 1, 0);
    vec_2d_real states_controls_bounds_mult_zeros(states_controls.size(), vec_1d_points_zeros);
    vec_2d_real algebraic_states_controls_bounds_mult_zeros(algebraic_states_controls.size(), vec_1d_intervals_zeros);
    vec_1d_real params_bounds_mult_zeros(params.size(), 0);
    vec_2d_real point_constr_mult_zeros(point_constr.size(), vec_1d_points_zeros);
    vec_2d_real path_constr_mult_zeros(path_constr.size(), vec_1d_intervals_zeros);
    vec_1d_real int_constr_mult_zeros(int_constr.size(), 0);
    vec_2d_real fo_eqns_mult_zeros(fo_eqns.size(), vec_1d_intervals_zeros);
    vec_1d_real bcs_mult_zero = vec_1d_real(boundary_conditions.size(), 0);

    setSolution(alpha,
                target,
                zeta,
                cumulative_target,
                integrand_target,
                states_controls,
                algebraic_states_controls,
                params,
                point_constr,
                path_constr,
                int_constr,
                fo_eqns,
                boundary_conditions,
                post_processing,
                differential_post_processing,
                integral_post_processing,

                states_controls_bounds_mult_zeros,
                states_controls_bounds_mult_zeros,
                algebraic_states_controls_bounds_mult_zeros,
                algebraic_states_controls_bounds_mult_zeros,
                params_bounds_mult_zeros,
                params_bounds_mult_zeros,
                point_constr_mult_zeros,
                path_constr_mult_zeros,
                int_constr_mult_zeros,
                fo_eqns_mult_zeros,
                bcs_mult_zero
    );
  }

  void RK1OcpSolutionSinglePhase::setSolution(real const alpha,
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
                                                   vec_1d_real const &bcs_multipliers) {

    _alpha = alpha;
    _target = target;
    _zeta = zeta;
    _cumulative_target = cumulative_target;
    _integrand_target = integrand_target;
    _states_controls = states_controls;
    _algebraic_states_controls = algebraic_states_controls;
    _parameters = params;
    _point_constraints = point_constr;
    _path_constr = path_constr;
    _int_constr = int_constr;
    _fo_eqns = fo_eqns;
    _boundary_conditions = boundary_conditions;
    _post_processing = post_processing;
    _differential_post_processing = differential_post_processing;
    _integral_post_processing = integral_post_processing;

    _states_controls_upper_bounds_multipliers = states_controls_upper_bounds_multipliers;
    _states_controls_lower_bounds_multipliers = states_controls_lower_bounds_multipliers;
    _algebraic_states_controls_upper_bounds_multipliers = algebraic_states_controls_upper_bounds_multipliers;
    _algebraic_states_controls_lower_bounds_multipliers = algebraic_states_controls_lower_bounds_multipliers;
    _parameters_upper_bounds_multipliers = parameters_upper_bounds_multipliers;
    _parameters_lower_bounds_multipliers = parameters_lower_bounds_multipliers;
    _point_constraints_multipliers = point_constraints_multipliers;
    _path_constr_multipliers = path_constr_multipliers;
    _int_constr_multipliers = int_constr_multipliers;
    _fo_eqns_multipliers = fo_eqns_multipliers;
    _bcs_multipliers = bcs_multipliers;

    if (!areVectorsConsistent()) {
      MaverickSingleton::getInstance().Log(InfoLevel::info_level_warning,
                                           "RK1OcpSolutionSinglePhase::setSolution: solution has been set with non consistent vectors length. Solution will be cleared.\n");
      clear();
      return;
    }

    buildGf1a();
  }

  bool RK1OcpSolutionSinglePhase::areVectorsConsistent() const {

    if ((_states_controls_upper_bounds_multipliers.size() != _states_controls.size()) ||
        (_states_controls_lower_bounds_multipliers.size() != _states_controls.size()) ||
        (_algebraic_states_controls_upper_bounds_multipliers.size() != _algebraic_states_controls.size()) ||
        (_algebraic_states_controls_lower_bounds_multipliers.size() != _algebraic_states_controls.size()) ||
        (_parameters_upper_bounds_multipliers.size() != _parameters.size()) ||
        (_parameters_lower_bounds_multipliers.size() != _parameters.size()) ||
        (_point_constraints_multipliers.size() != _point_constraints.size()) ||
        (_path_constr_multipliers.size() != _path_constr.size()) ||
        (_int_constr_multipliers.size() != _int_constr.size()) ||
        (_fo_eqns_multipliers.size() != _fo_eqns.size()) ||
        (_bcs_multipliers.size() != _boundary_conditions.size())
        )
      return false;

    auto point_size = _zeta.size();
    auto interval_size = point_size - 1;
    if (point_size == 0)
      interval_size = 0;

    auto point_size_vecs = {
      &_states_controls, &_point_constraints,
      &_int_constr, &_post_processing, &_integral_post_processing,
      &_states_controls_upper_bounds_multipliers,
      &_states_controls_lower_bounds_multipliers,
      &_point_constraints_multipliers
    };

    auto interval_size_vecs = {
      &_algebraic_states_controls, &_path_constr, &_fo_eqns,
      &_differential_post_processing,
      &_algebraic_states_controls_upper_bounds_multipliers,
      &_algebraic_states_controls_lower_bounds_multipliers,
      &_path_constr_multipliers,
      &_fo_eqns_multipliers
    };

    if (point_size != _cumulative_target.size()) return false;
    for (auto vec_p : point_size_vecs)
      for (auto & x : *vec_p) {
        if (x.size() != point_size)
          return false;
      }

    if (interval_size != _integrand_target.size()) return false;
    for (auto vec_p : interval_size_vecs)
      for (auto & x : *vec_p) {
        if (x.size() != interval_size)
          return false;
      }

    return true;
  }

  void RK1OcpSolutionSinglePhase::buildGf1a() {
    clearPointers();

    using data_touple = std::tuple<std::vector<MaverickUtils::GenericFunction1AInterface *> *,
      vec_2d_real *, bool>;

    auto all_spline_vecs_and_data = {
      data_touple(&_gf1a_states_controls, &_states_controls, true),
      data_touple(&_gf1a_states_controls_upper_bounds_multipliers, &_states_controls_upper_bounds_multipliers, true),
      data_touple(&_gf1a_states_controls_lower_bounds_multipliers, &_states_controls_lower_bounds_multipliers, true),
      data_touple(&_gf1a_algebraic_states_controls, &_algebraic_states_controls, false),
      data_touple(&_gf1a_algebraic_states_controls_upper_bounds_multipliers, &_algebraic_states_controls_upper_bounds_multipliers, false),
      data_touple(&_gf1a_algebraic_states_controls_lower_bounds_multipliers, &_algebraic_states_controls_lower_bounds_multipliers, false),
      data_touple(&_gf1a_point_constraints, &_point_constraints, true),
      data_touple(&_gf1a_point_constraints_multipliers, &_point_constraints_multipliers, true),
      data_touple(&_gf1a_path_constr, &_path_constr, false),
      data_touple(&_gf1a_path_constr_multipliers, &_path_constr_multipliers, false),
      data_touple(&_gf1a_int_constr, &_int_constr, true),
      data_touple(&_gf1a_fo_eqns, &_fo_eqns, false),
      data_touple(&_gf1a_fo_eqns_multipliers, &_fo_eqns_multipliers, false),
      data_touple(&_gf1a_post_processing, &_post_processing, true),
      data_touple(&_gf1a_differential_post_processing, &_differential_post_processing, false),
      data_touple(&_gf1a_integral_post_processing, &_integral_post_processing, true)
    };

    if (_zeta.size() == 0) { // in this case setup a null (zero) solution
      GC::GenericContainer gc;

      for (auto touple_data : all_spline_vecs_and_data) {
        auto p_spline_vec = std::get<0>(touple_data);
        auto p_data_vec = std::get<1>(touple_data);
          for (auto i = 0; i < p_data_vec->size(); i++) {
          MaverickUtils::GF1APolyFive *tmp_gf1a = new MaverickUtils::GF1APolyFive();
          tmp_gf1a->setup(gc);
          p_spline_vec->push_back(tmp_gf1a);
        }
      }

      {
        MaverickUtils::GF1APolyFive *tmp_gf1a = new MaverickUtils::GF1APolyFive();
        tmp_gf1a->setup(gc);
        _gf1a_integrand_target = tmp_gf1a;

        tmp_gf1a = new MaverickUtils::GF1APolyFive();
        tmp_gf1a->setup(gc);
        _gf1a_cumulative_target = tmp_gf1a;
      }
    } else if (_zeta.size() == 1) {
      // in this case setup a constant solution
      GC::GenericContainer gc;

      for (auto touple_data : all_spline_vecs_and_data) {
        auto p_spline_vec = std::get<0>(touple_data);
        auto p_data_vec = std::get<1>(touple_data);
        for (auto & data : *p_data_vec) {
          MaverickUtils::GF1APolyFive *tmp_gf1a = new MaverickUtils::GF1APolyFive();
          tmp_gf1a->setup(gc);
          tmp_gf1a->setA0(data[0]);
          p_spline_vec->push_back(tmp_gf1a);
        }
      }

      {
        MaverickUtils::GF1APolyFive *tmp_gf1a = new MaverickUtils::GF1APolyFive();
        tmp_gf1a->setup(gc);
        tmp_gf1a->setA0(_integrand_target[0]);
        _gf1a_integrand_target = tmp_gf1a;

        tmp_gf1a = new MaverickUtils::GF1APolyFive();
        tmp_gf1a->setup(gc);
        tmp_gf1a->setA0(_cumulative_target[0]);
        _gf1a_cumulative_target = tmp_gf1a;
      }
    } else { // in this case build the splines
      vec_1d_real zeta_alpha = extractAlphaPoints(_zeta, _alpha);

      for (auto touple_data : all_spline_vecs_and_data) {
        auto p_spline_vec = std::get<0>(touple_data);
        auto p_data_vec = std::get<1>(touple_data);
        auto is_point = std::get<2>(touple_data);
        auto const & this_zeta = is_point ? _zeta : zeta_alpha;

        for (auto & data : *p_data_vec) {
          GF1A_TYPE *tmp_gf1a = new GF1A_TYPE();
          tmp_gf1a->setCheckRange(SPLINE_CHECK_RANGE);
          tmp_gf1a->setup(SPLINE_TYPE, this_zeta, data, SPLINE_EXTEND_RANGE);
          p_spline_vec->push_back(tmp_gf1a);
        }
      }

      {
        GF1A_TYPE *tmp_gf1a = new GF1A_TYPE();
        tmp_gf1a->setCheckRange(SPLINE_CHECK_RANGE);
        tmp_gf1a->setup(SPLINE_TYPE, zeta_alpha, _integrand_target, SPLINE_EXTEND_RANGE);
        _gf1a_integrand_target = tmp_gf1a;

        tmp_gf1a = new GF1A_TYPE();
        tmp_gf1a->setCheckRange(SPLINE_CHECK_RANGE);
        tmp_gf1a->setup(SPLINE_TYPE, _zeta, _cumulative_target, SPLINE_EXTEND_RANGE);
        _gf1a_cumulative_target = tmp_gf1a;
      }

    }
  }

  void RK1OcpSolutionSinglePhase::clear() {
    _target = 0;
    _cumulative_target = {};
    _integrand_target = {};
    _states_controls = {};
    _algebraic_states_controls = {};
    _point_constraints = {};
    _path_constr = {};
    _int_constr = {};
    _fo_eqns = {};
    _parameters = {};
    _zeta = {};
    _boundary_conditions = {};
    _post_processing = {};
    _differential_post_processing = {};
    _integral_post_processing = {};

    _states_controls_upper_bounds_multipliers = {};
    _states_controls_lower_bounds_multipliers = {};
    _algebraic_states_controls_upper_bounds_multipliers = {};
    _algebraic_states_controls_lower_bounds_multipliers = {};
    _parameters_upper_bounds_multipliers = {};
    _parameters_lower_bounds_multipliers = {};
    _point_constraints_multipliers = {};
    _path_constr_multipliers = {};
    _int_constr_multipliers = {};
    _fo_eqns_multipliers = {};
    _bcs_multipliers = {};
  }

  void RK1OcpSolutionSinglePhase::writeMeshVarsToStream(ostream &body, ostream &header) const {
    writeMeshVarsToStream(body, header, false, 0);
  }

  void RK1OcpSolutionSinglePhase::writeMeshVarsToStream(ostream &body, ostream &header, bool const add_phase_index,
                                                             integer const phase_index) const {
    //write solution to output in the following sequence:

    //header
    if (add_phase_index)
      header << "phase" << StreamChars::separator;
    header << "zeta" << StreamChars::separator;

    using name_and_spline = std::pair<string,
        std::vector<MaverickUtils::GenericFunction1AInterface *> const * >;
    auto names_and_splines = {
      name_and_spline("state_control", &_gf1a_states_controls),
      name_and_spline("algebraic_state_control", &_gf1a_algebraic_states_controls),
      name_and_spline("post_processing", &_gf1a_post_processing),
      name_and_spline("diff_post_processing", &_gf1a_differential_post_processing),
      name_and_spline("int_post_processing", &_gf1a_integral_post_processing),
      name_and_spline("lambda_states_controls_upper", &_gf1a_states_controls_upper_bounds_multipliers),
      name_and_spline("lambda_states_controls_lower", &_gf1a_states_controls_lower_bounds_multipliers),
      name_and_spline("lambda_algebraic_states_controls_upper", &_gf1a_algebraic_states_controls_upper_bounds_multipliers),
      name_and_spline("lambda_algebraic_states_controls_lower", &_gf1a_algebraic_states_controls_lower_bounds_multipliers),
      name_and_spline("lambda_states_constraint", &_gf1a_point_constraints_multipliers),
      name_and_spline("lambda_path_constraint", &_gf1a_path_constr_multipliers),
      name_and_spline("lambda_point_constraint", &_gf1a_point_constraints_multipliers),
      name_and_spline("lambda_fo_equation", &_gf1a_fo_eqns_multipliers),
      name_and_spline("states_constraint", &_gf1a_point_constraints),
      name_and_spline("path_constraint", &_gf1a_path_constr),
      name_and_spline("integral_constraint", &_gf1a_int_constr),
      name_and_spline("point_constraint", &_gf1a_point_constraints),
      name_and_spline("fo_equation", &_gf1a_fo_eqns)
    };

    using name_and_constant = std::pair<string, vec_1d_real const * >;
    auto names_and_constants = {
      name_and_constant("parameter", &_parameters),
      name_and_constant("lambda_parameter_upper", &_parameters_upper_bounds_multipliers),
      name_and_constant("lambda_parameter_lower", &_parameters_lower_bounds_multipliers),
      name_and_constant("lambda_integral_constraint", &_int_constr_multipliers),
      name_and_constant("boundary_condition", &_boundary_conditions),
      name_and_constant("lambda_boundary_condition", &_bcs_multipliers)
    };

    for (auto & name_and_spline : names_and_splines)
      for (auto i = 0; i < name_and_spline.second->size(); i++)
        header << name_and_spline.first << i << StreamChars::separator;

    for (auto & name_and_constant : names_and_constants)
      for (auto i = 0; i < name_and_constant.second->size(); i++)
        header << "parameter" << i << StreamChars::separator;

    //target
    header << "integrand_target" << StreamChars::separator
           << "cumulative_target" << StreamChars::new_line;

    //body
    for (integer mesh_point = 0; mesh_point < _zeta.size(); mesh_point++) {
      if (add_phase_index)
        body << phase_index << StreamChars::separator;

      real const zeta = _zeta[mesh_point];
      body << zeta << StreamChars::separator;

      body << std::scientific << std::setprecision(15);

      for (auto & name_and_spline : names_and_splines)
        for (auto i = 0; i < name_and_spline.second->size(); i++)
          header << (*name_and_spline.second)[i]->funcEval(zeta) << StreamChars::separator;

      for (auto & name_and_constant : names_and_constants)
        for (auto i = 0; i < name_and_constant.second->size(); i++)
          header << (*name_and_constant.second)[i] << StreamChars::separator;

      //target
      body << _gf1a_integrand_target->funcEval(zeta) << StreamChars::separator
           << _cumulative_target[mesh_point] << StreamChars::new_line;
    }
  }

  void RK1OcpSolutionSinglePhase::evalAtMesh(real zeta,
                                                  integer const num_states_controls, real *states_controls,
                                                  real *states_controls_upper_bounds_mult,
                                                  real *states_controls_lower_bounds_mult,
                                                  integer const num_alg_states_controls,
                                                  real *algebraic_states_controls,
                                                  real *algebraic_states_controls_upper_bounds_mult,
                                                  real *algebraic_states_controls_lower_bounds_mult,
                                                  integer const num_fo_eqns, real *fo_eqns, real *fo_eqns_mult,
                                                  integer const num_point_constr, real *states_constr,
                                                  real *point_constr_mult,
                                                  integer const num_path_constr, real *path_constr,
                                                  real *path_constr_mult,
                                                  integer const num_int_constr, real *int_constr,

                                                  integer const num_post_proc, real *post_proc,
                                                  integer const num_diff_post_proc, real *diff_post_proc,
                                                  integer const num_int_post_proc, real *int_post_proc
  ) const {

    if ((states_controls != nullptr) || (states_controls_upper_bounds_mult != nullptr) ||
        (states_controls_lower_bounds_mult != nullptr))
      MAVERICK_ASSERT(num_states_controls == _states_controls.size(),
                      "RK1OcpSolutionSinglePhase::evalAtMesh: wrong states and controls size.\n")

    if ((algebraic_states_controls != nullptr) || (algebraic_states_controls_upper_bounds_mult != nullptr) ||
        (algebraic_states_controls_lower_bounds_mult != nullptr))
      MAVERICK_ASSERT(num_alg_states_controls == _algebraic_states_controls.size(),
                      "RK1OcpSolutionSinglePhase::evalAtMesh: wrong algerbaic states and controls size.\n")

    if (int_constr != nullptr)
      MAVERICK_ASSERT(num_int_constr == _int_constr.size(),
                      "OcpCompleteSolution::evalAtMesh: wrong integral constraints size.\n")

    if (post_proc != nullptr)
      MAVERICK_ASSERT(num_post_proc == _post_processing.size(),
                      "OcpCompleteSolution::evalAtMesh: wrong post processing size.\n")

    if (diff_post_proc != nullptr)
      MAVERICK_ASSERT(num_diff_post_proc == _differential_post_processing.size(),
                      "OcpCompleteSolution::evalAtMesh: wrong differential post processing size.\n")

    if (int_post_proc != nullptr)
      MAVERICK_ASSERT(num_int_post_proc == _integral_post_processing.size(),
                      "OcpCompleteSolution::evalAtMesh: wrong integral post processing size.\n")

    if ((states_constr != nullptr) || (point_constr_mult != nullptr))
      MAVERICK_ASSERT(num_point_constr == _point_constraints.size(),
                      "OcpCompleteSolution::evalAtMesh: wrong point constraints size.\n")

    if ((path_constr != nullptr) || (path_constr_mult != nullptr))
      MAVERICK_ASSERT(num_path_constr == _path_constr.size(),
                      "OcpCompleteSolution::evalAtMesh: wrong path constraints size.\n")

    if ((fo_eqns != nullptr) || (fo_eqns_mult != nullptr))
      MAVERICK_ASSERT(num_fo_eqns == _fo_eqns_multipliers.size(),
                      "OcpCompleteSolution::evalAtMesh: wrong f.o. equations size.\n")

    using pointer_and_spline = std::pair<real *, std::vector<MaverickUtils::GenericFunction1AInterface *> const *>;

    auto pointer_and_splines = {
      pointer_and_spline(states_controls, &_gf1a_states_controls),
      pointer_and_spline(states_controls_upper_bounds_mult, &_gf1a_states_controls_upper_bounds_multipliers),
      pointer_and_spline(states_controls_lower_bounds_mult, &_gf1a_states_controls_lower_bounds_multipliers),
      pointer_and_spline(algebraic_states_controls, &_gf1a_algebraic_states_controls),
      pointer_and_spline(algebraic_states_controls_upper_bounds_mult, &_gf1a_algebraic_states_controls_upper_bounds_multipliers),
      pointer_and_spline(algebraic_states_controls_lower_bounds_mult, &_gf1a_algebraic_states_controls_lower_bounds_multipliers),
      pointer_and_spline(fo_eqns, &_gf1a_fo_eqns),
      pointer_and_spline(fo_eqns_mult, &_gf1a_fo_eqns_multipliers),
      pointer_and_spline(states_constr, &_gf1a_point_constraints),
      pointer_and_spline(point_constr_mult, &_gf1a_point_constraints_multipliers),
      pointer_and_spline(path_constr, &_gf1a_path_constr),
      pointer_and_spline(path_constr_mult, &_gf1a_path_constr_multipliers),
      pointer_and_spline(int_constr, &_gf1a_int_constr),
      pointer_and_spline(post_proc, &_gf1a_post_processing),
      pointer_and_spline(diff_post_proc, &_gf1a_differential_post_processing),
      pointer_and_spline(int_post_proc, &_gf1a_integral_post_processing)
    };

    for (auto & data : pointer_and_splines)
      if (data.first != nullptr)
        for (size i = 0; i < data.second->size(); i++)
          data.first[i] = (*data.second)[i]->funcEval(zeta);
  }

  void RK1OcpSolutionSinglePhase::eval(real const initial_zeta, real const final_zeta,

                                            integer const num_parameters, real *parameters,
                                            real *params_upper_bounds_mult, real *params_lower_bounds_mult,
                                            integer const num_boundary_conditions, real *boundary_conditions,
                                            real *boundary_conditions_mult,
                                            integer const num_int_constr, real *int_constr_at_end,
                                            real *int_constr_mult,
                                            integer const num_int_post_proc, real *int_post_proc_at_end
                                            ) const
  {

    if ((parameters != nullptr) || (params_upper_bounds_mult != nullptr) || (params_lower_bounds_mult != nullptr))
      MAVERICK_ASSERT(num_parameters == _parameters.size(),
                      "RK1OcpSolutionSinglePhase::evalParams wrong parameter size.\n")

    if ((int_constr_at_end != nullptr) || (int_constr_mult != nullptr))
      MAVERICK_ASSERT(num_int_constr == _int_constr_multipliers.size(),
                      "OcpCompleteSolution::evalParams: wrong integral constraints size.\n")

    if ((boundary_conditions != nullptr) || (boundary_conditions_mult != nullptr))
      MAVERICK_ASSERT(num_boundary_conditions == _bcs_multipliers.size(),
                      "OcpCompleteSolution::evalParams: wrong boundary conditions size.\n")

    if (int_post_proc_at_end != nullptr)
      MAVERICK_ASSERT(num_int_post_proc == _integral_post_processing.size(),
                      "OcpCompleteSolution::eval: wrong integral post processing size.\n")

    using pointer_and_vector = std::pair<real *, vec_1d_real const *>;
    auto pointer_and_vectors = {
      pointer_and_vector(parameters, &_parameters),
      pointer_and_vector(params_upper_bounds_mult, &_parameters_upper_bounds_multipliers),
      pointer_and_vector(params_lower_bounds_mult, &_parameters_lower_bounds_multipliers),
      pointer_and_vector(boundary_conditions, &_boundary_conditions),
      pointer_and_vector(boundary_conditions_mult, &_bcs_multipliers)
    };

    for (auto & data : pointer_and_vectors)
      if (data.first != nullptr)
        copyVectorTo(data.second->data(), data.first, (integer) data.second->size());

    if (int_constr_at_end != nullptr)
      for (size i = 0; i < num_int_constr; i++)
        int_constr_at_end[i] = *(_int_constr[i].end() - 1);

    if (int_constr_mult != nullptr)
      copyVectorTo(_int_constr_multipliers.data(), int_constr_mult, num_int_constr);

    if (int_post_proc_at_end != nullptr)
      for (size i = 0; i < num_int_post_proc; i++)
        int_post_proc_at_end[i] = *(_integral_post_processing[i].end() - 1);
  }

  // getters
  real RK1OcpSolutionSinglePhase::getTarget() const { return _target; }

  vec_1d_real const &RK1OcpSolutionSinglePhase::getCumulativeTarget() const { return _cumulative_target; }

  vec_1d_real const &RK1OcpSolutionSinglePhase::getIntegrandTarget() const { return _integrand_target; }

  vec_2d_real const &RK1OcpSolutionSinglePhase::getStatesControls() const { return _states_controls; }

  vec_2d_real const &
  RK1OcpSolutionSinglePhase::getAlgebraicStatesControls() const { return _algebraic_states_controls; }

  vec_2d_real const &RK1OcpSolutionSinglePhase::getPointConstraints() const { return _point_constraints; }

  vec_2d_real const &RK1OcpSolutionSinglePhase::getPathConstraints() const { return _path_constr; }

  vec_2d_real const &RK1OcpSolutionSinglePhase::getIntegralConstraints() const { return _int_constr; }

  vec_2d_real const &RK1OcpSolutionSinglePhase::getFoEquations() const { return _fo_eqns; }

  vec_1d_real const &RK1OcpSolutionSinglePhase::getParameters() const { return _parameters; }

  vec_1d_real const &RK1OcpSolutionSinglePhase::getDiscretisationPoints() const { return _zeta; }

  vec_1d_real const &RK1OcpSolutionSinglePhase::getBoundaryConditions() const { return _boundary_conditions; }

  vec_2d_real const &RK1OcpSolutionSinglePhase::getPostProcessing() const { return _post_processing; }

  vec_2d_real const &
  RK1OcpSolutionSinglePhase::getDifferentialPostProcessing() const { return _differential_post_processing; }

  vec_2d_real const &
  RK1OcpSolutionSinglePhase::getIntegralPostProcessing() const { return _integral_post_processing; }

  vec_2d_real const &
  RK1OcpSolutionSinglePhase::getStatesControlsUpperBoundsMultipliers() const { return _states_controls_upper_bounds_multipliers; }

  vec_2d_real const &
  RK1OcpSolutionSinglePhase::getStatesControlsLowerBoundsMultipliers() const { return _states_controls_lower_bounds_multipliers; }

  vec_2d_real const &
  RK1OcpSolutionSinglePhase::getAlgebraicStatesControlsUpperBoundsMultipliers() const { return _algebraic_states_controls_upper_bounds_multipliers; }

  vec_2d_real const &
  RK1OcpSolutionSinglePhase::getAlgebraicStatesControlsLowerBoundsMultipliers() const { return _algebraic_states_controls_lower_bounds_multipliers; }

  vec_1d_real const &
  RK1OcpSolutionSinglePhase::getParametersUpperBoundsMultipliers() const { return _parameters_upper_bounds_multipliers; }

  vec_1d_real const &
  RK1OcpSolutionSinglePhase::getParametersLowerBoundsMultipliers() const { return _parameters_lower_bounds_multipliers; }

  vec_2d_real const &
  RK1OcpSolutionSinglePhase::getPointConstraintsMultipliers() const { return _point_constraints_multipliers; }

  vec_2d_real const &
  RK1OcpSolutionSinglePhase::getPathConstraintsMultipliers() const { return _path_constr_multipliers; }

  vec_1d_real const &
  RK1OcpSolutionSinglePhase::getIntConstraintsMultipliers() const { return _int_constr_multipliers; }

  vec_2d_real const &RK1OcpSolutionSinglePhase::getFoEqnsMultipliers() const { return _fo_eqns_multipliers; }

  vec_1d_real const &
  RK1OcpSolutionSinglePhase::getBoundaryConditionsMultipliers() const { return _bcs_multipliers; }

  void RK1OcpSolutionSinglePhase::writeContentToGC(GC::GenericContainer &out_gc, MaverickOcp const *const p_ocp,
                                                        integer const i_phase) const {

    using name_func = std::function<string(integer)>;
    name_func get_sc_name = [p_ocp, i_phase](integer i) {
      auto num_s = p_ocp->numberOfStates(i_phase);
      if (i < num_s)
        return p_ocp->stateName(i_phase, i);
      return p_ocp->controlName(i_phase, i - num_s);
    };
    name_func get_asc_name = [p_ocp, i_phase](integer i) {
      auto num_s = p_ocp->numberOfAlgebraicStates(i_phase);
      if (i < num_s)
        return p_ocp->algebraicStateName(i_phase, i);
      return p_ocp->algebraicControlName(i_phase, i - num_s);
    };
    using name_and_spline_vec = std::tuple<string, std::vector<MaverickUtils::GenericFunction1AInterface *> const *, name_func>;
    auto names_and_spline_vecs = {
      name_and_spline_vec("states_controls", &_gf1a_states_controls, get_sc_name),
      name_and_spline_vec("algebraic_states_controls", &_gf1a_algebraic_states_controls, get_asc_name),
      name_and_spline_vec("post_processings", &_gf1a_post_processing, [p_ocp, i_phase](integer i){return p_ocp->postProcessingName(i_phase, i); }),
      name_and_spline_vec("differential_post_processings", &_gf1a_differential_post_processing, [p_ocp, i_phase](integer i){return p_ocp->differentialPostProcessingName(i_phase, i); }),
      name_and_spline_vec("integral_post_processings", &_gf1a_integral_post_processing, [p_ocp, i_phase](integer i){return p_ocp->integralPostProcessingName(i_phase, i);}),
      name_and_spline_vec("lambda_states_controls_upper", &_gf1a_states_controls_upper_bounds_multipliers, [&get_sc_name](integer i){return "lambda_" + get_sc_name(i) + "_upper";}),
      name_and_spline_vec("lambda_states_controls_lower", &_gf1a_states_controls_lower_bounds_multipliers, [&get_sc_name](integer i){return "lambda_" + get_sc_name(i) + "_lower";}),
      name_and_spline_vec("lambda_algebraic_states_controls_upper", &_gf1a_algebraic_states_controls_upper_bounds_multipliers, [&get_asc_name](integer i){return "lambda_" + get_asc_name(i) + "_upper";}),
      name_and_spline_vec("lambda_algebraic_states_controls_lower", &_gf1a_algebraic_states_controls_lower_bounds_multipliers, [&get_asc_name](integer i){return "lambda_" + get_asc_name(i) + "_lower";}),
      name_and_spline_vec("lambda_point_constraints", &_gf1a_point_constraints_multipliers, [p_ocp, i_phase](integer i){return "lambda_"+p_ocp->pointConstraintName(i_phase, i);}),
      name_and_spline_vec("lambda_path_constraints", &_gf1a_path_constr_multipliers, [p_ocp, i_phase](integer i){return "lambda_"+p_ocp->pathConstraintName(i_phase, i);}),
      name_and_spline_vec("lambda_fo_equations", &_gf1a_fo_eqns_multipliers, [](integer i){return "lambda_fo_eqs" + std::to_string(i);}),
      name_and_spline_vec("point_constraints", &_gf1a_point_constraints, [p_ocp, i_phase](integer i){return p_ocp->pointConstraintName(i_phase, i);}),
      name_and_spline_vec("path_constraints", &_gf1a_path_constr, [p_ocp, i_phase](integer i){return p_ocp->pathConstraintName(i_phase, i);}),
      name_and_spline_vec("fo_equations", &_gf1a_fo_eqns, [](integer i){return "fo_eq" + std::to_string(i);}),
      name_and_spline_vec("integral_constraints", &_gf1a_int_constr, [p_ocp, i_phase](integer i){return p_ocp->intConstraintName(i_phase, i);})
    };
    using name_and_spline = std::pair<string, MaverickUtils::GenericFunction1AInterface const *>;
    auto names_and_splines = {
      name_and_spline("integrand_lagrange_target", _gf1a_integrand_target),
      name_and_spline("cumulative_lagrange_target", _gf1a_cumulative_target)
    };
    using name_and_vec = std::tuple<string, std::vector<real> const *, name_func>;
    auto names_and_vecs = {
      name_and_vec("parameters", &_parameters, [p_ocp, i_phase](integer i){return p_ocp->parameterName(i_phase, i);}),
      name_and_vec("lambda_parameters_upper", &_parameters_upper_bounds_multipliers, [p_ocp, i_phase](integer i){return "lambda_" + p_ocp->parameterName(i_phase, i) + "_upper";}),
      name_and_vec("lambda_parameters_lower", &_parameters_lower_bounds_multipliers, [p_ocp, i_phase](integer i){return "lambda_" + p_ocp->parameterName(i_phase, i) + "_lower";}),
      name_and_vec("boundary_conditions", &_boundary_conditions, [p_ocp, i_phase](integer i){return p_ocp->boundaryConditionName(i_phase, i);}),
      name_and_vec("lambda_boundary_conditions", &_bcs_multipliers, [p_ocp, i_phase](integer i){return "lambda_" + p_ocp->boundaryConditionName(i_phase, i);}),
      name_and_vec("lambda_integral_constraints", &_int_constr_multipliers, [p_ocp, i_phase](integer i){return "lambda_" + p_ocp->intConstraintName(i_phase, i);})
    };

    auto insert_values_to_gc_on_zeta = [&names_and_spline_vecs, &names_and_splines, p_ocp](GC::GenericContainer & out_gc, vec_1d_real const & zeta){
      for (auto & name_and_spline_vec : names_and_spline_vecs) {
        auto const & splines_vec = *(std::get<1>(name_and_spline_vec));
        if (p_ocp == nullptr) {
          auto const & name = std::get<0>(name_and_spline_vec);
          out_gc[name].set_vector(splines_vec.size());
          vector<real> tmp_vec;
          for (auto i = 0; i < splines_vec.size(); i++) {
            auto & spline = splines_vec[i];
            tmp_vec.clear();
            if (spline != nullptr)
              for (auto & this_zeta : zeta)
                tmp_vec.push_back(spline->funcEval(this_zeta));
            out_gc[name][i].set_vec_real(tmp_vec);
          }
        } else {
          vector<real> tmp_vec;
          auto const & get_name = std::get<2>(name_and_spline_vec);
          for (auto i = 0; i < splines_vec.size(); i++) {
            auto & spline = splines_vec[i];
            tmp_vec.clear();
            if (spline != nullptr)
              for (auto & this_zeta : zeta)
                tmp_vec.push_back(spline->funcEval(this_zeta));
            out_gc[get_name(i)].set_vec_real(tmp_vec);
          }
        }

      }

      for (auto & name_and_spline : names_and_splines) {
        auto & name = name_and_spline.first;
        auto  p_spline = name_and_spline.second;
        vector<real> tmp_vec;
        if (p_spline != nullptr)
          for (auto & this_zeta : zeta)
            tmp_vec.push_back(p_spline->funcEval(this_zeta));
        out_gc[name].set_vec_real(tmp_vec);
      }
    };

    out_gc.clear();
    out_gc["zeta"].set_vec_real(_zeta);
    out_gc["target"].set_real(getTarget());
    for (auto & names_and_vec : names_and_vecs) {
      auto zero_vec = vec_1d_real();
      auto * vec = std::get<1>(names_and_vec);
      if (vec == nullptr)
        vec = &zero_vec;
      if (p_ocp == nullptr)
        out_gc[std::get<0>(names_and_vec)].set_vec_real(*vec);
      else
        for (int i = 0; i < vec->size(); i++)
          out_gc[std::get<2>(names_and_vec)(i)] = (*vec)[i];
    }
    insert_values_to_gc_on_zeta(out_gc, _zeta);

    GC::GenericContainer &gc_alpha_point = out_gc["alpha_point_vars"];
    vec_1d_real zeta_alpha;
    zeta_alpha.reserve(_zeta.size()-1);
    for (size index = 0; index < _zeta.size() - 1; index++) {
      zeta_alpha.push_back( _zeta[index] * (1 - _alpha) + _zeta[index + 1] * _alpha);
    }
    gc_alpha_point["zeta"].set_vec_real(zeta_alpha);
    insert_values_to_gc_on_zeta(gc_alpha_point, zeta_alpha);
  }

  static std::vector<real>
  resampleVariableAtDifferentMesh(std::vector<real> const &given_mesh, std::vector<real> const &variable,
                                  std::vector<real> const &new_mesh) {
    MaverickUtils::GF1ASpline gf1a;
    gf1a.setup("Linear", given_mesh, variable,
               MaverickUtils::GF1ASpline::ExtendRange::keep_derivative); // "Akima", "Linear"

    std::vector<real> out;
    for (real x : new_mesh)
      out.push_back(gf1a.funcEval(x));

    return out;
  }

  std::unique_ptr<RK1OcpSolutionSinglePhase>
  RK1OcpSolutionSinglePhase::convertFromRealTable(real_table const &table, MaverickOcp const &ocp_problem,
                                                       integer const i_phase,
                                                       std::vector<std::string> &found_variables) {

    RK1OcpSolutionSinglePhase *sol = new RK1OcpSolutionSinglePhase();

    vec_1d_real const &zeta = table.at("zeta");
    vec_1d_real zeta_center;
    for (vec_1d_real::const_iterator it = zeta.begin(); it != zeta.end() - 1; it++) {
      zeta_center.push_back(((*it) + (*(it + 1))) / 2);
    }
    // zeta_center.push_back( zeta_center.back() ); // copy last element

    vec_1d_real zeros_zeta_1d = vec_1d_real(zeta.size(), 0);
    vec_1d_real zeros_zeta_center_1d = vec_1d_real(zeta_center.size(), 0);

    vec_2d_real point_constr = vec_2d_real(ocp_problem.numberOfPointConstraints(i_phase), zeros_zeta_1d);
    vec_2d_real path_constr = vec_2d_real(ocp_problem.numberOfPathConstraints(i_phase), zeros_zeta_center_1d);
    vec_2d_real int_constr = vec_2d_real(ocp_problem.numberOfIntConstraints(i_phase), zeros_zeta_1d);
    vec_2d_real fo_eqns = vec_2d_real(ocp_problem.numberOfStates(i_phase), zeros_zeta_center_1d);
    vec_1d_real boundary_conditions = vec_1d_real(ocp_problem.numberOfBoundaryConditions(i_phase), 0);
    vec_2d_real post_processing = vec_2d_real(ocp_problem.numberOfPostProcessing(i_phase), zeros_zeta_1d);
    vec_2d_real differential_post_processing = vec_2d_real(ocp_problem.numberOfDifferentialPostProcessing(i_phase),
                                                           zeros_zeta_center_1d);
    vec_2d_real integral_post_processing = vec_2d_real(ocp_problem.numberOfIntegralPostProcessing(i_phase),
                                                       zeros_zeta_1d);

    vec_2d_real states_controls = vec_2d_real(
        ocp_problem.numberOfStates(i_phase) + ocp_problem.numberOfControls(i_phase), zeros_zeta_center_1d);
    vec_2d_real algebraic_states_controls = vec_2d_real(
        ocp_problem.numberOfAlgebraicStates(i_phase) + ocp_problem.numberOfAlgebraicControls(i_phase),
        zeros_zeta_1d);
    vec_1d_real params = vec_1d_real(ocp_problem.numberOfParameters(i_phase), 0);

    // now check for states or controls declared in the table
    integer i_s;
    for (i_s = 0; i_s < ocp_problem.numberOfStates(i_phase); i_s++) {
      try {
        string name = ocp_problem.stateName(i_phase, i_s);
        vec_1d_real const &vec = table.at(name);
        states_controls[i_s] = vec;
        found_variables.push_back(name);
      } catch (...) {}
    }
    for (integer i_c = 0; i_c < ocp_problem.numberOfControls(i_phase); i_c++) {
      try {
        string name = ocp_problem.controlName(i_phase, i_c);
        vec_1d_real const &vec = table.at(name);
        states_controls[i_c + i_s] = vec;
        found_variables.push_back(name);
      } catch (...) {}
    }

    for (i_s = 0; i_s < ocp_problem.numberOfAlgebraicStates(i_phase); i_s++) {
      try {
        string name = ocp_problem.algebraicStateName(i_phase, i_s);
        vec_1d_real const &vec = table.at(name);
        algebraic_states_controls[i_s] = resampleVariableAtDifferentMesh(zeta, vec, zeta_center);
        found_variables.push_back(name);
      } catch (...) {}
    }
    for (integer i_c = 0; i_c < ocp_problem.numberOfAlgebraicControls(i_phase); i_c++) {
      try {
        string name = ocp_problem.algebraicControlName(i_phase, i_c);
        vec_1d_real const &vec = table.at(name);
        algebraic_states_controls[i_c + i_s] = resampleVariableAtDifferentMesh(zeta, vec, zeta_center);
        found_variables.push_back(name);
      } catch (...) {}
    }

    //TODO: find multiplier in guess
    sol->setSolution(0.5,
                     0, //target
                     zeta,
                     zeros_zeta_1d, zeros_zeta_center_1d, //cumulative and integrand target
                     states_controls, algebraic_states_controls, params,
                     point_constr, path_constr, int_constr, fo_eqns,
                     boundary_conditions,
                     post_processing, differential_post_processing, integral_post_processing);

    return std::unique_ptr<RK1OcpSolutionSinglePhase>(sol);
  }
}
