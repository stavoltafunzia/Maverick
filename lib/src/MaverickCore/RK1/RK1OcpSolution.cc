#include "RK1OcpSolution.hh"
#include "MaverickCore/MaverickPrivateDefinitions.hh"
#include "MaverickCore/MaverickFunctions.hh"

using namespace std;

namespace Maverick {

  RK1OcpSolution::RK1OcpSolution() {
  }

  RK1OcpSolution::RK1OcpSolution(RK1OcpSolution const &ocp_solution) {
    clear();
    for (integer i = 0; i < ocp_solution.getNumberOfPhases(); i++) {
      _solutions.push_back(ocp_solution(i));
    }
    _solutions.shrink_to_fit();
  }

  RK1OcpSolution &RK1OcpSolution::operator=(const RK1OcpSolution &ocp_solution) {
    clear();
    for (integer i = 0; i < ocp_solution.getNumberOfPhases(); i++) {
      _solutions.push_back(ocp_solution(i));
    }
    return *this;
  }

  OcpSolutionSinglePhase const &RK1OcpSolution::operator[](integer const i_phase) const {
    return this->operator()(i_phase);
  }

  RK1OcpSolutionSinglePhase const &RK1OcpSolution::operator()(integer const i_phase) const {
    MAVERICK_SKIPABLE_ASSERT(i_phase < safeCastToInt(_solutions.size()),
                             "RK1OcpSolutionSinglePhase::const operator(): number of mesh point " << i_phase
                                                                                                       << " greater than "
                                                                                                       <<
                                                                                                       _solutions.size() -
                                                                                                       1 << ".")
    return _solutions[i_phase];
  }

  RK1OcpSolutionSinglePhase &RK1OcpSolution::operator()(integer const i_phase) {
    MAVERICK_SKIPABLE_ASSERT(i_phase < safeCastToInt(_solutions.size()),
                             "RK1OcpSolutionSinglePhase::operator(): number of mesh point " << i_phase
                                                                                                 << " greater than "
                                                                                                 << _solutions.size() -
                                                                                                    1 << ".")
    return _solutions[i_phase];
  }

  unique_ptr<OcpSolution> RK1OcpSolution::copy() const {
    return unique_ptr<RK1OcpSolution>(new RK1OcpSolution(*this));
  }

  RK1OcpSolution &RK1OcpSolution::operator<<(RK1OcpSolutionSinglePhase const &ocp_solution) {
    _solutions.push_back(ocp_solution);
    return *this;
  }

  integer RK1OcpSolution::getNumberOfPhases() const {
    return safeCastToInt(_solutions.size());
  }

  void RK1OcpSolution::clear() {
    _solutions.clear();
  }

  void
  RK1OcpSolution::setSolutionAtPhase(integer const i_phase, RK1OcpSolutionSinglePhase const &ocp_solution) {
    integer additional_phases = i_phase - getNumberOfPhases() + 1;
    if (additional_phases <= 0) {
      _solutions[i_phase] = ocp_solution;
    } else {
      for (integer i = 0; i < additional_phases - 1; i++)
        _solutions.push_back(RK1OcpSolutionSinglePhase());
      _solutions.push_back(ocp_solution);
    }
  }

  real RK1OcpSolution::getTarget() const {
    real target = 0;
    for (size_t i = 0; i < _solutions.size(); i++)
      target += _solutions[i].getTarget();
    return target;
  }

  void RK1OcpSolution::evalAtMesh(integer const i_phase,
                                       real const zeta,

                                       integer const num_states_controls, real *states_controls,
                                       real *states_controls_upper_bounds_mult, real *states_controls_lower_bounds_mult,
                                       integer const num_alg_states_controls, real *algebraic_states_controls,
                                       real *algebraic_states_controls_upper_bounds_mult,
                                       real *algebraic_states_controls_lower_bounds_mult,
                                       integer const num_fo_eqns, real *fo_eqns_mult,
                                       integer const num_point_constr, real *point_constr_mult,
                                       integer const num_path_constr, real *path_constr_mult
  ) const {

    evalAtMesh(i_phase,
               zeta,

               num_states_controls, states_controls, states_controls_upper_bounds_mult,
               states_controls_lower_bounds_mult,
               num_alg_states_controls, algebraic_states_controls, algebraic_states_controls_upper_bounds_mult,
               algebraic_states_controls_lower_bounds_mult,
               num_fo_eqns, nullptr, fo_eqns_mult,
               num_point_constr, nullptr, point_constr_mult,
               num_path_constr, nullptr, path_constr_mult,

               0, nullptr,
               0, nullptr,
               0, nullptr,
               0, nullptr);
  }

  void RK1OcpSolution::eval(integer const i_phase,
                                 real const initial_zeta, real const final_zeta,

                                 integer const num_parameters, real *parameters, real *params_upper_bounds_mult,
                                 real *params_lower_bounds_mult,
                                 integer const num_boundary_conditions, real *boundary_conditions_mult,
                                 integer const num_int_constr, real *int_constr_mult
  ) const {
    eval(i_phase,
         initial_zeta, final_zeta,

         num_parameters, parameters, params_upper_bounds_mult, params_lower_bounds_mult,
         num_boundary_conditions, nullptr, boundary_conditions_mult,
         num_int_constr, nullptr, int_constr_mult,
         0, nullptr);
  }

  // full evaluation
  void RK1OcpSolution::evalAtMesh(integer const i_phase,
                                       real zeta,

                                       integer const num_states_controls, real *states_controls,
                                       real *states_controls_upper_bounds_mult, real *states_controls_lower_bounds_mult,
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
  ) const {
    MAVERICK_ASSERT(i_phase < getNumberOfPhases(),
                    "RK1OcpSolution::evalAtMesh: phase out of bounds. Requested phase " << i_phase
                                                                                             << " when phase bounds are 0 - "
                                                                                             << getNumberOfPhases() - 1
                                                                                             << ".\n")
    _solutions[i_phase].evalAtMesh(zeta,
                                   num_states_controls, states_controls, states_controls_upper_bounds_mult,
                                   states_controls_lower_bounds_mult,
                                   num_alg_states_controls, algebraic_states_controls,
                                   algebraic_states_controls_upper_bounds_mult,
                                   algebraic_states_controls_lower_bounds_mult,
                                   num_fo_eqns, fo_eqns, fo_eqns_mult,
                                   num_point_constr, states_constr, point_constr_mult,
                                   num_path_constr, path_constr, path_constr_mult,
                                   num_int_constr, int_constr,
                                   num_post_proc, post_proc,
                                   num_diff_post_proc, diff_post_proc,
                                   num_int_post_proc, int_post_proc);
  }

  void RK1OcpSolution::eval(integer const i_phase,
                                 real const initial_zeta, real const final_zeta,

                                 integer const num_parameters, real *parameters, real *params_upper_bounds_mult,
                                 real *params_lower_bounds_mult,
                                 integer const num_boundary_conditions, real *boundary_conditions,
                                 real *boundary_conditions_mult,
                                 integer const num_int_constr, real *int_constr_at_end, real *int_constr_mult,
                                 integer const num_int_post_proc, real *int_post_proc_at_end
  ) const {
    MAVERICK_ASSERT(i_phase < getNumberOfPhases(),
                    "RK1OcpSolution::eval: phase out of bounds. Requested phase " << i_phase
                                                                                       << " when phase bounds are 0 - "
                                                                                       << getNumberOfPhases() - 1
                                                                                       << ".\n")

    _solutions[i_phase].eval(initial_zeta, final_zeta,
                             num_parameters, parameters, params_upper_bounds_mult, params_lower_bounds_mult,
                             num_boundary_conditions, boundary_conditions, boundary_conditions_mult,
                             num_int_constr, int_constr_at_end, int_constr_mult,
                             num_int_post_proc, int_post_proc_at_end);
  }

  void RK1OcpSolution::writeAllMeshVarsToStream(std::ostream &out) const {
    for (size_t i_phase = 0; i_phase < _solutions.size(); i_phase++) {
      writeOnePhaseMeshVarsToStream(i_phase, out);
    }
  }

  void RK1OcpSolution::writeOnePhaseMeshVarsToStream(integer const i_phase, ostream &out) const {
    MAVERICK_ASSERT(i_phase < getNumberOfPhases(),
                    "RK1OcpSolution::writeMeshVarsToStream: phase out of bounds. Requested phase " << i_phase
                                                                                                        << " when phase bounds are 0 - "
                                                                                                        <<
                                                                                                        getNumberOfPhases() -
                                                                                                        1 << ".\n")

    RK1OcpSolutionSinglePhase const &current_sol = _solutions[i_phase];
    stringstream body;
    stringstream header;
    current_sol.writeMeshVarsToStream(body, header);

    stringstream number;
    number << i_phase;
    out << StreamChars::comment_begin << "Phase" << number.str() << "\n";
    out << header.str();
    out << body.str();
  }

  void RK1OcpSolution::writeContentToGC(GC::GenericContainer &out_gc, MaverickOcp const *const p_ocp) const {
    out_gc.clear();

    // target
    out_gc["target"].set_real(getTarget());
    // solution
    for (size_t i = 0; i < _solutions.size(); i++) {
      _solutions[i].writeContentToGC(out_gc["Phase" + std::to_string(i)], p_ocp, i);
    }
  }

  std::unique_ptr<RK1OcpSolution>
  RK1OcpSolution::getFromGuessTablesForOcpProblem(std::vector<real_table> const &guess_table,
                                                       MaverickOcp const &ocp_problem,
                                                       std::vector<std::vector<std::string>> &found_variables) {

    found_variables = {};
    RK1OcpSolution *solution = new RK1OcpSolution();
    for (size_t i = 0; i < guess_table.size(); i++) {
      std::vector<string> tmp_found;
      std::unique_ptr<RK1OcpSolutionSinglePhase> c_sol = RK1OcpSolutionSinglePhase::convertFromRealTable(
          guess_table[i], ocp_problem, i, tmp_found);
      solution->operator<<(*(c_sol.get()));
      found_variables.push_back(tmp_found);
    }
    return std::unique_ptr<RK1OcpSolution>(solution);
  }

}
