#include "MaverickOcp.hh"
#include "MaverickFunctions.hh"
#include "MaverickCore/OcpSolverOutput.hh"
#include "MaverickCore/MaverickPrivateDefinitions.hh"
#include <iomanip>
#include <fstream>

using namespace Maverick;
using namespace std;

MaverickOcp::MaverickOcp() {}

MaverickOcp::~MaverickOcp() {
  if (_model_params)
    delete[] _model_params;
}

void MaverickOcp::setupOcp(GC::GenericContainer const &gc) {
  derivedSetup(gc);
  _has_setup_ocp = true;

  if (!_has_setup_scaling)
    _scaling.setupForOcpAsNoScaling(*this);
}

void MaverickOcp::setupScaling(GC::GenericContainer const &gc_scaling, vec_2d_real const &zeta) {
  _scaling.setupForOcp(gc_scaling, *this, zeta);
  _has_setup_scaling = true;
}

OcpScaling const &MaverickOcp::getScaling() const {
  return _scaling;
}

string MaverickOcp::stateName(integer const i_phase, integer i) const {
  MAVERICK_ASSERT(i_phase >= 0 && i_phase < _num_p,
                  "in MaverickOcp::stateName( " << i_phase << " ) phase index out of range: [ 0.." << _num_p - 1
                                                << "]\n");
  MAVERICK_ASSERT(i >= 0 && i < _dim_x[i_phase],
                  "in MaverickOcp::stateName( " << i << " ) index out of range: [ 0.." << _dim_x[i_phase] - 1 << "]\n");
  return _states_names[i_phase][i];
}

string MaverickOcp::controlName(integer const i_phase, integer i) const {
  MAVERICK_ASSERT(i_phase >= 0 && i_phase < _num_p,
                  "in MaverickOcp::controlName( " << i_phase << " ) phase index out of range: [ 0.." << _num_p - 1
                                                  << "]\n");
  MAVERICK_ASSERT(i >= 0 && i < _dim_u[i_phase],
                  "in MaverickOcp::controlName( " << i << " ) index out of range: [ 0.." << _dim_u[i_phase] - 1
                                                  << "]\n");
  return _controls_names[i_phase][i];
}

string MaverickOcp::algebraicStateName(integer const i_phase, integer i) const {
  MAVERICK_ASSERT(i_phase >= 0 && i_phase < _num_p,
                  "in MaverickOcp::algebraicStateName( " << i_phase << " ) phase index out of range: [ 0.."
                                                         << _num_p - 1 << "]\n");
  MAVERICK_ASSERT(i >= 0 && i < _dim_ax[i_phase],
                  "in MaverickOcp::algebraicStateName( " << i << " ) index out of range: [ 0.." << _dim_ax[i_phase] - 1
                                                         << "]\n");
  return _algebraic_states_names[i_phase][i];
}

string MaverickOcp::algebraicControlName(integer const i_phase, integer i) const {
  MAVERICK_ASSERT(i_phase >= 0 && i_phase < _num_p,
                  "in MaverickOcp::algebraicControlName( " << i_phase << " ) phase index out of range: [ 0.."
                                                           << _num_p - 1 << "]\n");
  MAVERICK_ASSERT(i >= 0 && i < _dim_au[i_phase],
                  "in MaverickOcp::algebraicControlName( " << i << " ) index out of range: [ 0.."
                                                           << _dim_au[i_phase] - 1 << "]\n");
  return _algebraic_controls_names[i_phase][i];
}

string MaverickOcp::parameterName(integer const i_phase, integer i) const {
  MAVERICK_ASSERT(i_phase >= 0 && i_phase < _num_p,
                  "in MaverickOcp::parameterName( " << i_phase << " ) phase index out of range: [ 0.." << _num_p - 1
                                                    << "]\n");
  MAVERICK_ASSERT(i >= 0 && i < _dim_p[i_phase],
                  "in MaverickOcp::parameterName( " << i << " ) index out of range: [ 0.." << _dim_p[i_phase] - 1
                                                    << "]\n");
  return _parameters_names[i_phase][i];
}

string MaverickOcp::pointConstraintName(integer const i_phase, integer i) const {
  MAVERICK_ASSERT(i_phase >= 0 && i_phase < _num_p,
                  "in MaverickOcp::constraintName( " << i_phase << " ) phase index out of range: [ 0.." << _num_p - 1
                                                     << "]\n");
  MAVERICK_ASSERT(i >= 0 && i < _dim_poc[i_phase],
                  "in MaverickOcp::constraintName( " << i << " ) index out of range: [ 0.." << _dim_poc[i_phase] - 1
                                                     << "]\n");
  return _point_constraints_names[i_phase][i];
}

string MaverickOcp::pathConstraintName(integer const i_phase, integer i) const {
  MAVERICK_ASSERT(i_phase >= 0 && i_phase < _num_p,
                  "in MaverickOcp::pathConstraintName( " << i_phase << " ) phase index out of range: [ 0.."
                                                         << _num_p - 1 << "]\n");
  MAVERICK_ASSERT(i >= 0 && i < _dim_pc[i_phase],
                  "in MaverickOcp::pathConstraintName( " << i << " ) index out of range: [ 0.." << _dim_pc[i_phase] - 1
                                                         << "]\n");
  return _path_constraints_names[i_phase][i];
}

string MaverickOcp::intConstraintName(integer const i_phase, integer i) const {
  MAVERICK_ASSERT(i_phase >= 0 && i_phase < _num_p,
                  "in MaverickOcp::intConstraintName( " << i_phase << " ) phase index out of range: [ 0.." << _num_p - 1
                                                        << "]\n");
  MAVERICK_ASSERT(i >= 0 && i < _dim_ic[i_phase],
                  "in MaverickOcp::intConstraintName( " << i << " ) index out of range: [ 0.." << _dim_ic[i_phase] - 1
                                                        << "]\n");
  return _integral_constraints_names[i_phase][i];
}


string MaverickOcp::boundaryConditionName(integer const i_phase, integer i) const {
  MAVERICK_ASSERT(i_phase >= 0 && i_phase < _num_p,
                  "in MaverickOcp::boundaryConditionName( " << i_phase << " ) phase index out of range: [ 0.."
                                                            << _num_p - 1 << "]\n");
  MAVERICK_ASSERT(i >= 0 && i < _dim_bc[i_phase],
                  "in MaverickOcp::boundaryConditionName( " << i << " ) index out of range: [ 0.."
                                                            << _dim_bc[i_phase] - 1 << "]\n");
  return _boundary_conditions_names[i_phase][i];
}


string MaverickOcp::eventConstraintName(integer const i_phase, integer i) const {
  MAVERICK_ASSERT(i_phase >= 0 && i_phase < _num_p,
                  "in MaverickOcp::eventConstraintName( " << i_phase << " ) phase index out of range: [ 0.."
                                                          << _num_p - 1 << "]\n");
  MAVERICK_ASSERT(i >= 0 && i < _dim_ec[i_phase],
                  "in MaverickOcp::eventConstraintName( " << i << " ) index out of range: [ 0.." << _dim_ec[i_phase] - 1
                                                          << "]\n");
  return _event_constraints_names[i_phase][i];
}

// GUESS INTERFACE

void MaverickOcp::evalAtMesh(integer const i_phase,
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
  // consistency check
  if ((states_controls) || (states_controls_upper_bounds_mult) || (states_controls_lower_bounds_mult)) {
    MAVERICK_ASSERT(num_states_controls == numberOfStates(i_phase) + numberOfControls(i_phase),
                    "MaverickOcp::evalAtMesh: wrong number of states and controls.\n")
  }
  if ((algebraic_states_controls) || (algebraic_states_controls_upper_bounds_mult) ||
      (algebraic_states_controls_lower_bounds_mult)) {
    MAVERICK_ASSERT(num_alg_states_controls == numberOfAlgebraicStates(i_phase) + numberOfAlgebraicControls(i_phase),
                    "MaverickOcp::evalAtMesh: wrong number of algebraic states and controls.\n")
  }
  if (fo_eqns_mult) {
    MAVERICK_ASSERT(num_fo_eqns == numberOfFirstOrderEquations(i_phase),
                    "MaverickOcp::evalAtMesh: wrong number of first order equation.\n")
  }
  if (point_constr_mult) {
    MAVERICK_ASSERT(num_point_constr == numberOfPointConstraints(i_phase),
                    "MaverickOcp::evalAtMesh: wrong number of point constraints.\n")
  }
  if (path_constr_mult) {
    MAVERICK_ASSERT(num_path_constr == numberOfPathConstraints(i_phase),
                    "MaverickOcp::evalAtMesh: wrong number of differential point constraints.\n")
  }

  if (states_controls)
    for (integer i = 0; i < numberOfStates(i_phase) + numberOfControls(i_phase); i++)
      states_controls[i] = 0;

  if (states_controls_upper_bounds_mult)
    for (integer i = 0; i < numberOfStates(i_phase) + numberOfControls(i_phase); i++)
      states_controls_upper_bounds_mult[i] = 0;

  if (states_controls_lower_bounds_mult)
    for (integer i = 0; i < numberOfStates(i_phase) + numberOfControls(i_phase); i++)
      states_controls_lower_bounds_mult[i] = 0;

  if (algebraic_states_controls)
    for (integer i = 0; i < numberOfAlgebraicStates(i_phase) + numberOfAlgebraicControls(i_phase); i++)
      algebraic_states_controls[i] = 0;

  if (algebraic_states_controls_upper_bounds_mult)
    for (integer i = 0; i < numberOfAlgebraicStates(i_phase) + numberOfAlgebraicControls(i_phase); i++)
      algebraic_states_controls_upper_bounds_mult[i] = 0;

  if (algebraic_states_controls_lower_bounds_mult)
    for (integer i = 0; i < numberOfAlgebraicStates(i_phase) + numberOfAlgebraicControls(i_phase); i++)
      algebraic_states_controls_lower_bounds_mult[i] = 0;

  if (fo_eqns_mult)
    for (integer i = 0; i < numberOfFirstOrderEquations(i_phase); i++)
      fo_eqns_mult[i] = 0;

  if (point_constr_mult)
    for (integer i = 0; i < numberOfPointConstraints(i_phase); i++)
      point_constr_mult[i] = 0;

  if (path_constr_mult)
    for (integer i = 0; i < numberOfPathConstraints(i_phase); i++)
      path_constr_mult[i] = 0;

}

void MaverickOcp::eval(integer const i_phase,
                       real const initial_zeta, real const final_zeta,

                       integer const num_parameters, real *parameters, real *params_upper_bounds_mult,
                       real *params_lower_bounds_mult,
                       integer const num_boundary_conditions, real *boundary_conditions_mult,
                       integer const num_int_constr, real *int_constr_mult
) const {
  // consistency check
  if ((parameters) || (params_upper_bounds_mult) || (params_lower_bounds_mult)) {
    MAVERICK_ASSERT(num_parameters == numberOfParameters(i_phase), "MaverickOcp::eval: wrong number of parameters.\n")
  }

  if (boundary_conditions_mult) {
    MAVERICK_ASSERT(num_boundary_conditions == numberOfBoundaryConditions(i_phase),
                    "MaverickOcp::evalAtMesh: wrong number of boundary conditions.\n")
  }

  if (int_constr_mult) {
    MAVERICK_ASSERT(num_int_constr == numberOfIntConstraints(i_phase),
                    "MaverickOcp::evalAtMesh: wrong number of integral constraints.\n")
  }

  if (parameters)
    for (integer i = 0; i < numberOfParameters(i_phase); i++)
      parameters[i] = 0;

  if (params_upper_bounds_mult)
    for (integer i = 0; i < numberOfParameters(i_phase); i++)
      params_upper_bounds_mult[i] = 0;

  if (params_lower_bounds_mult)
    for (integer i = 0; i < numberOfParameters(i_phase); i++)
      params_lower_bounds_mult[i] = 0;

  if (boundary_conditions_mult)
    for (integer i = 0; i < numberOfBoundaryConditions(i_phase); i++)
      boundary_conditions_mult[i] = 0;

  if (int_constr_mult)
    for (integer i = 0; i < numberOfIntConstraints(i_phase); i++)
      int_constr_mult[i] = 0;
}


void MaverickOcp::printInfo(std::ostream &out, InfoLevel info_level) const {
  MAVERICK_ASSERT(_model_params, "MaverickOcp::printInfo: called when the pointer to the parameters is still nullptr\n")

  if (info_level >= info_level_more) {
    size_t max_length = 0;
    for (std::vector<string>::const_iterator it = _model_params_names.begin(); it != _model_params_names.end(); it++)
      max_length = std::max(max_length, it->length());

    max_length++;
    for (size_t i = 0; i < _model_params_names.size(); i++) {
      out << std::setw(max_length) << _model_params_names[i] << "   ";
      if (_model_params[i] < 0)
        out << std::scientific << _model_params[i];
      else
        out << " " << std::scientific << _model_params[i];
      out << "\n";

    }

    printDerivedInfo(out, info_level);
  }
}

void MaverickOcp::writeSolutionToFile(OcpSolution const &solution, std::string const &filename) const {
  MAVERICK_ASSERT(solution.getNumberOfPhases() == numberOfPhases(),
                  "MaverickOcp::writeSolutionToFile: solution provided has a wrong number of phases\n");

  for (integer i_phase = 0; i_phase < solution.getNumberOfPhases(); i_phase++) {

    std::ofstream out;
    string current_filename;
    if (numberOfPhases() == 1)
      current_filename = filename;
    else
      current_filename = filename + ".phase" + std::to_string(i_phase);

    out.open(current_filename, ios::out);

    OcpSolutionSinglePhase const &current_sol = solution[i_phase];

    //write mesh variables to the ouptut
    //zeta, center_zeta, states, controls
    stringstream body;
    stringstream header;
    current_sol.writeMeshVarsToStream(body, header);

    //translate the header and write it to output
    translateSolutionHeader(header, i_phase, out);
    //write the body
    out << body.str();

    out.close();

  }
}

void MaverickOcp::translateSolutionHeader(std::stringstream const &input_header, integer const i_phase,
                                          std::ostream &output) const {
  string header = input_header.str();

  //replace the generic state names with the problem one
  for (integer i = 0; i < numberOfStates(i_phase); i++) {
    stringstream ss;
    ss << "state_control" << i;
    stringReplace(header, ss.str(), stateName(i_phase, i));

    ss.str(string());
    ss << "lambda_states_controls_upper" << i;
    stringReplace(header, ss.str(), "lambda_" + stateName(i_phase, i) + "_upper");

    ss.str(string());
    ss << "lambda_states_controls_lower" << i;
    stringReplace(header, ss.str(), "lambda_" + stateName(i_phase, i) + "_lower");
  }

  //replace the generic control names with the problem one
  {
    int const ns = numberOfStates(i_phase);
    for (integer i = 0; i < numberOfControls(i_phase); i++) {
      stringstream ss;
      ss << "state_control" << i + ns;
      stringReplace(header, ss.str(), controlName(i_phase, i));

      ss.str(string());
      ss << "lambda_states_controls_upper" << i + ns;
      stringReplace(header, ss.str(), "lambda_" + controlName(i_phase, i) + "_upper");

      ss.str(string());
      ss << "lambda_states_controls_lower" << i + ns;
      stringReplace(header, ss.str(), "lambda_" + controlName(i_phase, i) + "_lower");
    }
  }

  //replace the generic algebraic state names with the problem one
  for (integer i = 0; i < numberOfAlgebraicStates(i_phase); i++) {
    stringstream ss;
    ss << "algebraic_state_control" << i;
    stringReplace(header, ss.str(), algebraicStateName(i_phase, i));

    ss.str(string());
    ss << "lambda_algebraic_states_controls_upper" << i;
    stringReplace(header, ss.str(), "lambda_" + algebraicStateName(i_phase, i) + "_upper");

    ss.str(string());
    ss << "lambda_algebraic_states_controls_lower" << i;
    stringReplace(header, ss.str(), "lambda_" + algebraicStateName(i_phase, i) + "_lower");
  }

  //replace the generic algebraic control names with the problem one
  {
    int const ns = numberOfAlgebraicStates(i_phase);
    for (integer i = 0; i < numberOfAlgebraicControls(i_phase); i++) {
      stringstream ss;
      ss << "algebraic_state_control" << i + ns;
      stringReplace(header, ss.str(), algebraicControlName(i_phase, i));

      ss.str(string());
      ss << "lambda_algebraic_states_controls_upper" << i + ns;
      stringReplace(header, ss.str(), "lambda_" + algebraicControlName(i_phase, i) + "_upper");

      ss.str(string());
      ss << "lambda_algebraic_states_controls_lower" << i + ns;
      stringReplace(header, ss.str(), "lambda_" + algebraicControlName(i_phase, i) + "_lower");
    }
  }

  //replace the generic point constraints multiplier name
  for (integer i = 0; i < numberOfPointConstraints(i_phase); i++) {
    stringstream ss;
    ss << "lambda_point_constraints" << i;
    stringReplace(header, ss.str(), "lambda_" + pointConstraintName(i_phase, i));
  }

  //replace the generic path constraints multiplier name
  for (integer i = 0; i < numberOfPathConstraints(i_phase); i++) {
    stringstream ss;
    ss << "lambda_path_constraints" << i;
    stringReplace(header, ss.str(), "lambda_" + pathConstraintName(i_phase, i));
  }

  //replace the generic parameter names with the problem one
  for (integer i = 0; i < numberOfParameters(i_phase); i++) {
    stringstream ss;
    ss << "parameter" << i;
    stringReplace(header, ss.str(), parameterName(i_phase, i));
  }

  //replace the generic post processing names with the problem one
  for (integer i = 0; i < numberOfPostProcessing(i_phase); i++) {
    stringstream ss;
    ss << "post_processing" << i;
    stringReplace(header, ss.str(), postProcessingName(i_phase, i));
  }

  //replace the generic differential post processing names with the problem one
  for (integer i = 0; i < numberOfDifferentialPostProcessing(i_phase); i++) {
    stringstream ss;
    ss << "differential_post_processing" << i;
    stringReplace(header, ss.str(), differentialPostProcessingName(i_phase, i));
  }

  //replace the generic integral post processing names with the problem one
  for (integer i = 0; i < numberOfIntegralPostProcessing(i_phase); i++) {
    stringstream ss;
    ss << "integral_post_processing" << i;
    stringReplace(header, ss.str(), integralPostProcessingName(i_phase, i));
  }

  output << header;
}

void MaverickOcp::writeMeshHistoryToFile(OcpSolverOutput const &solver_output, std::string const &filename) const {

  vec_3d_real const &mesh_history = solver_output.getMeshHistory();

  vec_3d_real working_mesh_history = solver_output.getMeshHistory();

  // find the max length
  size_t max_length = 0;
  for (vec_3d_real::iterator it = working_mesh_history.begin(); it != working_mesh_history.end(); it++) {
    for (vec_2d_real::iterator it2 = it->begin(); it2 != it->end(); it2++) {
      max_length = std::max(max_length, it2->size());
    }
  }

  // reshape vectors to have full length
  for (vec_3d_real::iterator it = working_mesh_history.begin(); it != working_mesh_history.end(); it++) {
    for (vec_2d_real::iterator it2 = it->begin(); it2 != it->end(); it2++) {
      real to_add = it2->back();
      it2->reserve(max_length);
      for (size_t i = it2->size(); i < max_length; i++) {
        it2->push_back(to_add);
      }

    }
  }

  // write the output to a stream
  ofstream ss;
  ss.open(filename, ios::out);

  //first, the header
  for (size_t i_run = 0; i_run < working_mesh_history.size(); i_run++) {
    for (integer i_phase = 0; i_phase < numberOfPhases(); i_phase++) {
      ss << "phase" << i_phase << "_it" << i_run;
      if (i_phase < (numberOfPhases() - 1)) {
        ss << StreamChars::separator;
      }
    }
    if (i_run < (working_mesh_history.size() - 1)) {
      ss << StreamChars::separator;
    }
  }
  ss << StreamChars::new_line;

  //then, the length of each mesh
  for (size_t i_run = 0; i_run < working_mesh_history.size(); i_run++) {
    for (integer i_phase = 0; i_phase < numberOfPhases(); i_phase++) {
      ss << mesh_history[i_run][i_phase].size();
      if (i_phase < (numberOfPhases() - 1)) {
        ss << StreamChars::separator;
      }
    }
    if (i_run < (mesh_history.size() - 1)) {
      ss << StreamChars::separator;
    }
  }
  ss << StreamChars::new_line;

  //finally, the zeta
  for (size_t i_zeta = 0; i_zeta < max_length; i_zeta++) {
    for (size_t i_run = 0; i_run < working_mesh_history.size(); i_run++) {
      for (integer i_phase = 0; i_phase < numberOfPhases(); i_phase++) {
        ss << std::scientific << working_mesh_history[i_run][i_phase][i_zeta];
        if (i_phase < (numberOfPhases() - 1)) {
          ss << StreamChars::separator;
        }
      }
      if (i_run < (working_mesh_history.size() - 1)) {
        ss << StreamChars::separator;
      }
    }
    ss << StreamChars::new_line;
  }

  ss.close();
}
