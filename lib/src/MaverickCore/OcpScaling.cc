#include "OcpScaling.hh"
#include "MaverickFunctions.hh"
#include "MaverickCore/MaverickSingleton.hh"
#include "MaverickCore/MaverickOcp.hh"
#include "MaverickCore/MaverickPrivateDefinitions.hh"
#include <iomanip>

using namespace Maverick;
using namespace std;

OcpScaling::OcpScaling() {}

OcpScalingOptions::OcpScalingOptions() {
  multiply_lagrange_by_n = false;
  multiply_int_constr_by_n = false;
  multiply_foeqns_by_dz = false;
  multiply_foeqns_by_n = false;
  divide_foeqns_by_z = false;
  multiply_path_constr_by_dz = false;
  multiply_path_constr_by_n = false;
  divide_path_constr_by_z = false;
  multiply_point_constr_by_dz = false;
  multiply_point_constr_by_n = false;
  divide_point_constr_by_z = false;
  divide_mayer_by_n = false;
  divide_bcs_by_n = false;

  // suggested alternative scaling (can be specified at runtime)
  /*
      bool multiply_lagrange_by_n      = false;
      bool multiply_int_constr_by_n    = false;
      bool multiply_foeqns_by_dz       = true;
      bool multiply_foeqns_by_n        = true;
      bool divide_foeqns_by_z          = true;
      bool multiply_path_constr_by_dz  = true;
      bool multiply_path_constr_by_n   = true;
      bool divide_path_constr_by_z     = true;
      bool multiply_point_constr_by_dz = true;
      bool multiply_point_constr_by_n  = true;
      bool divide_point_constr_by_z    = true;
      bool divide_mayer_by_n           = false;
      bool divide_bcs_by_n             = false;
  */
}

OcpScaling::OcpScaling(OcpScaling const &ocp_scaling) {
  _states_control = ocp_scaling._states_control;
  _algebraic_states_controls = ocp_scaling._algebraic_states_controls;
  _params = ocp_scaling._params;
  _target = ocp_scaling._target;
  _fo_equations = ocp_scaling._fo_equations;
  _point_constraints = ocp_scaling._point_constraints;
  _path_constraints = ocp_scaling._path_constraints;
  _int_constraints = ocp_scaling._int_constraints;
  _boundary_conditions = ocp_scaling._boundary_conditions;
  _event_constraints = ocp_scaling._event_constraints;
  _options = ocp_scaling._options;
}

OcpScaling const &OcpScaling::operator=(OcpScaling const &ocp_scaling) {
  _states_control = ocp_scaling._states_control;
  _algebraic_states_controls = ocp_scaling._algebraic_states_controls;
  _params = ocp_scaling._params;
  _target = ocp_scaling._target;
  _fo_equations = ocp_scaling._fo_equations;
  _point_constraints = ocp_scaling._point_constraints;
  _path_constraints = ocp_scaling._path_constraints;
  _int_constraints = ocp_scaling._int_constraints;
  _boundary_conditions = ocp_scaling._boundary_conditions;
  _event_constraints = ocp_scaling._event_constraints;
  _options = ocp_scaling._options;

  return *this;
}

void OcpScaling::clear() {
  _states_control = {};
  _algebraic_states_controls = {};
  _params = {};
  _target = {};
  _fo_equations = {};
  _point_constraints = {};
  _path_constraints = {};
  _int_constraints = {};
  _boundary_conditions = {};
  _event_constraints = {};
  _options = OcpScalingOptions();
}

void OcpScaling::setupForOcp(GC::GenericContainer const &gc, MaverickOcp const &ocp_problem, vec_2d_real const &zeta) {

  clear();

  // check if all or none phase are declared
  bool is_one_phase_declared;
  bool are_all_phases_declared;
  vector<integer> missing_phases;
  chechForAllPhasesDeclarationInGc(gc, ocp_problem.numberOfPhases(), are_all_phases_declared, is_one_phase_declared,
                                   missing_phases);

  if ((!is_one_phase_declared) &&
      (ocp_problem.numberOfPhases() == 1)) { // only in this case try to check if it is declared as root value
    setupForOcpAndOnePhase(gc, ocp_problem, zeta);
  } else {

    if (!are_all_phases_declared) {
      std::stringstream err_mess;
      err_mess << "Maverick ocp scaling: scaling is not declared for all phases. Missing phases are: ";
      for (integer i = 0; i < missing_phases.size(); i++)
        err_mess << missing_phases[i] << " ";
      err_mess << "\n";
      throw runtime_error(err_mess.str());
    }

    for (integer i_phase = 0; i_phase < ocp_problem.numberOfPhases(); i_phase++) {
      setupForOcpAndOnePhase(gc("Phase" + std::to_string(i_phase)), ocp_problem, zeta);
    }
  }

  setupOptions(gc["options"]);

}

void OcpScaling::setupForOcpAsNoScaling(MaverickOcp const &ocp_problem) {
  clear();

  integer const to_push = 1;

  for (integer i_phase = 0; i_phase < ocp_problem.numberOfPhases(); i_phase++) {
    _states_control.push_back(
        vec_1d_real(ocp_problem.numberOfStates(i_phase) + ocp_problem.numberOfControls(i_phase), to_push));
    _algebraic_states_controls.push_back(
        vec_1d_real(ocp_problem.numberOfAlgebraicStates(i_phase) + ocp_problem.numberOfAlgebraicControls(i_phase),
                    to_push));
    _params.push_back(vec_1d_real(ocp_problem.numberOfParameters(i_phase), to_push));
    _target.push_back(to_push);
    _fo_equations.push_back(vec_1d_real(ocp_problem.numberOfFirstOrderEquations(i_phase), to_push));
    _point_constraints.push_back(vec_1d_real(ocp_problem.numberOfPointConstraints(i_phase), to_push));
    _path_constraints.push_back(vec_1d_real(ocp_problem.numberOfPathConstraints(i_phase), to_push));
    _int_constraints.push_back(vec_1d_real(ocp_problem.numberOfIntConstraints(i_phase), to_push));
    _boundary_conditions.push_back(vec_1d_real(ocp_problem.numberOfBoundaryConditions(i_phase), to_push));
    _event_constraints.push_back(vec_1d_real(ocp_problem.numberOfEventConstraints(i_phase), to_push));
  }
}

void OcpScaling::writeScalingsToStream(std::ostream &out) const {

  for (integer i_phase = 0; i_phase < _states_control.size(); i_phase++) {
    out << "Phase" << i_phase << "\n";

    out << "\ttarget: " << _target[i_phase] << "\n";

    if (_states_control[i_phase].size() > 0) {
      out << "\tstates and controls: ";
      for (vec_1d_real::const_iterator it = _states_control[i_phase].begin();
           it != _states_control[i_phase].end(); it++) {
        out << *it << ", ";
      }
      out << "\n";
    }

    if (_algebraic_states_controls[i_phase].size() > 0) {
      out << "\talgebraic states and controls: ";
      for (vec_1d_real::const_iterator it = _algebraic_states_controls[i_phase].begin();
           it != _algebraic_states_controls[i_phase].end(); it++) {
        out << *it << ", ";
      }
      out << "\n";
    }

    if (_params[i_phase].size() > 0) {
      out << "\tparameters: ";
      for (vec_1d_real::const_iterator it = _params[i_phase].begin(); it != _params[i_phase].end(); it++) {
        out << *it << ", ";
      }
      out << "\n";
    }

    if (_fo_equations[i_phase].size() > 0) {
      out << "\tfirst order equations: ";
      for (vec_1d_real::const_iterator it = _fo_equations[i_phase].begin(); it != _fo_equations[i_phase].end(); it++) {
        out << *it << ", ";
      }
      out << "\n";
    }

    if (_point_constraints[i_phase].size() > 0) {
      out << "\tpoint constraints: ";
      for (vec_1d_real::const_iterator it = _point_constraints[i_phase].begin();
           it != _point_constraints[i_phase].end(); it++) {
        out << *it << ", ";
      }
      out << "\n";
    }

    if (_path_constraints[i_phase].size() > 0) {
      out << "\tpath constraints: ";
      for (vec_1d_real::const_iterator it = _path_constraints[i_phase].begin();
           it != _path_constraints[i_phase].end(); it++) {
        out << *it << ", ";
      }
      out << "\n";
    }

    if (_int_constraints[i_phase].size() > 0) {
      out << "\tintegral constraints: ";
      for (vec_1d_real::const_iterator it = _int_constraints[i_phase].begin();
           it != _int_constraints[i_phase].end(); it++) {
        out << *it << ", ";
      }
      out << "\n";
    }

    if (_boundary_conditions[i_phase].size() > 0) {
      out << "\tboundary_conditions: ";
      for (vec_1d_real::const_iterator it = _boundary_conditions[i_phase].begin();
           it != _boundary_conditions[i_phase].end(); it++) {
        out << *it << ", ";
      }
      out << "\n";
    }

    if (_event_constraints[i_phase].size() > 0) {
      out << "\tevent_constraints: ";
      for (vec_1d_real::const_iterator it = _event_constraints[i_phase].begin();
           it != _event_constraints[i_phase].end(); it++) {
        out << *it << ", ";
      }
      out << "\n";
    }
    if (i_phase < (_states_control.size() - 1))
      out << "\n";
  }
  // output scaling options
  out << "scaling options:\n";
  char const spacer = '.';
  integer const width = 35;

  out << "\t" << std::left << std::setfill(spacer) << std::setw(width) << "multiply_lagrange_by_n"
      << (_options.multiply_lagrange_by_n ? "true" : "false") << "\n";
  out << "\t" << std::left << std::setfill(spacer) << std::setw(width) << "multiply_int_constr_by_n"
      << (_options.multiply_int_constr_by_n ? "true" : "false") << "\n";
  out << "\t" << std::left << std::setfill(spacer) << std::setw(width) << "multiply_foeqns_by_dz"
      << (_options.multiply_foeqns_by_dz ? "true" : "false") << "\n";
  out << "\t" << std::left << std::setfill(spacer) << std::setw(width) << "multiply_foeqns_by_n"
      << (_options.multiply_foeqns_by_n ? "true" : "false") << "\n";
  out << "\t" << std::left << std::setfill(spacer) << std::setw(width) << "divide_foeqns_by_z"
      << (_options.divide_foeqns_by_z ? "true" : "false") << "\n";
  out << "\t" << std::left << std::setfill(spacer) << std::setw(width) << "multiply_path_constr_by_dz"
      << (_options.multiply_path_constr_by_dz ? "true" : "false") << "\n";
  out << "\t" << std::left << std::setfill(spacer) << std::setw(width) << "multiply_path_constr_by_n"
      << (_options.multiply_path_constr_by_n ? "true" : "false") << "\n";
  out << "\t" << std::left << std::setfill(spacer) << std::setw(width) << "divide_path_constr_by_z"
      << (_options.divide_path_constr_by_z ? "true" : "false") << "\n";
  out << "\t" << std::left << std::setfill(spacer) << std::setw(width) << "multiply_point_constr_by_dz"
      << (_options.multiply_point_constr_by_dz ? "true" : "false") << "\n";
  out << "\t" << std::left << std::setfill(spacer) << std::setw(width) << "multiply_point_constr_by_n"
      << (_options.multiply_point_constr_by_n ? "true" : "false") << "\n";
  out << "\t" << std::left << std::setfill(spacer) << std::setw(width) << "divide_point_constr_by_z"
      << (_options.divide_point_constr_by_z ? "true" : "false") << "\n";
  out << "\t" << std::left << std::setfill(spacer) << std::setw(width) << "divide_mayer_by_n"
      << (_options.divide_mayer_by_n ? "true" : "false") << "\n";
  out << "\t" << std::left << std::setfill(spacer) << std::setw(width) << "divide_bcs_by_n"
      << (_options.divide_bcs_by_n ? "true" : "false") << "\n";

  out << "\n";
}

OcpScalingOptions const &OcpScaling::getScalingOptions() const {
  return _options;
}

void OcpScaling::setupForOcpAndOnePhase(GC::GenericContainer const &gc, MaverickOcp const &ocp_problem,
                                        vec_2d_real const &all_zeta) {

  integer i_phase = (integer) _states_control.size();

  string not_found = "";
  //bool something_not_found = false;
  bool mesh_warning = false;

  string option_not_rec = "Ocp scaling option xxx for phase " + std::to_string(i_phase) +
                          " is not recognised. Possible options are: 'automatic-bounds'\n";

  {
    vec_1d_real states_controls;
    bool found = findVecRealFromGenericContainer(gc, "states_and_controls", states_controls);
    if (found) {
      integer read = (integer) states_controls.size();
      integer exp = (integer) ocp_problem.numberOfStates(i_phase) + (integer) ocp_problem.numberOfControls(i_phase);
      if (read != exp) {
        throw runtime_error(
            "Maverick ocp scaling: wrong states and controls scaling size at phase " + std::to_string(i_phase)
            + ". Found " + std::to_string(read) + " elements, expected " + std::to_string(exp) + "\n");
      }
      for (integer i = 0; i < ocp_problem.numberOfStates(i_phase); i++) {
        MAVERICK_ASSERT(states_controls[i] > 0, "State scaling for state at index " + std::to_string(i) + " at phase " +
                                                std::to_string(i_phase) + " cannot be non positive");
      }
      for (integer i = 0; i < ocp_problem.numberOfControls(i_phase); i++) {
        MAVERICK_ASSERT(states_controls[i + ocp_problem.numberOfStates(i_phase)] > 0,
                        "Scaling for control at index " + std::to_string(i) + " at phase " + std::to_string(i_phase) +
                        " cannot be non positive");
      }
      _states_control.push_back(states_controls);
    } else {
      string str;
      found = gc.get_if_exists("states_and_controls", str);
      if (found) {
        found = checkIfStringMatchesAutomaticBounds(str);
        if (found) {

          integer const nnz = ocp_problem.numberOfStates(i_phase) + ocp_problem.numberOfControls(i_phase);
          real lower_bounds[nnz];
          real upper_bounds[nnz];
          states_controls = vec_1d_real(nnz, 0);

          vec_1d_real zeta;
          if (all_zeta.size() > i_phase) {
            zeta = all_zeta[i_phase];
          } else {
            zeta = {0};
            mesh_warning = true;
          }

          for (integer i_mesh = 0; i_mesh < zeta.size(); i_mesh++) {
            ocp_problem.getStatesControlsBounds(i_phase, zeta[i_mesh], lower_bounds, upper_bounds);

            for (integer i = 0; i < nnz; i++) {
              real scale = upper_bounds[i] - lower_bounds[i];
              states_controls[i] = max(states_controls[i], scale);
            }
          }
          for (integer i = 0; i < nnz; i++)
            if (states_controls[i] <= 0)
              states_controls[i] = 1;

          _states_control.push_back(states_controls);
        } else {
          stringReplace(option_not_rec, "xxx", str);
          throw runtime_error(option_not_rec);
        }
      } else {
        _states_control.push_back(
            vec_1d_real(ocp_problem.numberOfStates(i_phase) + ocp_problem.numberOfControls(i_phase), 1));
      }
    }
  }

  {
    vec_1d_real algebraic_states_controls;
    bool found = findVecRealFromGenericContainer(gc, "algebraic_states_and_controls", algebraic_states_controls);
    if (found) {
      integer read = (integer) algebraic_states_controls.size();
      integer exp = (integer) ocp_problem.numberOfAlgebraicStates(i_phase) +
                    (integer) ocp_problem.numberOfAlgebraicControls(i_phase);
      if (read != exp) {
        throw runtime_error("Maverick ocp scaling: wrong algebraic states and controls scaling size at phase " +
                            std::to_string(i_phase)
                            + ". Found " + std::to_string(read) + " elements, expected " + std::to_string(exp) + "\n");
      }
      for (integer i = 0; i < ocp_problem.numberOfAlgebraicStates(i_phase); i++) {
        MAVERICK_ASSERT(algebraic_states_controls[i] > 0,
                        "Algebraic state scaling for state at index " + std::to_string(i) + " at phase " +
                        std::to_string(i_phase) + " cannot be non positive");
      }
      for (integer i = 0; i < ocp_problem.numberOfAlgebraicControls(i_phase); i++) {
        MAVERICK_ASSERT(algebraic_states_controls[i + ocp_problem.numberOfAlgebraicStates(i_phase)] > 0,
                        "Scaling for algebraic control at index " + std::to_string(i) + " at phase " +
                        std::to_string(i_phase) + " cannot be non positive");
      }
      _algebraic_states_controls.push_back(algebraic_states_controls);
    } else {
      string str;
      found = gc.get_if_exists("algebraic_states_and_controls", str);
      if (found) {
        found = checkIfStringMatchesAutomaticBounds(str);
        if (found) {
          integer const nnz =
              ocp_problem.numberOfAlgebraicStates(i_phase) + ocp_problem.numberOfAlgebraicControls(i_phase);
          real lower_bounds[nnz];
          real upper_bounds[nnz];
          algebraic_states_controls = vec_1d_real(nnz, 0);

          vec_1d_real zeta;
          if (all_zeta.size() > i_phase) {
            zeta = all_zeta[i_phase];
          } else {
            zeta = {0};
            mesh_warning = true;
          }

          for (integer i_mesh = 0; i_mesh < zeta.size(); i_mesh++) {
            ocp_problem.getAlgebraicStatesControlsBounds(i_phase, zeta[i_mesh], lower_bounds, upper_bounds);

            for (integer i = 0; i < nnz; i++) {
              real scale = upper_bounds[i] - lower_bounds[i];
              algebraic_states_controls[i] = max(algebraic_states_controls[i], scale);
            }
          }
          for (integer i = 0; i < nnz; i++)
            if (algebraic_states_controls[i] <= 0)
              algebraic_states_controls[i] = 1;

          _algebraic_states_controls.push_back(algebraic_states_controls);
        } else {
          stringReplace(option_not_rec, "xxx", str);
          throw runtime_error(option_not_rec);
        }
      } else {
        _algebraic_states_controls.push_back(vec_1d_real(
            ocp_problem.numberOfAlgebraicStates(i_phase) + ocp_problem.numberOfAlgebraicControls(i_phase), 1));
      }
    }
  }

  if (ocp_problem.numberOfParameters(i_phase) > 0) {
    vec_1d_real params;
    bool found = findVecRealFromGenericContainer(gc, "parameters", params);
    if (found) {
      integer read = (integer) params.size();
      integer exp = (integer) ocp_problem.numberOfParameters(i_phase);
      if (read != exp) {
        throw runtime_error("Maverick ocp scaling: wrong parameters scaling size at phase " + std::to_string(i_phase)
                            + ". Found " + std::to_string(read) + " elements, expected " + std::to_string(exp) + "\n");
      }
      for (integer i = 0; i < params.size(); i++) {
        MAVERICK_ASSERT(params[i] > 0,
                        "Scaling for parameter at index " + std::to_string(i) + " at phase " + std::to_string(i_phase) +
                        " cannot be non positive");
      }
      _params.push_back(params);
    } else {
      string str;
      found = gc.get_if_exists("parameters", str);
      if (found) {
        found = checkIfStringMatchesAutomaticBounds(str);
        if (found) {
          integer const nnz = ocp_problem.numberOfParameters(i_phase);
          real lower_bounds[nnz];
          real upper_bounds[nnz];
          ocp_problem.getParametersBounds(i_phase, lower_bounds, upper_bounds);

          params = {};
          params.reserve(nnz);
          for (integer i = 0; i < nnz; i++) {
            real scale = upper_bounds[i] - lower_bounds[i];
            if (scale <= 0)
              scale = 1;
            params.push_back(scale);
          }
          _params.push_back(params);
        } else {
          stringReplace(option_not_rec, "xxx", str);
          throw runtime_error(option_not_rec);
        }
      } else {
        _params.push_back(vec_1d_real(ocp_problem.numberOfParameters(i_phase), 1));
      }
    }
  } else {
    _params.push_back({});
  }

  {
    real tgt;
    bool found = findRealFromGenericContainer(gc, "target", tgt);
    if (found) {
      MAVERICK_ASSERT(tgt > 0, "Target scaling for phase " + std::to_string(i_phase) + " cannot be non positive");
      _target.push_back(tgt);
    } else {
      string str;
      found = gc.get_if_exists("target", str);
      if (found) {
        found = checkIfStringMatchesAutomaticBounds(str);
        if (found) {
          str = "Ocp scaling of type 'automatic-bounds' for mayer target for phase " + std::to_string(i_phase) +
                " is not allowed. Please specify the scaling manually, or do not specify it to use ones\n";
          throw runtime_error(str);
        } else {
          stringReplace(option_not_rec, "xxx", str);
          throw runtime_error(option_not_rec);
        }
      } else {
        _target.push_back(1);
      }
    }
  }

  {
    vec_1d_real fo_eqns;
    bool found = findVecRealFromGenericContainer(gc, "fo_equations", fo_eqns);
    if (found) {
      integer read = (integer) fo_eqns.size();
      integer exp = (integer) ocp_problem.numberOfFirstOrderEquations(i_phase);
      if (read != exp) {
        throw runtime_error(
            "Maverick ocp scaling: wrong f.o. equations scaling size at phase " + std::to_string(i_phase)
            + ". Found " + std::to_string(read) + " elements, expected " + std::to_string(exp) + "\n");
      }
      for (integer i = 0; i < fo_eqns.size(); i++) {
        MAVERICK_ASSERT(fo_eqns[i] > 0,
                        "Scaling for equation at index " + std::to_string(i) + " at phase " + std::to_string(i_phase) +
                        " cannot be non positive");
      }
      _fo_equations.push_back(fo_eqns);
    } else {
      string str;
      found = gc.get_if_exists("fo_equations", str);
      if (found) {
        found = checkIfStringMatchesAutomaticBounds(str);
        if (found) {
          str = "Ocp scaling of type 'automatic-bounds' for first-order-equations for phase " +
                std::to_string(i_phase) +
                " is not allowed. Please specify the scaling manually, or do not specify it to use ones\n";
          throw runtime_error(str);
        } else {
          stringReplace(option_not_rec, "xxx", str);
          throw runtime_error(option_not_rec);
        }
      } else {
        _fo_equations.push_back(vec_1d_real(ocp_problem.numberOfFirstOrderEquations(i_phase), 1));
      }
    }
  }

  if (ocp_problem.numberOfPointConstraints(i_phase) > 0) {
    vec_1d_real vec;
    bool found = findVecRealFromGenericContainer(gc, "point_constraints", vec);
    if (found) {
      integer read = (integer) vec.size();
      integer exp = (integer) ocp_problem.numberOfPointConstraints(i_phase);
      if (read != exp) {
        throw runtime_error(
            "Maverick ocp scaling: wrong point constraints scaling size at phase " + std::to_string(i_phase)
            + ". Found " + std::to_string(read) + " elements, expected " + std::to_string(exp) + "\n");
      }
      for (integer i = 0; i < vec.size(); i++) {
        MAVERICK_ASSERT(vec[i] > 0, "Scaling for point constraint at index " + std::to_string(i) + " at phase " +
                                    std::to_string(i_phase) + " cannot be non positive");
      }
      _point_constraints.push_back(vec);
    } else {
      string str;
      found = gc.get_if_exists("point_constraints", str);
      if (found) {
        found = checkIfStringMatchesAutomaticBounds(str);
        if (found) {
          integer const nnz = ocp_problem.numberOfPointConstraints(i_phase);
          real lower_bounds[nnz];
          real upper_bounds[nnz];
          vec = vec_1d_real(nnz, 0);

          vec_1d_real zeta;
          if (all_zeta.size() > i_phase) {
            zeta = all_zeta[i_phase];
          } else {
            zeta = {0};
            mesh_warning = true;
          }

          for (integer i_mesh = 0; i_mesh < zeta.size(); i_mesh++) {
            ocp_problem.getPointConstraintsBounds(i_phase, zeta[i_mesh], lower_bounds, upper_bounds);

            for (integer i = 0; i < nnz; i++) {
              real scale = upper_bounds[i] - lower_bounds[i];
              vec[i] = max(vec[i], scale);
            }
          }
          for (integer i = 0; i < nnz; i++)
            if (vec[i] <= 0)
              vec[i] = 1;

          _point_constraints.push_back(vec);
        } else {
          stringReplace(option_not_rec, "xxx", str);
          throw runtime_error(option_not_rec);
        }
      } else {
        _point_constraints.push_back(vec_1d_real(ocp_problem.numberOfPointConstraints(i_phase), 1));
      }
    }
  } else {
    _point_constraints.push_back({});
  }

  if (ocp_problem.numberOfPathConstraints(i_phase) > 0) {
    vec_1d_real vec;
    bool found = findVecRealFromGenericContainer(gc, "path_constraints", vec);
    if (found) {
      integer read = (integer) vec.size();
      integer exp = (integer) ocp_problem.numberOfPathConstraints(i_phase);
      if (read != exp) {
        throw runtime_error(
            "Maverick ocp scaling: wrong path constraints scaling size at phase " + std::to_string(i_phase)
            + ". Found " + std::to_string(read) + " elements, expected " + std::to_string(exp) + "\n");
      }
      for (integer i = 0; i < vec.size(); i++) {
        MAVERICK_ASSERT(vec[i] > 0, "Scaling for path constraint at index " + std::to_string(i) + " at phase " +
                                    std::to_string(i_phase) + " cannot be non positive");
      }
      _path_constraints.push_back(vec);
    } else {
      string str;
      found = gc.get_if_exists("path_constraints", str);
      if (found) {
        found = checkIfStringMatchesAutomaticBounds(str);
        if (found) {
          integer const nnz = ocp_problem.numberOfPathConstraints(i_phase);
          real lower_bounds[nnz];
          real upper_bounds[nnz];
          vec = vec_1d_real(nnz, 0);

          vec_1d_real zeta;
          if (all_zeta.size() > i_phase) {
            zeta = all_zeta[i_phase];
          } else {
            zeta = {0};
            mesh_warning = true;
          }

          for (integer i_mesh = 0; i_mesh < zeta.size(); i_mesh++) {
            ocp_problem.getPathConstraintsBounds(i_phase, zeta[i_mesh], lower_bounds, upper_bounds);

            for (integer i = 0; i < nnz; i++) {
              real scale = upper_bounds[i] - lower_bounds[i];
              vec[i] = max(vec[i], scale);
            }
          }
          for (integer i = 0; i < nnz; i++)
            if (vec[i] <= 0)
              vec[i] = 1;

          _path_constraints.push_back(vec);
        } else {
          stringReplace(option_not_rec, "xxx", str);
          throw runtime_error(option_not_rec);
        }
      } else {
        _path_constraints.push_back(vec_1d_real(ocp_problem.numberOfPathConstraints(i_phase), 1));
      }
    }
  } else {
    _path_constraints.push_back({});
  }

  if (ocp_problem.numberOfIntConstraints(i_phase) > 0) {
    vec_1d_real vec;
    bool found = findVecRealFromGenericContainer(gc, "integral_constraints", vec);
    if (found) {
      integer read = (integer) vec.size();
      integer exp = (integer) ocp_problem.numberOfIntConstraints(i_phase);
      if (read != exp) {
        throw runtime_error(
            "Maverick ocp scaling: wrong integral constraints scaling size at phase " + std::to_string(i_phase)
            + ". Found " + std::to_string(read) + " elements, expected " + std::to_string(exp) + "\n");
      }
      for (integer i = 0; i < vec.size(); i++) {
        MAVERICK_ASSERT(vec[i] > 0, "Scaling for integral constraint at index " + std::to_string(i) + " at phase " +
                                    std::to_string(i_phase) + " cannot be non positive");
      }
      _int_constraints.push_back(vec);
    } else {
      string str;
      found = gc.get_if_exists("integral_constraints", str);
      if (found) {
        found = checkIfStringMatchesAutomaticBounds(str);
        if (found) {
          integer const nnz = ocp_problem.numberOfIntConstraints(i_phase);
          real lower_bounds[nnz];
          real upper_bounds[nnz];

          real zeta_f = 1;
          real zeta_i = 0;
          if (all_zeta.size() > i_phase) {
            zeta_f = all_zeta[i_phase].back();
            zeta_i = all_zeta[i_phase].front();
          } else {
            mesh_warning = true;
          }
          ocp_problem.getIntConstraintsBounds(i_phase, zeta_i, zeta_f, lower_bounds, upper_bounds);

          vec = {};
          vec.reserve(nnz);
          for (integer i = 0; i < nnz; i++) {
            real scale = upper_bounds[i] - lower_bounds[i];
            if (scale <= 0)
              scale = 1;
            vec.push_back(scale);
          }
          _int_constraints.push_back(vec);
        } else {
          stringReplace(option_not_rec, "xxx", str);
          throw runtime_error(option_not_rec);
        }
      } else {
        _int_constraints.push_back(vec_1d_real(ocp_problem.numberOfIntConstraints(i_phase), 1));
      }
    }
  } else {
    _int_constraints.push_back({});
  }

  if (ocp_problem.numberOfBoundaryConditions(i_phase) > 0) {
    vec_1d_real vec;
    bool found = findVecRealFromGenericContainer(gc, "boundary_conditions", vec);
    if (found) {
      integer read = (integer) vec.size();
      integer exp = (integer) ocp_problem.numberOfBoundaryConditions(i_phase);
      if (read != exp) {
        throw runtime_error(
            "Maverick ocp scaling: wrong boundary conditions scaling size at phase " + std::to_string(i_phase)
            + ". Found " + std::to_string(read) + " elements, expected " + std::to_string(exp) + "\n");
      }
      for (integer i = 0; i < vec.size(); i++) {
        MAVERICK_ASSERT(vec[i] > 0, "Scaling for boundary condition at index " + std::to_string(i) + " at phase " +
                                    std::to_string(i_phase) + " cannot be non positive");
      }
      _boundary_conditions.push_back(vec);
    } else {
      string str;
      found = gc.get_if_exists("boundary_conditions", str);
      if (found) {
        found = checkIfStringMatchesAutomaticBounds(str);
        if (found) {
          integer const nnz = ocp_problem.numberOfBoundaryConditions(i_phase);
          real lower_bounds[nnz];
          real upper_bounds[nnz];

          real zeta_f = 1;
          real zeta_i = 0;
          if (all_zeta.size() > i_phase) {
            zeta_f = all_zeta[i_phase].back();
            zeta_i = all_zeta[i_phase].front();
          } else {
            mesh_warning = true;
          }
          ocp_problem.getBoundaryConditionsBounds(i_phase, zeta_i, zeta_f, lower_bounds, upper_bounds);

          vec = {};
          vec.reserve(nnz);
          for (integer i = 0; i < nnz; i++) {
            real scale = upper_bounds[i] - lower_bounds[i];
            if (scale <= 0)
              scale = 1;
            vec.push_back(scale);
          }
          _boundary_conditions.push_back(vec);
        } else {
          stringReplace(option_not_rec, "xxx", str);
          throw runtime_error(option_not_rec);
        }
      } else {
        _boundary_conditions.push_back(vec_1d_real(ocp_problem.numberOfBoundaryConditions(i_phase), 1));
      }
    }
  } else {
    _boundary_conditions.push_back({});
  }

  if (ocp_problem.numberOfEventConstraints(i_phase) > 0) {
    vec_1d_real vec;
    bool found = findVecRealFromGenericContainer(gc, "event_constraints", vec);
    if (found) {
      integer read = (integer) vec.size();
      integer exp = (integer) ocp_problem.numberOfEventConstraints(i_phase);
      if (read != exp) {
        throw runtime_error(
            "Maverick ocp scaling: wrong event constraints scaling size at phase " + std::to_string(i_phase)
            + ". Found " + std::to_string(read) + " elements, expected " + std::to_string(exp) + "\n");
      }
      for (integer i = 0; i < vec.size(); i++) {
        MAVERICK_ASSERT(vec[i] > 0, "Scaling for event constraint at index " + std::to_string(i) + " at phase " +
                                    std::to_string(i_phase) + " cannot be non positive");
      }
      _event_constraints.push_back(vec);
    } else {
      string str;
      found = gc.get_if_exists("event_constraints", str);
      if (found) {
        found = checkIfStringMatchesAutomaticBounds(str);
        if (found) {
          integer const nnz = ocp_problem.numberOfEventConstraints(i_phase);
          real lower_bounds[nnz];
          real upper_bounds[nnz];

          real zeta_l = 1;
          real zeta_r = 0;
          if (all_zeta.size() > i_phase + 1) {
            zeta_l = all_zeta[i_phase].back();
            zeta_r = all_zeta[i_phase + 1].front();
          } else {
            mesh_warning = true;
          }
          ocp_problem.getEventConstraintsBounds(i_phase, zeta_l, zeta_r, lower_bounds, upper_bounds);

          vec = {};
          vec.reserve(nnz);
          for (integer i = 0; i < nnz; i++) {
            real scale = upper_bounds[i] - lower_bounds[i];
            if (scale <= 0)
              scale = 1;
            vec.push_back(scale);
          }
          _event_constraints.push_back(vec);
        } else {
          stringReplace(option_not_rec, "xxx", str);
          throw runtime_error(option_not_rec);
        }
      } else {
        _event_constraints.push_back(vec_1d_real(ocp_problem.numberOfEventConstraints(i_phase), 1));
      }
    }
  } else {
    _event_constraints.push_back({});
  }

  if (mesh_warning) {
    string str = "Ocp scaling for phase " + std::to_string(i_phase) +
                 ": requested bounds-based scaling but no mesh has been set yet. Scaling for variables with mesh-dependent bounds may be inaccurate!\n";
    MaverickSingleton::getInstance().Log(InfoLevel::info_level_warning, str);
  }

}

bool OcpScaling::checkIfStringMatchesAutomaticBounds(string const &str) {
  if (str.compare("automatic-bounds") == 0)
    return true;

  return false;
}

//Maverick::real      OcpScaling::getDomainScaling(integer const i_phase) const {
//    MAVERICK_ASSERT(i_phase < _zeta.size(), "OcpScaling::getDomainScaling: phase index out of bounds.\n" );
//    return _zeta[i_phase];
//}
Maverick::real OcpScaling::getTargetScaling(integer const i_phase) const {
  MAVERICK_ASSERT(i_phase < _target.size(), "OcpScaling::getTargetScaling: phase index out of bounds.\n");
  return _target[i_phase];
}

// bool      OcpScaling::isTargetOfLagrangeType(integer const i_phase) const {
//     MAVERICK_ASSERT(i_phase < _is_tgt_lag_type.size(), "OcpScaling::isTargetOfLagrangeType: phase index out of bounds.\n" );
//     return _is_tgt_lag_type[i_phase];
// }
vec_1d_real const &OcpScaling::getStatesControlScaling(integer const i_phase) const {
  MAVERICK_ASSERT(i_phase < _states_control.size(),
                  "OcpScaling::getStatesControlScaling: phase index out of bounds.\n");
  return _states_control[i_phase];
}

vec_1d_real const &OcpScaling::getAlgebraicStatesControlScaling(integer const i_phase) const {
  MAVERICK_ASSERT(i_phase < _algebraic_states_controls.size(),
                  "OcpScaling::getAlgebraicStatesControlScaling: phase index out of bounds.\n");
  return _algebraic_states_controls[i_phase];
}

vec_1d_real const &OcpScaling::getParamsScaling(integer const i_phase) const {
  MAVERICK_ASSERT(i_phase < _params.size(), "OcpScaling::getParamsScaling: phase index out of bounds.\n");
  return _params[i_phase];
}

vec_1d_real const &OcpScaling::getFoEqnsScaling(integer const i_phase) const {
  MAVERICK_ASSERT(i_phase < _fo_equations.size(), "OcpScaling::getFoEqnsScaling: phase index out of bounds.\n");
  return _fo_equations[i_phase];
}

vec_1d_real const &OcpScaling::getPointConstraintsScaling(integer const i_phase) const {
  MAVERICK_ASSERT(i_phase < _point_constraints.size(),
                  "OcpScaling::getPointConstraintsScaling: phase index out of bounds.\n");
  return _point_constraints[i_phase];
}

vec_1d_real const &OcpScaling::getPathConstraintsScaling(integer const i_phase) const {
  MAVERICK_ASSERT(i_phase < _path_constraints.size(),
                  "OcpScaling::getPathConstraintsScaling: phase index out of bounds.\n");
  return _path_constraints[i_phase];
}

vec_1d_real const &OcpScaling::getIntConstraintsScaling(integer const i_phase) const {
  MAVERICK_ASSERT(i_phase < _int_constraints.size(),
                  "OcpScaling::getIntConstraintsScaling: phase index out of bounds.\n");
  return _int_constraints[i_phase];
}

vec_1d_real const &OcpScaling::getBoundaryConditionsScaling(integer const i_phase) const {
  MAVERICK_ASSERT(i_phase < _boundary_conditions.size(),
                  "OcpScaling::getBoundaryConditionsScaling: phase index out of bounds.\n");
  return _boundary_conditions[i_phase];
}

vec_1d_real const &OcpScaling::getEventConstraintsScaling(integer const i_phase) const {
  MAVERICK_ASSERT(i_phase < _event_constraints.size(),
                  "OcpScaling::getEventConstraintsScaling: phase index out of bounds.\n");
  return _event_constraints[i_phase];
}

void OcpScaling::setupOptions(GC::GenericContainer const &gc) {
  _options = OcpScalingOptions();

  gc.get_if_exists("multiply_lagrange_by_n", _options.multiply_lagrange_by_n);
  gc.get_if_exists("multiply_int_constr_by_n", _options.multiply_int_constr_by_n);
  gc.get_if_exists("multiply_foeqns_by_dz", _options.multiply_foeqns_by_dz);
  gc.get_if_exists("multiply_foeqns_by_n", _options.multiply_foeqns_by_n);
  gc.get_if_exists("divide_foeqns_by_z", _options.divide_foeqns_by_z);
  gc.get_if_exists("multiply_path_constr_by_dz", _options.multiply_path_constr_by_dz);
  gc.get_if_exists("multiply_path_constr_by_n", _options.multiply_path_constr_by_n);
  gc.get_if_exists("divide_path_constr_by_z", _options.divide_path_constr_by_z);
  gc.get_if_exists("multiply_point_constr_by_dz", _options.multiply_point_constr_by_dz);
  gc.get_if_exists("multiply_point_constr_by_n", _options.multiply_point_constr_by_n);
  gc.get_if_exists("divide_point_constr_by_z", _options.divide_point_constr_by_z);
  gc.get_if_exists("divide_mayer_by_n", _options.divide_mayer_by_n);
  gc.get_if_exists("divide_bcs_by_n", _options.divide_bcs_by_n);

}
