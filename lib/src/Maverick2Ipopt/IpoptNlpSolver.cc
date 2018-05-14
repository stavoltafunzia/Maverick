#include "IpoptNlpSolver.hh"
#include "MaverickCore/MaverickFunctions.hh"

using namespace Maverick;
using namespace std;
using namespace Ipopt;

IpoptNlpSolver::IpoptNlpSolver() : NlpSolver() {}

IpoptNlpSolver::IpoptNlpSolver(std::shared_ptr<const Ocp2Nlp> ocp_2_nlp) : NlpSolver(ocp_2_nlp) {
}

IpoptNlpSolver::~IpoptNlpSolver() {}

void IpoptNlpSolver::setOcp2NlpDerived(std::shared_ptr<const Ocp2Nlp> ocp_2_nlp) {}

void IpoptNlpSolver::setup(GC::GenericContainer const &gc_ipopt) {

  try {
    GC::GenericContainer const &gc_options = gc_ipopt("IpoptOptions");
    GC::map_type const &options = gc_options.get_map();
    for (GC::map_type::const_iterator it = options.begin(); it != options.end(); ++it) {
      integer int_value;
      if (findIntFromGenericContainer(gc_options, it->first, int_value)) {
        _ipopt_app.Options()->SetIntegerValue(it->first, int_value);
      } else {
        real real_value;
        if (findRealFromGenericContainer(gc_options, it->first, real_value)) {
          _ipopt_app.Options()->SetNumericValue(it->first, real_value);
        } else {
          try {
            string value = gc_options(it->first).get_string();
            if ((value.compare("max_iter") != 0) && (value.compare("warm_start_init_point") != 0))
              _ipopt_app.Options()->SetStringValue(it->first, value);
          } catch (...) {}
        }
      }
    }
  } catch (...) {}
}

IpoptNlpSolver::Output IpoptNlpSolver::solve(SolvingOptions const &options) {
  //set the start mode

  if ((options.initialize_multipliers_form_guess)) {
    _ipopt_app.Options()->SetStringValue("warm_start_init_point", "yes");
  } else {
    _ipopt_app.Options()->SetStringValue("warm_start_init_point", "no");
  }

  //set max iterations
  _ipopt_app.Options()->SetIntegerValue("max_iter", options.max_iterations);

  //supress the standard Ipopt message
  _ipopt_app.Options()->SetStringValue("sb", "yes");

  //set the guess pointer
  // use a smart ptr because Ipopt requires it
  Nlp2Ipopt *tmp = new Nlp2Ipopt(*(options.guess), *_p_ocp_2_nlp);
  Ipopt::SmartPtr <Nlp2Ipopt> nlp_2_ipopt(tmp);

  // start ipopt
  ApplicationReturnStatus status;
  status = _ipopt_app.Initialize();

  Output out;
  if (status != Solve_Succeeded) {
    out.return_status = Nlp2Ipopt::convertIpoptExitStatusToMaverick(status);
    out.return_status_message = Nlp2Ipopt::convertIpoptExitStatusToString(status);
    return out;
  }

  status = _ipopt_app.OptimizeTNLP(nlp_2_ipopt);
  out.return_status = Nlp2Ipopt::convertIpoptExitStatusToMaverick(status);
  out.return_status_message = Nlp2Ipopt::convertIpoptExitStatusToString(status);

  bool save_iter_and_target = false;

  if ((out.return_status == converged_optimal_solution) ||
      (out.return_status == converged_accetable_level) ||
      (out.return_status == infeasable_problem_detected) ||
      (out.return_status == number_of_iterations_exceeded) ||
      (out.return_status == not_converged)) {
    save_iter_and_target = true;
  }

  if (save_iter_and_target) {
    // Retrieve some statistics about the solve
    Index iter_count = _ipopt_app.Statistics()->IterationCount();
//        Number final_obj = _ipopt_app.Statistics()->FinalObjective();
    out.num_iterations = iter_count;
  }

  out.solution = make_shared<Nlp>(nlp_2_ipopt->getSolution());

  return out;
}

std::string IpoptNlpSolver::getNlpSolverType() const {
  return "Ipopt";
};
