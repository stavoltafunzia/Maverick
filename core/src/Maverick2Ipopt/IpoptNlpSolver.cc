#include "IpoptNlpSolver.hh"
#include "MaverickCore/MaverickPrivateDefs.hh"
#include "MaverickCore/MaverickFunctions.hh"

using namespace Maverick;
using namespace std;
using namespace Ipopt;

IpoptNlpSolver::IpoptNlpSolver( NlpSolution & nlp_solution ) : _nlp_solution(nlp_solution) {
    _p_nlp_2_ipopt = new Nlp2Ipopt(nlp_solution);
}

IpoptNlpSolver::IpoptNlpSolver( NlpSolution & nlp_solution, std::shared_ptr<const Ocp2Nlp> ocp_2_nlp ) : _nlp_solution(nlp_solution) {
    _p_nlp_2_ipopt = new Nlp2Ipopt(nlp_solution, ocp_2_nlp);
}

IpoptNlpSolver::~IpoptNlpSolver() {}

void IpoptNlpSolver::setOcp2Nlp( std::shared_ptr<const Ocp2Nlp> ocp_2_nlp ) {
    _p_nlp_2_ipopt->setOcp2Nlp(ocp_2_nlp);
}

void IpoptNlpSolver::setup( GC::GenericContainer const & gc_ipopt ) {

    try {
        GC::GenericContainer const & gc_options = gc_ipopt("IpoptOptions");
        GC::map_type const & options  = gc_options.get_map();
        for(GC::map_type::const_iterator it = options.begin(); it != options.end(); ++it) {
            integer int_value;
            if ( findIntFromGenericContainer(gc_options, it->first, int_value) ) {
                _ipopt_app.Options()->SetIntegerValue(it->first, int_value);
            } else {
                real real_value;
                if ( findRealFromGenericContainer(gc_options, it->first, real_value) ) {
                    _ipopt_app.Options()->SetNumericValue(it->first, real_value);
                } else {
                    try {
                        string value = gc_options( it->first ).get_string();
                        if ( (value.compare("max_iter")!=0) && (value.compare("warm_start_init_point")!=0) )
                            _ipopt_app.Options()->SetStringValue(it->first, value);
                    } catch (...) {}
                }
            }
        }
    } catch (...) {}
    //    _ipopt_app.Options()->SetStringValue("linear_solver", "ma27");
    //    _ipopt_app.Options()->SetStringValue("linear_solver", "ma57");
    //    _ipopt_app.Options()->SetStringValue("linear_solver", "ma77");
    //    _ipopt_app.Options()->SetStringValue("linear_solver", "ma86");

    //    _ipopt_app.Options()->SetStringValue("linear_solver", "ma97");

    //    _ipopt_app.Options()->SetStringValue("warm_start_init_point", "yes");
    //    _ipopt_app.Options()->SetStringValue("hessian_approximation", "limited-memory");

    //    _ipopt_app.Options()->SetStringValue("derivative_test", "first-order");
    //    _ipopt_app.Options()->SetStringValue("derivative_test", "second-order");
    //    _ipopt_app.Options()->SetNumericValue("derivative_test_perturbation", 1e-8);

}

IpoptNlpSolver::IpoptSolverReturnStatus IpoptNlpSolver::solve( SolverSettings const & solver_settings ) {
    //set the start mode
//    if ( (solver_settings.start_mode == warm_start_with_multipliers) || (solver_settings.start_mode == hot_start_with_multipliers) ) {
    if ( (solver_settings.start_mode == warm_start_with_multipliers) ) {
        _ipopt_app.Options()->SetStringValue("warm_start_init_point", "yes");
    } else {
        _ipopt_app.Options()->SetStringValue("warm_start_init_point", "no");
    }

    //set max iterations
    _ipopt_app.Options()->SetIntegerValue("max_iter", solver_settings.max_iterations);

    //supress the standard Ipopt message
    _ipopt_app.Options()->SetStringValue("sb","yes");

    //set the guess pointer
    _p_nlp_2_ipopt->setNlpGuessPtr( (Nlp const *) solver_settings.nlp_guess_ptr );

    // start ipopt
    ApplicationReturnStatus status;
    status = _ipopt_app.Initialize();
    if (status != Solve_Succeeded) {
        IpoptSolverReturnStatus mav_status;
        mav_status.maverick_return_status = crashed_before_start;
        mav_status.ipopt_return_status = status;
//        std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
        return mav_status;
    }

    status = _ipopt_app.OptimizeTNLP(_p_nlp_2_ipopt);

    IpoptSolverReturnStatus mav_status;
    mav_status.ipopt_return_status = status;
    mav_status.maverick_return_status = convertIpoptReturnStatusToMaverick(mav_status.ipopt_return_status);

    bool save_iter_and_target = false;

    if ( (mav_status.maverick_return_status == converged_optimal_solution)    ||
         (mav_status.maverick_return_status == converged_accetable_level)     ||
         (mav_status.maverick_return_status == infeasable_problem_detected)   ||
         (mav_status.maverick_return_status == number_of_iterations_exceeded) ||
         (mav_status.maverick_return_status == not_converged)) {
        save_iter_and_target = true;
    }

    if ( save_iter_and_target ) {
        // Retrieve some statistics about the solve
        Index iter_count = _ipopt_app.Statistics()->IterationCount();
//        std::cout << std::endl << std::endl << "*** The problem solved in " << iter_count << " iterations!" << std::endl;
        Number final_obj = _ipopt_app.Statistics()->FinalObjective();
//        std::cout << std::endl << std::endl << "*** The final value of the objective function is " << final_obj << '.' << std::endl;
        mav_status.num_iterations = iter_count;
        mav_status.unscaled_target = final_obj;
    }

    return mav_status;
}

SolverReturnStatus IpoptNlpSolver::convertIpoptReturnStatusToMaverick( Ipopt::ApplicationReturnStatus const ipopt_status ) {

    SolverReturnStatus mav_status;

    switch (ipopt_status) {
        case Solve_Succeeded:
            mav_status = converged_optimal_solution;
            break;
        case Solved_To_Acceptable_Level:
            mav_status = converged_accetable_level;
            break;
        case Infeasible_Problem_Detected:
            mav_status = infeasable_problem_detected;
            break;
        case Search_Direction_Becomes_Too_Small:
            mav_status = not_converged;
            break;
        case User_Requested_Stop:
            mav_status = not_converged;
            break;
        case Diverging_Iterates:
            mav_status = not_converged;
            break;
        case Feasible_Point_Found:
            mav_status = not_converged;
            break;
        case Maximum_Iterations_Exceeded:
            mav_status = number_of_iterations_exceeded;
            break;
        case Restoration_Failed:
            mav_status = crashed_during_iterations;
            break;
        case Error_In_Step_Computation:
            mav_status = crashed_during_iterations;
            break;
        case Maximum_CpuTime_Exceeded:
            mav_status = not_converged;
            break;
        case Not_Enough_Degrees_Of_Freedom:
            mav_status = problem_detected;
            break;
        case Invalid_Problem_Definition:
            mav_status = problem_detected;
            break;
        case Invalid_Option:
            mav_status = problem_detected;
            break;
        case Invalid_Number_Detected:
            mav_status = problem_detected;
            break;
        case Unrecoverable_Exception:
            mav_status = crashed_during_iterations;
            break;
        case NonIpopt_Exception_Thrown:
            mav_status = crashed_during_iterations;
            break;
        case Insufficient_Memory:
            mav_status = crashed_during_iterations;
            break;
        case Internal_Error:
            mav_status = crashed_during_iterations;
            break;
        default:
            mav_status = crashed_during_iterations;
            break;
    }

    return mav_status;
}

string IpoptNlpSolver::convertIpoptReturnStatusToString( Ipopt::ApplicationReturnStatus const ipopt_status ) {
    string return_string;

    switch (ipopt_status) {
        case Solve_Succeeded:
            return_string = "Solve succeeded";
            break;
        case Solved_To_Acceptable_Level:
            return_string = "Solved to acceptable level";
            break;
        case Infeasible_Problem_Detected:
            return_string = "Infeasible problem detected";
            break;
        case Search_Direction_Becomes_Too_Small:
            return_string = "Search direction becomes too small";
            break;
        case User_Requested_Stop:
            return_string = "User requested stop";
            break;
        case Diverging_Iterates:
            return_string = "Diverging iterates";
            break;
        case Feasible_Point_Found:
            return_string = "Feasible point found";
            break;
        case Maximum_Iterations_Exceeded:
            return_string = "Maximum iterations exceeded";
            break;
        case Restoration_Failed:
            return_string = "Restoration failed";
            break;
        case Error_In_Step_Computation:
            return_string = "Error in step computation";
            break;
        case Maximum_CpuTime_Exceeded:
            return_string = "Maximum cpu-time exceeded";
            break;
        case Not_Enough_Degrees_Of_Freedom:
            return_string = "Not enough degrees of freedom";
            break;
        case Invalid_Problem_Definition:
            return_string = "Invalid problem definition";
            break;
        case Invalid_Option:
            return_string = "Invalid option";
            break;
        case Invalid_Number_Detected:
            return_string = "Invalid number detected";
            break;
        case Unrecoverable_Exception:
            return_string = "Unrecoverable exception";
            break;
        case NonIpopt_Exception_Thrown:
            return_string = "NonIpopt exception thrown";
            break;
        case Insufficient_Memory:
            return_string = "Insufficient memory";
            break;
        case Internal_Error:
            return_string = "Internal error";
            break;
        default:
            return_string = "unkown status";
            break;
    }

    return return_string;
}
