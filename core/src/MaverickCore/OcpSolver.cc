#include "OcpSolver.hh"
#include "MaverickPrivateDefs.hh"
#include "MaverickFunctions.hh"
#include "MaverickCore/Midpoint/MidpointOcp2NlpSinglePhase.hh"
#include "MaverickCore/Midpoint/MidpointMeshSolutionRefiner.hh"
#include <iostream>
#include <string>
#include <chrono>
#include <iomanip>

#ifdef DO_NOT_USE_MAV_SHARED_LIB
#include "Maverick2Ipopt/getIpoptSolvers.hh"
#endif

using namespace Maverick;
using namespace Ipopt;
using namespace std;

OcpSolver::OcpSolver( MaverickOcp & ocp_problem ) : MaverickSolver(ocp_problem), _maverick(MaverickSingleton::getInstance()) {
    _solver_settings.nlp_guess_ptr = & _nlp_guess;

    u_integer num_th = _maverick.getHardwareConcurrencyNumThreads();
    for (u_integer i=0; i<num_th; i++)
        _th_affinity.push_back({});
    if (num_th == 0) _th_affinity = {{}};

}

OcpSolver::~OcpSolver() {}

void OcpSolver::reset() {
    _solver_settings = SolverSettings();
    _solver_status = SolverStatus();

    _nlp_guess = Nlp();
    _p_nlp_solver = nullptr;
    _p_ocp_2_nlp = nullptr;
    _ocp_solution = nullptr;
    _p_ext_guess = nullptr;
    _p_mesh = nullptr;
}


bool OcpSolver::isProblemDetectedInLastSolve() const {
    return  (_solver_status.return_status <= problem_detected);
}

// COMPUTE SOLUTION

SolverOutput OcpSolver::solve() {
    MAVERICK_ASSERT( _solver_status.has_setup_mesh, "OcpSolver::solve: you must setup the mesh before computing any solution. You may have forgotten the 'Mesh' container in the data file.\n")
    MAVERICK_ASSERT( _solver_status.has_setup_solver, "OcpSolver::solve: you must setup the solver before computing any solution. You may have forgotten the 'Solver' container in the data file.\n")
    MAVERICK_ASSERT( _ocp_problem.hasSetup(), "OcpSolver::solve: you must setup the ocp before computing any solution. You may have forgotten the 'Model' container in the data file.\n")

    SolverOutput solver_output;

    integer mesh_iterations = 0;

    while (true) { // mesh refinement loop
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        if (_p_ocp_2_nlp == nullptr)
            setupOcp2Nlp();

        // depending on the selected start code, set the guess
        setNlpGuess();

        // Log the number of threads used for function evaluation
        if (_maverick.getInfoLevel() >= InfoLevel::info_level_very_verbose) {
            for (integer i=0; i<_ocp_problem.numberOfPhases(); i++) {
                stringstream message;
                threads_affinity const & actual_th_aff = _p_ocp_2_nlp->getActualThreadsAffinityUsed(i);
                message << "Threads for evaluations at phase " << i << ": " <<actual_th_aff.size();
                bool specific_affinity = false;
                for (auto const & entry : actual_th_aff) {
                    if ( entry.size()!=0 ) {
                        specific_affinity = true;
                        break;
                    }
                }
                if (specific_affinity) {
                    for (integer i_t = 0; i_t<actual_th_aff.size(); i_t++) {
                        auto const & entry = actual_th_aff[i_t];
                        message << "\n\tthread " << i_t << " core affinity: ";
                        if (entry.size()>0) {
                            message << "[";
                            for (integer i_aff = 0; i_aff<entry.size(); i_aff++) {
                                message << entry[i_aff];
                                if (i_aff != (entry.size()-1)) message << ", ";
                            }
                            message << "]";
                        } else {
                            message << "all";
                        }
                    }
                }
                message << "\n";
                _maverick.Log(InfoLevel::info_level_verbose, message.str());
            }
        }

        // start solving problem log
        _maverick.Log(InfoLevel::info_level_normal, "Start solving the problem ...\n\n");

        IpoptNlpSolverInterface::IpoptSolverReturnStatus ipopt_return_status;

        // solve the problem
        if (_solver_settings.max_iterations>0) { //SOLVE THE PROBLEM
            ipopt_return_status = _p_nlp_solver->solve( _solver_settings );
            _solver_status.return_status = ipopt_return_status.maverick_return_status;
        } else { // if max iterations is zero then the solution is equal to the guess
            _nlp_solution = NlpSolution(_nlp_guess, SolverReturnStatus::number_of_iterations_exceeded); // the solution does not containe the constraints evaluated
            // make the solution to contain the evaluated constraints
            Nlp tmp_nlp(_nlp_solution);
            _p_ocp_2_nlp->scaleNlp(tmp_nlp); // scale the nlp
            _p_ocp_2_nlp->evalNlpConstraints(tmp_nlp);
            _p_ocp_2_nlp->scaleNlp(tmp_nlp, true); // unscale the nlp
            _nlp_solution.setConstraintsAndMultipliers(tmp_nlp.getNlpConstraintsSize(), tmp_nlp.getConstraints().data(), _nlp_solution.getConstraintsMultipliers().data());

            _solver_status.return_status = number_of_iterations_exceeded;
            ipopt_return_status.num_iterations = 0;
            _maverick.Log(InfoLevel::info_level_normal, "Maximum number of iterations exceeded.\n");
        }
        _maverick.Log(InfoLevel::info_level_normal, "\n");

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

        //caluclate elapsed time
        std::chrono::duration<real> time_span = std::chrono::duration_cast<std::chrono::duration<real>>(end - begin);
        _maverick.Log(InfoLevel::info_level_normal, "Computation time: " + std::to_string(time_span.count()) + "s\n");

        // update solver status
        _solver_status.is_last_solution_saved = false;
        if ( isProblemDetectedInLastSolve() ) {
            break;
        } else {
            _solver_status.has_mesh_changed_since_last_solution = false;
            _solver_status.is_one_solution_computed = true;
            saveLastOcpSolution();

            // update the solver output
            SolverOutput::SingleIterationOutput single_it_output;
            single_it_output.start_code = _solver_settings.start_mode;
            single_it_output.return_status = _solver_status.return_status;
            single_it_output.mesh = _p_mesh->copy();
            single_it_output.unscaled_target = _ocp_solution->getTarget();
            single_it_output.iterations = ipopt_return_status.num_iterations;
            single_it_output.calculation_ms = u_long_integer(time_span.count()*1000) ;

            solver_output.append(single_it_output, *_ocp_solution);
        }

        real mesh_increase_ratio = 1; //estimate of the increase in mesh size after mesh refinement
        std::shared_ptr<Mesh> future_mesh = nullptr;

        {
            bool will_break = false; // flag to stop mesh refinement iterations

            // check the number of iterations
            if ( ++mesh_iterations > _p_mesh->maxIterations() )
                will_break = true;

            // check the mesh tolerance
            real const tolerance_threshold = _p_mesh->tolerance();

            // now caclulate mesh error only if the user hasn't chosen to skip it
            if (_solver_settings.skip_mesh_error_calculus) {
                if (!will_break)
                    _maverick.Log(InfoLevel::info_level_warning, "Mesh tolerance is not satisfied but the mesh won't be refined because the mesh error calculus is skipped\n" );
                break; // stop the mesh refinement iterations

            } else {

                std::unique_ptr<MeshSolutionRefiner> refiner = _p_mesh->getMeshSolutionRefiner(_ocp_problem, _ocp_problem.getScaling() );
                refiner->setThreadsAffinity(_th_affinity);

                //calculate the errors and get the new mesh
                real max_mesh_error = 0;
                vec_2d_real mesh_errors;

                std::unique_ptr<Mesh> new_mesh = refiner->calculateMeshErrors(tolerance_threshold,
                                                                                 !will_break, // make new mesh
                                                                                 true,
                                                                                 *_ocp_solution,
                                                                                 *_p_mesh,
                                                                                 mesh_errors,
                                                                                 max_mesh_error);
                future_mesh = std::move(new_mesh) ;

                bool mesh_error_satisfied = abs(max_mesh_error) < tolerance_threshold;
                stringstream tmp;
                if ( mesh_error_satisfied ) {
                    tmp << "Mesh error is satisfied in current mesh";
                    if (    ( _solver_status.return_status == converged_optimal_solution )
                        || ( _solver_status.return_status == converged_accetable_level)     )
                        will_break = true;;
                    } else {
                    tmp << "Mesh error is NOT satisfied in current mesh";
                }
                tmp << " (iteration " << (mesh_iterations-1) << ") : ";
                tmp << std::scientific << max_mesh_error << " (threshold = " << tolerance_threshold << ")\n";
                _maverick.Log(InfoLevel::info_level_normal, tmp.str());

                if (will_break)
                    break; // stop the mesh refinement iterations

                // set the mesh
                setMesh(future_mesh);
            }
        }

        // set the start mode
        _solver_settings.start_mode = warm_start;
        if (mesh_increase_ratio < 3.5)
            _solver_settings.start_mode = warm_start_with_multipliers;
        if (_solver_settings.refinement_start == force_warm_start)
            _solver_settings.start_mode = warm_start;
        if (_solver_settings.refinement_start == force_warm_multiplier_start)
            _solver_settings.start_mode = warm_start_with_multipliers;
        if (_solver_settings.refinement_start == force_cold_start)
            _solver_settings.start_mode = cold_start;

    } // end of while loop for mesh refinement

    return solver_output;
}

SolverOutput OcpSolver::doSingleRun( GC::GenericContainer const &gc ) {
    setup( gc );

    // check if to write the mesh history
    string mh_filename = "no";
    gc.get_if_exists("write_mesh_history", mh_filename);
    if ( (mh_filename.compare("no")!=0) && (mh_filename.compare("No")!=0) && (mh_filename.compare("NO")!=0) ) {
        _solver_settings.save_mesh_history = true;
    } else {
        _solver_settings.save_mesh_history = false;
    }

    //solve
    SolverOutput solver_output = solve();

    // check if to write the solution
    string res_filename = "no";
    try {
        res_filename = gc("write_solution").get_string();
    } catch (...) {}
    if ( (res_filename.compare("no")!=0) && (res_filename.compare("No")!=0) && (res_filename.compare("NO")!=0) ) {
        if ( isProblemDetectedInLastSolve() )
            _maverick.Log(InfoLevel::info_level_warning, "Solution cannot be saved because it has not been computed\n");
        else {
            _maverick.Log(InfoLevel::info_level_normal, "Writing solution to file...");
            _ocp_problem.writeSolutionToFile(*_ocp_solution, res_filename);
            _maverick.Log(InfoLevel::info_level_normal, "Done\n");
        }
    }

    // write the mesh history if necessary
    if (_solver_settings.save_mesh_history) {
        _maverick.Log(InfoLevel::info_level_normal, "Writing mesh history...");
        _ocp_problem.writeMeshHistoryToFile(solver_output, mh_filename);
        _maverick.Log(InfoLevel::info_level_normal, "Done\n");
    }

    return solver_output;
}

SolverOutput OcpSolver::solve( GC::GenericContainer const &gc_run ) {
    SolverOutput complete_solver_output;

    integer run_index = 0;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    while (true) {
        GC::GenericContainer const * gc = nullptr;
        string run_index_string = std::to_string(run_index);
        try {
            gc = & gc_run("Run" + run_index_string);
        } catch (...) {}

        if ( gc == nullptr ) {
            break;
        } else {
            _maverick.Log(InfoLevel::info_level_normal, "\nMaverick will start run " + run_index_string + ":\n");
            complete_solver_output << doSingleRun(*gc);
            run_index ++;

            if ( isProblemDetectedInLastSolve() ) {
                _maverick.Log(InfoLevel::info_level_warning, "Problem detected. Iterations will stop now.\n");
                break;
            }
        }
    }

    if ( run_index == 0 )
        complete_solver_output << doSingleRun(gc_run);

    //caluclate elapsed time
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::chrono::duration<real> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(end - begin);

    // print summary of the calculation
    if (_maverick.getInfoLevel() >= InfoLevel::info_level_normal ) {
        integer const width = 35;
        _maverick.Log(InfoLevel::info_level_normal, "\n");
        // time elapsed
        stringstream ss;
        ss << std::setw(width) << std::left << "Total time elapsed:";
        real sec = time_span.count();
        if (sec > 1.0e3)
            ss << std::fixed << std::setprecision(0) << sec << "s\n";
        else
            ss << std::fixed << std::setprecision(3) << sec << "s\n";

        _maverick.Log(InfoLevel::info_level_normal, ss.str() );
        // total cpu time
        ss.str("");
        ss << std::setw(width) << std::left << "Total calculation time:";
        real msec = complete_solver_output.getTotalCalculationMs();
        if (msec > 1.0e3)
            ss << msec/1e3 << "s\n";
        else
            ss << msec << "ms\n";
        _maverick.Log(InfoLevel::info_level_normal, ss.str() );
        // total iterations
        ss.str("");
        ss << std::setw(width) << std::left << "Total iterations:";
        ss << complete_solver_output.getTotalIterations() << "\n";
        _maverick.Log(InfoLevel::info_level_normal, ss.str() );
        // target
        ss.str("");
        ss << std::setw(width) << std::left << "Target:";
        ss << std::scientific << std::setprecision(3) << complete_solver_output.getSolution()->getTarget();
        ss << "\n";
        _maverick.Log(InfoLevel::info_level_normal, ss.str() );
    }

    // print the info
    _maverick.printInfo();

    return complete_solver_output;
}

//SETUP

void OcpSolver::setNlpGuess() {
    _nlp_guess.clear();
    if ( ( _solver_settings.start_mode == warm_start ) || ( _solver_settings.start_mode == warm_start_with_multipliers ) ) {
        if ( isProblemDetectedInLastSolve() ) { // if an error occured in the last computation
            if ( _solver_status.is_one_solution_computed ) {
                _maverick.Log(InfoLevel::info_level_warning, "Warm start requested but solver crashed in last computation. Will use previous solution.\n");
            } else {
                _maverick.Log(InfoLevel::info_level_warning, "Warm start requested when no previous solution has been computed. Will switch to cold start.\n");
                _solver_settings.start_mode = cold_start;
            }
        }
    }

    if ( _solver_settings.start_mode == cold_start ) { // in this case provide the OcpProblem guess or the external one if provided
        if ( _p_ext_guess != nullptr ) {
            if (_solver_status.ext_guess_type == ext_guess_object)
                _maverick.Log(InfoLevel::info_level_more, "Using external object as guess ...\n");
            else
                _maverick.Log(InfoLevel::info_level_more, "Using guess tables as guess ...\n");

            _nlp_guess = _p_ocp_2_nlp->translateOcpGuess2Nlp( *_p_ext_guess );
            _p_ocp_2_nlp->setIsTargetLagrangeFromGuess( *_p_ext_guess );
        } else {
            _maverick.Log(InfoLevel::info_level_more, "Using guess from ocp problem ...\n");
            _nlp_guess = _p_ocp_2_nlp->translateOcpGuess2Nlp( _ocp_problem );
            _p_ocp_2_nlp->setIsTargetLagrangeFromGuess( _ocp_problem );
        }
        return;
    }

    if ( ( _solver_settings.start_mode == warm_start ) || ( _solver_settings.start_mode == warm_start_with_multipliers ) ) { // in this case interpolate the last nlp solution to the new mesh

        if ( _solver_settings.start_mode == warm_start )
            _maverick.Log(InfoLevel::info_level_more, "Using warm guess ...\n");
        else
            _maverick.Log(InfoLevel::info_level_more, "Using warm guess with multipliers ...\n");

#ifndef MAVERICK_DEBUG
        if (_solver_status.has_mesh_changed_since_last_solution) { // in this case interpolate the last nlp solution to the new mesh
            _nlp_guess = _p_ocp_2_nlp->translateOcpGuess2Nlp( *_ocp_solution );
            _p_ocp_2_nlp->setIsTargetLagrangeFromGuess( *_ocp_solution );
        } else { // in this case provide the last nlp solution
            _nlp_guess = Nlp(_nlp_solution);
            _p_ocp_2_nlp->setIsTargetLagrangeFromGuess( _nlp_guess );
        }
#else
        _nlp_guess = _p_ocp_2_nlp->translateOcpGuess2Nlp( *_ocp_solution );
        _p_ocp_2_nlp->setIsTargetLagrangeFromGuess( *_ocp_solution );
#endif

        return;
    }
}

// GENERAL SETUP
void OcpSolver::setup( GC::GenericContainer const & gc_setup ) {
    // setup info level
    integer info_level;
    if ( gc_setup.get_if_exists("info_level", info_level) ) {
        switch (info_level) {
            case 0:
                _maverick.setInfoLevel(InfoLevel::info_level_none);
                break;
            case 1:
                _maverick.setInfoLevel(InfoLevel::info_level_warning);
                break;
            case 2:
                _maverick.setInfoLevel(InfoLevel::info_level_few);
                break;
            case 3:
                _maverick.setInfoLevel(InfoLevel::info_level_normal);
                break;
            case 4:
                _maverick.setInfoLevel(InfoLevel::info_level_more);
                break;
            case 5:
                _maverick.setInfoLevel(InfoLevel::info_level_verbose);
                break;
            case 6:
                _maverick.setInfoLevel(InfoLevel::info_level_very_verbose);
                break;
            default:
                break;
        }
        if (info_level<0)
            _maverick.setInfoLevel(InfoLevel::info_level_none);
        if (info_level>6)
            _maverick.setInfoLevel(InfoLevel::info_level_very_verbose);
    }

    // setup the ocp
    GC::GenericContainer const * gc_ocp = nullptr;
    {
        try {
            gc_ocp = &gc_setup("Model");
        } catch ( ... ) {}

        if (gc_ocp != nullptr)
            setupOcpOnly(*gc_ocp);
    }

    // setup mesh
    {
        GC::GenericContainer const * gc_mesh = nullptr;
        try {
            gc_mesh = &(gc_setup("Mesh"));
        } catch (...) {}
        if ( gc_mesh != nullptr ) { // in this case, the mesh is declared in the generic container
            setupMeshOnly( *gc_mesh );
        }
    }

    // setup ocp scaling
    {
        GC::GenericContainer const * gc_scaling = nullptr;
        try {
            gc_scaling = &gc_setup("Model")("Scaling");
        } catch ( ... ) {}
        if (gc_scaling != nullptr)
            setupOcpScalingOnly( *gc_scaling );
    }

    // setup guess tables
    {
        GC::GenericContainer const * gc_tables = nullptr;
        try {
            gc_tables = &gc_setup("GuessTables");
        } catch ( ... ) {}
        if (gc_tables != nullptr)
            setupGuessTablesOnly(*gc_tables);
    }

    // setup solver
    {
        GC::GenericContainer const * gc_solver = nullptr;
        try {
            gc_solver = &gc_setup("Solver");
        } catch (...) {}

        if ( gc_solver != nullptr ) { // in this case, the mesh is declared in the generic container
            setupSolverOnly( *gc_solver );
        } else {
            if ( !_solver_status.has_setup_solver ) { //if the solver has not been setup, then use the default settings
                setupSolverOnly( GC::GenericContainer() );
            }
        }
    }

}

void OcpSolver::setMesh( std::shared_ptr< Mesh > mesh ) {
    _maverick.Log(InfoLevel::info_level_normal, "Setting up mesh ...\n");

    _p_mesh = mesh;
    _p_ocp_2_nlp = nullptr; // invalidate discretiser

    _solver_status.has_mesh_changed_since_last_solution = true;
    _solver_status.has_setup_mesh = true;

    MAVERICK_ASSERT( _p_mesh->getNumberOfPhases() == _ocp_problem.numberOfPhases(), "Maverick solver error: number of mesh phases " << _p_mesh->getNumberOfPhases() << " is different from number of ocp phases " << _ocp_problem.numberOfPhases() << "\n" )
}

void OcpSolver::setMesh( Mesh const & mesh ) {
    // copy the mesh from outside
    std::shared_ptr<Mesh> ptr ( mesh.copy() );
    setMesh(ptr);
}

//SETUP MESH ONLY
void OcpSolver::setupMeshOnly( GC::GenericContainer const & gc_mesh ) {
    shared_ptr<Mesh> mesh = shared_ptr<Mesh> (new MidpointMesh()); // currenty only this mesh type is implemented
    mesh->setup( gc_mesh );
    setMesh(mesh);
}


//SETUP OCP2NLP ONLY
void OcpSolver::setupOcp2Nlp() {

    integer min_nlp_vars_per_thread = 0;
    if ( _p_ocp_2_nlp != nullptr ) {
        min_nlp_vars_per_thread = _p_ocp_2_nlp->getMinNumberOfNlpVarsPerThreads();
    }
    _p_ocp_2_nlp = _p_mesh->getDiscretiser( _ocp_problem );
    _p_ocp_2_nlp->setThreadsAffinity(_th_affinity);

    //set min number of threads
    if (min_nlp_vars_per_thread > 0)
        _p_ocp_2_nlp->setMinNumberOfNlpVarsPerThreads(min_nlp_vars_per_thread);

    if (_p_nlp_solver != nullptr) //if the nlp solver already exist, update its reference to ocp2nlp
        _p_nlp_solver->setOcp2Nlp( _p_ocp_2_nlp );
}

//SETUP THE SOLVER ONLY
void OcpSolver::setupSolverOnly( GC::GenericContainer const & gc_solver ) {
    _maverick.Log(InfoLevel::info_level_normal, "Setting up nlp solver...\n");

    NlpSolverId requested_nlp_solver_id = NlpSolverId::nlp_id_ipopt; //default solver
    try { //try to get the diesired nlp solver
        string nlp_solver_type = gc_solver["nlp_solver"].get_string();
        if ( nlp_solver_type == "Ipopt" )
            requested_nlp_solver_id = NlpSolverId::nlp_id_ipopt;
        if ( nlp_solver_type == "Whorp" )
            requested_nlp_solver_id = NlpSolverId::nlp_id_whorp;
    } catch (...) {}

    if ( ( _p_nlp_solver != nullptr) && ( _p_nlp_solver->getNlpSolverId() != requested_nlp_solver_id) ) {
        //deleteIpoptNlpSolverFromSharedLib();
        _p_nlp_solver = nullptr;
    }
    if (_p_nlp_solver == nullptr) { // we have to initialise it
        if (requested_nlp_solver_id == NlpSolverId::nlp_id_ipopt ) {
            loadIpoptNlpSolverFromSharedLib();
        } else {
            throw runtime_error("Currently only Ipopt can be used as nlp solver.\n");
        }
    }
    _p_nlp_solver->setup( gc_solver ); // finally, setup

    // check for the warm or hot start
    try {
        string start_code_requested = gc_solver("start_mode").get_string();
        if (start_code_requested.compare("cold_start")==0)
            _solver_settings.start_mode = cold_start;
        else if (start_code_requested.compare("warm_start")==0)
            _solver_settings.start_mode = warm_start;
        else if (start_code_requested.compare("warm_start_with_multipliers")==0)
            _solver_settings.start_mode = warm_start_with_multipliers;
        else {
            _maverick.Log(InfoLevel::info_level_warning, "Requested start of type '" + start_code_requested + "', which is not recognised. Available start mode are:\n"
                + "cold_start, warm_start, warm_start_with_multipliers.\n");//, hot_start, hot_start_with_multipliers.\n");
        }
    } catch (...) {}

    // check for the max number of iterations
    {
        integer max_iter;
        if (findIntFromGenericContainer(gc_solver, "max_iterations", max_iter)) {
            if ( max_iter < 0 ) {
                _maverick.Log(InfoLevel::info_level_warning, "Negative number of maximum solver iterations is not allowed. Will set to zero.\n");
                max_iter = 0;
            }
            _solver_settings.max_iterations = max_iter;
        }
    }

    //check for the number of threads to use
    {
        // first check if the user specified the threads_affinity:
        try {
            GC::vector_type const & gc_aff = gc_solver("threads_affinity").get_vector();
            _th_affinity = {};

            for (integer i = 0; i<gc_aff.size(); i++) {
                _th_affinity.push_back({});
                vector<integer> c_aff;
                if (!findVecIntFromGenericContainer(gc_aff[i], c_aff)) throw runtime_error("");
                for (int x : c_aff) {
                    if ( x >= 0 )
                        _th_affinity[i].push_back( (u_integer) x );
                    else {
                        _th_affinity[i] = {};
                        break;
                    }
                }
            }
        } catch (...) {
            // in this case, the user may have specified the number of threads
            integer num_th;
            if (findIntFromGenericContainer(gc_solver, "num_threads", num_th)) {
                _th_affinity = {};
                if ( num_th < 0 ) {
                    _maverick.Log(InfoLevel::info_level_warning, "Negative number of threads to use is not allowed. Will set to default.\n");
                    num_th = 0;
                }
                for (integer i = 0; i<num_th; i++)
                    _th_affinity.push_back({});
            }
        }
        if (_th_affinity.size() == 0) _th_affinity = {{}};
        if (_p_ocp_2_nlp != nullptr)
            _p_ocp_2_nlp -> setThreadsAffinity(_th_affinity);

    }

    //check for the minimum number of nlp vars per thread
    {
        integer min_nlp_vars_p_t;
        if (findIntFromGenericContainer(gc_solver, "min_nlp_vars_per_thread", min_nlp_vars_p_t)) {
            if ( min_nlp_vars_p_t <= 0 ) {
                _maverick.Log(InfoLevel::info_level_warning, "Non positive number of nlp vars per thread is not allowed. Will set to 1.\n");
                min_nlp_vars_p_t = 1;
            }
            if (_p_ocp_2_nlp != nullptr)
                _p_ocp_2_nlp->setMinNumberOfNlpVarsPerThreads(min_nlp_vars_p_t);
        }
    }

    //check for continuation start
    try {
        string continuation_mode = gc_solver("continuation_mode").get_string();
        if (continuation_mode.compare("standard") == 0)
            _solver_settings.refinement_start = standad_start;
        else if (continuation_mode.compare("force_warm_start") == 0)
            _solver_settings.refinement_start = force_warm_start;
        else if (continuation_mode.compare("force_warm_start_with_multipliers") == 0)
            _solver_settings.refinement_start = force_warm_multiplier_start;
        else if (continuation_mode.compare("force_cold_start") == 0)
            _solver_settings.refinement_start = force_cold_start;
        else {
            stringstream ss;
            ss << "'" << continuation_mode << "' is an invalid option for 'continuation_mode' setting. Possible chiches are:\n";
            ss << "\t" << "standard" << "\n";
            ss << "\t" << "force_cold_start" << "\n";
            ss << "\t" << "force_warm_start" << "\n";
            ss << "\t" << "force_warm_start_with_multipliers" << "\n";
            _maverick.Log(InfoLevel::info_level_warning, ss.str() );
        }
    } catch (...) {}

    //check for integrator type
    {
        string integrator_type;
        bool found = gc_solver.get_if_exists("integrator", integrator_type);
        if (found) {
            if (integrator_type.compare("Tensolve") == 0)
                _solver_settings.integrator_type = integrator_tensolve;
            else if (integrator_type.compare("Ipopt") == 0)
                _solver_settings.integrator_type = integrator_ipopt;
            else {
                stringstream ss;
                ss << "'" << integrator_type << "' is an invalid option for 'integrator' setting. Possible chiches are:\n";
                ss << "\t" << "Tensolve" << "\n";
                ss << "\t" << "Ipopt" << "\n";
                _maverick.Log(InfoLevel::info_level_warning, ss.str() );
            }
        }
    }

    // check for the skip of mesh error calculus
    {
        bool skip_calculus = false;
        bool found = gc_solver.get_if_exists("skip_mesh_error_calculus", skip_calculus);
        if (found)
            _solver_settings.skip_mesh_error_calculus = skip_calculus;
    }

    _solver_status.has_setup_solver = true;
}

void OcpSolver::setupOcpOnly( GC::GenericContainer const &gc_setup ) {
    _maverick.Log(InfoLevel::info_level_normal, "Setting up ocp problem ...\n");
    _ocp_problem.setupOcp( gc_setup );

    Maverick::InfoLevel il = InfoLevel::info_level_more;
    if ( _maverick.getInfoLevel() >= il ) {
        stringstream ss;
        ss << "OCP model parameters detail:\n";
        _ocp_problem.printInfo(ss, _maverick.getInfoLevel());
        ss << "\n";
        _maverick.Log(il, ss.str());
    }
}

void OcpSolver::setupGuessTablesOnly( GC::GenericContainer const &gc_guess_tables ) {
    _maverick.Log(InfoLevel::info_level_normal, "Setting up guess tables ...\n");
    // check if all or none phase are declared
    bool is_one_phase_declared;
    bool are_all_phases_declared;
    vector<integer> missing_phases;
    chechForAllPhasesDeclarationInGc( gc_guess_tables, _ocp_problem.numberOfPhases(), are_all_phases_declared, is_one_phase_declared, missing_phases );


    if (is_one_phase_declared && (!are_all_phases_declared)) {
        std::stringstream err_mess;
        err_mess << "Guess tables are not declared for all phases. Missing phases are: ";
        for (integer i=0; i<missing_phases.size(); i++)
            err_mess << missing_phases[i] << " ";
        err_mess << "\n";
        throw runtime_error(err_mess.str());
    }

    GC::map_type const * mappa = nullptr;
    if ( (!is_one_phase_declared) && (_ocp_problem.numberOfPhases()==1) ) { //try to check if it is declared as root value
        try {
            mappa = &(gc_guess_tables.get_map());
            are_all_phases_declared = true;
        } catch ( ... ) {}
    }

    if (are_all_phases_declared) { //in this case, we have to set the table as guess
        vector<real_table> tables;
        if ( mappa != nullptr ) { // in this case, the problem is a single phase problem, and the map is set in the root of the generic container
            vec_1d_real tmp;
            findVecRealFromGenericContainer( (*mappa).at("zeta"), tmp );
            size lenght = tmp.size();
            if (lenght==0)
                throw runtime_error("Cannot find zeta entry, or wrong type, for guess table.\n");
            if (lenght<2)
                throw runtime_error("Zeta entry in guess table must have dimension of at least 2.\n");

            tables.push_back(real_table());
            string error_key;
            integer return_code = extractRealTableFromMapType( *mappa, tables[0], error_key);
            if (return_code!=0) {
                throw runtime_error("Guess table entry '" + error_key + "' is not a vector of numbers.\n");
            }
        } else { // in this case, all phases are declared in the form 'PhaseXX'
            for (integer i=0; i<_ocp_problem.numberOfPhases(); i++) {
                stringstream ss;
                ss << "Phase" << i;

                GC::map_type const * current_map;
                try {
                    current_map = &(gc_guess_tables(ss.str()).get_map());
                } catch (...) {
                    throw runtime_error("Guess table not found, or wrong table format, for phase " + std::to_string(i));
                }

                vec_1d_real tmp;
                findVecRealFromGenericContainer( (*current_map).at("zeta"), tmp );
                size lenght = tmp.size();
                if (lenght==0)
                    throw runtime_error("Cannot find zeta entry, or wrong type, for guess table for phase " + std::to_string(i) + ".\n");
                if (lenght<2)
                    throw runtime_error("Zeta entry in guess table for phase " + std::to_string(i) + " must have dimension of at least 2.\n");

                tables.push_back(real_table());
                string error_key;
                integer return_code = extractRealTableFromMapType( *current_map, tables[0], error_key);
                if (return_code!=0) {
                    throw runtime_error("Guess table entry '" + error_key + "' at phase " + std::to_string(i) + " is not a vector of numbers.\n");
                }
            }
        }

        // now convert the tables to an ocp guess

        vector<vector<string>> found_vars_vec;
        std::unique_ptr<OcpSolution> solution = convertGuessTable2OcpSolution(tables, _ocp_problem, found_vars_vec);
        stringstream found_vars;
        for (integer i=0; i<found_vars_vec.size(); i++) {
            vector<string> const & vars = found_vars_vec[i];
            found_vars << "\tPhase" << i << ", found guess for: ";
            for (vector<string>::const_iterator it = vars.begin(); it!=vars.end(); it ++) {
                found_vars << *it << " ";
            }
            if (vars.size()==0)
                found_vars << "nothing!";
            found_vars << "\n";
        }
        _maverick.Log(InfoLevel::info_level_more, found_vars.str() );
        _p_ext_guess = std::move(solution);
        _solver_status.ext_guess_type = ext_guess_tables;

    } else { //in this case, we have to remove the table as guess, if already set
        _maverick.Log(InfoLevel::info_level_normal, "\tno guess table found.\n");
        if (_solver_status.ext_guess_type == ext_guess_tables) {
            _p_ext_guess = nullptr; //it is a shared pointer
            _solver_status.ext_guess_type = ext_guess_none;
        }
    }
}

//SETUP SCALING ONLY
void OcpSolver::setupOcpScalingOnly( GC::GenericContainer const &gc_scaling ) {
    _maverick.Log(InfoLevel::info_level_normal, "Setting up nlp scaling ...\n");

    _ocp_problem.setupScaling(gc_scaling, _p_mesh->getDiscretisationPoints() );

    if ( _maverick.getInfoLevel() >= InfoLevel::info_level_more ) {
        stringstream scaling;
        scaling << "Scaling factors details:\n";
        _ocp_problem.getScaling().writeScalingsToStream( scaling );
        _maverick.Log(InfoLevel::info_level_more, scaling.str() );
    }

}

// INTERNAL METHODS

void OcpSolver::setExternalGuess( shared_ptr<OcpGuess const> p_ext_guess ) {
    _p_ext_guess = p_ext_guess;
    _solver_status.ext_guess_type = ext_guess_object;
}

void OcpSolver::setStartMode( SolverStartCode const code ) {
    if (code == cold_start) {
        _solver_settings.start_mode = code;
    } else {
        if (!_solver_status.is_one_solution_computed) {
            _maverick.Log(InfoLevel::info_level_warning, "OcpSolver: warm start has been selected when no previous solutions have been calculated. Will switch to cold start.\n");
            _solver_settings.start_mode = cold_start;
        } else {
            _solver_settings.start_mode = code;
        }
    }
}

bool OcpSolver::saveLastOcpSolution() {
    if ( isProblemDetectedInLastSolve() ) // if there were an error, we cannot save the last solution
        return false;

    // in this case, we can save the solution
    if ( !_solver_status.is_last_solution_saved ) { // if the last solution is not saved
        if ( _solver_status.has_mesh_changed_since_last_solution )  {
            throw runtime_error("Internal error at OcpSolver::saveLastOcpSolution. Please contact the developer.\n");
        }
        _ocp_solution = _p_ocp_2_nlp->translateNlp2OcpSolution( _nlp_solution );

        _solver_status.is_last_solution_saved = true;
    }
    return true;
}



void OcpSolver::loadIpoptNlpSolverFromSharedLib() {
#ifndef DO_NOT_USE_MAV_SHARED_LIB
    void * p_mavericktoip = _maverick.getMaverickToIpSharedLibHandle();
    typedef std::unique_ptr<Maverick::IpoptNlpSolverInterface> (*GetIpoptNlpSolver)(Maverick::NlpSolution &);

    GetIpoptNlpSolver p_getIpoptNlpSolver = nullptr;
    p_getIpoptNlpSolver = (GetIpoptNlpSolver)dlsym(p_mavericktoip, "getIpoptNlpSolver");

    // if sl_handle exist
    if (p_getIpoptNlpSolver) {
        _p_nlp_solver = p_getIpoptNlpSolver(_nlp_solution);
        _p_nlp_solver->setOcp2Nlp(_p_ocp_2_nlp);
	} else {
        string message = "error while loading shared library: libmavericktoip: " + string(dlerror()) + "\n";
		throw runtime_error(message);
	}
#else
    _p_nlp_solver = getIpoptNlpSolver(_nlp_solution);
    _p_nlp_solver->setOcp2Nlp(_p_ocp_2_nlp);
#endif
}

// static methods
std::string OcpSolver::convertSolverStartCodeToString(SolverStartCode start_code) {
    switch (start_code) {
        case cold_start:
            return "cold_start";
        case warm_start:
            return "warm_start";
        case warm_start_with_multipliers:
            return "warm_start_with_multipliers";
        default:
            return "not_recognized";
    }
}

std::string OcpSolver::convertSolverReturnStatusToString(SolverReturnStatus return_status) {
    switch (return_status) {
        case solution_not_computed           :
            return "solution_not_computed";
        case converged_optimal_solution      :
            return "converged_optimal_solution";
        case converged_accetable_level       :
            return "converged_accetable_level";
        case number_of_iterations_exceeded   :
            return "number_of_iterations_exceeded";
        case infeasable_problem_detected     :
            return "infeasable_problem_detected";
        case not_converged                   :
            return "not_converged";
        case problem_detected                :
            return "problem_detected";
        case crashed                         :
            return "crashed";
        case crashed_before_start            :
            return "crashed_before_start";
        case crashed_during_iterations       :
            return "crashed_during_iterations";
        default:
            return "not_recognized";
    }
}
