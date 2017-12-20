#include "MidpointOcp2NlpSinglePhase.hh"
#include "MaverickCore/MaverickFunctions.hh"
#include "MaverickCore/MaverickPrivateDefs.hh"
#include "MaverickCore/OcpScaling.hh"
#include "MaverickCore/Midpoint/MidpointOcpSolution.hh"
#include "MaverickUtils/GenericFunction/GF1ASpline.hh"
#include "MaverickCore/MaverickSingleton.hh"
#include <mutex>

using namespace Maverick;
using namespace std;

#define SPLINE_TYPE "Akima"
//#define SPLINE_TYPE "Linear"

#define SPLINE_EXTEND_RANGE MaverickUtils::GF1ASpline::ExtendRange::keep_derivative

MidpointOcp2NlpSinglePhase::MidpointOcp2NlpSinglePhase( MaverickOcp const & ocp_problem, Mesh const & mesh, integer const i_phase ) : _i_phase(i_phase), Ocp2Nlp(ocp_problem, mesh) {

    if (mesh.discretisationType() != DiscretisationType::midpoint)
        throw runtime_error("MidpointOcp2NlpSinglePhase::derivedInit: only a midpoint mesh can be used");
    _p_mesh = (MidpointMeshSinglePhase*) &( mesh[_i_phase]);

    vector < int ** > additional_int_pointers = {
        // target gradient pointers
        &_p_lagrange_target_j_y_pattern,
        &_p_target_j_yi_pattern,
        &_p_target_j_yf_pattern,
        &_p_target_j_p_pattern,

        // constraints jacobian pointers
        &_p_fo_eqns_j_xu_outer_start ,
        &_p_fo_eqns_j_dxu_outer_start,
        &_p_path_constr_j_xu_outer_start,
        &_p_path_constr_j_dxu_outer_start,
        &_p_int_constr_j_xu_outer_start,
        &_p_int_constr_j_dxu_outer_start,

        // hessian pointers
        &_p_lag_hess_xu_xu_outer_start,
        &_p_lag_hess_xu_dxu_outer_start,
        &_p_lag_hess_xu_axu_outer_start,
        &_p_lag_hess_xu_p_outer_start,
        &_p_lag_hess_dxu_dxu_outer_start,
        &_p_lag_hess_dxu_axu_outer_start,
        &_p_lag_hess_dxu_p_outer_start,
        &_p_lag_hess_axu_axu_outer_start,
        &_p_lag_hess_axu_p_outer_start,
        &_p_lag_hess_p_p_outer_start,
        &_p_fo_eqns_hess_xu_xu_outer_start,
        &_p_fo_eqns_hess_xu_dxu_outer_start,
        &_p_fo_eqns_hess_xu_axu_outer_start,
        &_p_fo_eqns_hess_xu_p_outer_start,
        &_p_fo_eqns_hess_dxu_dxu_outer_start,
        &_p_fo_eqns_hess_dxu_axu_outer_start,
        &_p_fo_eqns_hess_dxu_p_outer_start,
        &_p_fo_eqns_hess_axu_axu_outer_start,
        &_p_fo_eqns_hess_axu_p_outer_start,
        &_p_fo_eqns_hess_p_p_outer_start,
        &_p_path_constr_hess_xu_xu_outer_start,
        &_p_path_constr_hess_xu_dxu_outer_start,
        &_p_path_constr_hess_xu_axu_outer_start,
        &_p_path_constr_hess_xu_p_outer_start,
        &_p_path_constr_hess_dxu_dxu_outer_start,
        &_p_path_constr_hess_dxu_axu_outer_start,
        &_p_path_constr_hess_dxu_p_outer_start,
        &_p_path_constr_hess_axu_axu_outer_start,
        &_p_path_constr_hess_axu_p_outer_start,
        &_p_path_constr_hess_p_p_outer_start,
        &_p_int_constr_hess_xu_xu_outer_start,
        &_p_int_constr_hess_xu_dxu_outer_start,
        &_p_int_constr_hess_xu_axu_outer_start,
        &_p_int_constr_hess_xu_p_outer_start,
        &_p_int_constr_hess_dxu_dxu_outer_start,
        &_p_int_constr_hess_dxu_axu_outer_start,
        &_p_int_constr_hess_dxu_p_outer_start,
        &_p_int_constr_hess_axu_axu_outer_start,
        &_p_int_constr_hess_axu_p_outer_start,
        &_p_int_constr_hess_p_p_outer_start,
        &_p_point_constr_hess_xu_xu_outer_start,
        &_p_point_constr_hess_xu_p_outer_start,
        &_p_point_constr_hess_p_p_outer_start,
        &_p_bcs_hess_xu_init_xu_init_outer_start,
        &_p_bcs_hess_xu_init_xu_fin_outer_start,
        &_p_bcs_hess_xu_init_p_outer_start,
        &_p_bcs_hess_xu_fin_xu_fin_outer_start,
        &_p_bcs_hess_xu_fin_p_outer_start,
        &_p_bcs_hess_p_p_outer_start,
        &_p_mayer_hess_xu_init_xu_init_outer_start,
        &_p_mayer_hess_xu_init_xu_fin_outer_start,
        &_p_mayer_hess_xu_init_p_outer_start,
        &_p_mayer_hess_xu_fin_xu_fin_outer_start,
        &_p_mayer_hess_xu_fin_p_outer_start,
        &_p_mayer_hess_p_p_outer_start,
        &_p_hess_y_y_lower_mat_outer_starts,
        &_p_hess_y_y_lower_mat_rows,
        &_p_hess_y_p_mat_outer_starts,
        &_p_hess_y_p_mat_rows
    };

    vector<real **> additional_real_pointers = {
        //scaling quantities
        &_p_scaling_y    ,
        &_p_inv_scaling_y,
        &_p_scaling_ay    ,
        &_p_inv_scaling_ay,
        &_p_scaling_r    ,
        &_p_inv_scaling_r,

        &_p_inv_scaling_fo_eqns_global      ,
        &_p_inv_scaling_point_constr_global ,
        &_p_inv_scaling_path_constr_global  ,
        &_p_inv_scaling_int_constr          ,
        &_p_inv_scaling_bcs                 ,

        //target gradient scaling
        &_p_scale_factor_lagrange_target_j_y  ,
        &_p_scale_factor_lagrange_target_j_ay ,
        &_p_scale_factor_lagrange_target_j_p  ,
        &_p_scale_factor_mayer_j_yi           ,
        &_p_scale_factor_mayer_j_yf           ,
        &_p_scale_factor_mayer_j_p            ,

        //constraints jacobian scaling
        &_p_scale_factor_fo_eqns_j_y      ,
        &_p_scale_factor_fo_eqns_j_ay     ,
        &_p_scale_factor_fo_eqns_j_p      ,
        &_p_scale_factor_path_constr_j_y  ,
        &_p_scale_factor_path_constr_j_ay ,
        &_p_scale_factor_path_constr_j_p  ,
        &_p_scale_factor_int_constr_j_y   ,
        &_p_scale_factor_int_constr_j_ay  ,
        &_p_scale_factor_int_constr_j_p   ,
        &_p_scale_factor_point_constr_j_y ,
        &_p_scale_factor_point_constr_j_p ,
        &_p_scale_factor_bcs_j_xifp       ,

        // lagrangian hessian scaling
        &_p_scale_factor_hess_y_y_lower_mat   ,
        &_p_scale_factor_hess_ay_ay_lower_mat ,
        &_p_scale_factor_hess_y_ay_mat        ,
        &_p_scale_factor_hess_ay_y_mat        ,
        &_p_scale_factor_hess_y_p_mat         ,
        &_p_scale_factor_hess_ay_p_mat        ,
        &_p_scale_factor_hess_yleft_yright_mat
    };

    _int_vec_pointers.insert(std::end(_int_vec_pointers), std::begin(additional_int_pointers), std::end(additional_int_pointers));
    _real_vec_pointers.insert(std::end(_real_vec_pointers), std::begin(additional_real_pointers), std::end(additional_real_pointers));

    setup();

}


MidpointOcp2NlpSinglePhase::~MidpointOcp2NlpSinglePhase() {
    // pointers already deleted in super class
    clearThreadJobs();
}

void MidpointOcp2NlpSinglePhase::setup() {
    MAVERICK_DEBUG_ASSERT(_p_mesh!=nullptr, "MidpointOcp2NlpSinglePhase::setup: nullptr mesh pointer");

    deleteAllDataPointers();

    // ocp and nlp dimensions
    _dim_x  = _ocp_problem.numberOfStates(_i_phase); // states
    _dim_xu = _dim_x + _ocp_problem.numberOfControls(_i_phase); // states and controls
    _dim_ax  = _ocp_problem.numberOfAlgebraicStates(_i_phase); // states
    _dim_axu = _dim_ax + _ocp_problem.numberOfAlgebraicControls(_i_phase); // states and controls

    _dim_fo = _ocp_problem.numberOfFirstOrderEquations(_i_phase); //number of path constraints
    _dim_pc = _ocp_problem.numberOfPathConstraints(_i_phase); //number of path constraints
    _dim_poc = _ocp_problem.numberOfPointConstraints(_i_phase);
    _dim_ic = _ocp_problem.numberOfIntConstraints(_i_phase);
    _dim_q  = _dim_fo + _dim_pc + _dim_poc; // NLP constraints in all mesh intervals are dynamic equations plus path constraints
    _dim_bc = _ocp_problem.numberOfBoundaryConditions(_i_phase);
    _dim_p  = _ocp_problem.numberOfParameters(_i_phase); // nlp parameters are ocp parameters
    //_dim_p  = _dim_p;

    // scaling
    setupScaling();

    // nlp constrain jacobian related variables

    setupForNlpTargetGradient();
    setupForNlpConstraintsJacobianMatrixes();

    setupForNlpHessianMatrixes();

    calculateWorkForThreads();

}

void MidpointOcp2NlpSinglePhase::setupScaling() {
    OcpScaling const & scaling = _ocp_problem.getScaling();

    MAVERICK_DEBUG_ASSERT(scaling.getStatesControlScaling(_i_phase).size() == _dim_y,
                          std::string(__FILE__) + ": " + std::to_string(__LINE__) +  " wrong state and control scaling size.\n");
    _p_scaling_y = new real[_dim_y];
    copyVectorTo(scaling.getStatesControlScaling(_i_phase).data(), _p_scaling_y, _dim_y);
    _p_inv_scaling_y = new real[_dim_y];
    writeInverseVectorTo(_p_scaling_y, _p_inv_scaling_y, _dim_y);

    MAVERICK_DEBUG_ASSERT(scaling.getAlgebraicStatesControlScaling(_i_phase).size() == _dim_ay,
                          std::string(__FILE__) + ": " + std::to_string(__LINE__) +  " wrong algebraic state and control scaling size.\n");
    _p_scaling_ay = new real[_dim_ay];
    copyVectorTo(scaling.getAlgebraicStatesControlScaling(_i_phase).data(), _p_scaling_ay, _dim_ay);
    _p_inv_scaling_ay = new real[_dim_ay];
    writeInverseVectorTo(_p_scaling_ay, _p_inv_scaling_ay, _dim_ay);

    MAVERICK_DEBUG_ASSERT(scaling.getParamsScaling(_i_phase).size() == _dim_p,
                          std::string(__FILE__) + ": " + std::to_string(__LINE__) +  " wrong parameters scaling size.\n");
    _p_scaling_r = new real[_dim_p];
    copyVectorTo(scaling.getParamsScaling(_i_phase).data(), _p_scaling_r, _dim_p);
    _p_inv_scaling_r = new real[_dim_p];
    writeInverseVectorTo(_p_scaling_r, _p_inv_scaling_r, _dim_p);

    _inv_scaling_target = 1.0 / scaling.getTargetScaling(_i_phase);

    MAVERICK_DEBUG_ASSERT(scaling.getFoEqnsScaling(_i_phase).size() == _dim_fo,
                          std::string(__FILE__) + ": " + std::to_string(__LINE__) +  " wrong f.o. equations scaling size.\n");
    _p_inv_scaling_fo_eqns_global = new real[_dim_fo];
    writeInverseVectorTo(scaling.getFoEqnsScaling(_i_phase).data(), _p_inv_scaling_fo_eqns_global, _dim_fo);

    MAVERICK_DEBUG_ASSERT(scaling.getPointConstraintsScaling(_i_phase).size() == _dim_poc,
                          std::string(__FILE__) + ": " + std::to_string(__LINE__) +  " wrong point constraints scaling size.\n");
    _p_inv_scaling_point_constr_global = new real[_dim_poc];
    writeInverseVectorTo(scaling.getPointConstraintsScaling(_i_phase).data(), _p_inv_scaling_point_constr_global, _dim_poc);

    MAVERICK_DEBUG_ASSERT(scaling.getPathConstraintsScaling(_i_phase).size() == _dim_pc,
                          std::string(__FILE__) + ": " + std::to_string(__LINE__) +  " wrong path constraints scaling size.\n");
    _p_inv_scaling_path_constr_global = new real[_dim_pc];
    writeInverseVectorTo(scaling.getPathConstraintsScaling(_i_phase).data(), _p_inv_scaling_path_constr_global, _dim_pc);

    MAVERICK_DEBUG_ASSERT(scaling.getIntConstraintsScaling(_i_phase).size() == _dim_ic,
                          std::string(__FILE__) + ": " + std::to_string(__LINE__) +  " wrong integral constraints scaling size.\n");
    _p_inv_scaling_int_constr        = new real[_dim_ic];
    writeInverseVectorTo(scaling.getIntConstraintsScaling(_i_phase).data(), _p_inv_scaling_int_constr, _dim_ic);

    MAVERICK_DEBUG_ASSERT(scaling.getBoundaryConditionsScaling(_i_phase).size() == _dim_bc,
                          std::string(__FILE__) + ": " + std::to_string(__LINE__) +  " wrong boundary conditions constraints scaling size.\n");
    _p_inv_scaling_bcs               = new real[_dim_bc];
    writeInverseVectorTo(scaling.getBoundaryConditionsScaling(_i_phase).data(), _p_inv_scaling_bcs, _dim_bc);

    // check for corrections
    OcpScalingOptions const & options = scaling.getScalingOptions();
    integer const N = _p_mesh->getNumberOfIntervals();
    real const Z = _p_mesh->getFinalZeta()-_p_mesh->getInitialZeta();

    if (options.multiply_lagrange_by_n && _is_target_lagrange)
        _inv_scaling_target = _inv_scaling_target * N;

    if (options.multiply_int_constr_by_n   )
        multiplyVectorBy(_p_inv_scaling_int_constr, N, _dim_ic);

    _multiply_foeqns_by_dz = options.multiply_foeqns_by_dz;
    if (options.multiply_foeqns_by_n       )
        multiplyVectorBy(_p_inv_scaling_fo_eqns_global, N, _dim_fo);
    if (options.divide_foeqns_by_z         )
        multiplyVectorBy(_p_inv_scaling_fo_eqns_global, 1.0/Z, _dim_fo);
    _multiply_path_constr_by_dz = options.multiply_path_constr_by_dz;
    if (options.multiply_path_constr_by_n  )
        multiplyVectorBy(_p_inv_scaling_path_constr_global, N, _dim_pc);
    if (options.divide_path_constr_by_z    )
        multiplyVectorBy(_p_inv_scaling_path_constr_global, 1/Z, _dim_pc);
    _multiply_point_constr_by_dz = options.multiply_point_constr_by_dz;
    if (options.multiply_point_constr_by_n  )
        multiplyVectorBy(_p_inv_scaling_point_constr_global, N, _dim_poc);
    if (options.divide_point_constr_by_z    )
        multiplyVectorBy(_p_inv_scaling_point_constr_global, 1/Z, _dim_poc);

    if (options.divide_mayer_by_n && !_is_target_lagrange )
        _inv_scaling_target = _inv_scaling_target / N;
    if (options.divide_bcs_by_n            )
        multiplyVectorBy(_p_inv_scaling_bcs, N, _dim_bc);



}

integer MidpointOcp2NlpSinglePhase::getNlpSize() const {
    return (_dim_y + _dim_ay) * _p_mesh->getNumberOfIntervals() + _dim_y + _dim_p;
}

integer MidpointOcp2NlpSinglePhase::getNlpConstraintsSize() const {
    return ( _dim_q ) * _p_mesh->getNumberOfIntervals() + _dim_poc + _dim_bc + _dim_ic;
}

integer MidpointOcp2NlpSinglePhase::getNlpTargetGradientNnz() const {
    if (_is_gradient_dense)
        return getNlpSize();

    return _target_j_xi_nnz - _lagrange_target_j_y_nnz + _target_j_xf_nnz + (_p_mesh->getNumberOfIntervals()) * (_lagrange_target_j_y_nnz + _lagrange_target_j_ay_nnz ) + _target_j_p_nnz;
}

integer MidpointOcp2NlpSinglePhase::getNlpConstraintsJacobianNnz() const {
    return getNlpConstraintsJacobainMatrixYNnz() * _p_mesh->getNumberOfIntervals()
    + _point_constr_j_y_nnz + _point_constr_j_p_nnz
    + getNlpConstraintsJacobainMatrixFNnz()
    + (_int_constr_j_y_nnz + _int_constr_j_ay_nnz ) * _p_mesh->getNumberOfIntervals() + _int_constr_j_y_nnz +  _int_constr_j_p_nnz;
}

integer MidpointOcp2NlpSinglePhase::getNlpHessianNnz() const {
    return getNlpHessianFirstColumnBlockNnz() + getNlpHessianLeftColumnBlockNnz() * (_p_mesh->getNumberOfIntervals() - 1 )  + getNlpHessianLastColumnBlockNnz() + _p_mesh->getNumberOfIntervals() * getNlpHessianCentreColumnBlockNnz() +  _hess_p_p_lower_mat_nnz;
};

void MidpointOcp2NlpSinglePhase::clearThreadJobs() {
    // stop threads and detach them
    for (u_integer i_thread = 0; i_thread < _actual_num_threads; i_thread++) {
        ThreadJob & th_job = _thread_jobs[i_thread];
        {
            unique_lock<mutex> lock(*(th_job.job_mutex));
            th_job.kill = true;
        }
        th_job.cond_var->notify_one();
        th_job.th->join();
        delete th_job.th;
        delete th_job.cond_var;
        delete th_job.job_mutex;
    }
    delete[] _thread_jobs;
    _thread_jobs = nullptr;
}
void MidpointOcp2NlpSinglePhase::calculateWorkForThreads() {
    // clear previous thread jobs
    clearThreadJobs();

    // CALCULATE NUMBER OF THREADS TO USE
    integer const expected_num_threads = (integer) _th_affinity.size();
    integer const min_mesh_intervals_per_thread = ceil( _min_nlp_vars_per_thread / real(_dim_y) );

    integer mesh_intervals_per_thread = ceil(_p_mesh->getNumberOfIntervals() / real(expected_num_threads));

    if (mesh_intervals_per_thread < min_mesh_intervals_per_thread)
        mesh_intervals_per_thread = min_mesh_intervals_per_thread;

    _actual_num_threads = 0;
    vector<integer> thread_mesh_intervals = {0}; // array containing the mesh point extrema for each thread job
    thread_mesh_intervals.reserve(expected_num_threads+1);

    integer current_mesh_point = 0;
    integer const num_mesh_intervals = _p_mesh->getNumberOfIntervals();

    for (u_integer i_thread = 0; i_thread < expected_num_threads; i_thread++) {
        _actual_num_threads ++;
        if ((current_mesh_point + mesh_intervals_per_thread) < num_mesh_intervals ) {
            current_mesh_point += mesh_intervals_per_thread;
            thread_mesh_intervals.push_back(current_mesh_point);
        } else {
            current_mesh_point = num_mesh_intervals;
            thread_mesh_intervals.push_back(current_mesh_point);
            break;
        }
    }

    // now build the thread jobs
    _thread_jobs = new ThreadJob[_actual_num_threads];
    for (u_integer i_thread = 0; i_thread < _actual_num_threads; i_thread++) {
        ThreadJob & th_job = _thread_jobs[i_thread];

        // init pointers
        th_job.cond_var  = new condition_variable();
        th_job.job_mutex = new mutex();

        // set the mesh extrema
        th_job.start_mesh_interval = thread_mesh_intervals[i_thread];
        th_job.end_mesh_interval = thread_mesh_intervals[i_thread+1];
        th_job.affinity = _th_affinity[i_thread];

        // set the actual thread
        th_job.th = (new thread( //unique_ptr<thread>
            [& th_job](){
                auto check_predicate = [& th_job]() { return th_job.job_todo || th_job.kill; };
                while (true) {
                    {
                        unique_lock<mutex> lock(*(th_job.job_mutex));
                        th_job.cond_var->wait(lock, check_predicate);
                    }
                    if (th_job.kill) return; //exit from thread
                    //perform the job
                    th_job.job();
                    { // notify the job is done
                        unique_lock<mutex> lock(*(th_job.job_mutex));
                        th_job.job_todo = false;
                    }
                    th_job.cond_var->notify_one(); // notify
                }
            }
        ));
    }

#ifndef __linux__
    for (u_integer i=0; i<_th_affinity.size(); i++)
        _th_affinity[i] = {};  // Only in Linux thread affinity is supported

    for (ThreadJob & th_job : _thread_jobs)
        th_job.affinity = {};
#endif

    // now actually set the affinity of the threads
#ifdef __linux__
    for (u_integer i_thread = 0; i_thread < _actual_num_threads; i_thread++) {
        ThreadJob & th_job = _thread_jobs[i_thread];
        auto const & c_aff = th_job.affinity;
        if (c_aff.size()>0) {
            cpu_set_t cpuset;
            CPU_ZERO(&cpuset);
            for (u_integer x : c_aff)
                CPU_SET(x, &cpuset);
            pthread_setaffinity_np(th_job.th->native_handle(), sizeof(cpu_set_t), &cpuset);
        }
    }
#endif

    MAVERICK_DEBUG_ASSERT(current_mesh_point==num_mesh_intervals, "MidpointOcp2NlpSinglePhase: not all mesh points are spanned by threads")
    MAVERICK_DEBUG_ASSERT(_actual_num_threads==(thread_mesh_intervals.size()-1), "MidpointOcp2NlpSinglePhase: number of threads does not match _thread_mesh_intervals.size()")
}

void MidpointOcp2NlpSinglePhase::setThreadsAffinity( threads_affinity const & th_affinity ) {
    _th_affinity = th_affinity;
    calculateWorkForThreads();
}

void MidpointOcp2NlpSinglePhase::getNlpBounds( real lower_bounds[], real upper_bounds[], integer const n ) const {

    MAVERICK_ASSERT( n == getNlpSize(), "MidpointOcp2NlpSinglePhase::getNlpBounds: wrong nlp size.");

    MaverickSingleton const & _maverick = MaverickSingleton::getInstance();

    real * current_lower_bounds = lower_bounds;
    real * current_upper_bounds = upper_bounds;

    //now write the bounds at each mesh point
    for (integer c_mesh_point =0; c_mesh_point < _p_mesh->getNumberOfDiscretisationPoints(); c_mesh_point++ ) {
        real const current_zeta = _p_mesh->getZeta(c_mesh_point);

        //write the bounds for the states and controls
        real ocp_state_control_lower_bounds[_dim_xu];
        real ocp_state_control_upper_bounds[_dim_xu];
        _ocp_problem.getStatesControlsBounds(_i_phase, current_zeta, ocp_state_control_lower_bounds, ocp_state_control_upper_bounds);
        multiplyAndCopyVectorTo(ocp_state_control_lower_bounds, current_lower_bounds, _p_inv_scaling_y, _dim_xu);
        multiplyAndCopyVectorTo(ocp_state_control_upper_bounds, current_upper_bounds, _p_inv_scaling_y, _dim_xu);
        current_lower_bounds += _dim_xu;
        current_upper_bounds += _dim_xu;

        // check that the bounds are consistent
        for (integer i=0; i<_dim_x; i++) {
            if (ocp_state_control_upper_bounds[i] < ocp_state_control_lower_bounds[i]) {
                string message = "Upper bound for state " + _ocp_problem.stateName(_i_phase, i) + ", phase " + std::to_string(_i_phase) + ", mesh value " + std::to_string(current_zeta) + ", is lower than the lower bound. Problem will be infeasable.\n";
                _maverick.Log(InfoLevel::info_level_warning, message);
            }
        }
        for (integer i=_dim_x+1; i<_dim_xu; i++) {
            if (ocp_state_control_upper_bounds[i] < ocp_state_control_lower_bounds[i]) {
                string message = "Upper bound for control " + _ocp_problem.controlName(_i_phase, i - _dim_x) + ", phase " + std::to_string(_i_phase) + ", mesh value " + std::to_string(current_zeta) + ", is lower than the lower bound. Problem will be infeasable.\n";
                _maverick.Log(InfoLevel::info_level_warning, message);
            }
        }

        //write also algebraic states
        if (c_mesh_point < _p_mesh->getNumberOfIntervals() ) {
            real ocp_algebraic_state_control_lower_bounds[_dim_xu];
            real ocp_algebraic_state_control_upper_bounds[_dim_xu];
            _ocp_problem.getAlgebraicStatesControlsBounds(_i_phase, current_zeta, ocp_algebraic_state_control_lower_bounds, ocp_algebraic_state_control_upper_bounds);
            multiplyAndCopyVectorTo(ocp_algebraic_state_control_lower_bounds, current_lower_bounds, _p_inv_scaling_ay, _dim_axu);
            multiplyAndCopyVectorTo(ocp_algebraic_state_control_upper_bounds, current_upper_bounds, _p_inv_scaling_ay, _dim_axu);
            current_lower_bounds += _dim_axu;
            current_upper_bounds += _dim_axu;

            // check that the bounds are consistent
            for (integer i=0; i<_dim_ax; i++) {
                if (ocp_algebraic_state_control_upper_bounds[i] < ocp_algebraic_state_control_lower_bounds[i]) {
                    string message = "Upper bound for algebraic state " + _ocp_problem.algebraicStateName(_i_phase, i) + ", phase " + std::to_string(_i_phase) + ", mesh value " + std::to_string(current_zeta) + ", is lower than the lower bound. Problem will be infeasable.\n";
                    _maverick.Log(InfoLevel::info_level_warning, message);
                }
            }
            for (integer i=_dim_ax+1; i<_dim_axu; i++) {
                if (ocp_algebraic_state_control_upper_bounds[i] < ocp_algebraic_state_control_lower_bounds[i]) {
                    string message = "Upper bound for control " + _ocp_problem.algebraicControlName(_i_phase, i - _dim_x) + ", phase " + std::to_string(_i_phase) + ", mesh value " + std::to_string(current_zeta) + ", is lower than the lower bound. Problem will be infeasable.\n";
                    _maverick.Log(InfoLevel::info_level_warning, message);
                }
            }
        }
    }

    //write the bounds of the parameters
    real ocp_params_lower_bounds[_dim_p];
    real ocp_params_upper_bounds[_dim_p];
    _ocp_problem.getParametersBounds(_i_phase, ocp_params_lower_bounds, ocp_params_upper_bounds);
    multiplyAndCopyVectorTo(ocp_params_lower_bounds, current_lower_bounds, _p_inv_scaling_r, _dim_p);
    multiplyAndCopyVectorTo(ocp_params_upper_bounds, current_upper_bounds, _p_inv_scaling_r, _dim_p);
    current_lower_bounds += _dim_p;
    current_upper_bounds += _dim_p;

    // check that the bounds are consistent
    for (integer i=0; i<_dim_p; i++) {
        if (ocp_params_upper_bounds[i] < ocp_params_lower_bounds[i]) {
            string message = "Upper bound for parameter " + _ocp_problem.parameterName(_i_phase, i) + ", phase " + std::to_string(_i_phase) + ", is lower than the lower bound. Problem will be infeasable.\n";
            _maverick.Log(InfoLevel::info_level_warning, message);
        }
    }

    MAVERICK_DEBUG_ASSERT(current_lower_bounds == lower_bounds + getNlpSize(), "MidpointOcp2NlpSinglePhase::getNlpBounds: not all nlp lower bounds have been written.")
    MAVERICK_DEBUG_ASSERT(current_upper_bounds == upper_bounds + getNlpSize(), "MidpointOcp2NlpSinglePhase::getNlpBounds: not all nlp upper bounds have been written.")
}

void MidpointOcp2NlpSinglePhase::getNlpConstraintsBounds( real lower_bounds[], real upper_bounds[], integer const n ) const {

    MAVERICK_ASSERT( n == getNlpConstraintsSize(), "MidpointOcp2NlpSinglePhase::getNlpConstraintsBounds: wrong nlp constraints size.\n");

    MaverickSingleton const & _maverick = MaverickSingleton::getInstance();

    real * current_lower_bounds = lower_bounds;
    real * current_upper_bounds = upper_bounds;

    //now write the bounds at each mesh point
    for (integer c_mesh_interval =0; c_mesh_interval < _p_mesh->getNumberOfIntervals(); c_mesh_interval++ ) {
        real const current_zeta = _p_mesh->getZeta(c_mesh_interval);
        real const d_zeta = _p_mesh->getDz(c_mesh_interval);
        real const d_zeta_dual = _p_mesh->getDzDual(c_mesh_interval);

        //write the bounds (zero) for the equations
        writeRealToVector(current_lower_bounds, 0, _dim_fo);
        writeRealToVector(current_upper_bounds, 0, _dim_fo);
        current_lower_bounds += _dim_fo;
        current_upper_bounds += _dim_fo;

        // write bounds for the path constraints
        real ocp_path_constraints_lower_bounds[_dim_pc];
        real ocp_path_constraints_upper_bounds[_dim_pc];
        real p_inv_scaling_path_constr[_dim_pc];
        if (_multiply_path_constr_by_dz)
            multiplyAndCopyVectorTo(_p_inv_scaling_path_constr_global, p_inv_scaling_path_constr, d_zeta, _dim_pc);
        else
            copyVectorTo(_p_inv_scaling_path_constr_global, p_inv_scaling_path_constr, _dim_pc);

        _ocp_problem.getPathConstraintsBounds(_i_phase, current_zeta, ocp_path_constraints_lower_bounds, ocp_path_constraints_upper_bounds);
        multiplyAndCopyVectorTo(ocp_path_constraints_lower_bounds, current_lower_bounds, p_inv_scaling_path_constr, _dim_pc);
        multiplyAndCopyVectorTo(ocp_path_constraints_upper_bounds, current_upper_bounds, p_inv_scaling_path_constr, _dim_pc);
        current_lower_bounds += _dim_pc;
        current_upper_bounds += _dim_pc;

        // check that the bounds are consistent
        for (integer i=0; i<_dim_pc; i++) {
            if (ocp_path_constraints_upper_bounds[i] < ocp_path_constraints_lower_bounds[i]) {
                string message = "Upper bound for path constraint at index " + std::to_string(i) + ", phase " + std::to_string(_i_phase) + ", mesh value " + std::to_string(current_zeta) + ", is lower than the lower bound. Problem will be infeasable.\n";
                _maverick.Log(InfoLevel::info_level_warning, message);
            }
        }

        //write the bounds for the point constraints
        real ocp_point_constraints_lower_bounds[_dim_poc];
        real ocp_point_constraints_upper_bounds[_dim_poc];
        real p_inv_scaling_point_constr[_dim_poc];
        if (_multiply_point_constr_by_dz)
            multiplyAndCopyVectorTo(_p_inv_scaling_point_constr_global, p_inv_scaling_point_constr, d_zeta_dual, _dim_poc);
        else
            copyVectorTo(_p_inv_scaling_point_constr_global, p_inv_scaling_point_constr, _dim_poc);

        _ocp_problem.getPointConstraintsBounds(_i_phase, current_zeta, ocp_point_constraints_lower_bounds, ocp_point_constraints_upper_bounds);
        multiplyAndCopyVectorTo(ocp_point_constraints_lower_bounds, current_lower_bounds, p_inv_scaling_point_constr, _dim_poc);
        multiplyAndCopyVectorTo(ocp_point_constraints_upper_bounds, current_upper_bounds, p_inv_scaling_point_constr, _dim_poc);
        current_lower_bounds += _dim_poc;
        current_upper_bounds += _dim_poc;


        // check that the bounds are consistent
        for (integer i=0; i<_dim_poc; i++) {
            if (ocp_point_constraints_upper_bounds[i] < ocp_point_constraints_lower_bounds[i]) {
                string message = "Upper bound for point constraint at index " + std::to_string(i) + ", phase " + std::to_string(_i_phase) + ", mesh value " + std::to_string(current_zeta) + ", is lower than the lower bound. Problem will be infeasable.\n";
                _maverick.Log(InfoLevel::info_level_warning, message);
            }
        }
    }

    real const initial_zeta = _p_mesh->getZeta(0);
    real const final_zeta = _p_mesh->getZeta(_p_mesh->getNumberOfDiscretisationPoints()-1);

    //write the bounds of the point constraints for the last mesh point
    real const d_zeta_dual = _p_mesh->getDzDual(_p_mesh->getNumberOfIntervals());
    real ocp_point_constraints_lower_bounds[_dim_poc];
    real ocp_point_constraints_upper_bounds[_dim_poc];
    real p_inv_scaling_point_constr[_dim_poc];
    if (_multiply_point_constr_by_dz)
        multiplyAndCopyVectorTo(_p_inv_scaling_point_constr_global, p_inv_scaling_point_constr, d_zeta_dual, _dim_poc);
    else
        copyVectorTo(_p_inv_scaling_point_constr_global, p_inv_scaling_point_constr, _dim_poc);

    _ocp_problem.getPointConstraintsBounds(_i_phase, final_zeta, ocp_point_constraints_lower_bounds, ocp_point_constraints_upper_bounds);
    multiplyAndCopyVectorTo(ocp_point_constraints_lower_bounds, current_lower_bounds, p_inv_scaling_point_constr, _dim_poc);
    multiplyAndCopyVectorTo(ocp_point_constraints_upper_bounds, current_upper_bounds, p_inv_scaling_point_constr, _dim_poc);
    current_lower_bounds += _dim_poc;
    current_upper_bounds += _dim_poc;

    // check that the bounds are consistent
    for (integer i=0; i<_dim_poc; i++) {
        if (ocp_point_constraints_upper_bounds[i] < ocp_point_constraints_lower_bounds[i]) {
            string message = "Upper bound for point constraint at index " + std::to_string(i) + ", phase " + std::to_string(_i_phase) + ", mesh value " + std::to_string(final_zeta) + ", is lower than the lower bound. Problem will be infeasable.\n";
            _maverick.Log(InfoLevel::info_level_warning, message);
        }
    }

    //write the bounds for the boundary conditions
    real ocp_bcs_lower_bounds[_dim_bc];
    real ocp_bcs_upper_bounds[_dim_bc];
    _ocp_problem.getBoundaryConditionsBounds(_i_phase, initial_zeta, final_zeta, ocp_bcs_lower_bounds, ocp_bcs_upper_bounds);
    multiplyAndCopyVectorTo(ocp_bcs_lower_bounds, current_lower_bounds, _p_inv_scaling_bcs, _dim_bc);
    multiplyAndCopyVectorTo(ocp_bcs_upper_bounds, current_upper_bounds, _p_inv_scaling_bcs, _dim_bc);
    current_lower_bounds += _dim_bc;
    current_upper_bounds += _dim_bc;

    // check that the bounds are consistent
    for (integer i=0; i<_dim_bc; i++) {
        if (ocp_bcs_upper_bounds[i] < ocp_bcs_lower_bounds[i]) {
            string message = "Upper bound for boundary condition at index " + std::to_string(i) + ", phase " + std::to_string(_i_phase) + ", is lower than the lower bound. Problem will be infeasable.\n";
            _maverick.Log(InfoLevel::info_level_warning, message);
        }
    }

    //write the bounds for the integral constraints
    real ocp_int_constr_lower_bounds[_dim_ic];
    real ocp_int_constr_upper_bounds[_dim_ic];
    _ocp_problem.getIntConstraintsBounds(_i_phase, initial_zeta, final_zeta, ocp_int_constr_lower_bounds, ocp_int_constr_upper_bounds);
    multiplyAndCopyVectorTo(ocp_int_constr_lower_bounds, current_lower_bounds, _p_inv_scaling_int_constr, _dim_ic);
    multiplyAndCopyVectorTo(ocp_int_constr_upper_bounds, current_upper_bounds, _p_inv_scaling_int_constr, _dim_ic);
    current_lower_bounds += _dim_ic;
    current_upper_bounds += _dim_ic;

    // check that the bounds are consistent
    for (integer i=0; i<_dim_ic; i++) {
        if (ocp_int_constr_upper_bounds[i] < ocp_int_constr_lower_bounds[i]) {
            string message = "Upper bound for integral constraint at index " + std::to_string(i) + ", phase " + std::to_string(_i_phase) + ", is lower than the lower bound. Problem will be infeasable.\n";
            _maverick.Log(InfoLevel::info_level_warning, message);
        }
    }

    MAVERICK_DEBUG_ASSERT(current_lower_bounds == lower_bounds + getNlpConstraintsSize(), "MidpointOcp2NlpSinglePhase::getNlpConstraintsBounds: not all constraints lower bounds have been written.")
    MAVERICK_DEBUG_ASSERT(current_upper_bounds == upper_bounds + getNlpConstraintsSize(), "MidpointOcp2NlpSinglePhase::getNlpConstraintsBounds: not all constraints upper bounds have been written.")
}

threads_affinity MidpointOcp2NlpSinglePhase::getActualThreadsAffinityUsed(integer const i_phase) const {
    threads_affinity out = {};
    for (u_integer i_thread = 0; i_thread < _actual_num_threads; i_thread++) {
        out.push_back(_thread_jobs[i_thread].affinity);
    }
    return out;
}

void MidpointOcp2NlpSinglePhase::setIsTargetLagrangeFromGuess( Nlp const & nlp_guess ) {
    std::unique_ptr<MidpointOcpSolutionSinglePhase> sol_single_phase = translateNlp2MidpointOcpSolution( nlp_guess );
    MidpointOcpSolution sol;
    sol.setSolutionAtPhase(_i_phase, *sol_single_phase);
    setIsTargetLagrangeFromGuess( sol );
}

void MidpointOcp2NlpSinglePhase::setIsTargetLagrangeFromGuess( OcpGuess const & ocp_guess ) {

    real xu[_dim_xu];
    real xu_left[_dim_xu];
    real xu_right[_dim_xu];
    real dxu[_dim_xu];
    real axu[_dim_axu];
    real p[_dim_p];

    real const zeta_f = _p_mesh->getFinalZeta();
    real const zeta_i = _p_mesh->getInitialZeta();

    ocp_guess.eval(_i_phase, zeta_i, zeta_f, _dim_p, p, nullptr, nullptr, 0, nullptr, 0, nullptr);

    // evaluate the lagrange target
    real lagrange = 0;
    for (integer c_mesh_interval = 0; c_mesh_interval < _p_mesh->getNumberOfIntervals(); c_mesh_interval++) {
        real const zeta = _p_mesh->getZetaCenter(c_mesh_interval);
        ocp_guess.evalAtMesh(_i_phase, zeta, _dim_xu, xu, nullptr, nullptr, _dim_axu, axu, nullptr, nullptr, 0, nullptr, 0, nullptr, 0, nullptr);

        real const zeta_left = _p_mesh->getZetaLeft(c_mesh_interval);
        real const zeta_right = _p_mesh->getZetaRight(c_mesh_interval);
        ocp_guess.evalAtMesh(_i_phase, zeta_left, _dim_xu, xu_left, nullptr, nullptr, 0, nullptr, nullptr, nullptr, 0, nullptr, 0, nullptr, 0, nullptr);
        ocp_guess.evalAtMesh(_i_phase, zeta_right, _dim_xu, xu_right, nullptr, nullptr, 0, nullptr, nullptr, nullptr, 0, nullptr, 0, nullptr, 0, nullptr);
        computeTpzDerivativeWithoutScaling(xu_left, xu_right, dxu, 1.0/(zeta_right-zeta_left), _dim_xu);

        real c_lagrange;
        _ocp_problem.lagrange(_i_phase, xu, dxu, axu, p, zeta, c_lagrange);

        lagrange += c_lagrange * (zeta_right - zeta_left);
    }

    // now evaluate the mayer target
    ocp_guess.evalAtMesh(_i_phase, zeta_i, _dim_xu, xu_left, nullptr, nullptr, 0, nullptr, nullptr, nullptr, 0, nullptr, 0, nullptr, 0, nullptr);
    ocp_guess.evalAtMesh(_i_phase, zeta_f, _dim_xu, xu_right, nullptr, nullptr, 0, nullptr, nullptr, nullptr, 0, nullptr, 0, nullptr, 0, nullptr);
    real mayer;
    _ocp_problem.mayer(_i_phase, xu_left, xu_right, p, mayer);

    _is_target_lagrange = abs(lagrange) > abs(mayer);
}
