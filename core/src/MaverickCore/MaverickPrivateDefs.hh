/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef MAVERICK_PRIVATE_DEFS_HH
#define MAVERICK_PRIVATE_DEFS_HH

#include "Eigen/SparseCore"

#ifdef MAVERICK_DEBUG
    #ifndef MAVERICK_DEBUG_ASSERT
        #define MAVERICK_DEBUG_ASSERT(COND,MSG) if ( !(COND) ) MAVERICK_ERROR( MSG )
    #endif
#else
    #ifndef MAVERICK_DEBUG_ASSERT
        #define MAVERICK_DEBUG_ASSERT(COND,MSG)
    #endif
#endif

#ifndef MAVERICK_SKIP_CHECKS
    #ifndef MAVERICK_SKIPABLE_ASSERT
        #define MAVERICK_SKIPABLE_ASSERT(COND,MSG) if ( !(COND) ) MAVERICK_ERROR( MSG )
    #endif
#else
    #ifndef MAVERICK_SKIPABLE_ASSERT
        #define MAVERICK_SKIPABLE_ASSERT(COND,MSG)
    #endif
#endif

#if defined __linux__
    #define SHARED_LIB_EXTENSION "so"
#endif

#if  defined __APPLE__
    #define SHARED_LIB_EXTENSION "dylib"
#endif

#include <string>
#include "MaverickDefinitions.hh"

namespace Maverick {

    typedef Eigen::SparseMatrix<real>                                              SparseMatrix;
    typedef Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>   DenseMatrix;
    typedef Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>   DenseMatrixRowMajor;
    typedef Eigen::Matrix<real, Eigen::Dynamic, 1>                                 DenseColumnVector;
    typedef Eigen::VectorXd                                                        Eigen_vec_1d_real;
    typedef Eigen::VectorXi                                                        Eigen_vec_1d_int;
    typedef unsigned short ComputationCode;

    enum NlpSolverId {
        nlp_id_none = 0,
        nlp_id_ipopt = 1,
        nlp_id_whorp = 2
    };

    enum ExternalGuessType {
        ext_guess_none = 0,
        ext_guess_object,
        ext_guess_tables
    };

    enum MeshRefinementStart {
        standad_start = 0,
        force_cold_start,
        force_warm_start,
        force_warm_multiplier_start
    };

    struct StreamChars {
        static std::string const tab;
        static std::string const new_line;
        static std::string const comment_begin;
        static std::string const separator;
    };

    struct SolverSettings {
        SolverStartCode start_mode = cold_start;
        integer max_iterations = 300;
        void const * nlp_guess_ptr = nullptr;
        bool save_mesh_history = false;
        MeshRefinementStart refinement_start = standad_start;
        EquationIntegratorType integrator_type = integrator_tensolve;
        bool skip_mesh_error_calculus = false;
    };

    struct SolverStatus {
        SolverReturnStatus return_status = solution_not_computed;
        bool has_mesh_changed_since_last_solution = true;
        bool is_last_solution_saved = false;
        bool is_one_solution_computed = false;
        bool has_setup_mesh = false;
        bool has_setup_solver = false;
        bool must_setup_ocp2nlp = true;
        ExternalGuessType ext_guess_type = ext_guess_none;
    };

    /* OPTIONS first default behaviour (all jacobians = 1)
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

    /* OPTIONS second default behaviour (all jacobians = 1/N )
        bool multiply_lagrange_by_n      = false;
        bool multiply_int_constr_by_n    = false;
        bool multiply_foeqns_by_dz       = true;
        bool multiply_foeqns_by_n        = false;
        bool divide_foeqns_by_z          = true;
        bool multiply_path_constr_by_dz  = true;
        bool multiply_path_constr_by_n   = false;
        bool divide_path_constr_by_z     = true;
        bool multiply_point_constr_by_dz = true;
        bool multiply_point_constr_by_n  = false;
        bool divide_point_constr_by_z    = true;
        bool divide_mayer_by_n           = true;
        bool divide_bcs_by_n             = true;
    */
}

#endif
