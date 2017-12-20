/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef MAVERICK_DEFS_HH
#define MAVERICK_DEFS_HH

#include <stdexcept>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>

namespace Maverick {

    typedef double                                          real;
    typedef int                                             integer;
    typedef int64_t                                         long_integer;
    typedef unsigned int                                    u_integer;
    typedef uint64_t                                        u_long_integer;
    typedef std::size_t                                     size;
    typedef std::map<std::string,std::vector<real>>         real_table;

    typedef std::vector<real>                               vec_1d_real;
    typedef std::vector<std::vector<real>>                  vec_2d_real;
    typedef std::vector<std::vector<std::vector<real>>>     vec_3d_real;

    typedef std::vector<integer>                            vec_1d_integer;
    typedef std::vector<bool>                               vec_1d_bool;

    typedef std::vector<u_integer>                          single_thread_affinity;
    typedef std::vector<single_thread_affinity>             threads_affinity;

    enum InfoLevel {
        info_level_none = 0,
        info_level_warning,
        info_level_few,
        info_level_normal,
        info_level_more,
        info_level_verbose,
        info_level_very_verbose,
    };

    enum SolverStartCode {
        cold_start = 0,
        warm_start,
        warm_start_with_multipliers,
    };

    enum SolverReturnStatus {
        solution_not_computed           = -1,
        converged_optimal_solution      = 0,
        converged_accetable_level       = 2,
        number_of_iterations_exceeded   = 3,
        infeasable_problem_detected     = 10,
        not_converged                   = 100,
        problem_detected                = -10,
        crashed                         = -100,
        crashed_before_start            = -101,
        crashed_during_iterations       = -102
    };

    enum EquationIntegratorType {
        integrator_tensolve = 0,
        integrator_ipopt
    };


    enum DiscretisationType {
        midpoint = 0
    };

    #define MAVERICK_ERROR(MSG) { \
        std::ostringstream ost ; ost << MSG << "\n" ; \
        throw std::runtime_error(ost.str()) ; \
    }

    #ifndef MAVERICK_ASSERT
        #define MAVERICK_ASSERT(COND,MSG) if ( !(COND) ) MAVERICK_ERROR( MSG )
    #endif

}

#endif
