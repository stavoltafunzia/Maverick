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
#include "MaverickCore/MaverickCDefinitions.h"

#define MAVERICK_RESTRICT

namespace Maverick {

  using real = maverick_real;
  using integer = maverick_int;
  using u_integer = unsigned int;
  using size = std::size_t;
  using real_table = std::map<std::string, std::vector<real>>;

  using vec_1d_real = std::vector<real>;
  using vec_2d_real = std::vector<std::vector<real>> ;
  using vec_3d_real = std::vector<std::vector<std::vector<real>>>;

  using vec_1d_integer = std::vector<integer>;
  using vec_1d_bool = std::vector<bool>;

  using single_thread_affinity = std::vector<u_integer>;
  using threads_affinity = std::vector<single_thread_affinity>;

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

  enum SolverExitCode {
    solution_not_computed = -1,
    converged_optimal_solution = 0,
    converged_accetable_level = 2,
    number_of_iterations_exceeded = 3,
    infeasable_problem_detected = 10,
    not_converged = 100,
    problem_detected = -10,
    crashed = -100,
    crashed_before_start = -101,
    crashed_during_iterations = -102
  };

}

#endif
