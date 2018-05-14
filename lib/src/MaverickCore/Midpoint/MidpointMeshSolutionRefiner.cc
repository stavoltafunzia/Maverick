#include "MaverickCore/Midpoint/MidpointMeshSolutionRefiner.hh"
#include "MaverickCore/Midpoint/MidpointIntegrator.hh"
#include "MaverickCore/MaverickSingleton.hh"
#include "MaverickCore/MaverickPrivateDefinitions.hh"
#include <iomanip>
#include <thread>

#define SEP "   "
using namespace std;

namespace Maverick {

  MidpointMeshSolutionRefiner::MidpointMeshSolutionRefiner(MaverickOcp const &ocp_problem,
                                                           OcpScaling const &ocp_scaling) : MeshSolutionRefiner(
      ocp_problem, ocp_scaling) {};

  MidpointMeshSolutionRefiner::~MidpointMeshSolutionRefiner() {}


  std::unique_ptr<Mesh> MidpointMeshSolutionRefiner::calculateMeshErrors(real const mesh_error_threshold,
                                                                         bool const calculate_new_mesh,
                                                                         bool const log_mesh,
                                                                         OcpSolution const &sol,
                                                                         Mesh const &solution_mesh,
                                                                         vec_2d_real &all_mesh_errors,
                                                                         real &max_error) const {

    MaverickSingleton const &_maverick = MaverickSingleton::getInstance();

    if (solution_mesh.discretisationType() != Mesh::DiscretisationType::midpoint)
      throw std::runtime_error(
          "MidpointMeshSolutionRefiner::calculateMeshErrors: only a midpoint mesh can be anallysed");
    if (sol.discretisationType() != Mesh::DiscretisationType::midpoint)
      throw std::runtime_error(
          "MidpointMeshSolutionRefiner::calculateMeshErrors: only a midpoint solution can be anallysed");

    MidpointMesh const *mesh = (MidpointMesh *) &solution_mesh;

    all_mesh_errors.clear();

    if (log_mesh) {
      _maverick.Log(InfoLevel::info_level_verbose, "Analysis of mesh error...\n");
      stringstream tmp;
      tmp << "\t" << std::left << std::setw(5) << "Phase";
      tmp << SEP << std::left << std::setw(8) << "interval";
      tmp << SEP << std::left << std::setw(12) << "error";
      if (calculate_new_mesh)
        tmp << SEP << std::left << std::setw(5) << "# new points";
      tmp << "\n";
      _maverick.Log(InfoLevel::info_level_verbose, tmp.str());
    }

    // loop over phases
    MidpointMesh *new_mesh = nullptr;
    if (calculate_new_mesh)
      new_mesh = new MidpointMesh(*mesh);

    for (integer i_phase = 0; i_phase < _ocp_problem.numberOfPhases(); i_phase++) {
      MidpointMeshSinglePhase const &c_mesh = mesh->operator()(i_phase);

      all_mesh_errors.push_back(vec_1d_real());
      vec_1d_real &mesh_errors = all_mesh_errors[i_phase];
      mesh_errors.reserve(c_mesh.getNumberOfIntervals());

      // calculate the mesh errors for this phase
      MidpointOcpSolution const &solution = *((MidpointOcpSolution *) &sol);
      getMeshErrorsForSinglePhase(i_phase, *mesh, solution, mesh_errors);

      // analyse the errors and, eventually, increase the mesh size

      vec_1d_real new_zeta = {0};
      if (calculate_new_mesh)
        new_zeta.reserve(c_mesh.getNumberOfDiscretisationPoints() * 2);

      //loop over mesh intervals
      for (integer i_interval = 0; i_interval < mesh_errors.size(); i_interval++) {
        if (mesh_errors[i_interval] <
            0) // in this case, an error occure, therefore we consider it to be greater than the mesh_error_threshold
          mesh_errors[i_interval] = mesh_error_threshold * (1.01);

        max_error = max(max_error, mesh_errors[i_interval]);
        bool error_satisfied = mesh_errors[i_interval] <= mesh_error_threshold;

        InfoLevel il = InfoLevel::info_level_very_verbose;
        if (!error_satisfied)
          il = InfoLevel::info_level_verbose;

        stringstream message;

        if (log_mesh) {
          message << std::scientific << "\t" << std::left << std::setw(5) << i_phase;
          message << SEP << std::left << std::setw(8) << i_interval;
          message << SEP << std::left << std::setw(12) << mesh_errors[i_interval];
        }

        if (calculate_new_mesh) {
          if (error_satisfied) {
            new_zeta.push_back(c_mesh.getZetaRight(i_interval));
            if (log_mesh)
              message << SEP << std::left << std::setw(8) << "-";
          } else {
            real error_factor = mesh_errors[i_interval] / mesh_error_threshold;

            integer number_additional_mesh_points = ceil(log(error_factor) / log(mesh->logFactor()));

            MAVERICK_DEBUG_ASSERT(number_additional_mesh_points > 0, "non positive number of additional mesh points\n");
            number_additional_mesh_points = std::min(number_additional_mesh_points, mesh->maxNewPoints());

            real const last_z = *(new_zeta.end() - 1);
            real const dz = c_mesh.getDz(i_interval) / (number_additional_mesh_points + 1);

            for (integer i = 0; i < number_additional_mesh_points; i++) {
              new_zeta.push_back(last_z + dz * (i + 1));
            }
            new_zeta.push_back(c_mesh.getZetaRight(i_interval));

            if (log_mesh)
              message << SEP << std::left << std::setw(8) << number_additional_mesh_points;
          }
        }

        if (log_mesh) {
          message << "\n";
          _maverick.Log(il, message.str());
        }
      }
      if (calculate_new_mesh) {
        new_zeta.shrink_to_fit();
        new_mesh->setMeshForPhase(i_phase, MidpointMeshSinglePhase(mesh->operator()(i_phase)));
        new_mesh->operator()(i_phase).setDiscretisationPoints(new_zeta);
      }
    }
    return std::unique_ptr<MidpointMesh>(new_mesh);
  }

  void MidpointMeshSolutionRefiner::getMeshErrorsForSinglePhase(integer const i_phase, MidpointMesh const &mesh,
                                                                MidpointOcpSolution const &sol,
                                                                vec_1d_real &mesh_errors) const {
    integer const num_mesh_points = mesh(i_phase).getNumberOfDiscretisationPoints();
    integer const num_threads_to_use = (integer) _th_affinity.size();
    integer num_mesh_points_per_thread = ceil(num_mesh_points / real(num_threads_to_use));

    // calculate the number of mesh points to be spanned by each thread
    vec_1d_integer thread_mesh_points = {0};
    thread_mesh_points.reserve(num_threads_to_use + 1);

    for (integer i = 0; i < num_threads_to_use; i++) {
      thread_mesh_points.push_back((i + 1) * num_mesh_points_per_thread);
    }
    for (integer i = num_threads_to_use; i >= 0; i--) {
      if (thread_mesh_points[i] >= (num_mesh_points - 1))
        thread_mesh_points.pop_back();
    }
    thread_mesh_points.push_back(num_mesh_points - 1);

    mesh_errors = vec_1d_real(num_mesh_points - 1);
    std::thread *threads[(thread_mesh_points.size() - 1)];

    for (integer i_thread = 0; i_thread < (thread_mesh_points.size() - 1); i_thread++) {
      real *mesh_error_ptr = &mesh_errors[thread_mesh_points[i_phase]];
      threads[i_thread] = new std::thread(&MidpointMeshSolutionRefiner::calculateMeshErrorBetweenMeshPoints, this,
                                          i_phase,
                                          thread_mesh_points[i_thread],
                                          thread_mesh_points[i_thread + 1],
                                          mesh,
                                          sol,
                                          mesh_error_ptr);

    }

    for (integer i_thread = 0; i_thread < (thread_mesh_points.size() - 1); i_thread++) {
      threads[i_thread]->join();
    }

    for (integer i_thread = 0; i_thread < (thread_mesh_points.size() - 1); i_thread++) {
      delete threads[i_thread];
    }

  }

  void MidpointMeshSolutionRefiner::calculateMeshErrorBetweenMeshPoints(integer const i_phase,
                                                                        integer const first_mesh_point,
                                                                        integer const last_mesh_point,
                                                                        MidpointMesh const &mesh,
                                                                        MidpointOcpSolution const &sol,
                                                                        real mesh_error[]) const {
    vec_2d_real const &state_control = sol(i_phase).getStatesControls();
    vec_2d_real const &algebraic_state_control = sol(i_phase).getAlgebraicStatesControls();

    integer const dim_x = _ocp_problem.numberOfStates(i_phase);
    integer const dim_u = (integer) state_control.size() - dim_x;
    integer const dim_ax = _ocp_problem.numberOfAlgebraicStates(i_phase);
    integer const dim_au = (integer) algebraic_state_control.size() - dim_ax;
    integer const dim_p = _ocp_problem.numberOfParameters(i_phase);

    MidpointIntegrator calculator(_ocp_problem, _scaling, i_phase, _integrator_type);

    real x_left[dim_x];
    real x_right[dim_x];
    real u_left[dim_u];
    real u_right[dim_u];
    real ax[dim_ax];
    real au[dim_au];
    real const *params = sol(i_phase).getParameters().data();
    real const *zeta = mesh(i_phase).getDiscretisationPoints().data();
    real const *x_scaling = _scaling.getStatesControlScaling(i_phase).data();
    real const *ax_scaling = _scaling.getAlgebraicStatesControlScaling(i_phase).data();

    real x_solution[dim_x];
    real ax_solution[dim_ax];
    real solution_error;
    integer return_code;

    real x_tmp[dim_x];
    real u_center[dim_u];

    for (integer i_interval = first_mesh_point; i_interval < last_mesh_point; i_interval++) {
      MAVERICK_DEBUG_ASSERT(i_interval < state_control[0].size() - 1,
                            "MidpointMeshSolutionRefiner::calculateMeshErrorBetweenMeshPoints: index of interval exceeds solution size\n")

      // write the states and controls (store in x_tmp and u_tmp the middle point)
      for (integer i = 0; i < dim_x; i++) {
        x_left[i] = state_control[i][i_interval];
        x_right[i] = state_control[i][i_interval + 1];
        x_tmp[i] = (x_right[i] + x_left[i]) / 2.0;
      }
      for (integer i = 0; i < dim_u; i++) {
        u_left[i] = state_control[dim_x + i][i_interval];
        u_right[i] = state_control[dim_x + i][i_interval + 1];
        u_center[i] = (u_right[i] + u_left[i]) / 2.0;
      }
      for (integer i = 0; i < dim_ax; i++)
        ax[i] = algebraic_state_control[i][i_interval];
      for (integer i = 0; i < dim_au; i++)
        au[i] = algebraic_state_control[dim_ax + i][i_interval];

      // one step integration (as OCP solution)

      real const dz_half = (zeta[i_interval + 1] - zeta[i_interval]) / 2.f;

      // integrate the first half
      return_code = calculator.integrateForward(dim_x, x_left, x_tmp,   //solution will be in x_tmp
                                                dim_u, u_left, u_center,
                                                dim_ax, ax_solution, // solution will be in ax
                                                dim_au, au,
                                                dim_p, params,
                                                zeta[i_interval], dz_half,
                                                x_tmp,  // x_tmp is the x_center and is the guess
                                                ax, // guess
                                                solution_error);

      if (return_code != 0) {
        stringstream ss;
        ss << "Error in integrating mesh interval #" << i_interval << "\n";
        MaverickSingleton::getInstance().Log(InfoLevel::info_level_warning, ss.str());
        mesh_error[i_interval] = -1;
        continue;
      }
#ifdef MAVERICK_DEBUG
      real first_solution_error = solution_error;
#endif
      // integrate the second half
      return_code = calculator.integrateForward(dim_x, x_tmp, x_solution,   //solution will be in x_solution
                                                dim_u, u_center, u_right,
                                                dim_ax, ax_solution, // solution will be in ax
                                                dim_au, au,
                                                dim_p, params,
                                                zeta[i_interval] + dz_half, dz_half,
                                                x_right,  // guess
                                                ax_solution, // guess
                                                solution_error);

      if (return_code != 0) {
        stringstream ss;
        ss << "Error in integrating mesh interval #" << i_interval
           << ". Will suppose integration error = error threshold * 1.01\n";
        MaverickSingleton::getInstance().Log(InfoLevel::info_level_warning, ss.str());
        mesh_error[i_interval] = -1;
        continue;
      }
#ifdef MAVERICK_DEBUG
      solution_error = max(first_solution_error, solution_error);
      if (solution_error>1e-5) {
          MaverickSingleton::getInstance().Log(InfoLevel::info_level_warning,"Integration solution does not satisfy requested tolerance: "+ std::to_string(solution_error) + "\n");
      }
#endif
      // evaluate solution error
      real displacement[dim_x + dim_ax];

      for (integer i = 0; i < dim_x; i++) {
        displacement[i] = abs(x_solution[i] - x_right[i]) / x_scaling[i];
      }
      for (integer i = 0; i < dim_ax; i++) {
        displacement[dim_x + i] = abs(ax_solution[i] - ax[i]) / ax_scaling[i];
      }
      mesh_error[i_interval] = 0;
      for (integer i = 0; i < dim_x + dim_ax; i++) {
        if (displacement[i] > mesh_error[i_interval])
          mesh_error[i_interval] = displacement[i];
      }
    }
  }

}
