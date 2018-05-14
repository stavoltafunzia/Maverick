/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef MAVERICK_MESH_SOLUTION_REFINER_HH
#define MAVERICK_MESH_SOLUTION_REFINER_HH

#include "MaverickOcp.hh"
#include "MaverickCore/OcpSolution.hh"
#include "MaverickCore/Mesh.hh"
#include "MaverickCore/MaverickOcp.hh"
#include "MaverickCore/MaverickDefinitions.hh"

namespace Maverick {

  class OcpScaling;

  class MeshSolutionRefiner {

  public:

    enum EquationIntegratorType {
      integrator_tensolve = 0,
      integrator_ipopt
    };

    MeshSolutionRefiner() = delete;

    MeshSolutionRefiner(MaverickOcp const &ocp_problem, OcpScaling const &ocp_scaling);

    ~MeshSolutionRefiner();

    virtual std::unique_ptr<Mesh> calculateMeshErrors(real const mesh_error_threshold,
                                                      bool const calculate_new_mesh,
                                                      bool const log_mesh,
                                                      OcpSolution const &sol,
                                                      Mesh const &solution_mesh,
                                                      vec_2d_real &mesh_errors,
                                                      real &max_error) const = 0;

    void setThreadsAffinity(threads_affinity const &th_affinity);

    void setIntegratorType(EquationIntegratorType type);

  protected:

    MaverickOcp const &_ocp_problem;

    OcpScaling const &_scaling;

    EquationIntegratorType _integrator_type = integrator_tensolve;

    threads_affinity _th_affinity;

  };
}

#endif
