/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef MAVERICK_RK1_MESH_SOLUTION_REFINER_HH
#define MAVERICK_RK1_MESH_SOLUTION_REFINER_HH

#include "MaverickCore/MeshSolutionRefiner.hh"
#include "MaverickCore/EquationSolverInterface.hh"
#include "MaverickCore/EquationSolverSupplierInterface.hh"
#include "MaverickCore/RK1/RK1Mesh.hh"
#include "MaverickCore/OcpScaling.hh"
#include "MaverickCore/RK1/RK1OcpSolution.hh"

namespace Maverick {

  class RK1MeshSolutionRefiner : public MeshSolutionRefiner {

  public:

    RK1MeshSolutionRefiner(MaverickOcp const &ocp_problem, OcpScaling const &ocp_scaling);

    ~RK1MeshSolutionRefiner();

    virtual std::unique_ptr<Mesh> calculateMeshErrors(real const mesh_error_threshold,
                                                      bool const calculate_new_mesh,
                                                      bool const log_mesh,
                                                      OcpSolution const &sol,
                                                      Mesh const &solution_mesh,
                                                      vec_2d_real &mesh_errors,
                                                      real &max_error) const;

  protected:

    void getMeshErrorsForSinglePhase(integer const i_phase, RK1Mesh const &mesh, RK1OcpSolution const &sol,
                                     vec_1d_real &mesh_errors) const;

    void calculateMeshErrorBetweenMeshPoints(integer const i_phase,
                                             integer const first_mesh_point, integer const last_mesh_point,
                                             RK1Mesh const &mesh, RK1OcpSolution const &sol,
                                             real mesh_error[]) const;

  };
}

#endif
