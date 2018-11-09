/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef MAVERICK_RK1_MESH_HH
#define MAVERICK_RK1_MESH_HH

#include "MaverickCore/Mesh.hh"
#include "MaverickCore/RK1/RK1MeshSinglePhase.hh"

namespace Maverick {

  class RK1Mesh : public Mesh {

  public:

    RK1Mesh();

    RK1Mesh(RK1Mesh const &gc_mesh);

    virtual ~RK1Mesh();

    // Mesh class interface

    virtual DiscretisationType discretisationType() const { return DiscretisationType::runge_kutta_1; }

    virtual void setup(GC::GenericContainer const &gc);

    virtual void clear();

    virtual integer getNumberOfPhases() const;

    virtual std::unique_ptr<Mesh> copy() const;

    virtual std::unique_ptr<Ocp2Nlp> getDiscretiser(MaverickOcp const &ocp) const;

    virtual std::unique_ptr<MeshSolutionRefiner>
    getMeshSolutionRefiner(MaverickOcp const &ocp_problem, OcpScaling const &ocp_scaling) const;

    // operators

    virtual MeshSinglePhase const &operator[](integer const i_phase) const;

    // additional methods

    void copy(RK1Mesh const &mesh);

    void setMeshForPhase(integer const i_phase, RK1MeshSinglePhase const &mesh);

    integer maxNewPoints() const { return _max_new_points; }

    real logFactor() const { return _new_points_log_factor; }

    // operators

    RK1MeshSinglePhase const &operator()(integer const i_phase) const;

    RK1MeshSinglePhase &operator()(integer const i_phase);

    RK1Mesh &operator=(const RK1Mesh &mesh);

    RK1Mesh &operator<<(const RK1MeshSinglePhase &mesh);

  protected:

    real _default_alpha;

    integer _max_new_points;

    real _new_points_log_factor;

    std::vector<RK1MeshSinglePhase> _meshes;

  };
}

#endif
