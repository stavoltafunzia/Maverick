/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef MAVERICK_MIDPOINT_MESH_HH
#define MAVERICK_MIDPOINT_MESH_HH

#include "MaverickCore/Mesh.hh"
#include "MaverickCore/Midpoint/MidpointMeshSinglePhase.hh"

namespace Maverick {

    class MidpointMesh : public Mesh {

    public:

        MidpointMesh();

        MidpointMesh( MidpointMesh const & gc_mesh );

        virtual ~MidpointMesh();

        // Mesh class interface

        virtual DiscretisationType discretisationType() const { return DiscretisationType::midpoint; }

        virtual void setup( GC::GenericContainer const & gc );

        virtual void clear();

        virtual integer getNumberOfPhases() const;

        virtual std::unique_ptr<Mesh> copy() const;

        virtual std::unique_ptr<Ocp2Nlp> getDiscretiser( MaverickOcp const & ocp ) const;

        virtual std::unique_ptr<MeshSolutionRefiner> getMeshSolutionRefiner( MaverickOcp const & ocp_problem, OcpScaling const & ocp_scaling ) const;

        // operators

        virtual MeshSinglePhase const & operator[](integer const i_phase) const;

        // additional methods

        void copy( MidpointMesh const & mesh );

        void setMeshForPhase( integer const i_phase, MidpointMeshSinglePhase const & mesh );

        integer maxNewPoints() const { return _max_new_points; }

        real logFactor() const { return _new_points_log_factor; }

        // operators

        MidpointMeshSinglePhase const & operator()(integer const i_phase) const;

        MidpointMeshSinglePhase & operator()(integer const i_phase);

        MidpointMesh& operator=(const MidpointMesh & mesh);

        MidpointMesh& operator<<(const MidpointMeshSinglePhase & mesh);

    protected:

        integer _max_new_points;

        real _new_points_log_factor;

        std::vector< MidpointMeshSinglePhase > _meshes;

    };
}

#endif
