/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef MAVERICK_MESH_HH
#define MAVERICK_MESH_HH

#include "MaverickCore/MeshSinglePhase.hh"
#include "MaverickCore/MaverickOcp.hh"

namespace Maverick {

    class Ocp2Nlp;

    class MeshSolutionRefiner;

    class Mesh {

    public:

        Mesh();

        Mesh( Mesh const & mesh );

        virtual ~Mesh();

        virtual DiscretisationType discretisationType() const = 0;

        virtual integer getNumberOfDiscretisationPoints() const;

        virtual vec_2d_real getDiscretisationPoints() const;

        virtual void setup( GC::GenericContainer const & gc );

        virtual void clear();

        virtual integer getNumberOfPhases() const = 0;

        integer maxIterations() const { return _max_iterations; }

        real tolerance() const { return _tolerance; }

        virtual void copy( Mesh const & mesh );

        virtual std::unique_ptr<Mesh> copy() const = 0;

        virtual std::unique_ptr<Ocp2Nlp> getDiscretiser( MaverickOcp const & ocp ) const = 0;

        virtual std::unique_ptr<MeshSolutionRefiner> getMeshSolutionRefiner( MaverickOcp const & ocp_problem, OcpScaling const & ocp_scaling ) const = 0;

        virtual void writeContentToGC( GC::GenericContainer & out_gc ) const;

        // operators

        virtual MeshSinglePhase const & operator[](integer const i_phase) const = 0;

    protected:

        integer _max_iterations;

        real _tolerance;

    };
}

#endif
