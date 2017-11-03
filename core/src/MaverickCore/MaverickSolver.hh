/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef MAVERICK_SOLVER_HH
#define MAVERICK_SOLVER_HH

#include "MaverickCore/MaverickOcp.hh"
#include "MaverickCore/Mesh.hh"
#include "MaverickGC/GenericContainer.hh"
#include "MaverickCore/OcpGuess.hh"
#include "MaverickCore/SolverOutput.hh"
#include <memory>

namespace Maverick {

    class MaverickSolver {

    public:

        MaverickSolver( MaverickOcp & ocp_problem );

        virtual ~MaverickSolver() {}

        MaverickOcp const & getOcpProblem() const;

        virtual SolverOutput solve( GC::GenericContainer const &gc_run ) = 0;

        virtual SolverOutput solve() = 0;

        //general setup

        virtual void setup( GC::GenericContainer const &gc_setup ) = 0;

        // alternative setup methods to manually setup single features

        // set mesh
        virtual void setMesh( Mesh const & mesh ) = 0;

        // set external guess
        virtual void setExternalGuess( std::shared_ptr<OcpGuess const> p_ext_guess ) = 0;

        // set start mode
        virtual void setStartMode( SolverStartCode const code ) = 0;

        // reset the solver

        virtual void reset() = 0;

    protected:

        MaverickOcp & _ocp_problem;

    private:

        MaverickSolver();

        MaverickSolver(const MaverickSolver&);

        MaverickSolver& operator=(const MaverickSolver&);

    };
}

#endif
