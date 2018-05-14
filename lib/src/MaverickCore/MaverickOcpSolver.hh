/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef MAVERICK_OCP_SOLVER_HH
#define MAVERICK_OCP_SOLVER_HH

#include "MaverickCore/MaverickOcp.hh"
#include "MaverickCore/Mesh.hh"
#include "MaverickGC/GenericContainer.hh"
#include "MaverickCore/OcpGuess.hh"
#include "MaverickCore/OcpSolverOutput.hh"
#include <memory>

namespace Maverick {

  class MaverickOcpSolver {

  public:

    MaverickOcpSolver(MaverickOcp &ocp_problem);

    virtual ~MaverickOcpSolver() {}

    MaverickOcp const &getOcpProblem() const;

    virtual OcpSolverOutput solve(GC::GenericContainer const &gc_run) = 0;

    virtual OcpSolverOutput solve() = 0;

    //general setup

    virtual void setup(GC::GenericContainer const &gc_setup) = 0;

    // alternative setup methods to manually setup single features

    // set mesh
    virtual void setMesh(Mesh const &mesh) = 0;

    // set external guess
    virtual void setExternalGuess(std::shared_ptr<OcpGuess const> p_ext_guess) = 0;

    // set start mode
    virtual void setStartMode(SolverStartCode const code) = 0;

    // reset the solver

    virtual void reset() = 0;

  protected:

    MaverickOcp &_ocp_problem;

  private:

    MaverickOcpSolver();

    MaverickOcpSolver(const MaverickOcpSolver &);

    MaverickOcpSolver &operator=(const MaverickOcpSolver &);

  };
}

#endif
