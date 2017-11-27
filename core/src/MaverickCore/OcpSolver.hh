/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef MAVERICK_OCP_SOLVER_HH
#define MAVERICK_OCP_SOLVER_HH

#include "MaverickSolver.hh"
#include "Maverick2Ipopt/IpoptNlpSolverInterface.hh"
#include "Maverick2Ipopt/IpoptNlpSolver.hh"
#include "Ocp2Nlp.hh"
#include "MaverickOcp.hh"
#include "MaverickGC/GenericContainer.hh"
#include "OcpGuess.hh"
#include "NlpSolution.hh"
#include "MaverickSingleton.hh"
#include "MaverickPrivateDefs.hh"
#include "OcpScaling.hh"

namespace Maverick {

    class OcpSolver : public MaverickSolver {

    public:

        OcpSolver() = delete;

        OcpSolver( const OcpSolver & ) = delete;

        OcpSolver& operator = ( const OcpSolver & ) = delete; 

        OcpSolver( MaverickOcp & _ocp_problem );

        ~OcpSolver();

        SolverOutput solve();

        SolverOutput solve( GC::GenericContainer const &gc_run );

        //general setup
        void setup( GC::GenericContainer const &gc_setup );

        // set mesh
        void setMesh( Mesh const & mesh );
        
        // set guess
        void setExternalGuess( std::shared_ptr<OcpGuess const> p_ext_guess );

        // set start mode
        void setStartMode( SolverStartCode const code );

        // reset the solver
        void reset();

        // static methods
        static std::string convertSolverStartCodeToString(SolverStartCode start_code);
        static std::string convertSolverReturnStatusToString(SolverReturnStatus return_status);

    protected:

        SolverStatus _solver_status;

        SolverSettings _solver_settings;

        std::shared_ptr<OcpGuess const> _p_ext_guess = nullptr;

        Nlp _nlp_guess;

        std::shared_ptr<Mesh> _p_mesh = nullptr;

        std::unique_ptr<IpoptNlpSolverInterface> _p_nlp_solver = nullptr;

        std::shared_ptr<Ocp2Nlp> _p_ocp_2_nlp = nullptr;

        std::unique_ptr<OcpSolution> _ocp_solution = nullptr;

        NlpSolution _nlp_solution;

        void setupMeshOnly( GC::GenericContainer const &gc_mesh );

        void setupSolverOnly( GC::GenericContainer const &gc_solver );

        void setupOcpOnly( GC::GenericContainer const &gc_ocp );

        void setupGuessTablesOnly( GC::GenericContainer const &gc_guess_tables );

        void setupOcpScalingOnly( GC::GenericContainer const &gc_scaling );

        void setupOcp2Nlp();

        void setNlpGuess();
        
        void setMesh( std::shared_ptr< Mesh > mesh );

        // returns true if everything ok, false if the last solver call exited without giving any solution
        bool saveLastOcpSolution();

        bool isProblemDetectedInLastSolve() const;

        SolverOutput doSingleRun( GC::GenericContainer const &gc );

        void loadIpoptNlpSolverFromSharedLib();

        MaverickSingleton & _maverick;

        threads_affinity _th_affinity;

    };
}

#endif
