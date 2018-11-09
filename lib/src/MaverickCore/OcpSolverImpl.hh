/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef MAVERICK_OCP_SOLVER_IMPL_HH
#define MAVERICK_OCP_SOLVER_IMPL_HH

#include "MaverickOcpSolver.hh"
#include "Ocp2Nlp.hh"
#include "MaverickOcp.hh"
#include "MaverickGC/GenericContainer.hh"
#include "OcpGuess.hh"
#include "MaverickSingleton.hh"
#include "MaverickDefinitions.hh"
#include "OcpScaling.hh"
#include "NlpSolver.hh"
#include "OcpSolverOutput.hh"
#include "MaverickCore/RK1/RK1MeshSolutionRefiner.hh"

namespace Maverick {

  class OcpSolverImpl : public MaverickOcpSolver {

  public:

    OcpSolverImpl() = delete;

    OcpSolverImpl(const OcpSolverImpl &) = delete;

    OcpSolverImpl &operator=(const OcpSolverImpl &) = delete;

    OcpSolverImpl(MaverickOcp &_ocp_problem);

    ~OcpSolverImpl();

    OcpSolverOutput solve();

    OcpSolverOutput solve(GC::GenericContainer const &gc_run);

    //general setup
    void setup(GC::GenericContainer const &gc_setup);

    // set mesh
    void setMesh(Mesh const &mesh);

    // set guess
    void setExternalGuess(std::shared_ptr<OcpGuess const> p_ext_guess);

    // set start mode
    void setStartMode(SolverStartCode const code);

    // reset the solver
    void reset();

    // static methods
    static std::string convertSolverStartCodeToString(SolverStartCode start_code);

    static std::string convertSolverReturnStatusToString(SolverExitCode return_status);

  protected:

    enum ExternalGuessType {
      ext_guess_none = 0,
      ext_guess_object,
      ext_guess_tables
    };

    enum MeshRefinementStart {
      standad_start = 0,
      force_cold_start,
      force_warm_start,
      force_warm_multiplier_start
    };

    struct Settings {
      SolverStartCode start_mode = cold_start;
      bool save_mesh_history = false;
      MeshRefinementStart refinement_start = standad_start;
      MeshSolutionRefiner::EquationIntegratorType integrator_type = MeshSolutionRefiner::EquationIntegratorType::integrator_tensolve;
      bool skip_mesh_error_calculus = false;
      NlpSolver::SolvingOptions nlp_options;
    };

    struct Status {
      bool has_mesh_changed_since_last_solution = true;
      bool is_last_solution_saved = false;
      bool is_one_solution_computed = false;
      bool has_setup_mesh = false;
      bool has_setup_solver = false;
      ExternalGuessType ext_guess_type = ext_guess_none;
      SolverExitCode return_status;
    };

    Status _solver_status;

    Settings _solver_settings;

    std::shared_ptr<OcpGuess const> _p_ext_guess = nullptr;

    std::shared_ptr<const Nlp> _nlp_guess = nullptr;

    std::shared_ptr<const Nlp> _last_nlp_solution = nullptr;

    std::shared_ptr<Mesh> _p_mesh = nullptr;

    std::unique_ptr<NlpSolver> _p_nlp_solver = nullptr;

    std::shared_ptr<Ocp2Nlp> _p_ocp_2_nlp = nullptr;

    std::unique_ptr<OcpSolution> _ocp_solution = nullptr;

    void setupMeshOnly(GC::GenericContainer const &gc_mesh);

    void setupSolverOnly(GC::GenericContainer const &gc_solver);

    void setupOcpOnly(GC::GenericContainer const &gc_ocp);

    void setupGuessTablesOnly(GC::GenericContainer const &gc_guess_tables);

    void setupOcpScalingOnly(GC::GenericContainer const &gc_scaling);

    void setupOcp2Nlp();

    void setNlpGuess();

    void setMesh(std::shared_ptr<Mesh> mesh);

    // returns true if everything ok, false if the last solver call exited without giving any solution
    bool saveLastOcpSolution();

    bool isProblemDetectedInLastSolve() const;

    OcpSolverOutput doSingleRun(GC::GenericContainer const &gc);

    void loadIpoptNlpSolverFromSharedLib();

    void loadCustomNlpSolverFromSharedLib(std::string const &lib_name);

    MaverickSingleton &_maverick;

    threads_affinity _th_affinity;

  };
}

#endif
