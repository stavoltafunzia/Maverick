/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef NLP_SOLVER_HH
#define NLP_SOLVER_HH

#include "Nlp.hh"
#include "Ocp2Nlp.hh"

namespace Maverick {

  class NlpSolver {

  public:

    struct Output {
      SolverExitCode return_status = solution_not_computed;
      std::string return_status_message;
      integer num_iterations = 0;
      std::shared_ptr<const Nlp> solution;
    };

    struct SolvingOptions {
      std::shared_ptr<const Nlp> guess;
      integer max_iterations = 300;
      bool initialize_multipliers_form_guess = false;
    };

    NlpSolver();

    NlpSolver(std::shared_ptr<const Ocp2Nlp> ocp_2_nlp);

    NlpSolver(NlpSolver &) = delete;

    NlpSolver &operator=(const NlpSolver &) = delete;

    virtual ~NlpSolver();

    virtual void setup(GC::GenericContainer const &specific_settings) = 0;

    void setOcp2Nlp(std::shared_ptr<const Ocp2Nlp> ocp_2_nlp);

    virtual Output solve(SolvingOptions const &options) = 0;

    virtual std::string getNlpSolverType() const = 0;

  protected:

    std::shared_ptr<const Ocp2Nlp> _p_ocp_2_nlp = nullptr;

    virtual void setOcp2NlpDerived(std::shared_ptr<const Ocp2Nlp> ocp_2_nlp) = 0;

  };
}

#endif
