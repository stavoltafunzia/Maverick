/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef IPOPT_NLP_SOLVER_HH
#define IPOPT_NLP_SOLVER_HH

#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"
#include "Nlp2Ipopt.hh"
#include "MaverickCore/NlpSolver.hh"

namespace Maverick {

  class IpoptNlpSolver : public NlpSolver {

  public:

    IpoptNlpSolver();

    IpoptNlpSolver(std::shared_ptr<const Ocp2Nlp> ocp_2_nlp);

    IpoptNlpSolver(IpoptNlpSolver &) = delete;

    IpoptNlpSolver &operator=(const IpoptNlpSolver &) = delete;

    virtual ~IpoptNlpSolver();

    virtual void setup(GC::GenericContainer const &specific_settings);

    virtual Output solve(SolvingOptions const &options);

    virtual std::string getNlpSolverType() const;

  protected:

    Ipopt::IpoptApplication _ipopt_app;

    virtual void setOcp2NlpDerived(std::shared_ptr<const Ocp2Nlp> ocp_2_nlp);

  };
}

#endif
