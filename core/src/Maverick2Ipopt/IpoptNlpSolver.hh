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
#include "IpoptNlpSolverInterface.hh"

namespace Maverick {

    class IpoptNlpSolver : public IpoptNlpSolverInterface {

    public:

        IpoptNlpSolver( NlpSolution & nlp_solution );
        
        IpoptNlpSolver( NlpSolution & nlp_solution, std::shared_ptr<const Ocp2Nlp> ocp_2_nlp );

        ~IpoptNlpSolver();

        virtual void setup( GC::GenericContainer const & gc_solver );

        virtual IpoptSolverReturnStatus solve( SolverSettings const & solver_settings );

        virtual void setOcp2Nlp( std::shared_ptr<const Ocp2Nlp> ocp_2_nlp );

        virtual std::string convertIpoptReturnStatusToString( Ipopt::ApplicationReturnStatus const ipopt_status );

    protected:

        // use a smart ptr because Ipopt requires it
        Ipopt::SmartPtr<Nlp2Ipopt> _p_nlp_2_ipopt;

        Ipopt::IpoptApplication _ipopt_app;

        NlpSolution & _nlp_solution;

        static SolverReturnStatus convertIpoptReturnStatusToMaverick( Ipopt::ApplicationReturnStatus const ipopt_status );

    private:

        IpoptNlpSolver();

        IpoptNlpSolver(const IpoptNlpSolver&);

        IpoptNlpSolver& operator=(const IpoptNlpSolver&);

    };
}

#endif
