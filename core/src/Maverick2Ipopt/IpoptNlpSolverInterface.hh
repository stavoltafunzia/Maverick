/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef IPOPT_NLP_SOLVER_INTERFACE_HH
#define IPOPT_NLP_SOLVER_INTERFACE_HH

#include "MaverickCore/MaverickOcp.hh"
#include "MaverickCore/Ocp2Nlp.hh"
#include "MaverickGC/GenericContainer.hh"
#include "MaverickCore/NlpSolution.hh"
#include "MaverickCore/MaverickPrivateDefs.hh"
#include "IpIpoptNLP.hpp"

namespace Maverick {

    class IpoptNlpSolverInterface {

    public:

        struct IpoptSolverReturnStatus {
            SolverReturnStatus maverick_return_status;
            Ipopt::ApplicationReturnStatus ipopt_return_status;
            integer num_iterations;
            real unscaled_target;
        };

        IpoptNlpSolverInterface() {}

        virtual ~IpoptNlpSolverInterface() {}

        virtual void setup( GC::GenericContainer const & gc_solver ) = 0;

        virtual IpoptSolverReturnStatus solve( SolverSettings const & solver_settings ) = 0;

        virtual void setOcp2Nlp( std::shared_ptr<const Ocp2Nlp> ocp_2_nlp ) = 0;

        NlpSolverId getNlpSolverId() const { return NlpSolverId::nlp_id_ipopt; }

        virtual std::string convertIpoptReturnStatusToString( Ipopt::ApplicationReturnStatus const ipopt_status ) = 0;

    private:

        IpoptNlpSolverInterface(const IpoptNlpSolverInterface &) {}

    };
}

#endif
