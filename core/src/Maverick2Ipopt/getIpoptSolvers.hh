/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef GET_IPOPT_SOLVERS_HH
#define GET_IPOPT_SOLVERS_HH

#include "IpoptNlpSolverInterface.hh"
#include "MaverickCore/NlpSolution.hh"
#include "MaverickCore/EquationSolver/EquationSolverInterface.hh"
#include <memory>

#ifndef DO_NOT_USE_MAV_SHARED_LIB
extern "C" {
#endif
    std::unique_ptr<Maverick::IpoptNlpSolverInterface> getIpoptNlpSolver( Maverick::NlpSolution & nlp_solution );
    
    std::unique_ptr<Maverick::EquationSolverInterface> getIpoptEquationSolver( Maverick::EquationSolverSupplierInterface const & problem );
    
#ifndef DO_NOT_USE_MAV_SHARED_LIB
}
#endif

#endif
