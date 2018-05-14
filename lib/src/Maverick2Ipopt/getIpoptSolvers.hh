/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef GET_IPOPT_SOLVERS_HH
#define GET_IPOPT_SOLVERS_HH

#include "MaverickCore/EquationSolverInterface.hh"
#include "IpoptNlpSolver.hh"
#include <memory>

#ifndef DO_NOT_USE_MAV_SHARED_LIB
extern "C" {
#endif
std::unique_ptr<Maverick::IpoptNlpSolver> getIpoptNlpSolver();

std::unique_ptr<Maverick::EquationSolverInterface>
getIpoptEquationSolver(Maverick::EquationSolverSupplierInterface const &problem);

#ifndef DO_NOT_USE_MAV_SHARED_LIB
}
#endif

#endif
