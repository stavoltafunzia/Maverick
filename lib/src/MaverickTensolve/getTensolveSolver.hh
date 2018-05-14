/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef GET_TESNOLVE_SOLVERS_HH
#define GET_TESNOLVE_SOLVERS_HH

#include "TensolveEquationSolver.hh"
#include "MaverickCore/EquationSolverInterface.hh"
#include <memory>

#ifndef DO_NOT_USE_MAV_SHARED_LIB
extern "C" {
#endif

std::unique_ptr<Maverick::EquationSolverInterface>
getTensolveEquationSolver(Maverick::EquationSolverSupplierInterface const &problem);

#ifndef DO_NOT_USE_MAV_SHARED_LIB
}
#endif

#endif
