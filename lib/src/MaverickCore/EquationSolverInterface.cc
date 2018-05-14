#include "EquationSolverInterface.hh"

namespace Maverick {

  EquationSolverInterface::EquationSolverInterface(EquationSolverSupplierInterface const &problem) : _problem(
      problem) {}

}
