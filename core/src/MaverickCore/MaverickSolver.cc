#include "MaverickSolver.hh"

using namespace Maverick;

MaverickSolver::MaverickSolver( MaverickOcp & ocp_problem ) : _ocp_problem(ocp_problem) {}

MaverickOcp const & MaverickSolver::getOcpProblem() const { return _ocp_problem; }
