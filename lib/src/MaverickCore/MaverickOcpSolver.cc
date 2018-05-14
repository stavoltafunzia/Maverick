#include "MaverickOcpSolver.hh"

using namespace Maverick;

MaverickOcpSolver::MaverickOcpSolver(MaverickOcp &ocp_problem) : _ocp_problem(ocp_problem) {}

MaverickOcp const &MaverickOcpSolver::getOcpProblem() const { return _ocp_problem; }
