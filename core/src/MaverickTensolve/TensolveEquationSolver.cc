#include "TensolveEquationSolver.hh"
#include "MaverickCore/MaverickPrivateDefs.hh"
#include "MaverickCore/MaverickFunctions.hh"
#include "MaverickCore/MaverickSingleton.hh"
#include <functional>

using namespace Maverick;
using namespace Tensolve;
using namespace std;

#define DEFUALT_TENSOLVE_ITERATIONS 100

typedef void (*merda) ();

TensolveEquationSolver::TensolveEquationSolver( EquationSolverSupplierInterface const & problem ) :
                                                EquationSolverInterface(problem),
                                                EquationSolver(_problem.getNumEquations(), _problem.getNumUnknowns(), [this](Maverick::real const x[], Maverick::real f[], integer m, integer n){ this->evalFunctions(x, f, m, n); }, [this](Maverick::real const x[], Maverick::real jac[], integer m, integer n, integer max_m){ this->evalJacobian(x, jac, m, n, max_m); } ) {

    EquationSolver::setup(8,                          // message level: 0 default, 16 suppress output.
                          DEFUALT_TENSOLVE_ITERATIONS, // maximum nember of iterations.
                          -1,  // method
                          -1, //global
                          -1, //ipr
                          nullptr, nullptr, // typ_x, typ_f
                          -1.0, -1.0, -1.0, -1.0, -1.0 //grad_tol, step_tol, f_tol, step_max, dlt
                          );
    
}

TensolveEquationSolver::~TensolveEquationSolver() {

}

void TensolveEquationSolver::setMaxIterations(integer const max_iterations) {
    integer max_iter = max_iterations;
    if (max_iterations < 0) {
        MaverickSingleton & maverick = MaverickSingleton::getInstance();
        maverick.Log(InfoLevel::info_level_warning, "TensolveEquationSolver::setMaxIterations: negative number of maximum iterations not allowed. Will use the deualt one: " + std::to_string(DEFUALT_TENSOLVE_ITERATIONS) + "\n");
        max_iter = DEFUALT_TENSOLVE_ITERATIONS;
    }

    EquationSolver::setup(8,                          // message level: 0 default, 8 suppress output. 16 all
                          max_iter, // maximum nember of iterations.
                          -1,  // method
                          -1, //global
                          -1, //ipr
                          nullptr, nullptr, // typ_x, typ_f
                          -1.0, -1.0, -1.0, -1.0, -1.0 //grad_tol, step_tol, f_tol, step_max, dlt
                          );
}

TensolveEquationSolver::EquationSolverReturnStatus TensolveEquationSolver::solve() {
    real x0[_num_unknowns];
    _problem.getStartingPoint(_num_unknowns, x0);

    real sol[_num_unknowns];
    real func_at_sol[_num_equations];
    real grad_at_sol[_num_unknowns];

    integer return_code = EquationSolver::solve(x0, sol, func_at_sol, grad_at_sol);

    real error = 0.0;
    for (integer i=0; i<_num_unknowns; i++) {
        real tmp = abs(func_at_sol[i]*func_at_sol[i]);
        if (tmp>error)
            error = tmp;
    }

    _problem.finalizeSolution(_num_unknowns, sol, error);

    if (return_code == 5)
        return EquationSolverInterface::EquationSolverReturnStatus::max_iterations_exceeded;

    if (return_code == 0)
        return EquationSolverInterface::EquationSolverReturnStatus::problem_detected;

    return EquationSolverInterface::EquationSolverReturnStatus::solution_found;

}

void TensolveEquationSolver::evalFunctions(real const x[], real f[], integer const num_equations, integer const num_unknowns) const {
    real new_x[num_unknowns];
    copyVectorTo(x, new_x, num_unknowns);
    _problem.evalEquations(true,
                           num_unknowns, new_x,
                           num_equations, f);
}

void TensolveEquationSolver::evalJacobian(real const x[], real tensolve_jac[], integer const num_equations, integer const num_unknowns, integer const max_m) const {
    real new_x[num_unknowns];
    copyVectorTo(x, new_x, num_unknowns);

    real normal_jac[num_equations*num_unknowns];
    _problem.evalEquationsDenseJac(true,
                                   num_unknowns, new_x,
                                   normal_jac);
    
    // copy tensolve jac into the jacobian
    for(integer j_col = 0; j_col < num_unknowns; j_col++) {
        copyVectorTo(normal_jac + j_col * num_equations, tensolve_jac + j_col * max_m, num_equations);
    }
}
