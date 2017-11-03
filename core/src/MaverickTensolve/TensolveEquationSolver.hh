/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef TENSOLVE_EQUATION_SOLVER_HH
#define TENSOLVE_EQUATION_SOLVER_HH

#include "tensolve_Cpp_interface.hh"
#include "MaverickCore/EquationSolver/EquationSolverInterface.hh"

namespace Maverick {

    class TensolveEquationSolver : public EquationSolverInterface, protected Tensolve::EquationSolver {

    public:

        TensolveEquationSolver( EquationSolverSupplierInterface const & problem );

        ~TensolveEquationSolver();

        EquationSolverReturnStatus solve();

        void setMaxIterations(integer const max_iterations);

        TensolveEquationSolver& operator=(const TensolveEquationSolver&) = delete;

//    protected:

        virtual void evalFunctions(real const x[], real f[], integer const num_equations, integer const num_unknowns) const;

        virtual bool evalJacobian(real const x[], real jac[], integer const num_equations, integer const num_unknowns) const;

    private:

        TensolveEquationSolver();

        TensolveEquationSolver(const TensolveEquationSolver&);

    };
}

#endif
