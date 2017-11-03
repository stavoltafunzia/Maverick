/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef MAVERICK_EQUATION_SOLVER_INTERFACE_HH
#define MAVERICK_EQUATION_SOLVER_INTERFACE_HH

#include "MaverickCore/MaverickPrivateDefs.hh"
#include "EquationSolverSupplierInterface.hh"

namespace Maverick {

    class EquationSolverInterface {

    public:

        enum EquationSolverReturnStatus {
            solution_found = 0,
            singular_problem_detected = -10,
            max_iterations_exceeded = 1,
            solution_not_found = 2,
            problem_detected = -100,
        };

        EquationSolverInterface (EquationSolverSupplierInterface const & problem);

        virtual ~EquationSolverInterface() {}

        virtual EquationSolverReturnStatus solve() = 0;

        virtual void setMaxIterations(integer const max_iterations) = 0;

        EquationSolverInterface& operator=(const EquationSolverInterface&) = delete; 

    protected:

        EquationSolverSupplierInterface const & _problem;

    private:

        EquationSolverInterface();

        EquationSolverInterface(const EquationSolverInterface &);
        
    };
}

#endif
