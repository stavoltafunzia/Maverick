/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef MAVERICK_EQUATION_SOLVER_SUPPLIER_INTERFACE_HH
#define MAVERICK_EQUATION_SOLVER_SUPPLIER_INTERFACE_HH

#include "MaverickCore/MaverickDefinitions.hh"

namespace Maverick {

  class EquationSolverSupplierInterface {

  public:

    virtual ~EquationSolverSupplierInterface() {}

    virtual void getProblemInfo(integer &n_x, integer &n_eq) const = 0;

    virtual void getProblemInfo(integer &n_x, integer &n_eq,
                                integer &nnz_jac_sparse) const = 0;

    virtual integer getNumEquations() const = 0;

    virtual integer getNumUnknowns() const = 0;

    virtual void getVarsBounds(integer const n_x, real lower[], real upper[]) const = 0;

    virtual void getSparseJacStructure(integer const nnz_jac, integer cols[], integer rows[]) const = 0;

//        virtual void getHessStructure (integer const nnz_hess, integer cols[], integer rows[]) const = 0;

    virtual void evalEquations(bool const new_x,
                               integer const n_x, real const x[],
                               integer const n_eq, real eq[]) const = 0;

    virtual void evalEquationsSparseJac(bool const new_x,
                                        integer const n_x, real const x[],
                                        integer const nnz_jac, real jac[]) const = 0;

    // eval the jacobian in column major order
    virtual void evalEquationsDenseJac(bool const new_x,
                                       integer const n_x, real const x[],
                                       real jac[]) const = 0;

    virtual void evalEquationsDenseHess(bool const new_x,
                                        integer const n_x, real const x[],
                                        integer const n_eq, real const lambda[],
                                        real hess[]) const = 0;


    virtual void getStartingPoint(integer const n_x, real x[]) const = 0;

    virtual void finalizeSolution(integer const n_x, real const x_solution[], real const error) const = 0;

  };
}

#endif
