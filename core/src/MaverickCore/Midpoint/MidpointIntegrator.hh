/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef MAVERICK_MIDPOINT_INTEGRATOR_HH
#define MAVERICK_MIDPOINT_INTEGRATOR_HH

#include "MaverickCore/EquationSolver/EquationSolverSupplierInterface.hh"
#include "MaverickCore/MaverickOcp.hh"
#include "MaverickCore/Mesh.hh"
#include <memory>

namespace Maverick {

    class MidpointIntegrator : public EquationSolverSupplierInterface {

    public:

        MidpointIntegrator(MaverickOcp const & ocp_problem, OcpScaling const & ocp_scaling, integer const i_phase, EquationIntegratorType integrator_type);

        ~MidpointIntegrator();

        integer integrateForward(integer const n_x, real const x_left[], real x_right_solution[],
                                 integer const n_u, real const u_left[], real const u_right[],
                                 integer const n_alg_x, real alg_x_solution[],
                                 integer const n_alg_u, real const alg_u[],
                                 integer const n_p, real const p[],
                                 real const zeta_left, real const d_zeta,
                                 real const starting_x[],
                                 real const starting_alg_x[],
                                 real & solution_error);

        // EquationSolverSupplierInterface methods

        void getProblemInfo (integer & n_x, integer & n_eq) const;

        void getProblemInfo (integer & n_x, integer & n_eq,
                             integer & nnz_jac_sparse) const;

        integer getNumEquations() const { return _dim_eq; }

        integer getNumUnknowns() const { return _dim_unk; }

        void getVarsBounds (integer const n_x, real lower[], real upper[]) const;

        void getSparseJacStructure (integer const nnz_jac, integer cols[], integer rows[]) const;

        //        void getHessStructure (integer const nnz_hess, integer cols[], integer rows[]) const;

        void evalEquations (bool const new_x,
                            integer const n_x, real const x[],
                            integer const n_eq, real eq[]) const;

        void evalEquationsSparseJac(bool const new_x,
                                    integer const n_x, real const x[],
                                    integer const nnz_jac, real jac[]) const;

        // eval the jacobian in column major order
        void evalEquationsDenseJac(bool const new_x,
                                   integer const n_x, real const x[],
                                   real jac[]) const;

        void evalEquationsDenseHess(bool const new_x,
                                    integer const n_x, real const x[],
                                    integer const n_eq, real const lambda[],
                                    real hess[]) const;


        void getStartingPoint (integer const n_x, real x[]) const;

        void finalizeSolution (integer const n_x, real const x_solution[], real const error) const;

    protected:

        void loadIpoptEquationSolver();

        void loadTensolveEquationSolver();

        MaverickOcp const & _ocp;

        OcpScaling const & _scaling;

        integer const _i_phase;

        std::unique_ptr<EquationSolverInterface> _p_solver = nullptr;

        // data for calculation
        real _zeta_center  = 0;
        real _d_zeta_inv   = 0;
        integer _dim_x     = 0;
        integer _dim_u     = 0;
        integer _dim_ax    = 0;
        integer _dim_au    = 0;
        integer _dim_eq    = 0;
        integer _dim_unk   = 0;
        integer _dim_y     = 0;
        integer _dim_ay    = 0;
        integer _dim_p     = 0;
        mutable real _sol_error    = 0;

        integer _nnz_jac   = 0;

        //scaling
        real    * _p_scaling_y              = nullptr;
        real    * _p_inv_scaling_y          = nullptr;
        real    * _p_scaling_ay             = nullptr;
        real    * _p_inv_scaling_ay         = nullptr;
        real    * _p_inv_scaling_fo_eqns    = nullptr;
        real    * _p_scale_factor_jac       = nullptr;
        real    * _p_scale_factor_jac_dense = nullptr;
        real    * _p_scale_factor_hess      = nullptr;

        //data variables
        real    * _p_c_xu_left_ns           = nullptr;
        real    * _p_c_xu_right_ns          = nullptr;
        real    * _p_c_axu_ns               = nullptr;
        real    * _p_p_ns                   = nullptr;
        real    * _p_c_xu_center_ns         = nullptr;
        real    * _p_c_xu_diff_ns           = nullptr;

        //matrix variables
        integer * _p_j_xu_outer_starts      = nullptr;
        integer * _p_j_xu_rows              = nullptr;
        integer * _p_j_dxu_outer_starts     = nullptr;
        integer * _p_j_dxu_rows             = nullptr;
        integer * _p_j_axu_outer_starts     = nullptr;
        integer * _p_j_axu_rows             = nullptr;
        integer * _p_h_xu_xu_outer_starts   = nullptr;
        integer * _p_h_xu_xu_rows           = nullptr;
        integer * _p_h_xu_dxu_outer_starts  = nullptr;
        integer * _p_h_xu_dxu_rows          = nullptr;
        integer * _p_h_xu_axu_outer_starts  = nullptr;
        integer * _p_h_xu_axu_rows          = nullptr;
        integer * _p_h_dxu_dxu_outer_starts = nullptr;
        integer * _p_h_dxu_dxu_rows         = nullptr;
        integer * _p_h_dxu_axu_outer_starts = nullptr;
        integer * _p_h_dxu_axu_rows         = nullptr;
        integer * _p_h_axu_axu_outer_starts = nullptr;
        integer * _p_h_axu_axu_rows         = nullptr;

        integer * _p_jac_cols               = nullptr;
        integer * _p_jac_rows               = nullptr;

        // methods for calulation
        void writeStateScaled(real const x[], real const ax[]) const;

    private:

        void initializeMatrixes();

        MidpointIntegrator & operator = (MidpointIntegrator const &);

    };
}

#endif
