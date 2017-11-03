#include "MidpointOcp2NlpSinglePhase.hh"
#include "MaverickCore/MaverickFunctions.hh"
#include "MaverickCore/MaverickPrivateDefs.hh"

using namespace Maverick;

// +------------------------------------+
// |  _                  _              |
// | | |__   ___ ___ ___(_) __ _ _ __   |
// | | '_ \ / _ Y __/ __| |/ _` | '_ \  |
// | | | | |  __|__ \__ \ | (_| | | | | |
// | |_| |_|\___|___/___/_|\__,_|_| |_| |
// |                                    |
// +------------------------------------+

void MidpointOcp2NlpSinglePhase::setupForNlpHessianMatrixes() {

    // get all the required matrixes

    // get lagrange hessian
    integer nnz = _ocp_problem.lagrangeHessXuXuNnz(_i_phase);
    integer lag_hess_xu_xu_rows[ nnz ];
    integer lag_hess_xu_xu_cols[ nnz ];
    _ocp_problem.lagrangeHessXuXuPattern(_i_phase, lag_hess_xu_xu_rows, lag_hess_xu_xu_cols);
    SparseMatrix lag_hess_xu_xu_mat(_dim_xu,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    lag_hess_xu_xu_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        lag_hess_xu_xu_mat.insert(lag_hess_xu_xu_rows[i],lag_hess_xu_xu_cols[i]) = 1;
    lag_hess_xu_xu_mat.makeCompressed();
    _p_lag_hess_xu_xu_outer_start = new integer[lag_hess_xu_xu_mat.outerSize() + 1];
    copyVectorTo(lag_hess_xu_xu_mat.outerIndexPtr(), _p_lag_hess_xu_xu_outer_start, (integer) lag_hess_xu_xu_mat.outerSize() + 1);

    nnz = _ocp_problem.lagrangeHessXuDxuNnz(_i_phase);
    integer lag_hess_xu_dxu_rows[ nnz ];
    integer lag_hess_xu_dxu_cols[ nnz ];
    _ocp_problem.lagrangeHessXuDxuPattern(_i_phase, lag_hess_xu_dxu_rows, lag_hess_xu_dxu_cols);
    SparseMatrix lag_hess_xu_dxu_mat(_dim_xu,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    lag_hess_xu_dxu_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        lag_hess_xu_dxu_mat.insert(lag_hess_xu_dxu_rows[i],lag_hess_xu_dxu_cols[i]) = 1;
    lag_hess_xu_dxu_mat.makeCompressed();
    _p_lag_hess_xu_dxu_outer_start = new integer[lag_hess_xu_dxu_mat.outerSize() + 1];
    copyVectorTo(lag_hess_xu_dxu_mat.outerIndexPtr(), _p_lag_hess_xu_dxu_outer_start, (integer) lag_hess_xu_dxu_mat.outerSize() + 1);

    nnz = _ocp_problem.lagrangeHessXuAxuNnz(_i_phase);
    integer lag_hess_xu_axu_rows[ nnz ];
    integer lag_hess_xu_axu_cols[ nnz ];
    _ocp_problem.lagrangeHessXuAxuPattern(_i_phase, lag_hess_xu_axu_rows, lag_hess_xu_axu_cols);
    SparseMatrix lag_hess_xu_axu_mat(_dim_axu,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    lag_hess_xu_axu_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        lag_hess_xu_axu_mat.insert(lag_hess_xu_axu_rows[i],lag_hess_xu_axu_cols[i]) = 1;
    lag_hess_xu_axu_mat.makeCompressed();
    _p_lag_hess_xu_axu_outer_start = new integer[lag_hess_xu_axu_mat.outerSize() + 1];
    copyVectorTo(lag_hess_xu_axu_mat.outerIndexPtr(), _p_lag_hess_xu_axu_outer_start, (integer) lag_hess_xu_axu_mat.outerSize() + 1);

    nnz = _ocp_problem.lagrangeHessXuPNnz(_i_phase);
    integer lag_hess_xu_p_rows[ nnz ];
    integer lag_hess_xu_p_cols[ nnz ];
    _ocp_problem.lagrangeHessXuPPattern(_i_phase, lag_hess_xu_p_rows, lag_hess_xu_p_cols);
    SparseMatrix lag_hess_xu_p_mat(_dim_p,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    lag_hess_xu_p_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        lag_hess_xu_p_mat.insert(lag_hess_xu_p_rows[i],lag_hess_xu_p_cols[i]) = 1;
    lag_hess_xu_p_mat.makeCompressed();
    _p_lag_hess_xu_p_outer_start = new integer[lag_hess_xu_p_mat.outerSize() + 1];
    copyVectorTo(lag_hess_xu_p_mat.outerIndexPtr(), _p_lag_hess_xu_p_outer_start, (integer) lag_hess_xu_p_mat.outerSize() + 1);

    nnz = _ocp_problem.lagrangeHessDxuDxuNnz(_i_phase);
    integer lag_hess_dxu_dxu_rows[ nnz ];
    integer lag_hess_dxu_dxu_cols[ nnz ];
    _ocp_problem.lagrangeHessDxuDxuPattern(_i_phase, lag_hess_dxu_dxu_rows, lag_hess_dxu_dxu_cols);
    SparseMatrix lag_hess_dxu_dxu_mat(_dim_xu,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    lag_hess_dxu_dxu_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        lag_hess_dxu_dxu_mat.insert(lag_hess_dxu_dxu_rows[i],lag_hess_dxu_dxu_cols[i]) = 1;
    lag_hess_dxu_dxu_mat.makeCompressed();
    _p_lag_hess_dxu_dxu_outer_start = new integer[lag_hess_dxu_dxu_mat.outerSize() + 1];
    copyVectorTo(lag_hess_dxu_dxu_mat.outerIndexPtr(), _p_lag_hess_dxu_dxu_outer_start, (integer) lag_hess_dxu_dxu_mat.outerSize() + 1);

    nnz = _ocp_problem.lagrangeHessDxuAxuNnz(_i_phase);
    integer lag_hess_dxu_axu_rows[ nnz ];
    integer lag_hess_dxu_axu_cols[ nnz ];
    _ocp_problem.lagrangeHessDxuAxuPattern(_i_phase, lag_hess_dxu_axu_rows, lag_hess_dxu_axu_cols);
    SparseMatrix lag_hess_dxu_axu_mat(_dim_axu,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    lag_hess_dxu_axu_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        lag_hess_dxu_axu_mat.insert(lag_hess_dxu_axu_rows[i],lag_hess_dxu_axu_cols[i]) = 1;
    lag_hess_dxu_axu_mat.makeCompressed();
    _p_lag_hess_dxu_axu_outer_start = new integer[lag_hess_dxu_axu_mat.outerSize() + 1];
    copyVectorTo(lag_hess_dxu_axu_mat.outerIndexPtr(), _p_lag_hess_dxu_axu_outer_start, (integer) lag_hess_dxu_axu_mat.outerSize() + 1);

    nnz = _ocp_problem.lagrangeHessDxuPNnz(_i_phase);
    integer lag_hess_dxu_p_rows[ nnz ];
    integer lag_hess_dxu_p_cols[ nnz ];
    _ocp_problem.lagrangeHessDxuPPattern(_i_phase, lag_hess_dxu_p_rows, lag_hess_dxu_p_cols);
    SparseMatrix lag_hess_dxu_p_mat(_dim_p,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    lag_hess_dxu_p_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        lag_hess_dxu_p_mat.insert(lag_hess_dxu_p_rows[i],lag_hess_dxu_p_cols[i]) = 1;
    lag_hess_dxu_p_mat.makeCompressed();
    _p_lag_hess_dxu_p_outer_start = new integer[lag_hess_dxu_p_mat.outerSize() + 1];
    copyVectorTo(lag_hess_dxu_p_mat.outerIndexPtr(), _p_lag_hess_dxu_p_outer_start, (integer) lag_hess_dxu_p_mat.outerSize() + 1);

    nnz = _ocp_problem.lagrangeHessAxuAxuNnz(_i_phase);
    integer lag_hess_axu_axu_rows[ nnz ];
    integer lag_hess_axu_axu_cols[ nnz ];
    _ocp_problem.lagrangeHessAxuAxuPattern(_i_phase, lag_hess_axu_axu_rows, lag_hess_axu_axu_cols);
    SparseMatrix lag_hess_axu_axu_mat(_dim_axu,_dim_axu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    lag_hess_axu_axu_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        lag_hess_axu_axu_mat.insert(lag_hess_axu_axu_rows[i],lag_hess_axu_axu_cols[i]) = 1;
    lag_hess_axu_axu_mat.makeCompressed();
    _p_lag_hess_axu_axu_outer_start = new integer[lag_hess_axu_axu_mat.outerSize() + 1];
    copyVectorTo(lag_hess_axu_axu_mat.outerIndexPtr(), _p_lag_hess_axu_axu_outer_start, (integer) lag_hess_axu_axu_mat.outerSize() + 1);

    nnz = _ocp_problem.lagrangeHessAxuPNnz(_i_phase);
    integer lag_hess_axu_p_rows[ nnz ];
    integer lag_hess_axu_p_cols[ nnz ];
    _ocp_problem.lagrangeHessAxuPPattern(_i_phase, lag_hess_axu_p_rows, lag_hess_axu_p_cols);
    SparseMatrix lag_hess_axu_p_mat(_dim_p,_dim_axu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    lag_hess_axu_p_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        lag_hess_axu_p_mat.insert(lag_hess_axu_p_rows[i],lag_hess_axu_p_cols[i]) = 1;
    lag_hess_axu_p_mat.makeCompressed();
    _p_lag_hess_axu_p_outer_start = new integer[lag_hess_axu_p_mat.outerSize() + 1];
    copyVectorTo(lag_hess_axu_p_mat.outerIndexPtr(), _p_lag_hess_axu_p_outer_start, (integer) lag_hess_axu_p_mat.outerSize() + 1);

    nnz = _ocp_problem.lagrangeHessPPNnz(_i_phase);
    integer lag_hess_p_p_rows[ nnz ];
    integer lag_hess_p_p_cols[ nnz ];
    _ocp_problem.lagrangeHessPPPattern(_i_phase, lag_hess_p_p_rows, lag_hess_p_p_cols);
    SparseMatrix lag_hess_p_p_mat(_dim_p,_dim_p);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    lag_hess_p_p_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        lag_hess_p_p_mat.insert(lag_hess_p_p_rows[i],lag_hess_p_p_cols[i]) = 1;
    lag_hess_p_p_mat.makeCompressed();
    _p_lag_hess_p_p_outer_start = new integer[lag_hess_p_p_mat.outerSize() + 1];
    copyVectorTo(lag_hess_p_p_mat.outerIndexPtr(), _p_lag_hess_p_p_outer_start, (integer) lag_hess_p_p_mat.outerSize() + 1);

    // get fo_eqns hessian
    nnz = _ocp_problem.foEqnsHessXuXuNnz(_i_phase);
    integer fo_eqns_hess_xu_xu_rows[ nnz ];
    integer fo_eqns_hess_xu_xu_cols[ nnz ];
    _ocp_problem.foEqnsHessXuXuPattern(_i_phase, fo_eqns_hess_xu_xu_rows, fo_eqns_hess_xu_xu_cols);
    SparseMatrix fo_eqns_hess_xu_xu_mat(_dim_xu,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    fo_eqns_hess_xu_xu_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        fo_eqns_hess_xu_xu_mat.insert(fo_eqns_hess_xu_xu_rows[i],fo_eqns_hess_xu_xu_cols[i]) = 1;
    fo_eqns_hess_xu_xu_mat.makeCompressed();
    _p_fo_eqns_hess_xu_xu_outer_start = new integer[fo_eqns_hess_xu_xu_mat.outerSize() + 1];
    copyVectorTo(fo_eqns_hess_xu_xu_mat.outerIndexPtr(), _p_fo_eqns_hess_xu_xu_outer_start, (integer) fo_eqns_hess_xu_xu_mat.outerSize() + 1);

    nnz = _ocp_problem.foEqnsHessXuDxuNnz(_i_phase);
    integer fo_eqns_hess_xu_dxu_rows[ nnz ];
    integer fo_eqns_hess_xu_dxu_cols[ nnz ];
    _ocp_problem.foEqnsHessXuDxuPattern(_i_phase, fo_eqns_hess_xu_dxu_rows, fo_eqns_hess_xu_dxu_cols);
    SparseMatrix fo_eqns_hess_xu_dxu_mat(_dim_xu,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    fo_eqns_hess_xu_dxu_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        fo_eqns_hess_xu_dxu_mat.insert(fo_eqns_hess_xu_dxu_rows[i],fo_eqns_hess_xu_dxu_cols[i]) = 1;
    fo_eqns_hess_xu_dxu_mat.makeCompressed();
    _p_fo_eqns_hess_xu_dxu_outer_start = new integer[fo_eqns_hess_xu_dxu_mat.outerSize() + 1];
    copyVectorTo(fo_eqns_hess_xu_dxu_mat.outerIndexPtr(), _p_fo_eqns_hess_xu_dxu_outer_start, (integer) fo_eqns_hess_xu_dxu_mat.outerSize() + 1);

    nnz = _ocp_problem.foEqnsHessXuAxuNnz(_i_phase);
    integer fo_eqns_hess_xu_axu_rows[ nnz ];
    integer fo_eqns_hess_xu_axu_cols[ nnz ];
    _ocp_problem.foEqnsHessXuAxuPattern(_i_phase, fo_eqns_hess_xu_axu_rows, fo_eqns_hess_xu_axu_cols);
    SparseMatrix fo_eqns_hess_xu_axu_mat(_dim_axu,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    fo_eqns_hess_xu_axu_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        fo_eqns_hess_xu_axu_mat.insert(fo_eqns_hess_xu_axu_rows[i],fo_eqns_hess_xu_axu_cols[i]) = 1;
    fo_eqns_hess_xu_axu_mat.makeCompressed();
    _p_fo_eqns_hess_xu_axu_outer_start = new integer[fo_eqns_hess_xu_axu_mat.outerSize() + 1];
    copyVectorTo(fo_eqns_hess_xu_axu_mat.outerIndexPtr(), _p_fo_eqns_hess_xu_axu_outer_start, (integer) fo_eqns_hess_xu_axu_mat.outerSize() + 1);

    nnz = _ocp_problem.foEqnsHessXuPNnz(_i_phase);
    integer fo_eqns_hess_xu_p_rows[ nnz ];
    integer fo_eqns_hess_xu_p_cols[ nnz ];
    _ocp_problem.foEqnsHessXuPPattern(_i_phase, fo_eqns_hess_xu_p_rows, fo_eqns_hess_xu_p_cols);
    SparseMatrix fo_eqns_hess_xu_p_mat(_dim_p,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    fo_eqns_hess_xu_p_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        fo_eqns_hess_xu_p_mat.insert(fo_eqns_hess_xu_p_rows[i],fo_eqns_hess_xu_p_cols[i]) = 1;
    fo_eqns_hess_xu_p_mat.makeCompressed();
    _p_fo_eqns_hess_xu_p_outer_start = new integer[fo_eqns_hess_xu_p_mat.outerSize() + 1];
    copyVectorTo(fo_eqns_hess_xu_p_mat.outerIndexPtr(), _p_fo_eqns_hess_xu_p_outer_start, (integer) fo_eqns_hess_xu_p_mat.outerSize() + 1);

    nnz = _ocp_problem.foEqnsHessDxuDxuNnz(_i_phase);
    integer fo_eqns_hess_dxu_dxu_rows[ nnz ];
    integer fo_eqns_hess_dxu_dxu_cols[ nnz ];
    _ocp_problem.foEqnsHessDxuDxuPattern(_i_phase, fo_eqns_hess_dxu_dxu_rows, fo_eqns_hess_dxu_dxu_cols);
    SparseMatrix fo_eqns_hess_dxu_dxu_mat(_dim_xu,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    fo_eqns_hess_dxu_dxu_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        fo_eqns_hess_dxu_dxu_mat.insert(fo_eqns_hess_dxu_dxu_rows[i],fo_eqns_hess_dxu_dxu_cols[i]) = 1;
    fo_eqns_hess_dxu_dxu_mat.makeCompressed();
    _p_fo_eqns_hess_dxu_dxu_outer_start = new integer[fo_eqns_hess_dxu_dxu_mat.outerSize() + 1];
    copyVectorTo(fo_eqns_hess_dxu_dxu_mat.outerIndexPtr(), _p_fo_eqns_hess_dxu_dxu_outer_start, (integer) fo_eqns_hess_dxu_dxu_mat.outerSize() + 1);

    nnz = _ocp_problem.foEqnsHessDxuAxuNnz(_i_phase);
    integer fo_eqns_hess_dxu_axu_rows[ nnz ];
    integer fo_eqns_hess_dxu_axu_cols[ nnz ];
    _ocp_problem.foEqnsHessDxuAxuPattern(_i_phase, fo_eqns_hess_dxu_axu_rows, fo_eqns_hess_dxu_axu_cols);
    SparseMatrix fo_eqns_hess_dxu_axu_mat(_dim_axu,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    fo_eqns_hess_dxu_axu_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        fo_eqns_hess_dxu_axu_mat.insert(fo_eqns_hess_dxu_axu_rows[i],fo_eqns_hess_dxu_axu_cols[i]) = 1;
    fo_eqns_hess_dxu_axu_mat.makeCompressed();
    _p_fo_eqns_hess_dxu_axu_outer_start = new integer[fo_eqns_hess_dxu_axu_mat.outerSize() + 1];
    copyVectorTo(fo_eqns_hess_dxu_axu_mat.outerIndexPtr(), _p_fo_eqns_hess_dxu_axu_outer_start, (integer) fo_eqns_hess_dxu_axu_mat.outerSize() + 1);

    nnz = _ocp_problem.foEqnsHessDxuPNnz(_i_phase);
    integer fo_eqns_hess_dxu_p_rows[ nnz ];
    integer fo_eqns_hess_dxu_p_cols[ nnz ];
    _ocp_problem.foEqnsHessDxuPPattern(_i_phase, fo_eqns_hess_dxu_p_rows, fo_eqns_hess_dxu_p_cols);
    SparseMatrix fo_eqns_hess_dxu_p_mat(_dim_p,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    fo_eqns_hess_dxu_p_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        fo_eqns_hess_dxu_p_mat.insert(fo_eqns_hess_dxu_p_rows[i],fo_eqns_hess_dxu_p_cols[i]) = 1;
    fo_eqns_hess_dxu_p_mat.makeCompressed();
    _p_fo_eqns_hess_dxu_p_outer_start = new integer[fo_eqns_hess_dxu_p_mat.outerSize() + 1];
    copyVectorTo(fo_eqns_hess_dxu_p_mat.outerIndexPtr(), _p_fo_eqns_hess_dxu_p_outer_start, (integer) fo_eqns_hess_dxu_p_mat.outerSize() + 1);

    nnz = _ocp_problem.foEqnsHessAxuAxuNnz(_i_phase);
    integer fo_eqns_hess_axu_axu_rows[ nnz ];
    integer fo_eqns_hess_axu_axu_cols[ nnz ];
    _ocp_problem.foEqnsHessAxuAxuPattern(_i_phase, fo_eqns_hess_axu_axu_rows, fo_eqns_hess_axu_axu_cols);
    SparseMatrix fo_eqns_hess_axu_axu_mat(_dim_axu,_dim_axu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    fo_eqns_hess_axu_axu_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        fo_eqns_hess_axu_axu_mat.insert(fo_eqns_hess_axu_axu_rows[i],fo_eqns_hess_axu_axu_cols[i]) = 1;
    fo_eqns_hess_axu_axu_mat.makeCompressed();
    _p_fo_eqns_hess_axu_axu_outer_start = new integer[fo_eqns_hess_axu_axu_mat.outerSize() + 1];
    copyVectorTo(fo_eqns_hess_axu_axu_mat.outerIndexPtr(), _p_fo_eqns_hess_axu_axu_outer_start, (integer) fo_eqns_hess_axu_axu_mat.outerSize() + 1);

    nnz = _ocp_problem.foEqnsHessAxuPNnz(_i_phase);
    integer fo_eqns_hess_axu_p_rows[ nnz ];
    integer fo_eqns_hess_axu_p_cols[ nnz ];
    _ocp_problem.foEqnsHessAxuPPattern(_i_phase, fo_eqns_hess_axu_p_rows, fo_eqns_hess_axu_p_cols);
    SparseMatrix fo_eqns_hess_axu_p_mat(_dim_p,_dim_axu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    fo_eqns_hess_axu_p_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        fo_eqns_hess_axu_p_mat.insert(fo_eqns_hess_axu_p_rows[i],fo_eqns_hess_axu_p_cols[i]) = 1;
    fo_eqns_hess_axu_p_mat.makeCompressed();
    _p_fo_eqns_hess_axu_p_outer_start = new integer[fo_eqns_hess_axu_p_mat.outerSize() + 1];
    copyVectorTo(fo_eqns_hess_axu_p_mat.outerIndexPtr(), _p_fo_eqns_hess_axu_p_outer_start, (integer) fo_eqns_hess_axu_p_mat.outerSize() + 1);

    nnz = _ocp_problem.foEqnsHessPPNnz(_i_phase);
    integer fo_eqns_hess_p_p_rows[ nnz ];
    integer fo_eqns_hess_p_p_cols[ nnz ];
    _ocp_problem.foEqnsHessPPPattern(_i_phase, fo_eqns_hess_p_p_rows, fo_eqns_hess_p_p_cols);
    SparseMatrix fo_eqns_hess_p_p_mat(_dim_p,_dim_p);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    fo_eqns_hess_p_p_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        fo_eqns_hess_p_p_mat.insert(fo_eqns_hess_p_p_rows[i],fo_eqns_hess_p_p_cols[i]) = 1;
    fo_eqns_hess_p_p_mat.makeCompressed();
    _p_fo_eqns_hess_p_p_outer_start = new integer[fo_eqns_hess_p_p_mat.outerSize() + 1];
    copyVectorTo(fo_eqns_hess_p_p_mat.outerIndexPtr(), _p_fo_eqns_hess_p_p_outer_start, (integer) fo_eqns_hess_p_p_mat.outerSize() + 1);

    // get path constr hessian
    nnz = _ocp_problem.pathConstraintsHessXuXuNnz(_i_phase);
    integer path_constr_hess_xu_xu_rows[ nnz ];
    integer path_constr_hess_xu_xu_cols[ nnz ];
    _ocp_problem.pathConstraintsHessXuXuPattern(_i_phase, path_constr_hess_xu_xu_rows, path_constr_hess_xu_xu_cols);
    SparseMatrix path_constr_hess_xu_xu_mat(_dim_xu,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    path_constr_hess_xu_xu_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        path_constr_hess_xu_xu_mat.insert(path_constr_hess_xu_xu_rows[i],path_constr_hess_xu_xu_cols[i]) = 1;
    path_constr_hess_xu_xu_mat.makeCompressed();
    _p_path_constr_hess_xu_xu_outer_start = new integer[path_constr_hess_xu_xu_mat.outerSize() + 1];
    copyVectorTo(path_constr_hess_xu_xu_mat.outerIndexPtr(), _p_path_constr_hess_xu_xu_outer_start, (integer) path_constr_hess_xu_xu_mat.outerSize() + 1);

    nnz = _ocp_problem.pathConstraintsHessXuDxuNnz(_i_phase);
    integer path_constr_hess_xu_dxu_rows[ nnz ];
    integer path_constr_hess_xu_dxu_cols[ nnz ];
    _ocp_problem.pathConstraintsHessXuDxuPattern(_i_phase, path_constr_hess_xu_dxu_rows, path_constr_hess_xu_dxu_cols);
    SparseMatrix path_constr_hess_xu_dxu_mat(_dim_xu,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    path_constr_hess_xu_dxu_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        path_constr_hess_xu_dxu_mat.insert(path_constr_hess_xu_dxu_rows[i],path_constr_hess_xu_dxu_cols[i]) = 1;
    path_constr_hess_xu_dxu_mat.makeCompressed();
    _p_path_constr_hess_xu_dxu_outer_start = new integer[path_constr_hess_xu_dxu_mat.outerSize() + 1];
    copyVectorTo(path_constr_hess_xu_dxu_mat.outerIndexPtr(), _p_path_constr_hess_xu_dxu_outer_start, (integer) path_constr_hess_xu_dxu_mat.outerSize() + 1);

    nnz = _ocp_problem.pathConstraintsHessXuAxuNnz(_i_phase);
    integer path_constr_hess_xu_axu_rows[ nnz ];
    integer path_constr_hess_xu_axu_cols[ nnz ];
    _ocp_problem.pathConstraintsHessXuAxuPattern(_i_phase, path_constr_hess_xu_axu_rows, path_constr_hess_xu_axu_cols);
    SparseMatrix path_constr_hess_xu_axu_mat(_dim_axu,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    path_constr_hess_xu_axu_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        path_constr_hess_xu_axu_mat.insert(path_constr_hess_xu_axu_rows[i],path_constr_hess_xu_axu_cols[i]) = 1;
    path_constr_hess_xu_axu_mat.makeCompressed();
    _p_path_constr_hess_xu_axu_outer_start = new integer[path_constr_hess_xu_axu_mat.outerSize() + 1];
    copyVectorTo(path_constr_hess_xu_axu_mat.outerIndexPtr(), _p_path_constr_hess_xu_axu_outer_start, (integer) path_constr_hess_xu_axu_mat.outerSize() + 1);

    nnz = _ocp_problem.pathConstraintsHessXuPNnz(_i_phase);
    integer path_constr_hess_xu_p_rows[ nnz ];
    integer path_constr_hess_xu_p_cols[ nnz ];
    _ocp_problem.pathConstraintsHessXuPPattern(_i_phase, path_constr_hess_xu_p_rows, path_constr_hess_xu_p_cols);
    SparseMatrix path_constr_hess_xu_p_mat(_dim_p,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    path_constr_hess_xu_p_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        path_constr_hess_xu_p_mat.insert(path_constr_hess_xu_p_rows[i],path_constr_hess_xu_p_cols[i]) = 1;
    path_constr_hess_xu_p_mat.makeCompressed();
    _p_path_constr_hess_xu_p_outer_start = new integer[path_constr_hess_xu_p_mat.outerSize() + 1];
    copyVectorTo(path_constr_hess_xu_p_mat.outerIndexPtr(), _p_path_constr_hess_xu_p_outer_start, (integer) path_constr_hess_xu_p_mat.outerSize() + 1);

    nnz = _ocp_problem.pathConstraintsHessDxuDxuNnz(_i_phase);
    integer path_constr_hess_dxu_dxu_rows[ nnz ];
    integer path_constr_hess_dxu_dxu_cols[ nnz ];
    _ocp_problem.pathConstraintsHessDxuDxuPattern(_i_phase, path_constr_hess_dxu_dxu_rows, path_constr_hess_dxu_dxu_cols);
    SparseMatrix path_constr_hess_dxu_dxu_mat(_dim_xu,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    path_constr_hess_dxu_dxu_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        path_constr_hess_dxu_dxu_mat.insert(path_constr_hess_dxu_dxu_rows[i],path_constr_hess_dxu_dxu_cols[i]) = 1;
    path_constr_hess_dxu_dxu_mat.makeCompressed();
    _p_path_constr_hess_dxu_dxu_outer_start = new integer[path_constr_hess_dxu_dxu_mat.outerSize() + 1];
    copyVectorTo(path_constr_hess_dxu_dxu_mat.outerIndexPtr(), _p_path_constr_hess_dxu_dxu_outer_start, (integer) path_constr_hess_dxu_dxu_mat.outerSize() + 1);

    nnz = _ocp_problem.pathConstraintsHessDxuAxuNnz(_i_phase);
    integer path_constr_hess_dxu_axu_rows[ nnz ];
    integer path_constr_hess_dxu_axu_cols[ nnz ];
    _ocp_problem.pathConstraintsHessDxuAxuPattern(_i_phase, path_constr_hess_dxu_axu_rows, path_constr_hess_dxu_axu_cols);
    SparseMatrix path_constr_hess_dxu_axu_mat(_dim_axu,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    path_constr_hess_dxu_axu_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        path_constr_hess_dxu_axu_mat.insert(path_constr_hess_dxu_axu_rows[i],path_constr_hess_dxu_axu_cols[i]) = 1;
    path_constr_hess_dxu_axu_mat.makeCompressed();
    _p_path_constr_hess_dxu_axu_outer_start = new integer[path_constr_hess_dxu_axu_mat.outerSize() + 1];
    copyVectorTo(path_constr_hess_dxu_axu_mat.outerIndexPtr(), _p_path_constr_hess_dxu_axu_outer_start, (integer) path_constr_hess_dxu_axu_mat.outerSize() + 1);

    nnz = _ocp_problem.pathConstraintsHessDxuPNnz(_i_phase);
    integer path_constr_hess_dxu_p_rows[ nnz ];
    integer path_constr_hess_dxu_p_cols[ nnz ];
    _ocp_problem.pathConstraintsHessDxuPPattern(_i_phase, path_constr_hess_dxu_p_rows, path_constr_hess_dxu_p_cols);
    SparseMatrix path_constr_hess_dxu_p_mat(_dim_p,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    path_constr_hess_dxu_p_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        path_constr_hess_dxu_p_mat.insert(path_constr_hess_dxu_p_rows[i],path_constr_hess_dxu_p_cols[i]) = 1;
    path_constr_hess_dxu_p_mat.makeCompressed();
    _p_path_constr_hess_dxu_p_outer_start = new integer[path_constr_hess_dxu_p_mat.outerSize() + 1];
    copyVectorTo(path_constr_hess_dxu_p_mat.outerIndexPtr(), _p_path_constr_hess_dxu_p_outer_start, (integer) path_constr_hess_dxu_p_mat.outerSize() + 1);

    nnz = _ocp_problem.pathConstraintsHessAxuAxuNnz(_i_phase);
    integer path_constr_hess_axu_axu_rows[ nnz ];
    integer path_constr_hess_axu_axu_cols[ nnz ];
    _ocp_problem.pathConstraintsHessAxuAxuPattern(_i_phase, path_constr_hess_axu_axu_rows, path_constr_hess_axu_axu_cols);
    SparseMatrix path_constr_hess_axu_axu_mat(_dim_axu,_dim_axu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    path_constr_hess_axu_axu_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        path_constr_hess_axu_axu_mat.insert(path_constr_hess_axu_axu_rows[i],path_constr_hess_axu_axu_cols[i]) = 1;
    path_constr_hess_axu_axu_mat.makeCompressed();
    _p_path_constr_hess_axu_axu_outer_start = new integer[path_constr_hess_axu_axu_mat.outerSize() + 1];
    copyVectorTo(path_constr_hess_axu_axu_mat.outerIndexPtr(), _p_path_constr_hess_axu_axu_outer_start, (integer) path_constr_hess_axu_axu_mat.outerSize() + 1);

    nnz = _ocp_problem.pathConstraintsHessAxuPNnz(_i_phase);
    integer path_constr_hess_axu_p_rows[ nnz ];
    integer path_constr_hess_axu_p_cols[ nnz ];
    _ocp_problem.pathConstraintsHessAxuPPattern(_i_phase, path_constr_hess_axu_p_rows, path_constr_hess_axu_p_cols);
    SparseMatrix path_constr_hess_axu_p_mat(_dim_p,_dim_axu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    path_constr_hess_axu_p_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        path_constr_hess_axu_p_mat.insert(path_constr_hess_axu_p_rows[i],path_constr_hess_axu_p_cols[i]) = 1;
    path_constr_hess_axu_p_mat.makeCompressed();
    _p_path_constr_hess_axu_p_outer_start = new integer[path_constr_hess_axu_p_mat.outerSize() + 1];
    copyVectorTo(path_constr_hess_axu_p_mat.outerIndexPtr(), _p_path_constr_hess_axu_p_outer_start, (integer) path_constr_hess_axu_p_mat.outerSize() + 1);

    nnz = _ocp_problem.pathConstraintsHessPPNnz(_i_phase);
    integer path_constr_hess_p_p_rows[ nnz ];
    integer path_constr_hess_p_p_cols[ nnz ];
    _ocp_problem.pathConstraintsHessPPPattern(_i_phase, path_constr_hess_p_p_rows, path_constr_hess_p_p_cols);
    SparseMatrix path_constr_hess_p_p_mat(_dim_p,_dim_p);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    path_constr_hess_p_p_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        path_constr_hess_p_p_mat.insert(path_constr_hess_p_p_rows[i],path_constr_hess_p_p_cols[i]) = 1;
    path_constr_hess_p_p_mat.makeCompressed();
    _p_path_constr_hess_p_p_outer_start = new integer[path_constr_hess_p_p_mat.outerSize() + 1];
    copyVectorTo(path_constr_hess_p_p_mat.outerIndexPtr(), _p_path_constr_hess_p_p_outer_start, (integer) path_constr_hess_p_p_mat.outerSize() + 1);

    // get integral constr hessian
    nnz = _ocp_problem.intConstraintsHessXuXuNnz(_i_phase);
    integer int_constr_hess_xu_xu_rows[ nnz ];
    integer int_constr_hess_xu_xu_cols[ nnz ];
    _ocp_problem.intConstraintsHessXuXuPattern(_i_phase, int_constr_hess_xu_xu_rows, int_constr_hess_xu_xu_cols);
    SparseMatrix int_constr_hess_xu_xu_mat(_dim_xu,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    int_constr_hess_xu_xu_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        int_constr_hess_xu_xu_mat.insert(int_constr_hess_xu_xu_rows[i],int_constr_hess_xu_xu_cols[i]) = 1;
    int_constr_hess_xu_xu_mat.makeCompressed();
    _p_int_constr_hess_xu_xu_outer_start = new integer[int_constr_hess_xu_xu_mat.outerSize() + 1];
    copyVectorTo(int_constr_hess_xu_xu_mat.outerIndexPtr(), _p_int_constr_hess_xu_xu_outer_start, (integer) int_constr_hess_xu_xu_mat.outerSize() + 1);

    nnz = _ocp_problem.intConstraintsHessXuDxuNnz(_i_phase);
    integer int_constr_hess_xu_dxu_rows[ nnz ];
    integer int_constr_hess_xu_dxu_cols[ nnz ];
    _ocp_problem.intConstraintsHessXuDxuPattern(_i_phase, int_constr_hess_xu_dxu_rows, int_constr_hess_xu_dxu_cols);
    SparseMatrix int_constr_hess_xu_dxu_mat(_dim_xu,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    int_constr_hess_xu_dxu_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        int_constr_hess_xu_dxu_mat.insert(int_constr_hess_xu_dxu_rows[i],int_constr_hess_xu_dxu_cols[i]) = 1;
    int_constr_hess_xu_dxu_mat.makeCompressed();
    _p_int_constr_hess_xu_dxu_outer_start = new integer[int_constr_hess_xu_dxu_mat.outerSize() + 1];
    copyVectorTo(int_constr_hess_xu_dxu_mat.outerIndexPtr(), _p_int_constr_hess_xu_dxu_outer_start, (integer) int_constr_hess_xu_dxu_mat.outerSize() + 1);

    nnz = _ocp_problem.intConstraintsHessXuAxuNnz(_i_phase);
    integer int_constr_hess_xu_axu_rows[ nnz ];
    integer int_constr_hess_xu_axu_cols[ nnz ];
    _ocp_problem.intConstraintsHessXuAxuPattern(_i_phase, int_constr_hess_xu_axu_rows, int_constr_hess_xu_axu_cols);
    SparseMatrix int_constr_hess_xu_axu_mat(_dim_axu,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    int_constr_hess_xu_axu_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        int_constr_hess_xu_axu_mat.insert(int_constr_hess_xu_axu_rows[i],int_constr_hess_xu_axu_cols[i]) = 1;
    int_constr_hess_xu_axu_mat.makeCompressed();
    _p_int_constr_hess_xu_axu_outer_start = new integer[int_constr_hess_xu_axu_mat.outerSize() + 1];
    copyVectorTo(int_constr_hess_xu_axu_mat.outerIndexPtr(), _p_int_constr_hess_xu_axu_outer_start, (integer) int_constr_hess_xu_axu_mat.outerSize() + 1);

    nnz = _ocp_problem.intConstraintsHessXuPNnz(_i_phase);
    integer int_constr_hess_xu_p_rows[ nnz ];
    integer int_constr_hess_xu_p_cols[ nnz ];
    _ocp_problem.intConstraintsHessXuPPattern(_i_phase, int_constr_hess_xu_p_rows, int_constr_hess_xu_p_cols);
    SparseMatrix int_constr_hess_xu_p_mat(_dim_p,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    int_constr_hess_xu_p_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        int_constr_hess_xu_p_mat.insert(int_constr_hess_xu_p_rows[i],int_constr_hess_xu_p_cols[i]) = 1;
    int_constr_hess_xu_p_mat.makeCompressed();
    _p_int_constr_hess_xu_p_outer_start = new integer[int_constr_hess_xu_p_mat.outerSize() + 1];
    copyVectorTo(int_constr_hess_xu_p_mat.outerIndexPtr(), _p_int_constr_hess_xu_p_outer_start, (integer) int_constr_hess_xu_p_mat.outerSize() + 1);

    nnz = _ocp_problem.intConstraintsHessDxuDxuNnz(_i_phase);
    integer int_constr_hess_dxu_dxu_rows[ nnz ];
    integer int_constr_hess_dxu_dxu_cols[ nnz ];
    _ocp_problem.intConstraintsHessDxuDxuPattern(_i_phase, int_constr_hess_dxu_dxu_rows, int_constr_hess_dxu_dxu_cols);
    SparseMatrix int_constr_hess_dxu_dxu_mat(_dim_xu,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    int_constr_hess_dxu_dxu_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        int_constr_hess_dxu_dxu_mat.insert(int_constr_hess_dxu_dxu_rows[i],int_constr_hess_dxu_dxu_cols[i]) = 1;
    int_constr_hess_dxu_dxu_mat.makeCompressed();
    _p_int_constr_hess_dxu_dxu_outer_start = new integer[int_constr_hess_dxu_dxu_mat.outerSize() + 1];
    copyVectorTo(int_constr_hess_dxu_dxu_mat.outerIndexPtr(), _p_int_constr_hess_dxu_dxu_outer_start, (integer) int_constr_hess_dxu_dxu_mat.outerSize() + 1);

    nnz = _ocp_problem.intConstraintsHessDxuAxuNnz(_i_phase);
    integer int_constr_hess_dxu_axu_rows[ nnz ];
    integer int_constr_hess_dxu_axu_cols[ nnz ];
    _ocp_problem.intConstraintsHessDxuAxuPattern(_i_phase, int_constr_hess_dxu_axu_rows, int_constr_hess_dxu_axu_cols);
    SparseMatrix int_constr_hess_dxu_axu_mat(_dim_axu,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    int_constr_hess_dxu_axu_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        int_constr_hess_dxu_axu_mat.insert(int_constr_hess_dxu_axu_rows[i],int_constr_hess_dxu_axu_cols[i]) = 1;
    int_constr_hess_dxu_axu_mat.makeCompressed();
    _p_int_constr_hess_dxu_axu_outer_start = new integer[int_constr_hess_dxu_axu_mat.outerSize() + 1];
    copyVectorTo(int_constr_hess_dxu_axu_mat.outerIndexPtr(), _p_int_constr_hess_dxu_axu_outer_start, (integer) int_constr_hess_dxu_axu_mat.outerSize() + 1);

    nnz = _ocp_problem.intConstraintsHessDxuPNnz(_i_phase);
    integer int_constr_hess_dxu_p_rows[ nnz ];
    integer int_constr_hess_dxu_p_cols[ nnz ];
    _ocp_problem.intConstraintsHessDxuPPattern(_i_phase, int_constr_hess_dxu_p_rows, int_constr_hess_dxu_p_cols);
    SparseMatrix int_constr_hess_dxu_p_mat(_dim_p,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    int_constr_hess_dxu_p_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        int_constr_hess_dxu_p_mat.insert(int_constr_hess_dxu_p_rows[i],int_constr_hess_dxu_p_cols[i]) = 1;
    int_constr_hess_dxu_p_mat.makeCompressed();
    _p_int_constr_hess_dxu_p_outer_start = new integer[int_constr_hess_dxu_p_mat.outerSize() + 1];
    copyVectorTo(int_constr_hess_dxu_p_mat.outerIndexPtr(), _p_int_constr_hess_dxu_p_outer_start, (integer) int_constr_hess_dxu_p_mat.outerSize() + 1);

    nnz = _ocp_problem.intConstraintsHessAxuAxuNnz(_i_phase);
    integer int_constr_hess_axu_axu_rows[ nnz ];
    integer int_constr_hess_axu_axu_cols[ nnz ];
    _ocp_problem.intConstraintsHessAxuAxuPattern(_i_phase, int_constr_hess_axu_axu_rows, int_constr_hess_axu_axu_cols);
    SparseMatrix int_constr_hess_axu_axu_mat(_dim_axu,_dim_axu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    int_constr_hess_axu_axu_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        int_constr_hess_axu_axu_mat.insert(int_constr_hess_axu_axu_rows[i],int_constr_hess_axu_axu_cols[i]) = 1;
    int_constr_hess_axu_axu_mat.makeCompressed();
    _p_int_constr_hess_axu_axu_outer_start = new integer[int_constr_hess_axu_axu_mat.outerSize() + 1];
    copyVectorTo(int_constr_hess_axu_axu_mat.outerIndexPtr(), _p_int_constr_hess_axu_axu_outer_start, (integer) int_constr_hess_axu_axu_mat.outerSize() + 1);

    nnz = _ocp_problem.intConstraintsHessAxuPNnz(_i_phase);
    integer int_constr_hess_axu_p_rows[ nnz ];
    integer int_constr_hess_axu_p_cols[ nnz ];
    _ocp_problem.intConstraintsHessAxuPPattern(_i_phase, int_constr_hess_axu_p_rows, int_constr_hess_axu_p_cols);
    SparseMatrix int_constr_hess_axu_p_mat(_dim_p,_dim_axu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    int_constr_hess_axu_p_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        int_constr_hess_axu_p_mat.insert(int_constr_hess_axu_p_rows[i],int_constr_hess_axu_p_cols[i]) = 1;
    int_constr_hess_axu_p_mat.makeCompressed();
    _p_int_constr_hess_axu_p_outer_start = new integer[int_constr_hess_axu_p_mat.outerSize() + 1];
    copyVectorTo(int_constr_hess_axu_p_mat.outerIndexPtr(), _p_int_constr_hess_axu_p_outer_start, (integer) int_constr_hess_axu_p_mat.outerSize() + 1);

    nnz = _ocp_problem.intConstraintsHessPPNnz(_i_phase);
    integer int_constr_hess_p_p_rows[ nnz ];
    integer int_constr_hess_p_p_cols[ nnz ];
    _ocp_problem.intConstraintsHessPPPattern(_i_phase, int_constr_hess_p_p_rows, int_constr_hess_p_p_cols);
    SparseMatrix int_constr_hess_p_p_mat(_dim_p,_dim_p);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    int_constr_hess_p_p_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        int_constr_hess_p_p_mat.insert(int_constr_hess_p_p_rows[i],int_constr_hess_p_p_cols[i]) = 1;
    int_constr_hess_p_p_mat.makeCompressed();
    _p_int_constr_hess_p_p_outer_start = new integer[int_constr_hess_p_p_mat.outerSize() + 1];
    copyVectorTo(int_constr_hess_p_p_mat.outerIndexPtr(), _p_int_constr_hess_p_p_outer_start, (integer) int_constr_hess_p_p_mat.outerSize() + 1);

    // get point constraints hessian
    nnz = _ocp_problem.pointConstraintsHessXuXuNnz(_i_phase);
    integer point_constr_hess_xu_xu_rows[ nnz ];
    integer point_constr_hess_xu_xu_cols[ nnz ];
    _ocp_problem.pointConstraintsHessXuXuPattern(_i_phase, point_constr_hess_xu_xu_rows, point_constr_hess_xu_xu_cols);
    SparseMatrix point_constr_hess_xu_xu_mat(_dim_xu,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    point_constr_hess_xu_xu_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        point_constr_hess_xu_xu_mat.insert(point_constr_hess_xu_xu_rows[i],point_constr_hess_xu_xu_cols[i]) = 1;
    point_constr_hess_xu_xu_mat.makeCompressed();
    _p_point_constr_hess_xu_xu_outer_start = new integer[point_constr_hess_xu_xu_mat.outerSize() + 1];
    copyVectorTo(point_constr_hess_xu_xu_mat.outerIndexPtr(), _p_point_constr_hess_xu_xu_outer_start, (integer) point_constr_hess_xu_xu_mat.outerSize() + 1);

    nnz = _ocp_problem.pointConstraintsHessXuPNnz(_i_phase);
    integer point_constr_hess_xu_p_rows[ nnz ];
    integer point_constr_hess_xu_p_cols[ nnz ];
    _ocp_problem.pointConstraintsHessXuPPattern(_i_phase, point_constr_hess_xu_p_rows, point_constr_hess_xu_p_cols);
    SparseMatrix point_constr_hess_xu_p_mat(_dim_p,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    point_constr_hess_xu_p_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        point_constr_hess_xu_p_mat.insert(point_constr_hess_xu_p_rows[i],point_constr_hess_xu_p_cols[i]) = 1;
    point_constr_hess_xu_p_mat.makeCompressed();
    _p_point_constr_hess_xu_p_outer_start = new integer[point_constr_hess_xu_p_mat.outerSize() + 1];
    copyVectorTo(point_constr_hess_xu_p_mat.outerIndexPtr(), _p_point_constr_hess_xu_p_outer_start, (integer) point_constr_hess_xu_p_mat.outerSize() + 1);

    nnz = _ocp_problem.pointConstraintsHessPPNnz(_i_phase);
    integer point_constr_hess_p_p_rows[ nnz ];
    integer point_constr_hess_p_p_cols[ nnz ];
    _ocp_problem.pointConstraintsHessPPPattern(_i_phase, point_constr_hess_p_p_rows, point_constr_hess_p_p_cols);
    SparseMatrix point_constr_hess_p_p_mat(_dim_p,_dim_p);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    point_constr_hess_p_p_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        point_constr_hess_p_p_mat.insert(point_constr_hess_p_p_rows[i],point_constr_hess_p_p_cols[i]) = 1;
    point_constr_hess_p_p_mat.makeCompressed();
    _p_point_constr_hess_p_p_outer_start = new integer[point_constr_hess_p_p_mat.outerSize() + 1];
    copyVectorTo(point_constr_hess_p_p_mat.outerIndexPtr(), _p_point_constr_hess_p_p_outer_start, (integer) point_constr_hess_p_p_mat.outerSize() + 1);

    // get mayer hessian
    nnz = _ocp_problem.mayerHessXuInitXuInitNnz(_i_phase);
    integer mayer_hess_xu_init_xu_init_rows[ nnz ];
    integer mayer_hess_xu_init_xu_init_cols[ nnz ];
    _ocp_problem.mayerHessXuInitXuInitPattern(_i_phase, mayer_hess_xu_init_xu_init_rows, mayer_hess_xu_init_xu_init_cols);
    SparseMatrix mayer_hess_xu_init_xu_init_mat(_dim_xu,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    mayer_hess_xu_init_xu_init_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        mayer_hess_xu_init_xu_init_mat.insert(mayer_hess_xu_init_xu_init_rows[i],mayer_hess_xu_init_xu_init_cols[i]) = 1;
    mayer_hess_xu_init_xu_init_mat.makeCompressed();
    _p_mayer_hess_xu_init_xu_init_outer_start = new integer[mayer_hess_xu_init_xu_init_mat.outerSize() + 1];
    copyVectorTo(mayer_hess_xu_init_xu_init_mat.outerIndexPtr(), _p_mayer_hess_xu_init_xu_init_outer_start, (integer) mayer_hess_xu_init_xu_init_mat.outerSize() + 1);

    nnz = _ocp_problem.mayerHessXuInitXuFinNnz(_i_phase);
    integer mayer_hess_xu_init_xu_fin_rows[ nnz ];
    integer mayer_hess_xu_init_xu_fin_cols[ nnz ];
    _ocp_problem.mayerHessXuInitXuFinPattern(_i_phase, mayer_hess_xu_init_xu_fin_rows, mayer_hess_xu_init_xu_fin_cols);
    SparseMatrix mayer_hess_xu_init_xu_fin_mat(_dim_xu,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    mayer_hess_xu_init_xu_fin_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        mayer_hess_xu_init_xu_fin_mat.insert(mayer_hess_xu_init_xu_fin_rows[i],mayer_hess_xu_init_xu_fin_cols[i]) = 1;
    mayer_hess_xu_init_xu_fin_mat.makeCompressed();
    _p_mayer_hess_xu_init_xu_fin_outer_start = new integer[mayer_hess_xu_init_xu_fin_mat.outerSize() + 1];
    copyVectorTo(mayer_hess_xu_init_xu_fin_mat.outerIndexPtr(), _p_mayer_hess_xu_init_xu_fin_outer_start, (integer) mayer_hess_xu_init_xu_fin_mat.outerSize() + 1);

    nnz = _ocp_problem.mayerHessXuInitPNnz(_i_phase);
    integer mayer_hess_xu_init_p_rows[ nnz ];
    integer mayer_hess_xu_init_p_cols[ nnz ];
    _ocp_problem.mayerHessXuInitPPattern(_i_phase, mayer_hess_xu_init_p_rows, mayer_hess_xu_init_p_cols);
    SparseMatrix mayer_hess_xu_init_p_mat(_dim_p,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    mayer_hess_xu_init_p_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        mayer_hess_xu_init_p_mat.insert(mayer_hess_xu_init_p_rows[i],mayer_hess_xu_init_p_cols[i]) = 1;
    mayer_hess_xu_init_p_mat.makeCompressed();
    _p_mayer_hess_xu_init_p_outer_start = new integer[mayer_hess_xu_init_p_mat.outerSize() + 1];
    copyVectorTo(mayer_hess_xu_init_p_mat.outerIndexPtr(), _p_mayer_hess_xu_init_p_outer_start, (integer) mayer_hess_xu_init_p_mat.outerSize() + 1);

    nnz = _ocp_problem.mayerHessXuFinXuFinNnz(_i_phase);
    integer mayer_hess_xu_fin_xu_fin_rows[ nnz ];
    integer mayer_hess_xu_fin_xu_fin_cols[ nnz ];
    _ocp_problem.mayerHessXuFinXuFinPattern(_i_phase, mayer_hess_xu_fin_xu_fin_rows, mayer_hess_xu_fin_xu_fin_cols);
    SparseMatrix mayer_hess_xu_fin_xu_fin_mat(_dim_xu,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    mayer_hess_xu_fin_xu_fin_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        mayer_hess_xu_fin_xu_fin_mat.insert(mayer_hess_xu_fin_xu_fin_rows[i],mayer_hess_xu_fin_xu_fin_cols[i]) = 1;
    mayer_hess_xu_fin_xu_fin_mat.makeCompressed();
    _p_mayer_hess_xu_fin_xu_fin_outer_start = new integer[mayer_hess_xu_fin_xu_fin_mat.outerSize() + 1];
    copyVectorTo(mayer_hess_xu_fin_xu_fin_mat.outerIndexPtr(), _p_mayer_hess_xu_fin_xu_fin_outer_start, (integer) mayer_hess_xu_fin_xu_fin_mat.outerSize() + 1);

    nnz = _ocp_problem.mayerHessXuFinPNnz(_i_phase);
    integer mayer_hess_xu_fin_p_rows[ nnz ];
    integer mayer_hess_xu_fin_p_cols[ nnz ];
    _ocp_problem.mayerHessXuFinPPattern(_i_phase, mayer_hess_xu_fin_p_rows, mayer_hess_xu_fin_p_cols);
    SparseMatrix mayer_hess_xu_fin_p_mat(_dim_p,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    mayer_hess_xu_fin_p_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        mayer_hess_xu_fin_p_mat.insert(mayer_hess_xu_fin_p_rows[i],mayer_hess_xu_fin_p_cols[i]) = 1;
    mayer_hess_xu_fin_p_mat.makeCompressed();
    _p_mayer_hess_xu_fin_p_outer_start = new integer[mayer_hess_xu_fin_p_mat.outerSize() + 1];
    copyVectorTo(mayer_hess_xu_fin_p_mat.outerIndexPtr(), _p_mayer_hess_xu_fin_p_outer_start, (integer) mayer_hess_xu_fin_p_mat.outerSize() + 1);

    nnz = _ocp_problem.mayerHessPPNnz(_i_phase);
    integer mayer_hess_p_p_rows[ nnz ];
    integer mayer_hess_p_p_cols[ nnz ];
    _ocp_problem.mayerHessPPPattern(_i_phase, mayer_hess_p_p_rows, mayer_hess_p_p_cols);
    SparseMatrix mayer_hess_p_p_mat(_dim_p,_dim_p);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    mayer_hess_p_p_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        mayer_hess_p_p_mat.insert(mayer_hess_p_p_rows[i],mayer_hess_p_p_cols[i]) = 1;
    mayer_hess_p_p_mat.makeCompressed();
    _p_mayer_hess_p_p_outer_start = new integer[mayer_hess_p_p_mat.outerSize() + 1];
    copyVectorTo(mayer_hess_p_p_mat.outerIndexPtr(), _p_mayer_hess_p_p_outer_start, (integer) mayer_hess_p_p_mat.outerSize() + 1);

    // get boundary conditions hessian
    nnz = _ocp_problem.boundaryConditionsHessXuInitXuInitNnz(_i_phase);
    integer bcs_hess_xu_init_xu_init_rows[ nnz ];
    integer bcs_hess_xu_init_xu_init_cols[ nnz ];
    _ocp_problem.boundaryConditionsHessXuInitXuInitPattern(_i_phase, bcs_hess_xu_init_xu_init_rows, bcs_hess_xu_init_xu_init_cols);
    SparseMatrix bcs_hess_xu_init_xu_init_mat(_dim_xu,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    bcs_hess_xu_init_xu_init_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        bcs_hess_xu_init_xu_init_mat.insert(bcs_hess_xu_init_xu_init_rows[i],bcs_hess_xu_init_xu_init_cols[i]) = 1;
    bcs_hess_xu_init_xu_init_mat.makeCompressed();
    _p_bcs_hess_xu_init_xu_init_outer_start = new integer[bcs_hess_xu_init_xu_init_mat.outerSize() + 1];
    copyVectorTo(bcs_hess_xu_init_xu_init_mat.outerIndexPtr(), _p_bcs_hess_xu_init_xu_init_outer_start, (integer) bcs_hess_xu_init_xu_init_mat.outerSize() + 1);

    nnz = _ocp_problem.boundaryConditionsHessXuInitXuFinNnz(_i_phase);
    integer bcs_hess_xu_init_xu_fin_rows[ nnz ];
    integer bcs_hess_xu_init_xu_fin_cols[ nnz ];
    _ocp_problem.boundaryConditionsHessXuInitXuFinPattern(_i_phase, bcs_hess_xu_init_xu_fin_rows, bcs_hess_xu_init_xu_fin_cols);
    SparseMatrix bcs_hess_xu_init_xu_fin_mat(_dim_xu,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    bcs_hess_xu_init_xu_fin_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        bcs_hess_xu_init_xu_fin_mat.insert(bcs_hess_xu_init_xu_fin_rows[i],bcs_hess_xu_init_xu_fin_cols[i]) = 1;
    bcs_hess_xu_init_xu_fin_mat.makeCompressed();
    _p_bcs_hess_xu_init_xu_fin_outer_start = new integer[bcs_hess_xu_init_xu_fin_mat.outerSize() + 1];
    copyVectorTo(bcs_hess_xu_init_xu_fin_mat.outerIndexPtr(), _p_bcs_hess_xu_init_xu_fin_outer_start, (integer) bcs_hess_xu_init_xu_fin_mat.outerSize() + 1);

    nnz = _ocp_problem.boundaryConditionsHessXuInitPNnz(_i_phase);
    integer bcs_hess_xu_init_p_rows[ nnz ];
    integer bcs_hess_xu_init_p_cols[ nnz ];
    _ocp_problem.boundaryConditionsHessXuInitPPattern(_i_phase, bcs_hess_xu_init_p_rows, bcs_hess_xu_init_p_cols);
    SparseMatrix bcs_hess_xu_init_p_mat(_dim_p,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    bcs_hess_xu_init_p_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        bcs_hess_xu_init_p_mat.insert(bcs_hess_xu_init_p_rows[i],bcs_hess_xu_init_p_cols[i]) = 1;
    bcs_hess_xu_init_p_mat.makeCompressed();
    _p_bcs_hess_xu_init_p_outer_start = new integer[bcs_hess_xu_init_p_mat.outerSize() + 1];
    copyVectorTo(bcs_hess_xu_init_p_mat.outerIndexPtr(), _p_bcs_hess_xu_init_p_outer_start, (integer) bcs_hess_xu_init_p_mat.outerSize() + 1);

    nnz = _ocp_problem.boundaryConditionsHessXuFinXuFinNnz(_i_phase);
    integer bcs_hess_xu_fin_xu_fin_rows[ nnz ];
    integer bcs_hess_xu_fin_xu_fin_cols[ nnz ];
    _ocp_problem.boundaryConditionsHessXuFinXuFinPattern(_i_phase, bcs_hess_xu_fin_xu_fin_rows, bcs_hess_xu_fin_xu_fin_cols);
    SparseMatrix bcs_hess_xu_fin_xu_fin_mat(_dim_xu,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    bcs_hess_xu_fin_xu_fin_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        bcs_hess_xu_fin_xu_fin_mat.insert(bcs_hess_xu_fin_xu_fin_rows[i],bcs_hess_xu_fin_xu_fin_cols[i]) = 1;
    bcs_hess_xu_fin_xu_fin_mat.makeCompressed();
    _p_bcs_hess_xu_fin_xu_fin_outer_start = new integer[bcs_hess_xu_fin_xu_fin_mat.outerSize() + 1];
    copyVectorTo(bcs_hess_xu_fin_xu_fin_mat.outerIndexPtr(), _p_bcs_hess_xu_fin_xu_fin_outer_start, (integer) bcs_hess_xu_fin_xu_fin_mat.outerSize() + 1);

    nnz = _ocp_problem.boundaryConditionsHessXuFinPNnz(_i_phase);
    integer bcs_hess_xu_fin_p_rows[ nnz ];
    integer bcs_hess_xu_fin_p_cols[ nnz ];
    _ocp_problem.boundaryConditionsHessXuFinPPattern(_i_phase, bcs_hess_xu_fin_p_rows, bcs_hess_xu_fin_p_cols);
    SparseMatrix bcs_hess_xu_fin_p_mat(_dim_p,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    bcs_hess_xu_fin_p_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        bcs_hess_xu_fin_p_mat.insert(bcs_hess_xu_fin_p_rows[i],bcs_hess_xu_fin_p_cols[i]) = 1;
    bcs_hess_xu_fin_p_mat.makeCompressed();
    _p_bcs_hess_xu_fin_p_outer_start = new integer[bcs_hess_xu_fin_p_mat.outerSize() + 1];
    copyVectorTo(bcs_hess_xu_fin_p_mat.outerIndexPtr(), _p_bcs_hess_xu_fin_p_outer_start, (integer) bcs_hess_xu_fin_p_mat.outerSize() + 1);

    nnz = _ocp_problem.boundaryConditionsHessPPNnz(_i_phase);
    integer bcs_hess_p_p_rows[ nnz ];
    integer bcs_hess_p_p_cols[ nnz ];
    _ocp_problem.boundaryConditionsHessPPPattern(_i_phase, bcs_hess_p_p_rows, bcs_hess_p_p_cols);
    SparseMatrix bcs_hess_p_p_mat(_dim_p,_dim_p);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    bcs_hess_p_p_mat.reserve( nnz );
#endif
    for (integer i=0; i < nnz; i++)
        bcs_hess_p_p_mat.insert(bcs_hess_p_p_rows[i],bcs_hess_p_p_cols[i]) = 1;
    bcs_hess_p_p_mat.makeCompressed();
    _p_bcs_hess_p_p_outer_start = new integer[bcs_hess_p_p_mat.outerSize() + 1];
    copyVectorTo(bcs_hess_p_p_mat.outerIndexPtr(), _p_bcs_hess_p_p_outer_start, (integer) bcs_hess_p_p_mat.outerSize() + 1);


    //NOW CALCULATE NNZ OF HESSIAN BLOCKS
    SparseMatrix tmp_xu_xu_mat(_dim_xu, _dim_xu);
    tmp_xu_xu_mat = lag_hess_xu_xu_mat + fo_eqns_hess_xu_xu_mat + path_constr_hess_xu_xu_mat + int_constr_hess_xu_xu_mat;

    SparseMatrix tmp_dxu_dxu_mat(_dim_xu, _dim_xu);
    tmp_dxu_dxu_mat = lag_hess_dxu_dxu_mat + fo_eqns_hess_dxu_dxu_mat + path_constr_hess_dxu_dxu_mat + int_constr_hess_dxu_dxu_mat;

    SparseMatrix tmp_axu_axu_mat(_dim_axu, _dim_axu);
    tmp_axu_axu_mat = lag_hess_axu_axu_mat + fo_eqns_hess_axu_axu_mat + path_constr_hess_axu_axu_mat + int_constr_hess_axu_axu_mat;

    SparseMatrix tmp_xu_dxu_mat(_dim_xu, _dim_xu);
    tmp_xu_dxu_mat = lag_hess_xu_dxu_mat + fo_eqns_hess_xu_dxu_mat + path_constr_hess_xu_dxu_mat + int_constr_hess_xu_dxu_mat;

    SparseMatrix tmp_xu_axu_mat(_dim_axu, _dim_xu);
    tmp_xu_axu_mat = lag_hess_xu_axu_mat + fo_eqns_hess_xu_axu_mat + path_constr_hess_xu_axu_mat + int_constr_hess_xu_axu_mat;

    SparseMatrix tmp_dxu_axu_mat(_dim_axu, _dim_xu);
    tmp_dxu_axu_mat = lag_hess_dxu_axu_mat + fo_eqns_hess_dxu_axu_mat + path_constr_hess_dxu_axu_mat + int_constr_hess_dxu_axu_mat;

    SparseMatrix tmp_dxu_xu_mat(_dim_xu, _dim_xu);
    tmp_dxu_xu_mat = tmp_xu_dxu_mat.transpose();

    SparseMatrix tmp_axu_xu_mat(_dim_xu, _dim_axu);
    tmp_axu_xu_mat = tmp_xu_axu_mat.transpose();

    SparseMatrix tmp_axu_dxu_mat(_dim_xu, _dim_axu);
    tmp_axu_dxu_mat = tmp_dxu_axu_mat.transpose();

    // FIRST BLOCK: _y_y
    SparseMatrix hess_y_y_lower_mat(_dim_xu, _dim_xu);
    hess_y_y_lower_mat = tmp_xu_xu_mat.triangularView<Eigen::Lower>() + tmp_dxu_dxu_mat.triangularView<Eigen::Lower>()
    + tmp_xu_dxu_mat.triangularView<Eigen::Lower>() + tmp_dxu_xu_mat.triangularView<Eigen::Lower>()
    + point_constr_hess_xu_xu_mat.triangularView<Eigen::Lower>();
    _hess_y_y_lower_mat_nnz = (integer) hess_y_y_lower_mat.nonZeros();
    _p_hess_y_y_lower_mat_outer_starts = new integer[hess_y_y_lower_mat.outerSize() + 1];
    copyVectorTo( hess_y_y_lower_mat.outerIndexPtr(), _p_hess_y_y_lower_mat_outer_starts, (integer) hess_y_y_lower_mat.outerSize() + 1);
    _p_hess_y_y_lower_mat_rows = new integer[ _hess_y_y_lower_mat_nnz ];
    _p_scale_factor_hess_y_y_lower_mat = new real[ _hess_y_y_lower_mat_nnz ];
    integer counter = 0;
    for (integer k=0; k<hess_y_y_lower_mat.outerSize(); ++k) {
        for (SparseMatrix::InnerIterator it(hess_y_y_lower_mat,k); it; ++it) {
            integer i_row = (integer) it.row();
            integer i_col = (integer) it.col();
            _p_hess_y_y_lower_mat_rows[counter] = i_row;
            _p_scale_factor_hess_y_y_lower_mat[counter] = _p_scaling_y[i_row] * _p_scaling_y[i_col];
            counter ++;
        }
    }

    SparseMatrix hess_first_y_y_lower_mat(_dim_xu, _dim_xu);
    hess_first_y_y_lower_mat = hess_y_y_lower_mat + bcs_hess_xu_init_xu_init_mat.triangularView<Eigen::Lower>() + mayer_hess_xu_init_xu_init_mat.triangularView<Eigen::Lower>();
    _hess_first_y_y_lower_mat_nnz = (integer) hess_first_y_y_lower_mat.nonZeros();

    SparseMatrix hess_last_y_y_lower_mat(_dim_xu, _dim_xu);
    hess_last_y_y_lower_mat = hess_y_y_lower_mat + bcs_hess_xu_fin_xu_fin_mat.triangularView<Eigen::Lower>() + mayer_hess_xu_fin_xu_fin_mat.triangularView<Eigen::Lower>();
    _hess_last_y_y_lower_mat_nnz = (integer) hess_last_y_y_lower_mat.nonZeros();

    // SECOND BLOCK: _y_ay
    SparseMatrix hess_y_ay_mat(_dim_axu, _dim_xu);
    hess_y_ay_mat = tmp_xu_axu_mat + tmp_dxu_axu_mat;

    _hess_y_ay_mat_nnz = (integer) hess_y_ay_mat.nonZeros();
    // _p_hess_2_mat_outer_starts = new integer[hess_2_mat.outerSize() + 1];
    // copyVectorTo( hess_2_mat.outerIndexPtr(), _p_hess_2_mat_outer_starts, (integer) hess_2_mat.outerSize() + 1);
    _p_scale_factor_hess_y_ay_mat = new real[ _hess_y_ay_mat_nnz ];
    counter = 0;
    for (integer k=0; k<hess_y_ay_mat.outerSize(); ++k) {
        for (SparseMatrix::InnerIterator it(hess_y_ay_mat,k); it; ++it) {
            integer i_row = (integer) it.row();
            integer i_col = (integer) it.col();
            _p_scale_factor_hess_y_ay_mat[counter] = _p_scaling_ay[i_row] * _p_scaling_y[i_col];
            counter++;
        }
    }

    // THIRD BLOCK: _yleft_yright
    SparseMatrix hess_yleft_yright_mat(_dim_xu, _dim_xu);
    hess_yleft_yright_mat = tmp_xu_xu_mat + tmp_dxu_dxu_mat
    + tmp_xu_dxu_mat + tmp_dxu_xu_mat; // TODO: REMOVE THE NUMERICAL ZERO ENTRIES. TO MODIFY ALSO THE MATRIX CACLULATIONS IN SECTION BELOW

    _hess_yleft_yright_mat_nnz = (integer) hess_yleft_yright_mat.nonZeros();
    // _p_hess_2_mat_outer_starts = new integer[hess_2_mat.outerSize() + 1];
    // copyVectorTo( hess_2_mat.outerIndexPtr(), _p_hess_2_mat_outer_starts, (integer) hess_2_mat.outerSize() + 1);
    _p_scale_factor_hess_yleft_yright_mat = new real[ _hess_yleft_yright_mat_nnz ];
    counter = 0;
    for (integer k=0; k<hess_yleft_yright_mat.outerSize(); ++k) {
        for (SparseMatrix::InnerIterator it(hess_yleft_yright_mat,k); it; ++it) {
            integer i_row = (integer) it.row();
            integer i_col = (integer) it.col();
            _p_scale_factor_hess_yleft_yright_mat[counter] = _p_scaling_y[i_row] * _p_scaling_y[i_col];
            counter++;
        }
    }

    // FOURTH BLOCK: _y_p
    SparseMatrix tmp_xu_p_mat(_dim_p, _dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    tmp_xu_p_mat.reserve(_hess_y_p_mat_nnz);
#endif
    tmp_xu_p_mat = (lag_hess_xu_p_mat + fo_eqns_hess_xu_p_mat + path_constr_hess_xu_p_mat + int_constr_hess_xu_p_mat );

    SparseMatrix tmp_dxu_p_mat(_dim_p, _dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    tmp_dxu_p_mat.reserve(_hess_y_p_mat_nnz);
#endif
    tmp_dxu_p_mat = (lag_hess_dxu_p_mat + fo_eqns_hess_dxu_p_mat + path_constr_hess_dxu_p_mat + int_constr_hess_dxu_p_mat);

    SparseMatrix hess_y_p_mat(_dim_p, _dim_xu);
    hess_y_p_mat = tmp_xu_p_mat + tmp_dxu_p_mat + point_constr_hess_xu_p_mat;
    _hess_y_p_mat_nnz = (integer) hess_y_p_mat.nonZeros();
    _p_hess_y_p_mat_outer_starts = new integer[hess_y_p_mat.outerSize() + 1];
    copyVectorTo( hess_y_p_mat.outerIndexPtr(), _p_hess_y_p_mat_outer_starts, (integer) hess_y_p_mat.outerSize() + 1);
    _p_hess_y_p_mat_rows = new integer[ _hess_y_p_mat_nnz ];
    _p_scale_factor_hess_y_p_mat = new real[ _hess_y_p_mat_nnz ];
    counter = 0;
    for (integer k=0; k<hess_y_p_mat.outerSize(); ++k) {
        for (SparseMatrix::InnerIterator it(hess_y_p_mat,k); it; ++it) {
            integer i_row = (integer) it.row();
            integer i_col = (integer) it.col();
            _p_hess_y_p_mat_rows[counter] = i_row;
            _p_scale_factor_hess_y_p_mat[counter] = _p_scaling_r[i_row] * _p_scaling_y[i_col];
            counter++;
        }
    }

    SparseMatrix hess_first_y_p_mat(_dim_p, _dim_xu);
    hess_first_y_p_mat = hess_y_p_mat + bcs_hess_xu_init_p_mat + mayer_hess_xu_init_p_mat;
    _hess_first_y_p_mat_nnz = (integer) hess_first_y_p_mat.nonZeros();

    SparseMatrix hess_last_y_p_mat(_dim_p, _dim_xu);
    hess_last_y_p_mat = hess_y_p_mat + bcs_hess_xu_fin_p_mat + mayer_hess_xu_fin_p_mat;
    _hess_last_y_p_mat_nnz = (integer) hess_last_y_p_mat.nonZeros();

    // FIFTH BLOCK: AXU AXU
    SparseMatrix hess_ay_ay_lower_mat(_dim_axu, _dim_axu);
    hess_ay_ay_lower_mat = tmp_axu_axu_mat.triangularView<Eigen::Lower>();
    _hess_ay_ay_lower_mat_nnz = (integer) hess_ay_ay_lower_mat.nonZeros();
    _p_scale_factor_hess_ay_ay_lower_mat = new real[ _hess_ay_ay_lower_mat_nnz ];
    counter = 0;
    for (integer k=0; k<hess_ay_ay_lower_mat.outerSize(); ++k) {
        for (SparseMatrix::InnerIterator it(hess_ay_ay_lower_mat,k); it; ++it) {
            integer i_row = (integer) it.row();
            integer i_col = (integer) it.col();
            _p_scale_factor_hess_ay_ay_lower_mat[counter] = _p_scaling_ay[i_row] * _p_scaling_ay[i_col];
            counter++;
        }
    }

    // SIXTH BLOCK: _ay_y -> nnz equal to the second one
    SparseMatrix hess_ay_y_mat(_dim_xu, _dim_axu);
    hess_ay_y_mat = hess_y_ay_mat.transpose();

    // _p_hess_2_mat_outer_starts = new integer[hess_2_mat.outerSize() + 1];
    // copyVectorTo( hess_2_mat.outerIndexPtr(), _p_hess_2_mat_outer_starts, (integer) hess_2_mat.outerSize() + 1);
    _p_scale_factor_hess_ay_y_mat = new real[ _hess_y_ay_mat_nnz ];
    counter = 0;
    for (integer k=0; k<hess_ay_y_mat.outerSize(); ++k) {
        for (SparseMatrix::InnerIterator it(hess_ay_y_mat,k); it; ++it) {
            integer i_row = (integer) it.row();
            integer i_col = (integer) it.col();
            _p_scale_factor_hess_ay_y_mat[counter] = _p_scaling_y[i_row] * _p_scaling_ay[i_col];
            counter++;
        }
    }

    // SEVENTH BLOCK: _ay_p
    SparseMatrix hess_ay_p_mat(_dim_p, _dim_axu);
    hess_ay_p_mat = (lag_hess_axu_p_mat + fo_eqns_hess_axu_p_mat + path_constr_hess_axu_p_mat + int_constr_hess_axu_p_mat );
    _hess_ay_p_mat_nnz = (integer) hess_ay_p_mat.nonZeros();
    _p_scale_factor_hess_ay_p_mat = new real[ _hess_ay_p_mat_nnz ];
    counter = 0;
    for (integer k=0; k<hess_ay_p_mat.outerSize(); ++k) {
        for (SparseMatrix::InnerIterator it(hess_ay_p_mat,k); it; ++it) {
            integer i_row = (integer) it.row();
            integer i_col = (integer) it.col();
            _p_scale_factor_hess_ay_p_mat[counter] = _p_scaling_r[i_row] * _p_scaling_ay[i_col];
            counter++;
        }
    }

    // EIGHT BLOCK: _y_y -> equal to the first one

    // NINETH BLOCK: _y_p -> equal to the fourth one

    // TENTH BLOCK
    SparseMatrix hess_p_p_lower_mat(_dim_p, _dim_p);
    hess_p_p_lower_mat = lag_hess_p_p_mat.triangularView<Eigen::Lower>()
    + fo_eqns_hess_p_p_mat.triangularView<Eigen::Lower>()
    + path_constr_hess_p_p_mat.triangularView<Eigen::Lower>()
    + int_constr_hess_p_p_mat.triangularView<Eigen::Lower>()
    + point_constr_hess_p_p_mat.triangularView<Eigen::Lower>()
    + bcs_hess_p_p_mat.triangularView<Eigen::Lower>()
    + mayer_hess_p_p_mat.triangularView<Eigen::Lower>();
    _hess_p_p_lower_mat_nnz = (integer) hess_p_p_lower_mat.nonZeros();

    //XI XF BLOCK
    SparseMatrix hess_xi_xf_mat(_dim_xu, _dim_xu);
    hess_xi_xf_mat = bcs_hess_xu_init_xu_fin_mat + mayer_hess_xu_init_xu_fin_mat;
    _hess_xi_xf_mat_nnz = (integer) hess_xi_xf_mat.nonZeros();

    //now we can calculate the pattern of the nlp jacobian
    calculateNlpHessianPattern(hess_first_y_y_lower_mat,
                               hess_y_ay_mat,
                               hess_yleft_yright_mat,
                               hess_xi_xf_mat,
                               hess_first_y_p_mat,
                               hess_ay_ay_lower_mat,
                               hess_ay_p_mat,
                               hess_y_y_lower_mat,
                               hess_y_p_mat,
                               hess_last_y_y_lower_mat,
                               hess_last_y_p_mat,
                               hess_p_p_lower_mat);
}

//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________

void MidpointOcp2NlpSinglePhase::calculateNlpHessianPattern(SparseMatrix const & hess_first_y_y_lower_mat,
                                                       SparseMatrix const & hess_y_ay_mat,
                                                       SparseMatrix const & hess_yleft_yright_mat,
                                                       SparseMatrix const & hess_xi_xf_mat,
                                                       SparseMatrix const & hess_first_y_p_mat,
                                                       SparseMatrix const & hess_ay_ay_lower_mat,
                                                       SparseMatrix const & hess_ay_p_mat,
                                                       SparseMatrix const & hess_y_y_lower_mat,
                                                       SparseMatrix const & hess_y_p_mat,
                                                       SparseMatrix const & hess_last_y_y_lower_mat,
                                                       SparseMatrix const & hess_last_y_p_mat,
                                                       SparseMatrix const & hess_p_p_lower_mat) {
    integer index = 0;
    _p_nlp_hessian_rows = new integer[ getNlpHessianNnz() ];
    _p_nlp_hessian_cols = new integer[ getNlpHessianNnz() ];
    integer row_offset = 0;
    integer col_offset = 0;

    for ( integer mesh_point = 0; mesh_point < _p_mesh->getNumberOfDiscretisationPoints(); mesh_point++ ) {
        bool first_mesh_point = mesh_point == 0;
        bool last_mesh_point = mesh_point == (_p_mesh->getNumberOfDiscretisationPoints() -1 );
        bool middle_mesh_point = ! ( first_mesh_point || last_mesh_point );

        if (first_mesh_point) { //write first y_y hess matrix pattern
            for (integer k=0; k<hess_first_y_y_lower_mat.outerSize(); ++k) {
                for (SparseMatrix::InnerIterator it(hess_first_y_y_lower_mat,k); it; ++it) {
                    _p_nlp_hessian_rows[index] = (integer) it.row() + row_offset;   // row index
                    _p_nlp_hessian_cols[index] = (integer) it.col() + col_offset;   // col index
                    index ++;
                }
            }
        }

        if (middle_mesh_point) { //write hess y_y matrix pattern
            row_offset = getNlpYPtrIndexForInterval(mesh_point);
            col_offset = row_offset;
            for (integer k=0; k<hess_y_y_lower_mat.outerSize(); ++k) {
                for (SparseMatrix::InnerIterator it(hess_y_y_lower_mat,k); it; ++it) {
                    _p_nlp_hessian_rows[index] = (integer) it.row() + row_offset;   // row index
                    _p_nlp_hessian_cols[index] = (integer) it.col() + col_offset;   // col index
                    index ++;
                }
            }
        }

        if (last_mesh_point) { //write last y_y hess matrix pattern
            row_offset = getNlpYPtrIndexForInterval(mesh_point);
            col_offset = row_offset;
            for (integer k=0; k<hess_last_y_y_lower_mat.outerSize(); ++k) {
                for (SparseMatrix::InnerIterator it(hess_last_y_y_lower_mat,k); it; ++it) {
                    _p_nlp_hessian_rows[index] = (integer) it.row() + row_offset;   // row index
                    _p_nlp_hessian_cols[index] = (integer) it.col() + col_offset;   // col index
                    index ++;
                }
            }
        }

        if (!last_mesh_point) {
            // write the hess_y_ay_mat pattern
            row_offset = getNlpYPtrIndexForInterval(mesh_point) + _dim_y ;
            col_offset = row_offset - _dim_y ;
            for (integer k=0; k<hess_y_ay_mat.outerSize(); ++k) {
                for (SparseMatrix::InnerIterator it(hess_y_ay_mat,k); it; ++it) {
                    _p_nlp_hessian_rows[index] = (integer) it.row() + row_offset;   // row index
                    _p_nlp_hessian_cols[index] = (integer) it.col() + col_offset;   // col index
                    index ++;
                }
            }

            // write the hess_yleft_yright_mat pattern
            row_offset = getNlpYPtrIndexForInterval(mesh_point) + _dim_y + _dim_ay;
            col_offset = row_offset - _dim_y - _dim_ay;
            for (integer k=0; k<hess_yleft_yright_mat.outerSize(); ++k) {
                for (SparseMatrix::InnerIterator it(hess_yleft_yright_mat,k); it; ++it) {
                    _p_nlp_hessian_rows[index] = (integer) it.row() + row_offset;   // row index
                    _p_nlp_hessian_cols[index] = (integer) it.col() + col_offset;   // col index
                    index ++;
                }
            }
        }

        if (first_mesh_point) { //write xi_xf hess matrix pattern
            row_offset = getNlpYPtrIndexForInterval( _p_mesh->getNumberOfIntervals()) ;
            col_offset = 0 ;
            for (integer k=0; k<hess_xi_xf_mat.outerSize(); ++k) {
                for (SparseMatrix::InnerIterator it(hess_xi_xf_mat,k); it; ++it) {
                    _p_nlp_hessian_rows[index] = (integer) it.row() + row_offset;   // row index
                    _p_nlp_hessian_cols[index] = (integer) it.col() + col_offset;   // col index
                    index ++;
                }
            }
        }

        if (first_mesh_point) { //write hess first y_p matrix pattern
            row_offset = getNlpParamPtrIndex();
            col_offset = 0 ;
            for (integer k=0; k<hess_first_y_p_mat.outerSize(); ++k) {
                for (SparseMatrix::InnerIterator it(hess_first_y_p_mat,k); it; ++it) {
                    _p_nlp_hessian_rows[index] = (integer) it.row() + row_offset;   // row index
                    _p_nlp_hessian_cols[index] = (integer) it.col() + col_offset;   // col index
                    index ++;
                }
            }
        }

        if (middle_mesh_point) { //write hess y_p matrix pattern
            row_offset = getNlpParamPtrIndex();
            col_offset = getNlpYPtrIndexForInterval(mesh_point) ;
            for (integer k=0; k<hess_y_p_mat.outerSize(); ++k) {
                for (SparseMatrix::InnerIterator it(hess_y_p_mat,k); it; ++it) {
                    _p_nlp_hessian_rows[index] = (integer) it.row() + row_offset;   // row index
                    _p_nlp_hessian_cols[index] = (integer) it.col() + col_offset;   // col index
                    index ++;
                }
            }
        }

        if (last_mesh_point) { //write hess last y_p matrix pattern
            row_offset = getNlpParamPtrIndex();
            col_offset = getNlpYPtrIndexForInterval(mesh_point) ;
            for (integer k=0; k<hess_last_y_p_mat.outerSize(); ++k) {
                for (SparseMatrix::InnerIterator it(hess_last_y_p_mat,k); it; ++it) {
                    _p_nlp_hessian_rows[index] = (integer) it.row() + row_offset;   // row index
                    _p_nlp_hessian_cols[index] = (integer) it.col() + col_offset;   // col index
                    index ++;
                }
            }
        }

        if (!last_mesh_point) {
            // write the hess_ay_ay_mat pattern
            row_offset = getNlpYPtrIndexForInterval(mesh_point) + _dim_y ;
            col_offset = getNlpYPtrIndexForInterval(mesh_point) + _dim_y;
            for (integer k=0; k<hess_ay_ay_lower_mat.outerSize(); ++k) {
                for (SparseMatrix::InnerIterator it(hess_ay_ay_lower_mat,k); it; ++it) {
                    _p_nlp_hessian_rows[index] = (integer) it.row() + row_offset;   // row index
                    _p_nlp_hessian_cols[index] = (integer) it.col() + col_offset;   // col index
                    index ++;
                }
            }

            // write the hess_ay_y_mat pattern
            row_offset = getNlpYPtrIndexForInterval(mesh_point) + _dim_y + _dim_ay;
            col_offset = getNlpYPtrIndexForInterval(mesh_point) + _dim_y;
            SparseMatrix hess_ay_y_mat(_dim_y, _dim_ay);
            hess_ay_y_mat = hess_y_ay_mat.transpose();
            for (integer k=0; k<hess_ay_y_mat.outerSize(); ++k) {
                for (SparseMatrix::InnerIterator it(hess_ay_y_mat,k); it; ++it) {
                    _p_nlp_hessian_rows[index] = (integer) it.row() + row_offset;   // row index
                    _p_nlp_hessian_cols[index] = (integer) it.col() + col_offset;   // col index
                    index ++;
                }
            }

            // write the hess_ay_p_mat pattern
            row_offset = getNlpParamPtrIndex();
            col_offset = getNlpYPtrIndexForInterval(mesh_point) + _dim_y;
            for (integer k=0; k<hess_ay_p_mat.outerSize(); ++k) {
                for (SparseMatrix::InnerIterator it(hess_ay_p_mat,k); it; ++it) {
                    _p_nlp_hessian_rows[index] = (integer) it.row() + row_offset;   // row index
                    _p_nlp_hessian_cols[index] = (integer) it.col() + col_offset;   // col index
                    index ++;
                }
            }
        }
    }

    //end of mesh loop, write p_p hessian part
    row_offset = getNlpParamPtrIndex();
    col_offset = row_offset;
    for (integer k=0; k<hess_p_p_lower_mat.outerSize(); ++k) {
        for (SparseMatrix::InnerIterator it(hess_p_p_lower_mat,k); it; ++it) {
            _p_nlp_hessian_rows[index] = (integer) it.row() + row_offset;   // row index
            _p_nlp_hessian_cols[index] = (integer) it.col() + col_offset;   // col index
            index ++;
        }
    }
    //    cout << _hess_lower_mat_nnz << "\n";
    //    cout << _hess_first_lower_mat_nnz << "\n";
    //    cout << _hess_2_mat_nnz << "\n";
    //    cout << _hess_35_mat_nnz << "\n";
    //    cout << _hess_first_35_mat_nnz << "\n";
    //    cout << _hess_last_35_mat_nnz << "\n";
    //    cout << _hess_xi_xf_mat_nnz << "\n";
    //    cout << _hess_p_p_lower_mat_nnz << "\n";
    //    cout << getNlpHessianFirstColumnBlockNnz() << "\n";
    //    cout << getNlpHessianMiddleColumnBlockNnz() << "\n";
    //    cout << getNlpHessianLastColumnBlockNnz() << "\n";
#ifdef MAVERICK_DEBUG
    MAVERICK_ASSERT( index == getNlpHessianNnz(), "MidpointOcp2NlpSinglePhase::calculateNlpHessianPattern: wrong number of hessian nnz. Counted " << index << ",expected, " << getNlpHessianNnz() << ".\n")
#endif
}


//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________

void MidpointOcp2NlpSinglePhase::calculateHessianBlockAtMeshMiddle(real const ocp_left_state_control[],
                                                              real const ocp_state_control[],
                                                              real const ocp_state_control_derivative[],
                                                              real const ocp_algebraic_state_control[],
                                                              real const ocp_params[],
                                                              real const zeta,
                                                              real const d_zeta,
                                                              real const d_zeta_dual,
                                                              real const lambda_0_not_scaled,
                                                              real const lambda_not_scaled[],
                                                              real const int_constr_lambda_scaled[],
                                                              real       hessian_y_y[],
                                                              real       hessian_y_y_next[],
                                                              real       hessian_y_p_next[],
                                                              SparseMatrix &hess_p_p_lower_mat) const {

    // at the mesh middle we have to consider: lagrrange target, fo_eqns, path constraints and constraints

    real const * p_current_lambda = lambda_not_scaled;
    real const d_zeta_inv = 1.0/d_zeta;
    real       * p_current_hessian = hessian_y_y;

    // get lagrange hessian
    real lag_hess_xu_xu[ _ocp_problem.lagrangeHessXuXuNnz(_i_phase) ];
    real lag_hess_xu_dxu[ _ocp_problem.lagrangeHessXuDxuNnz(_i_phase) ];
    real lag_hess_xu_axu[ _ocp_problem.lagrangeHessXuAxuNnz(_i_phase) ];
    real lag_hess_xu_p[ _ocp_problem.lagrangeHessXuPNnz(_i_phase) ];
    real lag_hess_dxu_dxu[ _ocp_problem.lagrangeHessDxuDxuNnz(_i_phase) ];
    real lag_hess_dxu_axu[ _ocp_problem.lagrangeHessDxuAxuNnz(_i_phase) ];
    real lag_hess_dxu_p[ _ocp_problem.lagrangeHessDxuPNnz(_i_phase) ];
    real lag_hess_axu_axu[ _ocp_problem.lagrangeHessAxuAxuNnz(_i_phase) ];
    real lag_hess_axu_p[ _ocp_problem.lagrangeHessAxuPNnz(_i_phase) ];
    real lag_hess_p_p[ _ocp_problem.lagrangeHessPPNnz(_i_phase) ];
    _ocp_problem.lagrangeHess(_i_phase, ocp_state_control, ocp_state_control_derivative, ocp_algebraic_state_control, ocp_params, zeta, lambda_0_not_scaled * _inv_scaling_target,
                              lag_hess_xu_xu, lag_hess_xu_dxu, lag_hess_xu_axu, lag_hess_xu_p, lag_hess_dxu_dxu, lag_hess_dxu_axu, lag_hess_dxu_p, lag_hess_axu_axu, lag_hess_axu_p, lag_hess_p_p);

    // get fo_eqns hessian
    real fo_eqns_hess_xu_xu[ _ocp_problem.foEqnsHessXuXuNnz(_i_phase) ];
    real fo_eqns_hess_xu_dxu[ _ocp_problem.foEqnsHessXuDxuNnz(_i_phase) ];
    real fo_eqns_hess_xu_axu[ _ocp_problem.foEqnsHessXuAxuNnz(_i_phase) ];
    real fo_eqns_hess_xu_p[ _ocp_problem.foEqnsHessXuPNnz(_i_phase) ];
    real fo_eqns_hess_dxu_dxu[ _ocp_problem.foEqnsHessDxuDxuNnz(_i_phase) ];
    real fo_eqns_hess_dxu_axu[ _ocp_problem.foEqnsHessDxuAxuNnz(_i_phase) ];
    real fo_eqns_hess_dxu_p[ _ocp_problem.foEqnsHessDxuPNnz(_i_phase) ];
    real fo_eqns_hess_axu_axu[ _ocp_problem.foEqnsHessAxuAxuNnz(_i_phase) ];
    real fo_eqns_hess_axu_p[ _ocp_problem.foEqnsHessAxuPNnz(_i_phase) ];
    real fo_eqns_hess_p_p[ _ocp_problem.foEqnsHessPPNnz(_i_phase) ];
    real * lambda_scaled = new real[_dim_fo];
    real p_inv_scaling_fo_eqns[_dim_fo];

    if (_multiply_foeqns_by_dz)
        multiplyAndCopyVectorTo(_p_inv_scaling_fo_eqns_global, p_inv_scaling_fo_eqns, d_zeta, _dim_fo);
    else
        copyVectorTo(_p_inv_scaling_fo_eqns_global, p_inv_scaling_fo_eqns, _dim_fo);

    multiplyAndCopyVectorTo(p_current_lambda, lambda_scaled, p_inv_scaling_fo_eqns, _dim_fo);
    _ocp_problem.foEqnsHess(_i_phase, ocp_state_control, ocp_state_control_derivative, ocp_algebraic_state_control, ocp_params, zeta, lambda_scaled,
                            fo_eqns_hess_xu_xu, fo_eqns_hess_xu_dxu, fo_eqns_hess_xu_axu, fo_eqns_hess_xu_p, fo_eqns_hess_dxu_dxu, fo_eqns_hess_dxu_axu, fo_eqns_hess_dxu_p, fo_eqns_hess_axu_axu, fo_eqns_hess_axu_p, fo_eqns_hess_p_p);
    p_current_lambda += _dim_fo;
    delete [] lambda_scaled;

    // get path_constr hessian
    real path_constr_hess_xu_xu[ _ocp_problem.pathConstraintsHessXuXuNnz(_i_phase) ];
    real path_constr_hess_xu_dxu[ _ocp_problem.pathConstraintsHessXuDxuNnz(_i_phase) ];
    real path_constr_hess_xu_axu[ _ocp_problem.pathConstraintsHessXuAxuNnz(_i_phase) ];
    real path_constr_hess_xu_p[ _ocp_problem.pathConstraintsHessXuPNnz(_i_phase) ];
    real path_constr_hess_dxu_dxu[ _ocp_problem.pathConstraintsHessDxuDxuNnz(_i_phase) ];
    real path_constr_hess_dxu_axu[ _ocp_problem.pathConstraintsHessDxuAxuNnz(_i_phase) ];
    real path_constr_hess_dxu_p[ _ocp_problem.pathConstraintsHessDxuPNnz(_i_phase) ];
    real path_constr_hess_axu_axu[ _ocp_problem.pathConstraintsHessAxuAxuNnz(_i_phase) ];
    real path_constr_hess_axu_p[ _ocp_problem.pathConstraintsHessAxuPNnz(_i_phase) ];
    real path_constr_hess_p_p[ _ocp_problem.pathConstraintsHessPPNnz(_i_phase) ];
    lambda_scaled = new real[_dim_pc];
    real p_inv_scaling_path_constr[_dim_pc];
    if (_multiply_path_constr_by_dz)
        multiplyAndCopyVectorTo(_p_inv_scaling_path_constr_global, p_inv_scaling_path_constr, d_zeta, _dim_pc);
    else
        copyVectorTo(_p_inv_scaling_path_constr_global, p_inv_scaling_path_constr, _dim_pc);

    multiplyAndCopyVectorTo(p_current_lambda, lambda_scaled, p_inv_scaling_path_constr, _dim_pc);
    _ocp_problem.pathConstraintsHess(_i_phase, ocp_state_control, ocp_state_control_derivative, ocp_algebraic_state_control, ocp_params, zeta, lambda_scaled,
                                     path_constr_hess_xu_xu, path_constr_hess_xu_dxu, path_constr_hess_xu_axu, path_constr_hess_xu_p, path_constr_hess_dxu_dxu, path_constr_hess_dxu_axu, path_constr_hess_dxu_p, path_constr_hess_axu_axu, path_constr_hess_axu_p, path_constr_hess_p_p);
    p_current_lambda += _dim_pc;
    delete [] lambda_scaled;

    // get integral constraints hessian
    real int_constr_hess_xu_xu[ _ocp_problem.intConstraintsHessXuXuNnz(_i_phase) ];
    real int_constr_hess_xu_dxu[ _ocp_problem.intConstraintsHessXuDxuNnz(_i_phase) ];
    real int_constr_hess_xu_axu[ _ocp_problem.intConstraintsHessXuAxuNnz(_i_phase) ];
    real int_constr_hess_xu_p[ _ocp_problem.intConstraintsHessXuPNnz(_i_phase) ];
    real int_constr_hess_dxu_dxu[ _ocp_problem.intConstraintsHessDxuDxuNnz(_i_phase) ];
    real int_constr_hess_dxu_axu[ _ocp_problem.intConstraintsHessDxuAxuNnz(_i_phase) ];
    real int_constr_hess_dxu_p[ _ocp_problem.intConstraintsHessDxuPNnz(_i_phase) ];
    real int_constr_hess_axu_axu[ _ocp_problem.intConstraintsHessAxuAxuNnz(_i_phase) ];
    real int_constr_hess_axu_p[ _ocp_problem.intConstraintsHessAxuPNnz(_i_phase) ];
    real int_constr_hess_p_p[ _ocp_problem.intConstraintsHessPPNnz(_i_phase) ];
    _ocp_problem.intConstraintsHess(_i_phase, ocp_state_control, ocp_state_control_derivative, ocp_algebraic_state_control, ocp_params, zeta, int_constr_lambda_scaled,
                                    int_constr_hess_xu_xu, int_constr_hess_xu_dxu, int_constr_hess_xu_axu, int_constr_hess_xu_p, int_constr_hess_dxu_dxu, int_constr_hess_dxu_axu, int_constr_hess_dxu_p, int_constr_hess_axu_axu, int_constr_hess_axu_p, int_constr_hess_p_p);

    // get point constr hessian
    real point_constr_hess_xu_xu[ _ocp_problem.pointConstraintsHessXuXuNnz(_i_phase) ];
    real point_constr_hess_xu_p[ _ocp_problem.pointConstraintsHessXuPNnz(_i_phase) ];
    real point_constr_hess_p_p[ _ocp_problem.pointConstraintsHessPPNnz(_i_phase) ];
    lambda_scaled = new real[_dim_poc];
    real p_inv_scaling_point_constr[_dim_poc];
    if (_multiply_point_constr_by_dz)
        multiplyAndCopyVectorTo(_p_inv_scaling_point_constr_global, p_inv_scaling_point_constr, d_zeta_dual, _dim_poc);
    else
        copyVectorTo(_p_inv_scaling_point_constr_global, p_inv_scaling_point_constr, _dim_poc);

    multiplyAndCopyVectorTo(p_current_lambda, lambda_scaled, p_inv_scaling_point_constr, _dim_poc);
    _ocp_problem.pointConstraintsHess(_i_phase, ocp_left_state_control, ocp_params, zeta, lambda_scaled,
                                      point_constr_hess_xu_xu, point_constr_hess_xu_p, point_constr_hess_p_p);
    p_current_lambda += _dim_poc;
    delete [] lambda_scaled;

    // create lagrange matrixes
    integer lag_hess_xu_xu_rows[_ocp_problem.lagrangeHessXuXuNnz(_i_phase)];
    integer lag_hess_xu_xu_cols[_ocp_problem.lagrangeHessXuXuNnz(_i_phase)];
    _ocp_problem.lagrangeHessXuXuPattern(_i_phase, lag_hess_xu_xu_rows, lag_hess_xu_xu_cols);
    Eigen::Map<SparseMatrix> lag_hess_xu_xu_mat(_dim_xu, _dim_xu, _ocp_problem.lagrangeHessXuXuNnz(_i_phase),
                                                _p_lag_hess_xu_xu_outer_start,
                                                lag_hess_xu_xu_rows,
                                                lag_hess_xu_xu,
                                                0);

    integer lag_hess_xu_dxu_rows[_ocp_problem.lagrangeHessXuDxuNnz(_i_phase)];
    integer lag_hess_xu_dxu_cols[_ocp_problem.lagrangeHessXuDxuNnz(_i_phase)];
    _ocp_problem.lagrangeHessXuDxuPattern(_i_phase, lag_hess_xu_dxu_rows, lag_hess_xu_dxu_cols);
    Eigen::Map<SparseMatrix> lag_hess_xu_dxu_mat(_dim_xu, _dim_xu, _ocp_problem.lagrangeHessXuDxuNnz(_i_phase),
                                                 _p_lag_hess_xu_dxu_outer_start,
                                                 lag_hess_xu_dxu_rows,
                                                 lag_hess_xu_dxu,
                                                 0);

    integer lag_hess_xu_axu_rows[_ocp_problem.lagrangeHessXuAxuNnz(_i_phase)];
    integer lag_hess_xu_axu_cols[_ocp_problem.lagrangeHessXuAxuNnz(_i_phase)];
    _ocp_problem.lagrangeHessXuAxuPattern(_i_phase, lag_hess_xu_axu_rows, lag_hess_xu_axu_cols);
    Eigen::Map<SparseMatrix> lag_hess_xu_axu_mat(_dim_axu, _dim_xu, _ocp_problem.lagrangeHessXuAxuNnz(_i_phase),
                                                 _p_lag_hess_xu_axu_outer_start,
                                                 lag_hess_xu_axu_rows,
                                                 lag_hess_xu_axu,
                                                 0);

    integer lag_hess_xu_p_rows[_ocp_problem.lagrangeHessXuPNnz(_i_phase)];
    integer lag_hess_xu_p_cols[_ocp_problem.lagrangeHessXuPNnz(_i_phase)];
    _ocp_problem.lagrangeHessXuPPattern(_i_phase, lag_hess_xu_p_rows, lag_hess_xu_p_cols);
    Eigen::Map<SparseMatrix> lag_hess_xu_p_mat(_dim_p, _dim_xu, _ocp_problem.lagrangeHessXuPNnz(_i_phase),
                                               _p_lag_hess_xu_p_outer_start,
                                               lag_hess_xu_p_rows,
                                               lag_hess_xu_p,
                                               0);

    integer lag_hess_dxu_dxu_rows[_ocp_problem.lagrangeHessDxuDxuNnz(_i_phase)];
    integer lag_hess_dxu_dxu_cols[_ocp_problem.lagrangeHessDxuDxuNnz(_i_phase)];
    _ocp_problem.lagrangeHessDxuDxuPattern(_i_phase, lag_hess_dxu_dxu_rows, lag_hess_dxu_dxu_cols);
    Eigen::Map<SparseMatrix> lag_hess_dxu_dxu_mat(_dim_xu, _dim_xu, _ocp_problem.lagrangeHessDxuDxuNnz(_i_phase),
                                                  _p_lag_hess_dxu_dxu_outer_start,
                                                  lag_hess_dxu_dxu_rows,
                                                  lag_hess_dxu_dxu,
                                                  0);

    integer lag_hess_dxu_axu_rows[_ocp_problem.lagrangeHessDxuAxuNnz(_i_phase)];
    integer lag_hess_dxu_axu_cols[_ocp_problem.lagrangeHessDxuAxuNnz(_i_phase)];
    _ocp_problem.lagrangeHessDxuAxuPattern(_i_phase, lag_hess_dxu_axu_rows, lag_hess_dxu_axu_cols);
    Eigen::Map<SparseMatrix> lag_hess_dxu_axu_mat(_dim_axu, _dim_xu, _ocp_problem.lagrangeHessDxuAxuNnz(_i_phase),
                                                _p_lag_hess_dxu_axu_outer_start,
                                                lag_hess_dxu_axu_rows,
                                                lag_hess_dxu_axu,
                                                0);

    integer lag_hess_dxu_p_rows[_ocp_problem.lagrangeHessDxuPNnz(_i_phase)];
    integer lag_hess_dxu_p_cols[_ocp_problem.lagrangeHessDxuPNnz(_i_phase)];
    _ocp_problem.lagrangeHessDxuPPattern(_i_phase, lag_hess_dxu_p_rows, lag_hess_dxu_p_cols);
    Eigen::Map<SparseMatrix> lag_hess_dxu_p_mat(_dim_p, _dim_xu, _ocp_problem.lagrangeHessDxuPNnz(_i_phase),
                                                _p_lag_hess_dxu_p_outer_start,
                                                lag_hess_dxu_p_rows,
                                                lag_hess_dxu_p,
                                                0);

    integer lag_hess_axu_axu_rows[_ocp_problem.lagrangeHessAxuAxuNnz(_i_phase)];
    integer lag_hess_axu_axu_cols[_ocp_problem.lagrangeHessAxuAxuNnz(_i_phase)];
    _ocp_problem.lagrangeHessAxuAxuPattern(_i_phase, lag_hess_axu_axu_rows, lag_hess_axu_axu_cols);
    Eigen::Map<SparseMatrix> lag_hess_axu_axu_mat(_dim_axu, _dim_axu, _ocp_problem.lagrangeHessAxuAxuNnz(_i_phase),
                                                _p_lag_hess_axu_axu_outer_start,
                                                lag_hess_axu_axu_rows,
                                                lag_hess_axu_axu,
                                                0);

    integer lag_hess_axu_p_rows[_ocp_problem.lagrangeHessAxuPNnz(_i_phase)];
    integer lag_hess_axu_p_cols[_ocp_problem.lagrangeHessAxuPNnz(_i_phase)];
    _ocp_problem.lagrangeHessAxuAxuPattern(_i_phase, lag_hess_axu_p_rows, lag_hess_axu_p_cols);
    Eigen::Map<SparseMatrix> lag_hess_axu_p_mat(_dim_p, _dim_axu, _ocp_problem.lagrangeHessAxuPNnz(_i_phase),
                                                _p_lag_hess_axu_p_outer_start,
                                                lag_hess_axu_p_rows,
                                                lag_hess_axu_p,
                                                0);

    integer lag_hess_p_p_rows[_ocp_problem.lagrangeHessPPNnz(_i_phase)];
    integer lag_hess_p_p_cols[_ocp_problem.lagrangeHessPPNnz(_i_phase)];
    _ocp_problem.lagrangeHessPPPattern(_i_phase, lag_hess_p_p_rows, lag_hess_p_p_cols);
    Eigen::Map<SparseMatrix> lag_hess_p_p_mat(_dim_p, _dim_p, _ocp_problem.lagrangeHessPPNnz(_i_phase),
                                              _p_lag_hess_p_p_outer_start,
                                              lag_hess_p_p_rows,
                                              lag_hess_p_p,
                                              0);

    // create fo_eqns matrixes
    integer fo_eqns_hess_xu_xu_rows[_ocp_problem.foEqnsHessXuXuNnz(_i_phase)];
    integer fo_eqns_hess_xu_xu_cols[_ocp_problem.foEqnsHessXuXuNnz(_i_phase)];
    _ocp_problem.foEqnsHessXuXuPattern(_i_phase, fo_eqns_hess_xu_xu_rows, fo_eqns_hess_xu_xu_cols);
    Eigen::Map<SparseMatrix> fo_eqns_hess_xu_xu_mat(_dim_xu, _dim_xu, _ocp_problem.foEqnsHessXuXuNnz(_i_phase),
                                                _p_fo_eqns_hess_xu_xu_outer_start,
                                                fo_eqns_hess_xu_xu_rows,
                                                fo_eqns_hess_xu_xu,
                                                0);

    integer fo_eqns_hess_xu_dxu_rows[_ocp_problem.foEqnsHessXuDxuNnz(_i_phase)];
    integer fo_eqns_hess_xu_dxu_cols[_ocp_problem.foEqnsHessXuDxuNnz(_i_phase)];
    _ocp_problem.foEqnsHessXuDxuPattern(_i_phase, fo_eqns_hess_xu_dxu_rows, fo_eqns_hess_xu_dxu_cols);
    Eigen::Map<SparseMatrix> fo_eqns_hess_xu_dxu_mat(_dim_xu, _dim_xu, _ocp_problem.foEqnsHessXuDxuNnz(_i_phase),
                                                 _p_fo_eqns_hess_xu_dxu_outer_start,
                                                 fo_eqns_hess_xu_dxu_rows,
                                                 fo_eqns_hess_xu_dxu,
                                                 0);

    integer fo_eqns_hess_xu_axu_rows[_ocp_problem.foEqnsHessXuAxuNnz(_i_phase)];
    integer fo_eqns_hess_xu_axu_cols[_ocp_problem.foEqnsHessXuAxuNnz(_i_phase)];
    _ocp_problem.foEqnsHessXuAxuPattern(_i_phase, fo_eqns_hess_xu_axu_rows, fo_eqns_hess_xu_axu_cols);
    Eigen::Map<SparseMatrix> fo_eqns_hess_xu_axu_mat(_dim_axu, _dim_xu, _ocp_problem.foEqnsHessXuAxuNnz(_i_phase),
                                                 _p_fo_eqns_hess_xu_axu_outer_start,
                                                 fo_eqns_hess_xu_axu_rows,
                                                 fo_eqns_hess_xu_axu,
                                                 0);

    integer fo_eqns_hess_xu_p_rows[_ocp_problem.foEqnsHessXuPNnz(_i_phase)];
    integer fo_eqns_hess_xu_p_cols[_ocp_problem.foEqnsHessXuPNnz(_i_phase)];
    _ocp_problem.foEqnsHessXuPPattern(_i_phase, fo_eqns_hess_xu_p_rows, fo_eqns_hess_xu_p_cols);
    Eigen::Map<SparseMatrix> fo_eqns_hess_xu_p_mat(_dim_p, _dim_xu, _ocp_problem.foEqnsHessXuPNnz(_i_phase),
                                               _p_fo_eqns_hess_xu_p_outer_start,
                                               fo_eqns_hess_xu_p_rows,
                                               fo_eqns_hess_xu_p,
                                               0);

    integer fo_eqns_hess_dxu_dxu_rows[_ocp_problem.foEqnsHessDxuDxuNnz(_i_phase)];
    integer fo_eqns_hess_dxu_dxu_cols[_ocp_problem.foEqnsHessDxuDxuNnz(_i_phase)];
    _ocp_problem.foEqnsHessDxuDxuPattern(_i_phase, fo_eqns_hess_dxu_dxu_rows, fo_eqns_hess_dxu_dxu_cols);
    Eigen::Map<SparseMatrix> fo_eqns_hess_dxu_dxu_mat(_dim_xu, _dim_xu, _ocp_problem.foEqnsHessDxuDxuNnz(_i_phase),
                                                  _p_fo_eqns_hess_dxu_dxu_outer_start,
                                                  fo_eqns_hess_dxu_dxu_rows,
                                                  fo_eqns_hess_dxu_dxu,
                                                  0);

    integer fo_eqns_hess_dxu_axu_rows[_ocp_problem.foEqnsHessDxuAxuNnz(_i_phase)];
    integer fo_eqns_hess_dxu_axu_cols[_ocp_problem.foEqnsHessDxuAxuNnz(_i_phase)];
    _ocp_problem.foEqnsHessDxuAxuPattern(_i_phase, fo_eqns_hess_dxu_axu_rows, fo_eqns_hess_dxu_axu_cols);
    Eigen::Map<SparseMatrix> fo_eqns_hess_dxu_axu_mat(_dim_axu, _dim_xu, _ocp_problem.foEqnsHessDxuAxuNnz(_i_phase),
                                                _p_fo_eqns_hess_dxu_axu_outer_start,
                                                fo_eqns_hess_dxu_axu_rows,
                                                fo_eqns_hess_dxu_axu,
                                                0);

    integer fo_eqns_hess_dxu_p_rows[_ocp_problem.foEqnsHessDxuPNnz(_i_phase)];
    integer fo_eqns_hess_dxu_p_cols[_ocp_problem.foEqnsHessDxuPNnz(_i_phase)];
    _ocp_problem.foEqnsHessDxuPPattern(_i_phase, fo_eqns_hess_dxu_p_rows, fo_eqns_hess_dxu_p_cols);
    Eigen::Map<SparseMatrix> fo_eqns_hess_dxu_p_mat(_dim_p, _dim_xu, _ocp_problem.foEqnsHessDxuPNnz(_i_phase),
                                                _p_fo_eqns_hess_dxu_p_outer_start,
                                                fo_eqns_hess_dxu_p_rows,
                                                fo_eqns_hess_dxu_p,
                                                0);

    integer fo_eqns_hess_axu_axu_rows[_ocp_problem.foEqnsHessAxuAxuNnz(_i_phase)];
    integer fo_eqns_hess_axu_axu_cols[_ocp_problem.foEqnsHessAxuAxuNnz(_i_phase)];
    _ocp_problem.foEqnsHessAxuAxuPattern(_i_phase, fo_eqns_hess_axu_axu_rows, fo_eqns_hess_axu_axu_cols);
    Eigen::Map<SparseMatrix> fo_eqns_hess_axu_axu_mat(_dim_axu, _dim_axu, _ocp_problem.foEqnsHessAxuAxuNnz(_i_phase),
                                                _p_fo_eqns_hess_axu_axu_outer_start,
                                                fo_eqns_hess_axu_axu_rows,
                                                fo_eqns_hess_axu_axu,
                                                0);

    integer fo_eqns_hess_axu_p_rows[_ocp_problem.foEqnsHessAxuPNnz(_i_phase)];
    integer fo_eqns_hess_axu_p_cols[_ocp_problem.foEqnsHessAxuPNnz(_i_phase)];
    _ocp_problem.foEqnsHessAxuAxuPattern(_i_phase, fo_eqns_hess_axu_p_rows, fo_eqns_hess_axu_p_cols);
    Eigen::Map<SparseMatrix> fo_eqns_hess_axu_p_mat(_dim_p, _dim_axu, _ocp_problem.foEqnsHessAxuPNnz(_i_phase),
                                                _p_fo_eqns_hess_axu_p_outer_start,
                                                fo_eqns_hess_axu_p_rows,
                                                fo_eqns_hess_axu_p,
                                                0);

    integer fo_eqns_hess_p_p_rows[_ocp_problem.foEqnsHessPPNnz(_i_phase)];
    integer fo_eqns_hess_p_p_cols[_ocp_problem.foEqnsHessPPNnz(_i_phase)];
    _ocp_problem.foEqnsHessPPPattern(_i_phase, fo_eqns_hess_p_p_rows, fo_eqns_hess_p_p_cols);
    Eigen::Map<SparseMatrix> fo_eqns_hess_p_p_mat(_dim_p, _dim_p, _ocp_problem.foEqnsHessPPNnz(_i_phase),
                                              _p_fo_eqns_hess_p_p_outer_start,
                                              fo_eqns_hess_p_p_rows,
                                              fo_eqns_hess_p_p,
                                              0);

    // create path_constr matrixes
    integer path_constr_hess_xu_xu_rows[_ocp_problem.pathConstraintsHessXuXuNnz(_i_phase)];
    integer path_constr_hess_xu_xu_cols[_ocp_problem.pathConstraintsHessXuXuNnz(_i_phase)];
    _ocp_problem.pathConstraintsHessXuXuPattern(_i_phase, path_constr_hess_xu_xu_rows, path_constr_hess_xu_xu_cols);
    Eigen::Map<SparseMatrix> path_constr_hess_xu_xu_mat(_dim_xu, _dim_xu, _ocp_problem.pathConstraintsHessXuXuNnz(_i_phase),
                                                _p_path_constr_hess_xu_xu_outer_start,
                                                path_constr_hess_xu_xu_rows,
                                                path_constr_hess_xu_xu,
                                                0);

    integer path_constr_hess_xu_dxu_rows[_ocp_problem.pathConstraintsHessXuDxuNnz(_i_phase)];
    integer path_constr_hess_xu_dxu_cols[_ocp_problem.pathConstraintsHessXuDxuNnz(_i_phase)];
    _ocp_problem.pathConstraintsHessXuDxuPattern(_i_phase, path_constr_hess_xu_dxu_rows, path_constr_hess_xu_dxu_cols);
    Eigen::Map<SparseMatrix> path_constr_hess_xu_dxu_mat(_dim_xu, _dim_xu, _ocp_problem.pathConstraintsHessXuDxuNnz(_i_phase),
                                                 _p_path_constr_hess_xu_dxu_outer_start,
                                                 path_constr_hess_xu_dxu_rows,
                                                 path_constr_hess_xu_dxu,
                                                 0);

    integer path_constr_hess_xu_axu_rows[_ocp_problem.pathConstraintsHessXuAxuNnz(_i_phase)];
    integer path_constr_hess_xu_axu_cols[_ocp_problem.pathConstraintsHessXuAxuNnz(_i_phase)];
    _ocp_problem.pathConstraintsHessXuAxuPattern(_i_phase, path_constr_hess_xu_axu_rows, path_constr_hess_xu_axu_cols);
    Eigen::Map<SparseMatrix> path_constr_hess_xu_axu_mat(_dim_axu, _dim_xu, _ocp_problem.pathConstraintsHessXuAxuNnz(_i_phase),
                                                 _p_path_constr_hess_xu_axu_outer_start,
                                                 path_constr_hess_xu_axu_rows,
                                                 path_constr_hess_xu_axu,
                                                 0);

    integer path_constr_hess_xu_p_rows[_ocp_problem.pathConstraintsHessXuPNnz(_i_phase)];
    integer path_constr_hess_xu_p_cols[_ocp_problem.pathConstraintsHessXuPNnz(_i_phase)];
    _ocp_problem.pathConstraintsHessXuPPattern(_i_phase, path_constr_hess_xu_p_rows, path_constr_hess_xu_p_cols);
    Eigen::Map<SparseMatrix> path_constr_hess_xu_p_mat(_dim_p, _dim_xu, _ocp_problem.pathConstraintsHessXuPNnz(_i_phase),
                                               _p_path_constr_hess_xu_p_outer_start,
                                               path_constr_hess_xu_p_rows,
                                               path_constr_hess_xu_p,
                                               0);

    integer path_constr_hess_dxu_dxu_rows[_ocp_problem.pathConstraintsHessDxuDxuNnz(_i_phase)];
    integer path_constr_hess_dxu_dxu_cols[_ocp_problem.pathConstraintsHessDxuDxuNnz(_i_phase)];
    _ocp_problem.pathConstraintsHessDxuDxuPattern(_i_phase, path_constr_hess_dxu_dxu_rows, path_constr_hess_dxu_dxu_cols);
    Eigen::Map<SparseMatrix> path_constr_hess_dxu_dxu_mat(_dim_xu, _dim_xu, _ocp_problem.pathConstraintsHessDxuDxuNnz(_i_phase),
                                                  _p_path_constr_hess_dxu_dxu_outer_start,
                                                  path_constr_hess_dxu_dxu_rows,
                                                  path_constr_hess_dxu_dxu,
                                                  0);

    integer path_constr_hess_dxu_axu_rows[_ocp_problem.pathConstraintsHessDxuAxuNnz(_i_phase)];
    integer path_constr_hess_dxu_axu_cols[_ocp_problem.pathConstraintsHessDxuAxuNnz(_i_phase)];
    _ocp_problem.pathConstraintsHessDxuAxuPattern(_i_phase, path_constr_hess_dxu_axu_rows, path_constr_hess_dxu_axu_cols);
    Eigen::Map<SparseMatrix> path_constr_hess_dxu_axu_mat(_dim_axu, _dim_xu, _ocp_problem.pathConstraintsHessDxuAxuNnz(_i_phase),
                                                _p_path_constr_hess_dxu_axu_outer_start,
                                                path_constr_hess_dxu_axu_rows,
                                                path_constr_hess_dxu_axu,
                                                0);

    integer path_constr_hess_dxu_p_rows[_ocp_problem.pathConstraintsHessDxuPNnz(_i_phase)];
    integer path_constr_hess_dxu_p_cols[_ocp_problem.pathConstraintsHessDxuPNnz(_i_phase)];
    _ocp_problem.pathConstraintsHessDxuPPattern(_i_phase, path_constr_hess_dxu_p_rows, path_constr_hess_dxu_p_cols);
    Eigen::Map<SparseMatrix> path_constr_hess_dxu_p_mat(_dim_p, _dim_xu, _ocp_problem.pathConstraintsHessDxuPNnz(_i_phase),
                                                _p_path_constr_hess_dxu_p_outer_start,
                                                path_constr_hess_dxu_p_rows,
                                                path_constr_hess_dxu_p,
                                                0);

    integer path_constr_hess_axu_axu_rows[_ocp_problem.pathConstraintsHessAxuAxuNnz(_i_phase)];
    integer path_constr_hess_axu_axu_cols[_ocp_problem.pathConstraintsHessAxuAxuNnz(_i_phase)];
    _ocp_problem.pathConstraintsHessAxuAxuPattern(_i_phase, path_constr_hess_axu_axu_rows, path_constr_hess_axu_axu_cols);
    Eigen::Map<SparseMatrix> path_constr_hess_axu_axu_mat(_dim_axu, _dim_axu, _ocp_problem.pathConstraintsHessAxuAxuNnz(_i_phase),
                                                _p_path_constr_hess_axu_axu_outer_start,
                                                path_constr_hess_axu_axu_rows,
                                                path_constr_hess_axu_axu,
                                                0);

    integer path_constr_hess_axu_p_rows[_ocp_problem.pathConstraintsHessAxuPNnz(_i_phase)];
    integer path_constr_hess_axu_p_cols[_ocp_problem.pathConstraintsHessAxuPNnz(_i_phase)];
    _ocp_problem.pathConstraintsHessAxuAxuPattern(_i_phase, path_constr_hess_axu_p_rows, path_constr_hess_axu_p_cols);
    Eigen::Map<SparseMatrix> path_constr_hess_axu_p_mat(_dim_p, _dim_axu, _ocp_problem.pathConstraintsHessAxuPNnz(_i_phase),
                                                _p_path_constr_hess_axu_p_outer_start,
                                                path_constr_hess_axu_p_rows,
                                                path_constr_hess_axu_p,
                                                0);

    integer path_constr_hess_p_p_rows[_ocp_problem.pathConstraintsHessPPNnz(_i_phase)];
    integer path_constr_hess_p_p_cols[_ocp_problem.pathConstraintsHessPPNnz(_i_phase)];
    _ocp_problem.pathConstraintsHessPPPattern(_i_phase, path_constr_hess_p_p_rows, path_constr_hess_p_p_cols);
    Eigen::Map<SparseMatrix> path_constr_hess_p_p_mat(_dim_p, _dim_p, _ocp_problem.pathConstraintsHessPPNnz(_i_phase),
                                              _p_path_constr_hess_p_p_outer_start,
                                              path_constr_hess_p_p_rows,
                                              path_constr_hess_p_p,
                                              0);

    // create integral constraints matrixes
    integer int_constr_hess_xu_xu_rows[_ocp_problem.intConstraintsHessXuXuNnz(_i_phase)];
    integer int_constr_hess_xu_xu_cols[_ocp_problem.intConstraintsHessXuXuNnz(_i_phase)];
    _ocp_problem.intConstraintsHessXuXuPattern(_i_phase, int_constr_hess_xu_xu_rows, int_constr_hess_xu_xu_cols);
    Eigen::Map<SparseMatrix> int_constr_hess_xu_xu_mat(_dim_xu, _dim_xu, _ocp_problem.intConstraintsHessXuXuNnz(_i_phase),
                                                _p_int_constr_hess_xu_xu_outer_start,
                                                int_constr_hess_xu_xu_rows,
                                                int_constr_hess_xu_xu,
                                                0);

    integer int_constr_hess_xu_dxu_rows[_ocp_problem.intConstraintsHessXuDxuNnz(_i_phase)];
    integer int_constr_hess_xu_dxu_cols[_ocp_problem.intConstraintsHessXuDxuNnz(_i_phase)];
    _ocp_problem.intConstraintsHessXuDxuPattern(_i_phase, int_constr_hess_xu_dxu_rows, int_constr_hess_xu_dxu_cols);
    Eigen::Map<SparseMatrix> int_constr_hess_xu_dxu_mat(_dim_xu, _dim_xu, _ocp_problem.intConstraintsHessXuDxuNnz(_i_phase),
                                                 _p_int_constr_hess_xu_dxu_outer_start,
                                                 int_constr_hess_xu_dxu_rows,
                                                 int_constr_hess_xu_dxu,
                                                 0);

    integer int_constr_hess_xu_axu_rows[_ocp_problem.intConstraintsHessXuAxuNnz(_i_phase)];
    integer int_constr_hess_xu_axu_cols[_ocp_problem.intConstraintsHessXuAxuNnz(_i_phase)];
    _ocp_problem.intConstraintsHessXuAxuPattern(_i_phase, int_constr_hess_xu_axu_rows, int_constr_hess_xu_axu_cols);
    Eigen::Map<SparseMatrix> int_constr_hess_xu_axu_mat(_dim_axu, _dim_xu, _ocp_problem.intConstraintsHessXuAxuNnz(_i_phase),
                                                 _p_int_constr_hess_xu_axu_outer_start,
                                                 int_constr_hess_xu_axu_rows,
                                                 int_constr_hess_xu_axu,
                                                 0);

    integer int_constr_hess_xu_p_rows[_ocp_problem.intConstraintsHessXuPNnz(_i_phase)];
    integer int_constr_hess_xu_p_cols[_ocp_problem.intConstraintsHessXuPNnz(_i_phase)];
    _ocp_problem.intConstraintsHessXuPPattern(_i_phase, int_constr_hess_xu_p_rows, int_constr_hess_xu_p_cols);
    Eigen::Map<SparseMatrix> int_constr_hess_xu_p_mat(_dim_p, _dim_xu, _ocp_problem.intConstraintsHessXuPNnz(_i_phase),
                                               _p_int_constr_hess_xu_p_outer_start,
                                               int_constr_hess_xu_p_rows,
                                               int_constr_hess_xu_p,
                                               0);

    integer int_constr_hess_dxu_dxu_rows[_ocp_problem.intConstraintsHessDxuDxuNnz(_i_phase)];
    integer int_constr_hess_dxu_dxu_cols[_ocp_problem.intConstraintsHessDxuDxuNnz(_i_phase)];
    _ocp_problem.intConstraintsHessDxuDxuPattern(_i_phase, int_constr_hess_dxu_dxu_rows, int_constr_hess_dxu_dxu_cols);
    Eigen::Map<SparseMatrix> int_constr_hess_dxu_dxu_mat(_dim_xu, _dim_xu, _ocp_problem.intConstraintsHessDxuDxuNnz(_i_phase),
                                                  _p_int_constr_hess_dxu_dxu_outer_start,
                                                  int_constr_hess_dxu_dxu_rows,
                                                  int_constr_hess_dxu_dxu,
                                                  0);

    integer int_constr_hess_dxu_axu_rows[_ocp_problem.intConstraintsHessDxuAxuNnz(_i_phase)];
    integer int_constr_hess_dxu_axu_cols[_ocp_problem.intConstraintsHessDxuAxuNnz(_i_phase)];
    _ocp_problem.intConstraintsHessDxuAxuPattern(_i_phase, int_constr_hess_dxu_axu_rows, int_constr_hess_dxu_axu_cols);
    Eigen::Map<SparseMatrix> int_constr_hess_dxu_axu_mat(_dim_axu, _dim_xu, _ocp_problem.intConstraintsHessDxuAxuNnz(_i_phase),
                                                _p_int_constr_hess_dxu_axu_outer_start,
                                                int_constr_hess_dxu_axu_rows,
                                                int_constr_hess_dxu_axu,
                                                0);

    integer int_constr_hess_dxu_p_rows[_ocp_problem.intConstraintsHessDxuPNnz(_i_phase)];
    integer int_constr_hess_dxu_p_cols[_ocp_problem.intConstraintsHessDxuPNnz(_i_phase)];
    _ocp_problem.intConstraintsHessDxuPPattern(_i_phase, int_constr_hess_dxu_p_rows, int_constr_hess_dxu_p_cols);
    Eigen::Map<SparseMatrix> int_constr_hess_dxu_p_mat(_dim_p, _dim_xu, _ocp_problem.intConstraintsHessDxuPNnz(_i_phase),
                                                _p_int_constr_hess_dxu_p_outer_start,
                                                int_constr_hess_dxu_p_rows,
                                                int_constr_hess_dxu_p,
                                                0);

    integer int_constr_hess_axu_axu_rows[_ocp_problem.intConstraintsHessAxuAxuNnz(_i_phase)];
    integer int_constr_hess_axu_axu_cols[_ocp_problem.intConstraintsHessAxuAxuNnz(_i_phase)];
    _ocp_problem.intConstraintsHessAxuAxuPattern(_i_phase, int_constr_hess_axu_axu_rows, int_constr_hess_axu_axu_cols);
    Eigen::Map<SparseMatrix> int_constr_hess_axu_axu_mat(_dim_axu, _dim_axu, _ocp_problem.intConstraintsHessAxuAxuNnz(_i_phase),
                                                _p_int_constr_hess_axu_axu_outer_start,
                                                int_constr_hess_axu_axu_rows,
                                                int_constr_hess_axu_axu,
                                                0);

    integer int_constr_hess_axu_p_rows[_ocp_problem.intConstraintsHessAxuPNnz(_i_phase)];
    integer int_constr_hess_axu_p_cols[_ocp_problem.intConstraintsHessAxuPNnz(_i_phase)];
    _ocp_problem.intConstraintsHessAxuAxuPattern(_i_phase, int_constr_hess_axu_p_rows, int_constr_hess_axu_p_cols);
    Eigen::Map<SparseMatrix> int_constr_hess_axu_p_mat(_dim_p, _dim_axu, _ocp_problem.intConstraintsHessAxuPNnz(_i_phase),
                                                _p_int_constr_hess_axu_p_outer_start,
                                                int_constr_hess_axu_p_rows,
                                                int_constr_hess_axu_p,
                                                0);

    integer int_constr_hess_p_p_rows[_ocp_problem.intConstraintsHessPPNnz(_i_phase)];
    integer int_constr_hess_p_p_cols[_ocp_problem.intConstraintsHessPPNnz(_i_phase)];
    _ocp_problem.intConstraintsHessPPPattern(_i_phase, int_constr_hess_p_p_rows, int_constr_hess_p_p_cols);
    Eigen::Map<SparseMatrix> int_constr_hess_p_p_mat(_dim_p, _dim_p, _ocp_problem.intConstraintsHessPPNnz(_i_phase),
                                              _p_int_constr_hess_p_p_outer_start,
                                              int_constr_hess_p_p_rows,
                                              int_constr_hess_p_p,
                                              0);

    // create point_constr matrixes
    integer point_constr_hess_xu_xu_rows[_ocp_problem.pointConstraintsHessXuXuNnz(_i_phase)];
    integer point_constr_hess_xu_xu_cols[_ocp_problem.pointConstraintsHessXuXuNnz(_i_phase)];
    _ocp_problem.pointConstraintsHessXuXuPattern(_i_phase, point_constr_hess_xu_xu_rows, point_constr_hess_xu_xu_cols);
    Eigen::Map<SparseMatrix> point_constr_hess_xu_xu_mat(_dim_xu, _dim_xu, _ocp_problem.pointConstraintsHessXuXuNnz(_i_phase),
                                                         _p_point_constr_hess_xu_xu_outer_start,
                                                         point_constr_hess_xu_xu_rows,
                                                         point_constr_hess_xu_xu,
                                                         0);

    integer point_constr_hess_xu_p_rows[_ocp_problem.pointConstraintsHessXuPNnz(_i_phase)];
    integer point_constr_hess_xu_p_cols[_ocp_problem.pointConstraintsHessXuPNnz(_i_phase)];
    _ocp_problem.pointConstraintsHessXuPPattern(_i_phase, point_constr_hess_xu_p_rows, point_constr_hess_xu_p_cols);
    Eigen::Map<SparseMatrix> point_constr_hess_xu_p_mat(_dim_p, _dim_xu, _ocp_problem.pointConstraintsHessXuPNnz(_i_phase),
                                                        _p_point_constr_hess_xu_p_outer_start,
                                                        point_constr_hess_xu_p_rows,
                                                        point_constr_hess_xu_p,
                                                        0);

    integer point_constr_hess_p_p_rows[_ocp_problem.pointConstraintsHessPPNnz(_i_phase)];
    integer point_constr_hess_p_p_cols[_ocp_problem.pointConstraintsHessPPNnz(_i_phase)];
    _ocp_problem.pointConstraintsHessPPPattern(_i_phase, point_constr_hess_p_p_rows, point_constr_hess_p_p_cols);
    Eigen::Map<SparseMatrix> point_constr_hess_p_p_mat(_dim_p, _dim_p, _ocp_problem.pointConstraintsHessPPNnz(_i_phase),
                                                       _p_point_constr_hess_p_p_outer_start,
                                                       point_constr_hess_p_p_rows,
                                                       point_constr_hess_p_p,
                                                       0);


    SparseMatrix tmp_xu_xu_mat(_dim_xu, _dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    tmp_xu_xu_mat.reserve(_hess_y_y_lower_mat_nnz*2);
#endif
    tmp_xu_xu_mat = ( lag_hess_xu_xu_mat * d_zeta + fo_eqns_hess_xu_xu_mat + path_constr_hess_xu_xu_mat
                        + int_constr_hess_xu_xu_mat * d_zeta) * 0.25;

    SparseMatrix tmp_dxu_dxu_mat(_dim_xu, _dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    tmp_dxu_dxu_mat.reserve(_hess_y_y_lower_mat_nnz*2);
#endif
    tmp_dxu_dxu_mat = ( lag_hess_dxu_dxu_mat * d_zeta + fo_eqns_hess_dxu_dxu_mat + path_constr_hess_dxu_dxu_mat
                          + int_constr_hess_dxu_dxu_mat * d_zeta ) * ( d_zeta_inv * d_zeta_inv );

    SparseMatrix tmp_xu_dxu_mat(_dim_xu, _dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    tmp_xu_dxu_mat.reserve(_hess_y_y_lower_mat_nnz*2);
#endif
    tmp_xu_dxu_mat = ( lag_hess_xu_dxu_mat
                                + ( fo_eqns_hess_xu_dxu_mat + path_constr_hess_xu_dxu_mat ) * d_zeta_inv
                                + int_constr_hess_xu_dxu_mat ) * 0.5;

    SparseMatrix tmp_dxu_xu_mat(_dim_xu, _dim_xu);
    tmp_dxu_xu_mat = tmp_xu_dxu_mat.transpose();

    // FIRST BLOCK
    SparseMatrix hess_y_y_lower_mat(_dim_xu, _dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    hess_y_y_lower_mat.reserve(_hess_y_y_lower_mat_nnz);
#endif
    hess_y_y_lower_mat = tmp_xu_xu_mat.triangularView<Eigen::Lower>() + tmp_dxu_dxu_mat.triangularView<Eigen::Lower>()
                               - tmp_xu_dxu_mat.triangularView<Eigen::Lower>() - tmp_dxu_xu_mat.triangularView<Eigen::Lower>()
                               + point_constr_hess_xu_xu_mat.triangularView<Eigen::Lower>();
    multiplyAndSumVectorTo((real*) hess_y_y_lower_mat.valuePtr(), p_current_hessian, _p_scale_factor_hess_y_y_lower_mat, _hess_y_y_lower_mat_nnz);
    p_current_hessian += _hess_y_y_lower_mat_nnz;

    // EIGHT BLOCK: it is very similar to the first, except for a sign
    SparseMatrix hess_y_y_next_lower_mat(_dim_xu, _dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    hess_y_y_next_lower_mat.reserve(_hess_y_y_lower_mat_nnz);
#endif
    hess_y_y_next_lower_mat = tmp_xu_xu_mat.triangularView<Eigen::Lower>() + tmp_dxu_dxu_mat.triangularView<Eigen::Lower>()
                                + tmp_xu_dxu_mat.triangularView<Eigen::Lower>() + tmp_dxu_xu_mat.triangularView<Eigen::Lower>()
                                + 0 * point_constr_hess_xu_xu_mat.triangularView<Eigen::Lower>();
    multiplyAndCopyVectorTo((real*) hess_y_y_next_lower_mat.valuePtr(), hessian_y_y_next , _p_scale_factor_hess_y_y_lower_mat, _hess_y_y_lower_mat_nnz);

    // SECOND BLOCK: _y_ay
    SparseMatrix tmp_xu_axu_mat(_dim_axu, _dim_xu);
    tmp_xu_axu_mat = ( lag_hess_xu_axu_mat * d_zeta + fo_eqns_hess_xu_axu_mat + path_constr_hess_xu_axu_mat
                       + int_constr_hess_xu_axu_mat * d_zeta) * 0.5;

    SparseMatrix tmp_dxu_axu_mat(_dim_axu, _dim_xu);
    tmp_dxu_axu_mat =  lag_hess_dxu_axu_mat
                      + (fo_eqns_hess_dxu_axu_mat + path_constr_hess_dxu_axu_mat) * d_zeta_inv
                      + int_constr_hess_dxu_axu_mat ;


    SparseMatrix hess_y_ay_mat(_dim_axu, _dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    hess_y_ay_mat.reserve(_hess_y_ay_mat_nnz);
#endif
    hess_y_ay_mat = tmp_xu_axu_mat - tmp_dxu_axu_mat;
    multiplyAndCopyVectorTo((real*) hess_y_ay_mat.valuePtr(), p_current_hessian , _p_scale_factor_hess_y_ay_mat, _hess_y_ay_mat_nnz);
    p_current_hessian += _hess_y_ay_mat_nnz;

    // THIRD BLOCK: _yleft_yright
    SparseMatrix hess_yleft_yright_mat(_dim_xu, _dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    hess_yleft_yright_mat.reserve(_hess_yleft_yright_mat_nnz);
#endif
    hess_yleft_yright_mat = tmp_xu_xu_mat - tmp_dxu_dxu_mat
    + tmp_xu_dxu_mat
    - tmp_dxu_xu_mat; // TODO: REMOVE THE NUMERICAL ZERO ENTRIES. TO MODIFY ALSO THE PATTERN CACLULATIONS
    multiplyAndCopyVectorTo((real*) hess_yleft_yright_mat.valuePtr(), p_current_hessian , _p_scale_factor_hess_yleft_yright_mat, _hess_yleft_yright_mat_nnz);
    p_current_hessian += _hess_yleft_yright_mat_nnz;
    //    cout << *(p_current_hessian - _hess_2_mat_nnz) <<  "\t" << *(p_current_hessian - _hess_2_mat_nnz +1 ) <<  "\t" << *(p_current_hessian - _hess_2_mat_nnz + 2) <<  "\t";
    //    cout << "heess 2 mat:\n";
    //    cout << hess_2_mat;
    //    cout << "\n";

    // FOURTH BLOCK: _y_p
    SparseMatrix tmp_xu_p_mat(_dim_p, _dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    tmp_xu_p_mat.reserve(_hess_y_p_mat_nnz);
#endif
    tmp_xu_p_mat = (lag_hess_xu_p_mat*d_zeta + fo_eqns_hess_xu_p_mat + path_constr_hess_xu_p_mat + int_constr_hess_xu_p_mat*d_zeta ) * 0.5;

    SparseMatrix tmp_dxu_p_mat(_dim_p, _dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    tmp_dxu_p_mat.reserve(_hess_y_p_mat_nnz);
#endif
    tmp_dxu_p_mat =   lag_hess_dxu_p_mat
                       + (fo_eqns_hess_dxu_p_mat + path_constr_hess_dxu_p_mat) * d_zeta_inv
                       + int_constr_hess_dxu_p_mat ;

    SparseMatrix hess_y_p_mat(_dim_p, _dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    hess_y_p_mat.reserve(_hess_y_p_mat_nnz);
#endif
    hess_y_p_mat = tmp_xu_p_mat - tmp_dxu_p_mat + point_constr_hess_xu_p_mat;
    multiplyAndSumVectorTo((real*) hess_y_p_mat.valuePtr(), p_current_hessian, _p_scale_factor_hess_y_p_mat, _hess_y_p_mat_nnz);
    p_current_hessian += _hess_y_p_mat_nnz ;
    //    cout << "heess 3 mat:\n";
    //    cout << hess_3_mat;
    //    cout << "\n";

    // FIFTH BLOCK: _ay_ay
    SparseMatrix hess_ay_ay_lower_mat(_dim_axu, _dim_axu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    hess_ay_ay_mat.reserve(_hess_ay_ay_lower_mat_nnz);
#endif
    hess_ay_ay_lower_mat =  lag_hess_axu_axu_mat.triangularView<Eigen::Lower>() * d_zeta + fo_eqns_hess_axu_axu_mat.triangularView<Eigen::Lower>() + path_constr_hess_axu_axu_mat.triangularView<Eigen::Lower>()
                      + int_constr_hess_axu_axu_mat.triangularView<Eigen::Lower>() * d_zeta;
    multiplyAndCopyVectorTo((real*) hess_ay_ay_lower_mat.valuePtr(), p_current_hessian, _p_scale_factor_hess_ay_ay_lower_mat, _hess_ay_ay_lower_mat_nnz);
    p_current_hessian += _hess_ay_ay_lower_mat_nnz ;

    // SIXT BLOCK: _ay_y -> very similar to the second block
    SparseMatrix hess_ay_y_mat(_dim_xu, _dim_axu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    hess_ay_y_mat.reserve(_hess_y_ay_mat_nnz);
#endif
    hess_ay_y_mat = (tmp_xu_axu_mat + tmp_dxu_axu_mat).transpose();
    multiplyAndCopyVectorTo((real*) hess_ay_y_mat.valuePtr(), p_current_hessian , _p_scale_factor_hess_ay_y_mat, _hess_y_ay_mat_nnz);
    p_current_hessian += _hess_y_ay_mat_nnz;

    // SEVENTH BLOCK: _ay_p
    SparseMatrix hess_ay_p_mat(_dim_p, _dim_axu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    hess_ay_p_mat.reserve(_hess_ay_p_mat_nnz);
#endif
    hess_ay_p_mat = lag_hess_axu_p_mat*d_zeta + fo_eqns_hess_axu_p_mat + path_constr_hess_axu_p_mat + int_constr_hess_axu_p_mat*d_zeta ;
    multiplyAndCopyVectorTo((real*) hess_ay_p_mat.valuePtr(), p_current_hessian , _p_scale_factor_hess_ay_p_mat, _hess_ay_p_mat_nnz);
    p_current_hessian += _hess_ay_p_mat_nnz;

    //EIGHT BLOCK already done

    // NINETH BLOCK: it is very similar to the FOURTH, except for a sign
    SparseMatrix hess_y_p_next_mat(_dim_p, _dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    hess_y_p_next_mat.reserve(_hess_y_p_mat_nnz);
#endif
    hess_y_p_next_mat = tmp_xu_p_mat + tmp_dxu_p_mat + 0 * point_constr_hess_xu_p_mat;
    multiplyAndCopyVectorTo((real*) hess_y_p_next_mat.valuePtr(), hessian_y_p_next, _p_scale_factor_hess_y_p_mat, _hess_y_p_mat_nnz);

    //    cout << "heess 5 mat:\n";
    //    cout << hess_5_mat;
    //    cout << "\n";

#ifdef MAVERICK_DEBUG
    real * p_expected = hessian_y_y + getNlpHessianLeftColumnBlockNnz() + getNlpHessianCentreColumnBlockNnz();
    MAVERICK_ASSERT( p_current_hessian == p_expected, "MidpointOcp2NlpSinglePhase::calculateHessianBlockAtMeshMiddle: wrong pointer to first half hessian. Current: " << p_current_hessian << ", expected: " << p_expected << " difference is " << p_current_hessian - p_expected << ".\n")
#endif

    // SIXTH BLOCK
    hess_p_p_lower_mat += lag_hess_p_p_mat.triangularView<Eigen::Lower>()*d_zeta
                          + fo_eqns_hess_p_p_mat.triangularView<Eigen::Lower>()
                          + path_constr_hess_p_p_mat.triangularView<Eigen::Lower>()
                          + point_constr_hess_p_p_mat.triangularView<Eigen::Lower>()
                          + int_constr_hess_p_p_mat.triangularView<Eigen::Lower>() * d_zeta;

}

//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________

void MidpointOcp2NlpSinglePhase::calculateHessianLastConstrBcsMayer(real const initial_state_control[], real const final_state_control[], real const ocp_params[],
                                                               real const initial_zeta,
                                                               real const final_zeta,
                                                               real const last_d_z_dual,
                                                               real const lambda_not_scaled[], real const last_constr_lambda_not_scaled[], real const lambda_0_not_scaled,
                                                               SparseMatrix & out_last_point_constr_hess_xu_xu_lower_mat,
                                                               SparseMatrix & out_last_point_constr_hess_xu_p_mat,
                                                               SparseMatrix & out_last_point_constr_hess_p_p_lower_mat,
                                                               SparseMatrix & out_bcs_hess_xu_init_xu_init_lower_mat,
                                                               SparseMatrix & out_bcs_hess_xu_init_xu_fin_mat,
                                                               SparseMatrix & out_bcs_hess_xu_init_p_mat,
                                                               SparseMatrix & out_bcs_hess_xu_fin_xu_fin_lower_mat,
                                                               SparseMatrix & out_bcs_hess_xu_fin_p_mat,
                                                               SparseMatrix & out_bcs_hess_p_p_lower_mat,
                                                               SparseMatrix & out_mayer_hess_xu_init_xu_init_lower_mat,
                                                               SparseMatrix & out_mayer_hess_xu_init_xu_fin_mat,
                                                               SparseMatrix & out_mayer_hess_xu_init_p_mat,
                                                               SparseMatrix & out_mayer_hess_xu_fin_xu_fin_lower_mat,
                                                               SparseMatrix & out_mayer_hess_xu_fin_p_mat,
                                                               SparseMatrix & out_mayer_hess_p_p_lower_mat) const {
    real const * p_current_constraint_lambda = last_constr_lambda_not_scaled;
#ifdef MAVERICK_DEBUG
    real const * exp_lambda = lambda_not_scaled + getNlpConstraintsPtrIndexForInterval( _p_mesh->getNumberOfIntervals() );

    MAVERICK_ASSERT(p_current_constraint_lambda == exp_lambda, "MidpointOcp2NlpSinglePhase::calculateNLPQuantities: wrong lambda pointer for last point constraints. Current " << p_current_constraint_lambda << ", expected " << exp_lambda << ".\n")
#endif
    // get last point constr hessian
    real point_constr_hess_xu_xu[ _ocp_problem.pointConstraintsHessXuXuNnz(_i_phase) ];
    real point_constr_hess_xu_p[ _ocp_problem.pointConstraintsHessXuPNnz(_i_phase) ];
    real point_constr_hess_p_p[ _ocp_problem.pointConstraintsHessPPNnz(_i_phase) ];
    real * lambda_scaled = new real[_dim_poc];
    real p_inv_scaling_point_constr[_dim_poc];
    if (_multiply_point_constr_by_dz)
        multiplyAndCopyVectorTo(_p_inv_scaling_point_constr_global, p_inv_scaling_point_constr, last_d_z_dual, _dim_poc);
    else
        copyVectorTo(_p_inv_scaling_point_constr_global, p_inv_scaling_point_constr, _dim_poc);

    multiplyAndCopyVectorTo(p_current_constraint_lambda, lambda_scaled, p_inv_scaling_point_constr, _dim_poc);
    _ocp_problem.pointConstraintsHess(_i_phase, final_state_control, ocp_params, final_zeta, lambda_scaled,
                                      point_constr_hess_xu_xu, point_constr_hess_xu_p, point_constr_hess_p_p);
    p_current_constraint_lambda += _dim_poc;
    delete [] lambda_scaled;

    // create last point_constr matrixes
    integer point_constr_hess_xu_xu_rows[_ocp_problem.pointConstraintsHessXuXuNnz(_i_phase)];
    integer point_constr_hess_xu_xu_cols[_ocp_problem.pointConstraintsHessXuXuNnz(_i_phase)];
    _ocp_problem.pointConstraintsHessXuXuPattern(_i_phase, point_constr_hess_xu_xu_rows, point_constr_hess_xu_xu_cols);
    Eigen::Map<SparseMatrix> point_constr_hess_xu_xu_mat(_dim_xu, _dim_xu, _ocp_problem.pointConstraintsHessXuXuNnz(_i_phase),
                                                         _p_point_constr_hess_xu_xu_outer_start,
                                                         point_constr_hess_xu_xu_rows,
                                                         point_constr_hess_xu_xu,
                                                         0);
    out_last_point_constr_hess_xu_xu_lower_mat = point_constr_hess_xu_xu_mat.triangularView<Eigen::Lower>();

    integer point_constr_hess_xu_p_rows[_ocp_problem.pointConstraintsHessXuPNnz(_i_phase)];
    integer point_constr_hess_xu_p_cols[_ocp_problem.pointConstraintsHessXuPNnz(_i_phase)];
    _ocp_problem.pointConstraintsHessXuPPattern(_i_phase, point_constr_hess_xu_p_rows, point_constr_hess_xu_p_cols);
    Eigen::Map<SparseMatrix> point_constr_hess_xu_p_mat(_dim_p, _dim_xu, _ocp_problem.pointConstraintsHessXuPNnz(_i_phase),
                                                        _p_point_constr_hess_xu_p_outer_start,
                                                        point_constr_hess_xu_p_rows,
                                                        point_constr_hess_xu_p,
                                                        0);
    out_last_point_constr_hess_xu_p_mat = point_constr_hess_xu_p_mat;

    integer point_constr_hess_p_p_rows[_ocp_problem.pointConstraintsHessPPNnz(_i_phase)];
    integer point_constr_hess_p_p_cols[_ocp_problem.pointConstraintsHessPPNnz(_i_phase)];
    _ocp_problem.pointConstraintsHessPPPattern(_i_phase, point_constr_hess_p_p_rows, point_constr_hess_p_p_cols);
    Eigen::Map<SparseMatrix> point_constr_hess_p_p_mat(_dim_p, _dim_p, _ocp_problem.pointConstraintsHessPPNnz(_i_phase),
                                                       _p_point_constr_hess_p_p_outer_start,
                                                       point_constr_hess_p_p_rows,
                                                       point_constr_hess_p_p,
                                                       0);
    out_last_point_constr_hess_p_p_lower_mat = point_constr_hess_p_p_mat.triangularView<Eigen::Lower>();


#ifdef MAVERICK_DEBUG
    exp_lambda = lambda_not_scaled + getNlpConstraintsSize() - _dim_ic - _dim_bc;

    MAVERICK_ASSERT(p_current_constraint_lambda == exp_lambda, "MidpointOcp2NlpSinglePhase::calculateNLPQuantities: wrong lambda pointer for boundary conditions.\n")
#endif
    real bcs_hess_xu_init_xu_init[ _ocp_problem.boundaryConditionsHessXuInitXuInitNnz(_i_phase) ];
    real bcs_hess_xu_init_xu_fin[ _ocp_problem.boundaryConditionsHessXuInitXuFinNnz(_i_phase) ];
    real bcs_hess_xu_init_p[ _ocp_problem.boundaryConditionsHessXuInitPNnz(_i_phase) ];
    real bcs_hess_xu_fin_xu_fin[ _ocp_problem.boundaryConditionsHessXuFinXuFinNnz(_i_phase) ];
    real bcs_hess_xu_fin_p[ _ocp_problem.boundaryConditionsHessXuFinPNnz(_i_phase) ];
    real bcs_hess_p_p[ _ocp_problem.boundaryConditionsHessPPNnz(_i_phase) ];
    lambda_scaled = new real[_dim_bc];
    multiplyAndCopyVectorTo(p_current_constraint_lambda, lambda_scaled, _p_inv_scaling_bcs, _dim_bc);
    _ocp_problem.boundaryConditionsHess(_i_phase, initial_state_control, final_state_control, ocp_params, initial_zeta, final_zeta, lambda_scaled,
                                        bcs_hess_xu_init_xu_init, bcs_hess_xu_init_xu_fin, bcs_hess_xu_init_p, bcs_hess_xu_fin_xu_fin, bcs_hess_xu_fin_p, bcs_hess_p_p);
    p_current_constraint_lambda += _dim_bc;
    delete [] lambda_scaled;
#ifdef MAVERICK_DEBUG
    exp_lambda = lambda_not_scaled + getNlpConstraintsSize() - _dim_ic ;

    MAVERICK_ASSERT(p_current_constraint_lambda == exp_lambda, "MidpointOcp2NlpSinglePhase::calculateNLPQuantities: wrong final lambda pointer.\n")
#endif
    //create matrixes
    integer bcs_hess_xu_init_xu_init_rows[ _ocp_problem.boundaryConditionsHessXuInitXuInitNnz(_i_phase) ];
    integer bcs_hess_xu_init_xu_init_cols[ _ocp_problem.boundaryConditionsHessXuInitXuInitNnz(_i_phase) ];
    _ocp_problem.boundaryConditionsHessXuInitXuInitPattern(_i_phase, bcs_hess_xu_init_xu_init_rows, bcs_hess_xu_init_xu_init_cols);
    Eigen::Map<SparseMatrix> bcs_hess_xu_init_xu_init_mat(_dim_xu, _dim_xu, _ocp_problem.boundaryConditionsHessXuInitXuInitNnz(_i_phase),
                                                          _p_bcs_hess_xu_init_xu_init_outer_start,
                                                          bcs_hess_xu_init_xu_init_rows,
                                                          bcs_hess_xu_init_xu_init,
                                                          0);
    out_bcs_hess_xu_init_xu_init_lower_mat = bcs_hess_xu_init_xu_init_mat.triangularView<Eigen::Lower>();

    integer bcs_hess_xu_init_xu_fin_rows[ _ocp_problem.boundaryConditionsHessXuInitXuFinNnz(_i_phase) ];
    integer bcs_hess_xu_init_xu_fin_cols[ _ocp_problem.boundaryConditionsHessXuInitXuFinNnz(_i_phase) ];
    _ocp_problem.boundaryConditionsHessXuInitXuFinPattern(_i_phase, bcs_hess_xu_init_xu_fin_rows, bcs_hess_xu_init_xu_fin_cols);
    Eigen::Map<SparseMatrix> bcs_hess_xu_init_xu_fin_mat(_dim_xu, _dim_xu, _ocp_problem.boundaryConditionsHessXuInitXuFinNnz(_i_phase),
                                                         _p_bcs_hess_xu_init_xu_fin_outer_start,
                                                         bcs_hess_xu_init_xu_fin_rows,
                                                         bcs_hess_xu_init_xu_fin,
                                                         0);
    out_bcs_hess_xu_init_xu_fin_mat = bcs_hess_xu_init_xu_fin_mat;

    integer bcs_hess_xu_init_p_rows[ _ocp_problem.boundaryConditionsHessXuInitPNnz(_i_phase) ];
    integer bcs_hess_xu_init_p_cols[ _ocp_problem.boundaryConditionsHessXuInitPNnz(_i_phase) ];
    _ocp_problem.boundaryConditionsHessXuInitPPattern(_i_phase, bcs_hess_xu_init_p_rows, bcs_hess_xu_init_p_cols);
    Eigen::Map<SparseMatrix> bcs_hess_xu_init_p_mat(_dim_p, _dim_xu, _ocp_problem.boundaryConditionsHessXuInitPNnz(_i_phase),
                                                    _p_bcs_hess_xu_init_p_outer_start,
                                                    bcs_hess_xu_init_p_rows,
                                                    bcs_hess_xu_init_p,
                                                    0);
    out_bcs_hess_xu_init_p_mat = bcs_hess_xu_init_p_mat;

    integer bcs_hess_xu_fin_xu_fin_rows[ _ocp_problem.boundaryConditionsHessXuFinXuFinNnz(_i_phase) ];
    integer bcs_hess_xu_fin_xu_fin_cols[ _ocp_problem.boundaryConditionsHessXuFinXuFinNnz(_i_phase) ];
    _ocp_problem.boundaryConditionsHessXuFinXuFinPattern(_i_phase, bcs_hess_xu_fin_xu_fin_rows, bcs_hess_xu_fin_xu_fin_cols);
    Eigen::Map<SparseMatrix> bcs_hess_xu_fin_xu_fin_mat(_dim_xu, _dim_xu, _ocp_problem.boundaryConditionsHessXuFinXuFinNnz(_i_phase),
                                                        _p_bcs_hess_xu_fin_xu_fin_outer_start,
                                                        bcs_hess_xu_fin_xu_fin_rows,
                                                        bcs_hess_xu_fin_xu_fin,
                                                        0);
    out_bcs_hess_xu_fin_xu_fin_lower_mat = bcs_hess_xu_fin_xu_fin_mat.triangularView<Eigen::Lower>();

    integer bcs_hess_xu_fin_p_rows[ _ocp_problem.boundaryConditionsHessXuFinPNnz(_i_phase) ];
    integer bcs_hess_xu_fin_p_cols[ _ocp_problem.boundaryConditionsHessXuFinPNnz(_i_phase) ];
    _ocp_problem.boundaryConditionsHessXuFinPPattern(_i_phase, bcs_hess_xu_fin_p_rows, bcs_hess_xu_fin_p_cols);
    Eigen::Map<SparseMatrix> bcs_hess_xu_fin_p_mat(_dim_p, _dim_xu, _ocp_problem.boundaryConditionsHessXuFinPNnz(_i_phase),
                                                   _p_bcs_hess_xu_fin_p_outer_start,
                                                   bcs_hess_xu_fin_p_rows,
                                                   bcs_hess_xu_fin_p,
                                                   0);
    out_bcs_hess_xu_fin_p_mat = bcs_hess_xu_fin_p_mat;

    integer bcs_hess_p_p_rows[ _ocp_problem.boundaryConditionsHessPPNnz(_i_phase) ];
    integer bcs_hess_p_p_cols[ _ocp_problem.boundaryConditionsHessPPNnz(_i_phase) ];
    _ocp_problem.boundaryConditionsHessPPPattern(_i_phase, bcs_hess_p_p_rows, bcs_hess_p_p_cols);
    Eigen::Map<SparseMatrix> bcs_hess_p_p_mat(_dim_p, _dim_p, _ocp_problem.boundaryConditionsHessPPNnz(_i_phase),
                                              _p_bcs_hess_p_p_outer_start,
                                              bcs_hess_p_p_rows,
                                              bcs_hess_p_p,
                                              0);
    out_bcs_hess_p_p_lower_mat = bcs_hess_p_p_mat.triangularView<Eigen::Lower>();

    // Mayer target hessian
    real mayer_hess_xu_init_xu_init[ _ocp_problem.mayerHessXuInitXuInitNnz(_i_phase) ];
    real mayer_hess_xu_init_xu_fin[ _ocp_problem.mayerHessXuInitXuFinNnz(_i_phase) ];
    real mayer_hess_xu_init_p[ _ocp_problem.mayerHessXuInitPNnz(_i_phase) ];
    real mayer_hess_xu_fin_xu_fin[ _ocp_problem.mayerHessXuFinXuFinNnz(_i_phase) ];
    real mayer_hess_xu_fin_p[ _ocp_problem.mayerHessXuFinPNnz(_i_phase) ];
    real mayer_hess_p_p[ _ocp_problem.mayerHessPPNnz(_i_phase) ];
    _ocp_problem.mayerHess(_i_phase, initial_state_control, final_state_control, ocp_params, lambda_0_not_scaled * _inv_scaling_target,
                           mayer_hess_xu_init_xu_init, mayer_hess_xu_init_xu_fin, mayer_hess_xu_init_p, mayer_hess_xu_fin_xu_fin, mayer_hess_xu_fin_p, mayer_hess_p_p);

    //create matrixes
    integer mayer_hess_xu_init_xu_init_rows[ _ocp_problem.mayerHessXuInitXuInitNnz(_i_phase) ];
    integer mayer_hess_xu_init_xu_init_cols[ _ocp_problem.mayerHessXuInitXuInitNnz(_i_phase) ];
    _ocp_problem.mayerHessXuInitXuInitPattern(_i_phase, mayer_hess_xu_init_xu_init_rows, mayer_hess_xu_init_xu_init_cols);
    Eigen::Map<SparseMatrix> mayer_hess_xu_init_xu_init_mat(_dim_xu, _dim_xu, _ocp_problem.mayerHessXuInitXuInitNnz(_i_phase),
                                                            _p_mayer_hess_xu_init_xu_init_outer_start,
                                                            mayer_hess_xu_init_xu_init_rows,
                                                            mayer_hess_xu_init_xu_init,
                                                            0);
    out_mayer_hess_xu_init_xu_init_lower_mat = mayer_hess_xu_init_xu_init_mat.triangularView<Eigen::Lower>();

    integer mayer_hess_xu_init_xu_fin_rows[ _ocp_problem.mayerHessXuInitXuFinNnz(_i_phase) ];
    integer mayer_hess_xu_init_xu_fin_cols[ _ocp_problem.mayerHessXuInitXuFinNnz(_i_phase) ];
    _ocp_problem.mayerHessXuInitXuFinPattern(_i_phase, mayer_hess_xu_init_xu_fin_rows, mayer_hess_xu_init_xu_fin_cols);
    Eigen::Map<SparseMatrix> mayer_hess_xu_init_xu_fin_mat(_dim_xu, _dim_xu, _ocp_problem.mayerHessXuInitXuFinNnz(_i_phase),
                                                           _p_mayer_hess_xu_init_xu_fin_outer_start,
                                                           mayer_hess_xu_init_xu_fin_rows,
                                                           mayer_hess_xu_init_xu_fin,
                                                           0);
    out_mayer_hess_xu_init_xu_fin_mat = mayer_hess_xu_init_xu_fin_mat;

    integer mayer_hess_xu_init_p_rows[ _ocp_problem.mayerHessXuInitPNnz(_i_phase) ];
    integer mayer_hess_xu_init_p_cols[ _ocp_problem.mayerHessXuInitPNnz(_i_phase) ];
    _ocp_problem.mayerHessXuInitPPattern(_i_phase, mayer_hess_xu_init_p_rows, mayer_hess_xu_init_p_cols);
    Eigen::Map<SparseMatrix> mayer_hess_xu_init_p_mat(_dim_p, _dim_xu, _ocp_problem.mayerHessXuInitPNnz(_i_phase),
                                                      _p_mayer_hess_xu_init_p_outer_start,
                                                      mayer_hess_xu_init_p_rows,
                                                      mayer_hess_xu_init_p,
                                                      0);
    out_mayer_hess_xu_init_p_mat = mayer_hess_xu_init_p_mat;

    integer mayer_hess_xu_fin_xu_fin_rows[ _ocp_problem.mayerHessXuFinXuFinNnz(_i_phase) ];
    integer mayer_hess_xu_fin_xu_fin_cols[ _ocp_problem.mayerHessXuFinXuFinNnz(_i_phase) ];
    _ocp_problem.mayerHessXuFinXuFinPattern(_i_phase, mayer_hess_xu_fin_xu_fin_rows, mayer_hess_xu_fin_xu_fin_cols);
    Eigen::Map<SparseMatrix> mayer_hess_xu_fin_xu_fin_mat(_dim_xu, _dim_xu, _ocp_problem.mayerHessXuFinXuFinNnz(_i_phase),
                                                          _p_mayer_hess_xu_fin_xu_fin_outer_start,
                                                          mayer_hess_xu_fin_xu_fin_rows,
                                                          mayer_hess_xu_fin_xu_fin,
                                                          0);
    out_mayer_hess_xu_fin_xu_fin_lower_mat = mayer_hess_xu_fin_xu_fin_mat.triangularView<Eigen::Lower>();

    integer mayer_hess_xu_fin_p_rows[ _ocp_problem.mayerHessXuFinPNnz(_i_phase) ];
    integer mayer_hess_xu_fin_p_cols[ _ocp_problem.mayerHessXuFinPNnz(_i_phase) ];
    _ocp_problem.mayerHessXuFinPPattern(_i_phase, mayer_hess_xu_fin_p_rows, mayer_hess_xu_fin_p_cols);
    Eigen::Map<SparseMatrix> mayer_hess_xu_fin_p_mat(_dim_p, _dim_xu, _ocp_problem.mayerHessXuFinPNnz(_i_phase),
                                                     _p_mayer_hess_xu_fin_p_outer_start,
                                                     mayer_hess_xu_fin_p_rows,
                                                     mayer_hess_xu_fin_p,
                                                     0);
    out_mayer_hess_xu_fin_p_mat = mayer_hess_xu_fin_p_mat;

    integer mayer_hess_p_p_rows[ _ocp_problem.mayerHessPPNnz(_i_phase) ];
    integer mayer_hess_p_p_cols[ _ocp_problem.mayerHessPPNnz(_i_phase) ];
    _ocp_problem.mayerHessPPPattern(_i_phase, mayer_hess_p_p_rows, mayer_hess_p_p_cols);
    Eigen::Map<SparseMatrix> mayer_hess_p_p_mat(_dim_p, _dim_p, _ocp_problem.mayerHessPPNnz(_i_phase),
                                                _p_mayer_hess_p_p_outer_start,
                                                mayer_hess_p_p_rows,
                                                mayer_hess_p_p,
                                                0);
    out_mayer_hess_p_p_lower_mat = mayer_hess_p_p_mat.triangularView<Eigen::Lower>();
}
