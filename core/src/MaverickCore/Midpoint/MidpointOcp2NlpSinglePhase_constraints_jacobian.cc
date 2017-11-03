#include "MidpointOcp2NlpSinglePhase.hh"
#include "MaverickCore/MaverickFunctions.hh"
#include "MaverickCore/MaverickPrivateDefs.hh"

using namespace Maverick;


// +-------------------------------------------+
// |    _                 _     _              |
// |   (_) __ _  ___ ___ | |__ (_) __ _ _ __   |
// |   | |/ _` |/ __/ _ \| '_ \| |/ _` | '_ \  |
// |   | | (_| | (_| (_) | |_) | | (_| | | | | |
// |  _/ |\__,_|\___\___/|_.__/|_|\__,_|_| |_| |
// | |__/                                      |
// +-------------------------------------------+


void MidpointOcp2NlpSinglePhase::setupForNlpConstraintsJacobianMatrixes() {
    //Matrix Y is organised in 10 blocks, in order:
    // 1,2,3,4: fo_eqns_j_y_left_mat, fo_eqns_j_ay_mat, fo_eqns_j_y_right_mat, fo_eqns_j_p_mat,
    // 5,6,7,8: path_constr_j_y_left_mat, path_constr_j_ay_mat, path_constr_j_y_right_mat, path_constr_j_p_mat,
    // 9,10:   point_constr_j_y_left_mat, constr_j_p_mat (constraints does not depend on y right or algebraic y)

    //create the matrix fo_eqns_j_xu
    integer nnz = _ocp_problem.foEqnsJacXuNnz(_i_phase);
    integer fo_eqns_j_xu_rows[nnz];
    integer fo_eqns_j_xu_cols[nnz];
    _ocp_problem.foEqnsJacXuPattern(_i_phase, fo_eqns_j_xu_rows, fo_eqns_j_xu_cols);
    SparseMatrix fo_eqns_j_xu_mat(_dim_fo,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    fo_eqns_j_xu_mat.reserve(nnz);
#endif
    for (integer i=0; i<nnz; i++)
        fo_eqns_j_xu_mat.insert(fo_eqns_j_xu_rows[i],fo_eqns_j_xu_cols[i]) = 1;
    fo_eqns_j_xu_mat.makeCompressed();
    _p_fo_eqns_j_xu_outer_start = new integer[fo_eqns_j_xu_mat.outerSize() + 1];
    copyVectorTo(fo_eqns_j_xu_mat.outerIndexPtr(), _p_fo_eqns_j_xu_outer_start, (integer) fo_eqns_j_xu_mat.outerSize() + 1);


    //create the matrix fo_eqns_j_dxu
    nnz = _ocp_problem.foEqnsJacDxuNnz(_i_phase);
    integer fo_eqns_j_dxu_rows[nnz];
    integer fo_eqns_j_dxu_cols[nnz];
    _ocp_problem.foEqnsJacDxuPattern(_i_phase, fo_eqns_j_dxu_rows, fo_eqns_j_dxu_cols);
    SparseMatrix fo_eqns_j_dxu_mat(_dim_fo,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    fo_eqns_j_dxu_mat.reserve(nnz);
#endif
    for (integer i=0; i<nnz; i++)
        fo_eqns_j_dxu_mat.insert(fo_eqns_j_dxu_rows[i],fo_eqns_j_dxu_cols[i]) = 1;
    fo_eqns_j_dxu_mat.makeCompressed();
    _p_fo_eqns_j_dxu_outer_start = new integer[fo_eqns_j_dxu_mat.outerSize() + 1];
    copyVectorTo(fo_eqns_j_dxu_mat.outerIndexPtr(), _p_fo_eqns_j_dxu_outer_start, (integer) fo_eqns_j_dxu_mat.outerSize() + 1);

    //calculate pattern for fo_eqns_j_y
    SparseMatrix fo_eqns_j_y(_dim_fo,_dim_xu);
    fo_eqns_j_y = fo_eqns_j_xu_mat + fo_eqns_j_dxu_mat;
    _fo_eqns_j_y_nnz = (integer) fo_eqns_j_y.nonZeros();
    nnz = _fo_eqns_j_y_nnz;
    integer fo_eqns_j_y_rows[nnz];
    integer fo_eqns_j_y_cols[nnz];
    _p_scale_factor_fo_eqns_j_y = new real[nnz];

    integer counter = 0;
    for (integer k=0; k<fo_eqns_j_y.outerSize(); ++k) {
        for (SparseMatrix::InnerIterator it(fo_eqns_j_y,k); it; ++it) {
            // it.value();
            integer i_row = (integer) it.row();
            integer i_col = (integer) it.col();
            fo_eqns_j_y_rows[counter] = i_row;   // row index
            fo_eqns_j_y_cols[counter] = i_col;   // col index (here it is equal to k)
            _p_scale_factor_fo_eqns_j_y[counter] = _p_inv_scaling_fo_eqns_global[i_row] * _p_scaling_y[i_col];
#ifdef MAVERICK_DEBUG
            MAVERICK_ASSERT( fo_eqns_j_y_rows[counter] == it.index(), "MidpointOcp2NlpSinglePhase::setupForNlpConstraintJacobianMatrixes: wrong inner index for fo_eqns_j_y. Expected " << fo_eqns_j_y_rows[counter] << ", real " << it.row() << "\n")
#endif
            counter ++;
        }
    }
#ifdef MAVERICK_DEBUG
    MAVERICK_ASSERT( counter == nnz, "MidpointOcp2NlpSinglePhase::setupForNlpConstraintJacobianMatrixes: wrong estimation of nonzero entries for fo_eqns_j_y. Expected: " << nnz << ", real: " << counter << "\n")
#endif

    //fo_eqns j ay
    nnz = _ocp_problem.foEqnsJacAxuNnz(_i_phase);
    _p_scale_factor_fo_eqns_j_ay = new real[nnz];
    integer * p_row_indexes = new integer[nnz];
    integer * p_col_indexes = new integer[nnz];
    _ocp_problem.foEqnsJacAxuPattern(_i_phase, p_row_indexes, p_col_indexes);
    for (integer i=0; i<nnz; i++) {
        integer i_row = p_row_indexes[i];
        integer i_col = p_col_indexes[i];
        _p_scale_factor_fo_eqns_j_ay[i] = _p_inv_scaling_fo_eqns_global[i_row] * _p_scaling_ay[i_col];
    }
    delete [] p_row_indexes;
    delete [] p_col_indexes;

    //fo_eqns j p
    nnz = _ocp_problem.foEqnsJacPNnz(_i_phase);
    _p_scale_factor_fo_eqns_j_p = new real[nnz];
    p_row_indexes = new integer[nnz];
    p_col_indexes = new integer[nnz];
    _ocp_problem.foEqnsJacPPattern(_i_phase, p_row_indexes, p_col_indexes);
    for (integer i=0; i<nnz; i++) {
        integer i_row = p_row_indexes[i];
        integer i_col = p_col_indexes[i];
        _p_scale_factor_fo_eqns_j_p[i] = _p_inv_scaling_fo_eqns_global[i_row] * _p_scaling_r[i_col];
    }
    delete [] p_row_indexes;
    delete [] p_col_indexes;

    //create the matrix path_constr_j_xu
    nnz = _ocp_problem.pathConstraintsJacXuNnz(_i_phase);
    integer path_constr_j_xu_rows[nnz];
    integer path_constr_j_xu_cols[nnz];
    _ocp_problem.pathConstraintsJacXuPattern(_i_phase, path_constr_j_xu_rows, path_constr_j_xu_cols);
    SparseMatrix path_constr_j_xu_mat(_dim_pc,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    path_constr_j_xu_mat.reserve(nnz);
#endif
    for (integer i=0; i<nnz; i++)
        path_constr_j_xu_mat.insert(path_constr_j_xu_rows[i],path_constr_j_xu_cols[i]) = 1;
    path_constr_j_xu_mat.makeCompressed();
    _p_path_constr_j_xu_outer_start = new integer[path_constr_j_xu_mat.outerSize() + 1];
    copyVectorTo(path_constr_j_xu_mat.outerIndexPtr(), _p_path_constr_j_xu_outer_start, (integer) path_constr_j_xu_mat.outerSize() + 1);

    //create the matrix path_constr_j_dxu
    nnz = _ocp_problem.pathConstraintsJacDxuNnz(_i_phase);
    integer path_constr_j_dxu_rows[nnz];
    integer path_constr_j_dxu_cols[nnz];
    _ocp_problem.pathConstraintsJacDxuPattern(_i_phase, path_constr_j_dxu_rows, path_constr_j_dxu_cols);
    SparseMatrix path_constr_j_dxu_mat(_dim_pc,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    path_constr_j_dxu_mat.reserve(nnz);
#endif
    for (integer i=0; i<nnz; i++)
        path_constr_j_dxu_mat.insert(path_constr_j_dxu_rows[i],path_constr_j_dxu_cols[i]) = 1;
    path_constr_j_dxu_mat.makeCompressed();
    _p_path_constr_j_dxu_outer_start = new integer[path_constr_j_dxu_mat.outerSize() + 1];
    copyVectorTo(path_constr_j_dxu_mat.outerIndexPtr(), _p_path_constr_j_dxu_outer_start, (integer) path_constr_j_dxu_mat.outerSize() + 1);

    //calculate pattern for path_constr_j_y
    SparseMatrix path_constr_j_y(_dim_pc,_dim_xu);
    path_constr_j_y = path_constr_j_xu_mat + path_constr_j_dxu_mat;
    _path_constr_j_y_nnz = (integer) path_constr_j_y.nonZeros();
    nnz = _path_constr_j_y_nnz;
    integer path_constr_j_y_rows[nnz];
    integer path_constr_j_y_cols[nnz];
    _p_scale_factor_path_constr_j_y = new real[nnz];

    counter = 0;
    for (integer k=0; k<path_constr_j_y.outerSize(); ++k) {
        for (SparseMatrix::InnerIterator it(path_constr_j_y,k); it; ++it) {
            // it.value();
            integer i_row = (integer) it.row();
            integer i_col = (integer) it.col();
            path_constr_j_y_rows[counter] = i_row;   // row index
            path_constr_j_y_cols[counter] = i_col;   // col index (here it is equal to k)
            _p_scale_factor_path_constr_j_y[counter] = _p_inv_scaling_path_constr_global[i_row] * _p_scaling_y[i_col];

#ifdef MAVERICK_DEBUG
            MAVERICK_ASSERT( path_constr_j_y_rows[counter] == it.index(), "MidpointOcp2NlpSinglePhase::setupForNlpConstraintJacobianMatrixes: wrong inner index for path_constr_j_y.")
#endif
            counter ++;
        }
    }
#ifdef MAVERICK_DEBUG
    MAVERICK_ASSERT( counter == nnz, "MidpointOcp2NlpSinglePhase::setupForNlpConstraintJacobianMatrixes: wrong estimation of nonzero entries for path_constr_j_y.")
#endif
    // path constr j ay
    nnz = _ocp_problem.pathConstraintsJacAxuNnz(_i_phase);
    _p_scale_factor_path_constr_j_ay = new real[nnz];
    p_row_indexes = new integer[nnz];
    p_col_indexes = new integer[nnz];
    _ocp_problem.pathConstraintsJacAxuPattern(_i_phase, p_row_indexes, p_col_indexes);
    for (integer i=0; i<nnz; i++) {
        integer i_row = p_row_indexes[i];
        integer i_col = p_col_indexes[i];
        _p_scale_factor_path_constr_j_ay[i] = _p_inv_scaling_path_constr_global[i_row] * _p_scaling_ay[i_col];
    }
    delete [] p_row_indexes;
    delete [] p_col_indexes;

    // path constr j_p
    nnz = _ocp_problem.pathConstraintsJacPNnz(_i_phase);
    _p_scale_factor_path_constr_j_p = new real[nnz];
    p_row_indexes = new integer[nnz];
    p_col_indexes = new integer[nnz];
    _ocp_problem.pathConstraintsJacPPattern(_i_phase, p_row_indexes, p_col_indexes);
    for (integer i=0; i<nnz; i++) {
        integer i_row = p_row_indexes[i];
        integer i_col = p_col_indexes[i];
        _p_scale_factor_path_constr_j_p[i] = _p_inv_scaling_path_constr_global[i_row] * _p_scaling_r[i_col];
    }
    delete [] p_row_indexes;
    delete [] p_col_indexes;

    //create the matrix int_constr_j_xu
    nnz = _ocp_problem.intConstraintsJacXuNnz(_i_phase);
    integer int_constr_j_xu_rows[nnz];
    integer int_constr_j_xu_cols[nnz];
    _ocp_problem.intConstraintsJacXuPattern(_i_phase, int_constr_j_xu_rows, int_constr_j_xu_cols);
    SparseMatrix int_constr_j_xu_mat(_dim_ic,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    int_constr_j_xu_mat.reserve(nnz);
#endif
    for (integer i=0; i<nnz; i++)
        int_constr_j_xu_mat.insert(int_constr_j_xu_rows[i],int_constr_j_xu_cols[i]) = 1;
    int_constr_j_xu_mat.makeCompressed();
    _p_int_constr_j_xu_outer_start = new integer[int_constr_j_xu_mat.outerSize() + 1];
    copyVectorTo(int_constr_j_xu_mat.outerIndexPtr(), _p_int_constr_j_xu_outer_start, (integer) int_constr_j_xu_mat.outerSize() + 1);

    //create the matrix int_constr_j_dxu
    nnz = _ocp_problem.intConstraintsJacDxuNnz(_i_phase);
    integer int_constr_j_dxu_rows[nnz];
    integer int_constr_j_dxu_cols[nnz];
    _ocp_problem.intConstraintsJacDxuPattern(_i_phase, int_constr_j_dxu_rows, int_constr_j_dxu_cols);
    SparseMatrix int_constr_j_dxu_mat(_dim_ic,_dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    int_constr_j_dxu_mat.reserve(nnz);
#endif
    for (integer i=0; i<nnz; i++)
        int_constr_j_dxu_mat.insert(int_constr_j_dxu_rows[i],int_constr_j_dxu_cols[i]) = 1;
    int_constr_j_dxu_mat.makeCompressed();
    _p_int_constr_j_dxu_outer_start = new integer[int_constr_j_dxu_mat.outerSize() + 1];
    copyVectorTo(int_constr_j_dxu_mat.outerIndexPtr(), _p_int_constr_j_dxu_outer_start, (integer) int_constr_j_dxu_mat.outerSize() + 1);

    //calculate pattern for int_constr_j_y
    SparseMatrix int_constr_j_y(_dim_ic,_dim_xu);
    int_constr_j_y = int_constr_j_xu_mat + int_constr_j_dxu_mat;
    _int_constr_j_y_nnz = (integer) int_constr_j_y.nonZeros();
    integer int_constr_j_y_rows[_int_constr_j_y_nnz];
    integer int_constr_j_y_cols[_int_constr_j_y_nnz];
    _p_scale_factor_int_constr_j_y = new real[_int_constr_j_y_nnz];

    counter = 0;
    for (integer k=0; k<int_constr_j_y.outerSize(); ++k) {
        for (SparseMatrix::InnerIterator it(int_constr_j_y,k); it; ++it) {
            // it.value();
            integer i_row = (integer) it.row();
            integer i_col = (integer) it.col();
            int_constr_j_y_rows[counter] = i_row;   // row index
            int_constr_j_y_cols[counter] = i_col;   // col index (here it is equal to k)
            _p_scale_factor_int_constr_j_y[counter] = _p_inv_scaling_int_constr[i_row] * _p_scaling_y[i_col];
#ifdef MAVERICK_DEBUG
            MAVERICK_ASSERT( int_constr_j_y_rows[counter] == it.index(), "MidpointOcp2NlpSinglePhase::setupForNlpConstraintJacobianMatrixes: wrong inner index for int_constr_j_y.")
#endif
            counter ++;
        }
    }
#ifdef MAVERICK_DEBUG
    MAVERICK_ASSERT( counter == _int_constr_j_y_nnz, "MidpointOcp2NlpSinglePhase::setupForNlpConstraintJacobianMatrixes: wrong estimation of nonzero entries for path_constr_j_y.")
#endif

    // int constr j_ay
    nnz = _ocp_problem.intConstraintsJacAxuNnz(_i_phase);
    _p_scale_factor_int_constr_j_ay = new real[nnz];
    p_row_indexes = new integer[nnz];
    p_col_indexes = new integer[nnz];
    _ocp_problem.intConstraintsJacAxuPattern(_i_phase, p_row_indexes, p_col_indexes);
    for (integer i=0; i<nnz; i++) {
        integer i_row = p_row_indexes[i];
        integer i_col = p_col_indexes[i];
        _p_scale_factor_int_constr_j_ay[i] = _p_inv_scaling_int_constr[i_row] * _p_scaling_ay[i_col];
    }
    delete [] p_row_indexes;
    delete [] p_col_indexes;

    // int constr j_p
    nnz = _ocp_problem.intConstraintsJacPNnz(_i_phase);
    _p_scale_factor_int_constr_j_p = new real[nnz];
    p_row_indexes = new integer[nnz];
    p_col_indexes = new integer[nnz];
    _ocp_problem.intConstraintsJacPPattern(_i_phase, p_row_indexes, p_col_indexes);
    for (integer i=0; i<nnz; i++) {
        integer i_row = p_row_indexes[i];
        integer i_col = p_col_indexes[i];
        _p_scale_factor_int_constr_j_p[i] = _p_inv_scaling_int_constr[i_row] * _p_scaling_r[i_col];
    }
    delete [] p_row_indexes;
    delete [] p_col_indexes;


    //point constr j y
    nnz = _ocp_problem.pointConstraintsJacXuNnz(_i_phase);
    _p_scale_factor_point_constr_j_y = new real[nnz];
    p_row_indexes = new integer[nnz];
    p_col_indexes = new integer[nnz];
    _ocp_problem.pointConstraintsJacXuPattern(_i_phase, p_row_indexes, p_col_indexes);
    for (integer i=0; i<nnz; i++) {
        integer i_row = p_row_indexes[i];
        integer i_col = p_col_indexes[i];
        _p_scale_factor_point_constr_j_y[i] = _p_inv_scaling_point_constr_global[i_row] * _p_scaling_y[i_col];
    }
    delete [] p_row_indexes;
    delete [] p_col_indexes;

    //point constr j p
    nnz = _ocp_problem.pointConstraintsJacPNnz(_i_phase);
    _p_scale_factor_point_constr_j_p = new real[nnz];
    p_row_indexes = new integer[nnz];
    p_col_indexes = new integer[nnz];
    _ocp_problem.pointConstraintsJacPPattern(_i_phase, p_row_indexes, p_col_indexes);
    for (integer i=0; i<nnz; i++) {
        integer i_row = p_row_indexes[i];
        integer i_col = p_col_indexes[i];
        _p_scale_factor_point_constr_j_p[i] = _p_inv_scaling_point_constr_global[i_row] * _p_scaling_r[i_col];
    }
    delete [] p_row_indexes;
    delete [] p_col_indexes;

    // boundary conditions
    nnz = _ocp_problem.boundaryConditionsJacXuInitNnz(_i_phase) + _ocp_problem.boundaryConditionsJacXuFinNnz(_i_phase) + _ocp_problem.boundaryConditionsJacPNnz(_i_phase);
    _p_scale_factor_bcs_j_xifp = new real[nnz];
    real * p_current_scale = _p_scale_factor_bcs_j_xifp;

    nnz = _ocp_problem.boundaryConditionsJacXuInitNnz(_i_phase);
    p_row_indexes = new integer[nnz];
    p_col_indexes = new integer[nnz];
    _ocp_problem.boundaryConditionsJacXuInitPattern(_i_phase, p_row_indexes, p_col_indexes);
    for (integer i=0; i<nnz; i++) {
        integer i_row = p_row_indexes[i];
        integer i_col = p_col_indexes[i];
        p_current_scale[i] = _p_inv_scaling_bcs[i_row] * _p_scaling_y[i_col];
    }
    delete [] p_row_indexes;
    delete [] p_col_indexes;
    p_current_scale += nnz;

    nnz = _ocp_problem.boundaryConditionsJacXuFinNnz(_i_phase);
    p_row_indexes = new integer[nnz];
    p_col_indexes = new integer[nnz];
    _ocp_problem.boundaryConditionsJacXuFinPattern(_i_phase, p_row_indexes, p_col_indexes);
    for (integer i=0; i<nnz; i++) {
        integer i_row = p_row_indexes[i];
        integer i_col = p_col_indexes[i];
        p_current_scale[i] = _p_inv_scaling_bcs[i_row] * _p_scaling_y[i_col];
    }
    delete [] p_row_indexes;
    delete [] p_col_indexes;
    p_current_scale += nnz;

    nnz = _ocp_problem.boundaryConditionsJacPNnz(_i_phase);
    p_row_indexes = new integer[nnz];
    p_col_indexes = new integer[nnz];
    _ocp_problem.boundaryConditionsJacPPattern(_i_phase, p_row_indexes, p_col_indexes);
    for (integer i=0; i<nnz; i++) {
        integer i_row = p_row_indexes[i];
        integer i_col = p_col_indexes[i];
        p_current_scale[i] = _p_inv_scaling_bcs[i_row] * _p_scaling_r[i_col];
    }
    delete [] p_row_indexes;
    delete [] p_col_indexes;
    p_current_scale += nnz;


    //now we can calculate the pattern of the nlp jacobian
    calculateNlpConstraintsJacobainPattern(fo_eqns_j_y_rows, fo_eqns_j_y_cols,
                                           path_constr_j_y_rows, path_constr_j_y_cols,
                                           int_constr_j_y_rows, int_constr_j_y_cols);
}

//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________

void MidpointOcp2NlpSinglePhase::calculateNlpConstraintsJacobainPattern(integer const fo_eqns_j_y_rows[],
                                                                   integer const fo_eqns_j_y_cols[],
                                                                   integer const path_constr_j_y_rows[],
                                                                   integer const path_constr_j_y_cols[],
                                                                   integer const int_constr_j_y_rows[],
                                                                   integer const int_constr_j_y_cols[]) {

    // initialize the memory
    _p_nlp_constraints_j_rows = new integer[getNlpConstraintsJacobianNnz()];
    _p_nlp_constraints_j_cols = new integer[getNlpConstraintsJacobianNnz()];

    integer * p_current_nlp_constraints_j_rows = _p_nlp_constraints_j_rows;
    integer * p_current_nlp_constraints_j_cols = _p_nlp_constraints_j_cols;

    integer row_offset;
    integer col_offset;
    integer nnz;
    integer * p_row_indexes = nullptr;
    integer * p_col_indexes = nullptr;

    for (integer mesh_interval=0; mesh_interval<_p_mesh->getNumberOfDiscretisationPoints(); mesh_interval++) {
        if ( mesh_interval < _p_mesh->getNumberOfIntervals() ) { //the first order equations and path constraints do not apply to the last mesh points

            //write matrix Y first block indexes: fo_eqns_j_y_left
            row_offset = mesh_interval * _dim_q;
            col_offset = mesh_interval * (_dim_y+_dim_ay);
            sumAndWriteVectorTo(fo_eqns_j_y_rows, p_current_nlp_constraints_j_rows, row_offset, _fo_eqns_j_y_nnz);
            p_current_nlp_constraints_j_rows += _fo_eqns_j_y_nnz;
            sumAndWriteVectorTo(fo_eqns_j_y_cols, p_current_nlp_constraints_j_cols, col_offset, _fo_eqns_j_y_nnz);
            p_current_nlp_constraints_j_cols += _fo_eqns_j_y_nnz;

            //write matrix Y second block indexes: fo_eqns_j_ay
            row_offset = mesh_interval * _dim_q;
            col_offset = mesh_interval * (_dim_y+_dim_ay) + _dim_y;
            integer nnz = _fo_eqns_j_ay_nnz;
            p_row_indexes = new integer[nnz];
            p_col_indexes = new integer[nnz];
            _ocp_problem.foEqnsJacAxuPattern(_i_phase, p_row_indexes, p_col_indexes);
            sumAndWriteVectorTo(p_row_indexes, p_current_nlp_constraints_j_rows, row_offset, nnz);
            p_current_nlp_constraints_j_rows += nnz;
            sumAndWriteVectorTo(p_col_indexes, p_current_nlp_constraints_j_cols, col_offset, nnz);
            p_current_nlp_constraints_j_cols += nnz;
            delete [] p_row_indexes;
            delete [] p_col_indexes;

            //write matrix Y second block indexes: fo_eqns_j_xu_right
            row_offset = mesh_interval * _dim_q;
            col_offset = mesh_interval * (_dim_y+_dim_ay) + _dim_y + _dim_ay;
            sumAndWriteVectorTo(fo_eqns_j_y_rows, p_current_nlp_constraints_j_rows, row_offset, _fo_eqns_j_y_nnz);
            p_current_nlp_constraints_j_rows += _fo_eqns_j_y_nnz;
            sumAndWriteVectorTo(fo_eqns_j_y_cols, p_current_nlp_constraints_j_cols, col_offset, _fo_eqns_j_y_nnz);
            p_current_nlp_constraints_j_cols += _fo_eqns_j_y_nnz;

            //write matrix Y third block indexes: fo_eqns_j_p
            row_offset = mesh_interval * _dim_q;
            col_offset = getNlpParamPtrIndex();
            nnz = _fo_eqns_j_p_nnz;
            p_row_indexes = new integer[nnz];
            p_col_indexes = new integer[nnz];
            _ocp_problem.foEqnsJacPPattern(_i_phase, p_row_indexes, p_col_indexes);
            sumAndWriteVectorTo(p_row_indexes, p_current_nlp_constraints_j_rows, row_offset, nnz);
            p_current_nlp_constraints_j_rows += nnz;
            sumAndWriteVectorTo(p_col_indexes, p_current_nlp_constraints_j_cols, col_offset, nnz);
            p_current_nlp_constraints_j_cols += nnz;
            delete [] p_row_indexes;
            delete [] p_col_indexes;

            //write matrix Y fourth block indexes: path_constr_j_xu_left
            row_offset = mesh_interval * _dim_q + _dim_fo;
            col_offset = mesh_interval * (_dim_y+_dim_ay);
            sumAndWriteVectorTo(path_constr_j_y_rows, p_current_nlp_constraints_j_rows, row_offset, _path_constr_j_y_nnz);
            p_current_nlp_constraints_j_rows += _path_constr_j_y_nnz;
            sumAndWriteVectorTo(path_constr_j_y_cols, p_current_nlp_constraints_j_cols, col_offset, _path_constr_j_y_nnz);
            p_current_nlp_constraints_j_cols += _path_constr_j_y_nnz;

            //write matrix Y fifth block indexes: path_constr_j_ay
            row_offset = mesh_interval * _dim_q + _dim_fo;
            col_offset = mesh_interval * (_dim_y+_dim_ay) + _dim_y;
            nnz = _path_constr_j_ay_nnz;
            p_row_indexes = new integer[nnz];
            p_col_indexes = new integer[nnz];
            _ocp_problem.pathConstraintsJacAxuPattern(_i_phase, p_row_indexes, p_col_indexes);
            sumAndWriteVectorTo(p_row_indexes, p_current_nlp_constraints_j_rows, row_offset, nnz);
            p_current_nlp_constraints_j_rows += nnz;
            sumAndWriteVectorTo(p_col_indexes, p_current_nlp_constraints_j_cols, col_offset, nnz);
            p_current_nlp_constraints_j_cols += nnz;
            delete [] p_row_indexes;
            delete [] p_col_indexes;

            //write matrix Y sixth block indexes: path_constr_j_xu_rigth
            row_offset = mesh_interval * _dim_q + _dim_fo;
            col_offset = mesh_interval * (_dim_y+_dim_ay) + _dim_y + _dim_ay;
            sumAndWriteVectorTo(path_constr_j_y_rows, p_current_nlp_constraints_j_rows, row_offset, _path_constr_j_y_nnz);
            p_current_nlp_constraints_j_rows += _path_constr_j_y_nnz;
            sumAndWriteVectorTo(path_constr_j_y_cols, p_current_nlp_constraints_j_cols, col_offset, _path_constr_j_y_nnz);
            p_current_nlp_constraints_j_cols += _path_constr_j_y_nnz;

            //write matrix Y seventh block indexes: path_constr_j_p
            row_offset = mesh_interval * _dim_q + _dim_fo;
            col_offset = getNlpParamPtrIndex();
            nnz = _path_constr_j_p_nnz;
            p_row_indexes = new integer[nnz];
            p_col_indexes = new integer[nnz];
            _ocp_problem.pathConstraintsJacPPattern(_i_phase, p_row_indexes, p_col_indexes);
            sumAndWriteVectorTo(p_row_indexes, p_current_nlp_constraints_j_rows, row_offset, nnz);
            p_current_nlp_constraints_j_rows += nnz;
            sumAndWriteVectorTo(p_col_indexes, p_current_nlp_constraints_j_cols, col_offset, nnz);
            p_current_nlp_constraints_j_cols += nnz;
            delete [] p_row_indexes;
            delete [] p_col_indexes;
        }

        //write matrix Y eight block indexes : point_constr_j_xu_left
        row_offset = mesh_interval * _dim_q;
        if ( mesh_interval < _p_mesh->getNumberOfIntervals() ) {
            row_offset += _dim_fo + _dim_pc;
        }
        col_offset = mesh_interval * (_dim_y+_dim_ay);
        nnz = _point_constr_j_y_nnz;
        p_row_indexes = new integer[nnz];
        p_col_indexes = new integer[nnz];
        _ocp_problem.pointConstraintsJacXuPattern(_i_phase, p_row_indexes, p_col_indexes);
        sumAndWriteVectorTo(p_row_indexes, p_current_nlp_constraints_j_rows, row_offset, nnz);
        p_current_nlp_constraints_j_rows += nnz;
        sumAndWriteVectorTo(p_col_indexes, p_current_nlp_constraints_j_cols, col_offset, nnz);
        p_current_nlp_constraints_j_cols += nnz;
        delete [] p_row_indexes;
        delete [] p_col_indexes;

        //write matrix Y nineth block indexes: point_constr_j_p
        // row_offset is unchanged
        col_offset = getNlpParamPtrIndex();
        nnz = _point_constr_j_p_nnz;
        p_row_indexes = new integer[nnz];
        p_col_indexes = new integer[nnz];
        _ocp_problem.pointConstraintsJacPPattern(_i_phase, p_row_indexes, p_col_indexes);
        sumAndWriteVectorTo(p_row_indexes, p_current_nlp_constraints_j_rows, row_offset, nnz);
        p_current_nlp_constraints_j_rows += nnz;
        sumAndWriteVectorTo(p_col_indexes, p_current_nlp_constraints_j_cols, col_offset, nnz);
        p_current_nlp_constraints_j_cols += nnz;
        delete [] p_row_indexes;
        delete [] p_col_indexes;
    }

    // now write the pattern of the boundary conditions
    row_offset = _p_mesh->getNumberOfIntervals() * _dim_q + _dim_poc;
    col_offset = 0;
    nnz = _ocp_problem.boundaryConditionsJacXuInitNnz(_i_phase);
    p_row_indexes = new integer[nnz];
    p_col_indexes = new integer[nnz];
    _ocp_problem.boundaryConditionsJacXuInitPattern(_i_phase, p_row_indexes, p_col_indexes);
    sumAndWriteVectorTo(p_row_indexes, p_current_nlp_constraints_j_rows, row_offset, nnz);
    p_current_nlp_constraints_j_rows += nnz;
    sumAndWriteVectorTo(p_col_indexes, p_current_nlp_constraints_j_cols, col_offset, nnz);
    p_current_nlp_constraints_j_cols += nnz;
    delete [] p_row_indexes;
    delete [] p_col_indexes;

    //row_offset is unchanged
    col_offset = _p_mesh->getNumberOfIntervals() * (_dim_y+_dim_ay);
    nnz = _ocp_problem.boundaryConditionsJacXuFinNnz(_i_phase);
    p_row_indexes = new integer[nnz];
    p_col_indexes = new integer[nnz];
    _ocp_problem.boundaryConditionsJacXuFinPattern(_i_phase, p_row_indexes, p_col_indexes);
    sumAndWriteVectorTo(p_row_indexes, p_current_nlp_constraints_j_rows, row_offset, nnz);
    p_current_nlp_constraints_j_rows += nnz;
    sumAndWriteVectorTo(p_col_indexes, p_current_nlp_constraints_j_cols, col_offset, nnz);
    p_current_nlp_constraints_j_cols += nnz;
    delete [] p_row_indexes;
    delete [] p_col_indexes;

    //row_offset is unchanged
    col_offset = getNlpParamPtrIndex();
    nnz = _ocp_problem.boundaryConditionsJacPNnz(_i_phase);
    p_row_indexes = new integer[nnz];
    p_col_indexes = new integer[nnz];
    _ocp_problem.boundaryConditionsJacPPattern(_i_phase, p_row_indexes, p_col_indexes);
    sumAndWriteVectorTo(p_row_indexes, p_current_nlp_constraints_j_rows, row_offset, nnz);
    p_current_nlp_constraints_j_rows += nnz;
    sumAndWriteVectorTo(p_col_indexes, p_current_nlp_constraints_j_cols, col_offset, nnz);
    p_current_nlp_constraints_j_cols += nnz;
    delete [] p_row_indexes;
    delete [] p_col_indexes;

    // now write the pattern for the integral constraints jac y and ay
    integer int_constr_j_ay_rows[_int_constr_j_ay_nnz];
    integer int_constr_j_ay_cols[_int_constr_j_ay_nnz];
    _ocp_problem.intConstraintsJacAxuPattern(_i_phase, int_constr_j_ay_rows, int_constr_j_ay_cols);
    row_offset = _p_mesh->getNumberOfIntervals() * _dim_q + _dim_poc + _dim_bc;
    for (integer mesh_point=0; mesh_point<_p_mesh->getNumberOfDiscretisationPoints(); mesh_point++) {
        col_offset = mesh_point * (_dim_y + _dim_ay);
        sumAndWriteVectorTo(int_constr_j_y_rows, p_current_nlp_constraints_j_rows, row_offset, _int_constr_j_y_nnz);
        p_current_nlp_constraints_j_rows += _int_constr_j_y_nnz;
        sumAndWriteVectorTo(int_constr_j_y_cols, p_current_nlp_constraints_j_cols, col_offset, _int_constr_j_y_nnz);
        p_current_nlp_constraints_j_cols += _int_constr_j_y_nnz;

        if (mesh_point < _p_mesh->getNumberOfIntervals() ) {
            col_offset = mesh_point * (_dim_y + _dim_ay) + _dim_y;
            sumAndWriteVectorTo(int_constr_j_ay_rows, p_current_nlp_constraints_j_rows, row_offset, _int_constr_j_ay_nnz);
            p_current_nlp_constraints_j_rows += _int_constr_j_ay_nnz;
            sumAndWriteVectorTo(int_constr_j_ay_cols, p_current_nlp_constraints_j_cols, col_offset, _int_constr_j_ay_nnz);
            p_current_nlp_constraints_j_cols += _int_constr_j_ay_nnz;
        }
    }

    // now write the pattern for the integral constr jac p
    // row offset is unchanged
    col_offset = getNlpParamPtrIndex();
    nnz = _ocp_problem.intConstraintsJacPNnz(_i_phase);
    p_row_indexes = new integer[nnz];
    p_col_indexes = new integer[nnz];
    _ocp_problem.intConstraintsJacPPattern(_i_phase, p_row_indexes, p_col_indexes);
    sumAndWriteVectorTo(p_row_indexes, p_current_nlp_constraints_j_rows, row_offset, nnz);
    p_current_nlp_constraints_j_rows += nnz;
    sumAndWriteVectorTo(p_col_indexes, p_current_nlp_constraints_j_cols, col_offset, nnz);
    p_current_nlp_constraints_j_cols += nnz;
    delete [] p_row_indexes;
    delete [] p_col_indexes;

    // end

#ifdef MAVERICK_DEBUG
    {
        //    cout << getNlpConstraintsJacobianNnz() << "\n";
        //    cout << getNlpConstraintsJacobainMatrixFNnz() << "\n";
        //    cout << _ocp_problem.intConstraintsJacPNnz(_i_phase) << "\n";
        integer * p_expected = _p_nlp_constraints_j_rows + getNlpConstraintsJacobianNnz();
        MAVERICK_ASSERT( p_current_nlp_constraints_j_rows == p_expected, "MidpointOcp2NlpSinglePhase::calculateNlpConstraintsJacobainPattern: wrong final value of p_current_nlp_constraints_j_rows. Current " << p_current_nlp_constraints_j_rows << ", expected " <<  p_expected << " difference: " << p_current_nlp_constraints_j_rows-p_expected << "\n")
        p_expected =  _p_nlp_constraints_j_cols + getNlpConstraintsJacobianNnz();
        MAVERICK_ASSERT( p_current_nlp_constraints_j_cols == p_expected, "MidpointOcp2NlpSinglePhase::calculateNlpConstraintsJacobainPattern: wrong final value of p_current_nlp_constraints_j_cols" )
    }
#endif

}

//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________


void MidpointOcp2NlpSinglePhase::calculateNlpConstraintsJacobianMatrixY(real const left_state_control[],
                                                                   real const state_control[],
                                                                   real const state_control_derivative[],
                                                                   real const algebraic_state_control[],
                                                                   real const parameters[],
                                                                   real const zeta,
                                                                   real const d_zeta,
                                                                   real const d_zeta_dual,
                                                                   real       values[]) const {
    real * p_current_value_ptr = values;
    real const d_zeta_inv = 1. / d_zeta;

    integer const fo_eqns_j_xu_nnz = _fo_eqns_j_xu_nnz;
    real fo_eqns_j_xu_values[fo_eqns_j_xu_nnz];
    integer fo_eqns_j_xu_rows[fo_eqns_j_xu_nnz];
    integer fo_eqns_j_xu_cols[fo_eqns_j_xu_nnz];

    integer const fo_eqns_j_dxu_nnz = _fo_eqns_j_dxu_nnz;
    real fo_eqns_j_dxu_values[fo_eqns_j_dxu_nnz];
    integer fo_eqns_j_dxu_rows[fo_eqns_j_dxu_nnz];
    integer fo_eqns_j_dxu_cols[fo_eqns_j_dxu_nnz];

    integer const fo_eqns_j_ay_nnz = _fo_eqns_j_ay_nnz;

    //get fo_eqns_j_xu and dxu and directly write the third block
    _ocp_problem.foEqnsJac(_i_phase,
                           state_control,
                           state_control_derivative,
                           algebraic_state_control,
                           parameters,
                           zeta,
                           fo_eqns_j_xu_values,
                           fo_eqns_j_dxu_values,
                           p_current_value_ptr + _fo_eqns_j_y_nnz, // jac_ay
                           p_current_value_ptr + 2*_fo_eqns_j_y_nnz + fo_eqns_j_ay_nnz //jac_p
                           );

    // scale if necessary
    if (_multiply_foeqns_by_dz) {
        multiplyVectorBy(fo_eqns_j_xu_values, d_zeta, fo_eqns_j_xu_nnz);
        multiplyVectorBy(fo_eqns_j_dxu_values, d_zeta, fo_eqns_j_xu_nnz);
        multiplyVectorBy(p_current_value_ptr + _fo_eqns_j_y_nnz, d_zeta, fo_eqns_j_ay_nnz); // jac ay
        multiplyVectorBy(p_current_value_ptr + 2*_fo_eqns_j_y_nnz + fo_eqns_j_ay_nnz, d_zeta, _fo_eqns_j_p_nnz); // jac p
    }

    _ocp_problem.foEqnsJacXuPattern(_i_phase, fo_eqns_j_xu_rows, fo_eqns_j_xu_cols);
    Eigen::Map<SparseMatrix> fo_eqns_j_xu_mat(_dim_fo, _dim_xu, fo_eqns_j_xu_nnz,
                                              _p_fo_eqns_j_xu_outer_start,
                                              fo_eqns_j_xu_rows,
                                              fo_eqns_j_xu_values,
                                              0);

    _ocp_problem.foEqnsJacDxuPattern(_i_phase, fo_eqns_j_dxu_rows, fo_eqns_j_dxu_cols);
    Eigen::Map<SparseMatrix> fo_eqns_j_dxu_mat(_dim_fo, _dim_xu, fo_eqns_j_dxu_nnz,
                                               _p_fo_eqns_j_dxu_outer_start,
                                               fo_eqns_j_dxu_rows,
                                               fo_eqns_j_dxu_values,
                                               0);

    //FIRST AND SECOND BLOCKS
    //sum the fo_eqns_j_xu and fo_eqns_j_dxu and multiply by the left or right values
    SparseMatrix fo_eqns_j_y_left_mat(_dim_fo, _dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    fo_eqns_j_y_left_mat.reserve(_fo_eqns_j_y_nnz);
#endif
    SparseMatrix fo_eqns_j_y_right_mat(_dim_fo, _dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    fo_eqns_j_y_right_mat.reserve(_fo_eqns_j_y_nnz);
#endif

    fo_eqns_j_y_left_mat = fo_eqns_j_xu_mat * 0.5 - fo_eqns_j_dxu_mat * d_zeta_inv;
    fo_eqns_j_y_right_mat = fo_eqns_j_xu_mat * 0.5 + fo_eqns_j_dxu_mat * d_zeta_inv;

    //write the first and second matrix block to the output
    multiplyAndCopyVectorTo((real*) fo_eqns_j_y_left_mat.valuePtr(), p_current_value_ptr, _p_scale_factor_fo_eqns_j_y, _fo_eqns_j_y_nnz); //TODO: remove cast
    p_current_value_ptr += _fo_eqns_j_y_nnz;

    // second block. already written, just scale it
    multiplyVectorBy(p_current_value_ptr, _p_scale_factor_fo_eqns_j_ay, fo_eqns_j_ay_nnz);
    p_current_value_ptr += fo_eqns_j_ay_nnz;

    //write the third matrix block to the output
    multiplyAndCopyVectorTo((real*) fo_eqns_j_y_right_mat.valuePtr(), p_current_value_ptr, _p_scale_factor_fo_eqns_j_y, _fo_eqns_j_y_nnz); //TODO: remove cast
    p_current_value_ptr += _fo_eqns_j_y_nnz;

    // THIRD BLOCK
    //already written before, we need only to scale it
    {
        integer const nnz = _fo_eqns_j_p_nnz;
        multiplyVectorBy(p_current_value_ptr, _p_scale_factor_fo_eqns_j_p, nnz);
        p_current_value_ptr += nnz;
    }

    //get path_constr_j_xu and path_constr_j_dxu
    integer const path_constr_j_xu_nnz = _path_constr_j_xu_nnz;
    real path_constr_j_xu_values[path_constr_j_xu_nnz];
    integer path_constr_j_xu_rows[path_constr_j_xu_nnz];
    integer path_constr_j_xu_cols[path_constr_j_xu_nnz];

    integer const path_constr_j_dxu_nnz = _path_constr_j_dxu_nnz;
    real path_constr_j_dxu_values[path_constr_j_dxu_nnz];
    integer path_constr_j_dxu_rows[path_constr_j_dxu_nnz];
    integer path_constr_j_dxu_cols[path_constr_j_dxu_nnz];

    integer const path_constr_j_ay_nnz = _path_constr_j_ay_nnz;

    _ocp_problem.pathConstraintsJac(_i_phase, state_control, state_control_derivative, algebraic_state_control, parameters, zeta,
                                    path_constr_j_xu_values,
                                    path_constr_j_dxu_values,
                                    p_current_value_ptr + _path_constr_j_y_nnz, //jac_ay
                                    p_current_value_ptr + 2 * _path_constr_j_y_nnz + path_constr_j_ay_nnz //jac_p
                                    );

    if (_multiply_path_constr_by_dz) {
        multiplyVectorBy(path_constr_j_xu_values, d_zeta, path_constr_j_xu_nnz);
        multiplyVectorBy(path_constr_j_dxu_values, d_zeta, path_constr_j_dxu_nnz);
        multiplyVectorBy(p_current_value_ptr + _path_constr_j_y_nnz, d_zeta, path_constr_j_ay_nnz); // jac ay
        multiplyVectorBy(p_current_value_ptr + 2 * _path_constr_j_y_nnz + path_constr_j_ay_nnz, d_zeta, _path_constr_j_p_nnz); // jac p
    }

    _ocp_problem.pathConstraintsJacXuPattern(_i_phase, path_constr_j_xu_rows, path_constr_j_xu_cols);
    Eigen::Map<SparseMatrix> path_constr_j_xu_mat(_dim_pc, _dim_xu, path_constr_j_xu_nnz,
                                                  _p_path_constr_j_xu_outer_start,
                                                  path_constr_j_xu_rows,
                                                  path_constr_j_xu_values,
                                                  0);

    _ocp_problem.pathConstraintsJacDxuPattern(_i_phase, path_constr_j_dxu_rows, path_constr_j_dxu_cols);
    Eigen::Map<SparseMatrix> path_constr_j_dxu_mat(_dim_pc, _dim_xu, path_constr_j_dxu_nnz,
                                                   _p_path_constr_j_dxu_outer_start,
                                                   path_constr_j_dxu_rows,
                                                   path_constr_j_dxu_values,
                                                   0);

    // FOURTH BLOCK
    //sum the path_constr_j_xu and path_constr_j_dxu and multiply by the left or right values
    SparseMatrix path_constr_j_y_left_mat(_dim_pc, _dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    path_constr_j_y_left_mat.reserve(_path_constr_j_y_nnz);
#endif
    SparseMatrix path_constr_j_y_right_mat(_dim_pc, _dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    path_constr_j_y_right_mat.reserve(_path_constr_j_y_nnz);
#endif

    path_constr_j_y_left_mat = path_constr_j_xu_mat * 0.5 - path_constr_j_dxu_mat * d_zeta_inv;
    path_constr_j_y_right_mat = path_constr_j_xu_mat * 0.5 + path_constr_j_dxu_mat * d_zeta_inv;

    //write the fifth matrix block
    multiplyAndCopyVectorTo((real*) path_constr_j_y_left_mat.valuePtr(), p_current_value_ptr, _p_scale_factor_path_constr_j_y, _path_constr_j_y_nnz);
    p_current_value_ptr += _path_constr_j_y_nnz;

    //write the sixth block (algebraic)
    // already written, just scale
    multiplyVectorBy(p_current_value_ptr, _p_scale_factor_path_constr_j_ay, path_constr_j_ay_nnz);
    p_current_value_ptr += path_constr_j_ay_nnz;

    // seventh block
    multiplyAndCopyVectorTo((real*) path_constr_j_y_right_mat.valuePtr(), p_current_value_ptr, _p_scale_factor_path_constr_j_y, _path_constr_j_y_nnz); //TODO: remove cast
    p_current_value_ptr += _path_constr_j_y_nnz;

    // eigth block
    //already written before, we need only to scale it
    {
        integer const nnz = _path_constr_j_p_nnz;
        multiplyVectorBy(p_current_value_ptr, _p_scale_factor_path_constr_j_p, nnz);
        p_current_value_ptr += nnz;
    }

    // nineth and tenth blocks
    {
        integer const nnz = _point_constr_j_y_nnz;
        _ocp_problem.pointConstraintsJac(_i_phase, left_state_control, parameters, zeta,
                                         p_current_value_ptr,
                                         p_current_value_ptr + nnz);

        if (_multiply_point_constr_by_dz) {
            multiplyVectorBy(p_current_value_ptr, d_zeta_dual, nnz);
            multiplyVectorBy(p_current_value_ptr + nnz, d_zeta_dual, _point_constr_j_p_nnz);
        }
        multiplyVectorBy(p_current_value_ptr, _p_scale_factor_point_constr_j_y, nnz);
        p_current_value_ptr += nnz;
    }

    {
        integer const nnz = _point_constr_j_p_nnz;
        multiplyVectorBy(p_current_value_ptr, _p_scale_factor_point_constr_j_p, nnz);
        p_current_value_ptr += nnz;
    }

    MAVERICK_DEBUG_ASSERT( p_current_value_ptr == values + getNlpConstraintsJacobainMatrixYNnz(), "MidpointOcp2NlpSinglePhase::writeNlpConstraintsJacobianMatrixY: wrong evaluation of pointer in matrix CY calculation")

}

//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________

void MidpointOcp2NlpSinglePhase::calculateNlpConstraintsJacobianMatrixF (real const initial_state_control[],
                                                                    real const final_state_control[],
                                                                    real const parameters[],
                                                                    real const initial_zeta,
                                                                    real const final_zeta,
                                                                    real       values[]) const {
    //Matrix CF is organised in 3 blocks, in order: boundary_conditions_j_xui, boundary_conditions_j_xuf, boundary_conditions_j_p
    real * p_current_value_ptr = values;

    // FIRST, SECOND AND THIRD BLOCK
    real * p_jac_xu_ini = p_current_value_ptr;
    integer const xi_nnz = _ocp_problem.boundaryConditionsJacXuInitNnz(_i_phase);
    real * p_jac_xu_fin = p_jac_xu_ini + xi_nnz;
    integer const xf_nnz = _ocp_problem.boundaryConditionsJacXuFinNnz(_i_phase);
    real * p_jac_p = p_jac_xu_fin + xf_nnz;
    integer const p_nnz = _ocp_problem.boundaryConditionsJacPNnz(_i_phase);
    _ocp_problem.boundaryConditionsJac(_i_phase, initial_state_control, final_state_control, parameters, initial_zeta, final_zeta,
                                       p_jac_xu_ini,
                                       p_jac_xu_fin,
                                       p_jac_p );

    //scale
    multiplyVectorBy(p_current_value_ptr, _p_scale_factor_bcs_j_xifp, xi_nnz + xf_nnz + p_nnz);

    p_current_value_ptr = p_jac_p + p_nnz;

    MAVERICK_DEBUG_ASSERT( p_current_value_ptr == values + getNlpConstraintsJacobainMatrixFNnz(), "MidpointOcp2NlpSinglePhase::writeNLPConstraintJacobianMatrixF: wrong evaluation of pointer in matrix CF calculation")

}

//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________
//_______________________________________________________________________________________________________________________________________

void MidpointOcp2NlpSinglePhase::calculateNlpConstraintsJacobianMatrixI(real const state_control[],
                                                                   real const state_control_derivative[],
                                                                   real const algebraic_state_control[],
                                                                   real const parameters[],
                                                                   real const zeta,
                                                                   real const d_zeta,
                                                                   real       values_left[],
                                                                   real       values_right[],
                                                                   real       jac_p_values[] ) const {

    integer nnz = _int_constr_j_xu_nnz;
    real int_constr_j_xu_values[nnz];
    integer int_constr_j_xu_rows[nnz];
    integer int_constr_j_xu_cols[nnz];

    nnz = _int_constr_j_dxu_nnz;
    real int_constr_j_dxu_values[nnz];
    integer int_constr_j_dxu_rows[nnz];
    integer int_constr_j_dxu_cols[nnz];

    nnz = _int_constr_j_ay_nnz;
    real int_constr_j_axu_values[nnz];

    integer const & jac_p_nnz = _int_constr_j_p_nnz;
    real int_constr_j_p_values[ jac_p_nnz ];

    //get int constr jac
    _ocp_problem.intConstraintsJac(_i_phase,
                                   state_control,
                                   state_control_derivative,
                                   algebraic_state_control,
                                   parameters,
                                   zeta,
                                   int_constr_j_xu_values,
                                   int_constr_j_dxu_values,
                                   int_constr_j_axu_values,
                                   int_constr_j_p_values);
    // jacobian w.r.t p
    multiplyVectorBy(int_constr_j_p_values, _p_scale_factor_int_constr_j_p, jac_p_nnz);
    multiplyAndSumVectorTo(int_constr_j_p_values, jac_p_values, d_zeta, jac_p_nnz);

    // jacobian w.r.t xu
    _ocp_problem.intConstraintsJacXuPattern(_i_phase, int_constr_j_xu_rows, int_constr_j_xu_cols);
    Eigen::Map<SparseMatrix> int_constr_j_xu_mat(_dim_ic, _dim_xu, _ocp_problem.intConstraintsJacXuNnz(_i_phase),
                                                 _p_int_constr_j_xu_outer_start,
                                                 int_constr_j_xu_rows,
                                                 int_constr_j_xu_values,
                                                 0);

    _ocp_problem.intConstraintsJacDxuPattern(_i_phase, int_constr_j_dxu_rows, int_constr_j_dxu_cols);
    Eigen::Map<SparseMatrix> int_constr_j_dxu_mat(_dim_ic, _dim_xu, _ocp_problem.intConstraintsJacDxuNnz(_i_phase),
                                                  _p_int_constr_j_dxu_outer_start,
                                                  int_constr_j_dxu_rows,
                                                  int_constr_j_dxu_values,
                                                  0);

    //LEFT AND RIGHT
    //sum the int_constr_j_xu and int_constr_j_dxu and multiply by the left or right values
    SparseMatrix int_constr_j_y_left_mat(_dim_ic, _dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    int_constr_j_y_left_mat.reserve(_int_constr_j_y_nnz);
#endif
    SparseMatrix int_constr_j_y_right_mat(_dim_ic, _dim_xu);
#ifdef MAVERICK_RESERVE_MATRIX_SPACE
    int_constr_j_y_right_mat.reserve(_int_constr_j_y_nnz);
#endif

    int_constr_j_y_left_mat = int_constr_j_xu_mat * (0.5 * d_zeta) - int_constr_j_dxu_mat ;
    int_constr_j_y_right_mat = int_constr_j_xu_mat * (0.5 * d_zeta) + int_constr_j_dxu_mat ;

    //write the first matrix block to the output
    multiplyAndSumVectorTo((real*) int_constr_j_y_left_mat.valuePtr(), values_left, _p_scale_factor_int_constr_j_y, _int_constr_j_y_nnz);

    // now algebraic states and controls
    multiplyAndSumVectorTo(int_constr_j_axu_values, values_left + _int_constr_j_y_nnz, _p_scale_factor_int_constr_j_ay, d_zeta, _int_constr_j_ay_nnz);

    //write the second matrix block to the output
    multiplyAndCopyVectorTo((real*) int_constr_j_y_right_mat.valuePtr(), values_right, _p_scale_factor_int_constr_j_y, _int_constr_j_y_nnz);
}
