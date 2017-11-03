#include "Nlp2Ipopt.hh"
#include "MaverickCore/MaverickFunctions.hh"
#include "MaverickCore/MaverickPrivateDefs.hh"
#include "Eigen/SparseCore"
#include "iomanip"
#include <fstream>

using namespace Maverick;
using namespace Ipopt;
using namespace std;

Nlp2Ipopt::Nlp2Ipopt( NlpSolution & nlp_solution_unscaled ) : _nlp_solution_unscaled(nlp_solution_unscaled) {}

Nlp2Ipopt::Nlp2Ipopt( NlpSolution & nlp_solution_unscaled, std::shared_ptr<const Ocp2Nlp> ocp_2_nlp ) : _nlp_solution_unscaled(nlp_solution_unscaled) {
    setOcp2Nlp(ocp_2_nlp);
}

Nlp2Ipopt::~Nlp2Ipopt() {}


void Nlp2Ipopt::setOcp2Nlp( std::shared_ptr<const Ocp2Nlp> ocp_2_nlp ) {
    _ocp_2_nlp = ocp_2_nlp;
}

/** Method to return some info about the nlp */
bool Nlp2Ipopt::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                             Index& nnz_h_lag, IndexStyleEnum& index_style) {

    _num_constr = _ocp_2_nlp->getNlpConstraintsSize();
    _num_constr_jac = _ocp_2_nlp->getNlpConstraintsJacobianNnz();

    n = _ocp_2_nlp->getNlpSize();
    m = _num_constr;
    nnz_jac_g = _num_constr_jac;
    nnz_h_lag = _ocp_2_nlp->getNlpHessianNnz();

    index_style = C_STYLE;

    return true;
}

/** Method to return the bounds for my problem */
bool Nlp2Ipopt::get_bounds_info(Index n, Number* x_l, Number* x_u,
                                Index m, Number* g_l, Number* g_u) {

    _ocp_2_nlp->getNlpBounds( x_l, x_u, n );
    _ocp_2_nlp->getNlpConstraintsBounds( g_l, g_u, m );
    return true;
}

/** Method to return the starting point for the algorithm */
bool Nlp2Ipopt::get_starting_point(Index n, bool init_x, Number* x,
                                   bool init_z, Number* z_L, Number* z_U,
                                   Index m, bool init_lambda,
                                   Number* lambda) {

    if (init_x) {
        if ( _p_ext_nlp_guess_unscaled == nullptr ) {// by default, use all zero
            writeRealToVector( x, 0, n );
        } else { //otherwise use the provided guess
            MAVERICK_ASSERT( _p_ext_nlp_guess_unscaled->getNlpSize() == n, "Nlp2Ipopt::get_starting_point: nlp guess size is different from real nlp size.\n")
            Nlp nlp_scaled(*_p_ext_nlp_guess_unscaled);
            _ocp_2_nlp->scaleNlp(nlp_scaled);
            copyVectorTo(nlp_scaled.getY().data(), x, n);
        }
    }

    if (init_z) {
        if ( _p_ext_nlp_guess_unscaled == nullptr ) {// by default, use all zero
            writeRealToVector( z_L, 0, n );
            writeRealToVector( z_U, 0, n );
        } else { //otherwise use the provided guess
            MAVERICK_ASSERT( _p_ext_nlp_guess_unscaled->getNlpSize() == n, "Nlp2Ipopt::get_starting_point: nlp guess size is different from real nlp size.\n")
            Nlp nlp_scaled(*_p_ext_nlp_guess_unscaled);
            _ocp_2_nlp->scaleNlp(nlp_scaled);
            copyVectorTo(nlp_scaled.getLowerBoundsMultiplier().data(), z_L, n);
            copyVectorTo(nlp_scaled.getUpperBoundsMultiplier().data(), z_U, n);
        }
    }

    if (init_lambda) {
        if ( _p_ext_nlp_guess_unscaled == nullptr ) {// by default, use all zero
            writeRealToVector( lambda, 0, m );
        } else { //otherwise use the provided guess
            MAVERICK_ASSERT( _p_ext_nlp_guess_unscaled->getNlpConstraintsSize() == m, "Nlp2Ipopt::get_starting_point: nlp constraints guess size is different from real nlp constraints size.\n")
            Nlp nlp_scaled(*_p_ext_nlp_guess_unscaled);
            _ocp_2_nlp->scaleNlp(nlp_scaled);
            copyVectorTo(nlp_scaled.getConstraintsMultipliers().data(), lambda, m);
        }
    }
    return true;
}

/** Method to return the objective value */
bool Nlp2Ipopt::eval_f(Index n, const Number* x, bool new_x, Number& obj_value) {
    _ocp_2_nlp->calculateNlpQuantities(x, n, nullptr, 0, &obj_value, nullptr, 0, nullptr, 0, nullptr, 0, nullptr, 0);
    return true;
}

/** Method to return the gradient of the objective */
bool Nlp2Ipopt::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f) {
    _ocp_2_nlp->calculateNlpQuantities(x, n, nullptr, 0, nullptr, grad_f, n, nullptr, 0, nullptr, 0, nullptr, 0);
    return true;
}

/** Method to return the constraint residuals */
bool Nlp2Ipopt::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g) {
    _ocp_2_nlp->calculateNlpQuantities(x, n, nullptr, 0,
                                       nullptr,
                                       nullptr, 0,
                                       g, m,
                                       nullptr, 0,
                                       nullptr, 0
                                       );
    return true;
}

/** Method to return:
 *   1) The structure of the jacobian (if "values" is nullptr)
 *   2) The values of the jacobian (if "values" is not nullptr)
 */
bool Nlp2Ipopt::eval_jac_g(Index n, const Number* x, bool new_x,
                           Index m, Index nele_jac, Index* iRow, Index *jCol,
                           Number* values) {
    if ( values != nullptr )
        _ocp_2_nlp->calculateNlpQuantities(x, n, nullptr, 0, nullptr, nullptr, 0, nullptr, 0, values, nele_jac, nullptr, 0);
    else
        _ocp_2_nlp->getNlpConstraintsJacobianPattern( iRow, jCol, 0, 0, nele_jac );
    return true;
}

/** Method to return:
 *   1) The structure of the hessian of the lagrangian (if "values" is nullptr)
 *   2) The values of the hessian of the lagrangian (if "values" is not nullptr)
 */
bool Nlp2Ipopt::eval_h(Index n, const Number* x, bool new_x,
                       Number obj_factor, Index m, const Number* lambda,
                       bool new_lambda, Index nele_hess, Index* iRow,
                       Index* jCol, Number* values) {
    if ( values != nullptr )
        _ocp_2_nlp->calculateNlpQuantities(x, n, lambda, obj_factor, nullptr, nullptr, 0, nullptr, 0, nullptr, 0, values, nele_hess);
    else
        _ocp_2_nlp->getNlpHessianPattern( iRow, jCol, 0, 0, nele_hess );
    return true;
}

/** This method is called when the algorithm is complete so the TNLP can store/write the solution */
void Nlp2Ipopt::finalize_solution(SolverReturn status,
                                  Index n, const Number* x, const Number* z_L, const Number* z_U,
                                  Index m, const Number* g, const Number* lambda,
                                  Number obj_value,
                                  const IpoptData* ip_data,
                                  IpoptCalculatedQuantities* ip_cq) {
    _nlp_solution_unscaled.setNlp(n, x, z_U, z_L, m, g, lambda);
    _nlp_solution_unscaled.setSolverReturnStatus((integer) status);
    _ocp_2_nlp->scaleNlp(_nlp_solution_unscaled, true); // perform unscale
}

void Nlp2Ipopt::setNlpGuessPtr( Nlp const * p_nlp_guess_unscaled ) {
    _p_ext_nlp_guess_unscaled = p_nlp_guess_unscaled;
}
