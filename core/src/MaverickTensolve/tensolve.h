/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef TENSOLVE_H
#define TENSOLVE_H

#include "MaverickCore/MaverickDefinitions.hh"

#ifdef __cplusplus
extern "C" {
#endif

// function that evaluates at the point x the functions to be solved
typedef void (*TensolveEvalFunctions) (Maverick::real const x[],       // x value vector of dimension n (input)
                                       Maverick::real f[],             // function value vector of dimension m (output)
                                       Maverick::integer const * m,    // number of equations (input)
                                       Maverick::integer const * n);   // number of unknowns (input)

// function that evaluates the jacobian of the functions to be solved
/* the jacobian matrix, jac, has dimension max_m * n, and it is stored with column-major ordering in
a one-dimensional array. The user MUST FILL ONLY the m * n upper block, the other part is a working
memory area for tensolve. All the components of the upper m * n block must be written. */
typedef void (*TensolveEvalJacobian) (Maverick::real const x[],        // x value vector of dimension n (input)
                                      Maverick::real jac[],            // jacobian value vector of dimension max_m * n (output)
                                      Maverick::integer const * m,     // number of equations (input)
                                      Maverick::integer const * n);    // number of unknowns (input)


// Tensolve package functions

// simple interface for tensolve
void tensolve_tsnesi(Maverick::integer* max_m, Maverick::integer* max_n, Maverick::integer* max_p,
                          Maverick::real* x0, Maverick::integer const * m, Maverick::integer const * n,
                          TensolveEvalFunctions,
                          Maverick::integer* msg, Maverick::real* xp, Maverick::real* fp, Maverick::real* gp, Maverick::integer* termcd);

// complex interface for tensolve
void tensolve_tsneci(Maverick::integer* max_m, Maverick::integer* max_n, Maverick::integer* max_p, Maverick::real* x0, Maverick::integer const * m, Maverick::integer const * n,
                           Maverick::real* typ_x, Maverick::real* typ_f, Maverick::integer* it_lim,
                           Maverick::integer* jac_flag, Maverick::real* grad_tl, Maverick::real* step_tl, Maverick::real* f_tol,
                           Maverick::integer* method, Maverick::integer* global, Maverick::real* step_mx, Maverick::real* dlt, Maverick::integer* ipr,
                           TensolveEvalFunctions, TensolveEvalJacobian, Maverick::integer* msg,
                           Maverick::real* xp, Maverick::real* fp, Maverick::real* gp, Maverick::integer* termcd);

// dummy jacobian function
void tensolve_tsdumj(Maverick::real const * x, Maverick::real * jac, Maverick::integer const * m, Maverick::integer const * n);

// get tensolve default parameters
void tensolve_tsdflt(Maverick::integer const * m, Maverick::integer const * n, Maverick::integer * itnlim, Maverick::integer * jacflg, Maverick::real * gradtl, Maverick::real * steptl,
                           Maverick::real * ftol, Maverick::integer * method, Maverick::integer * global, Maverick::real * stepmx, Maverick::real * dlt,
                           Maverick::real * typx, Maverick::real * typf, Maverick::integer * ipr);

 #ifdef __cplusplus
 }
 #endif

#endif
