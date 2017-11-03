#include "MidpointOcp2NlpSinglePhase.hh"
#include "MaverickCore/MaverickFunctions.hh"
#include "MaverickCore/MaverickPrivateDefs.hh"
#include "MaverickCore/OcpScaling.hh"
#include "MaverickUtils/GenericFunction/GF1ASpline.hh"
#include "MaverickCore/Midpoint/MidpointOcpSolution.hh"

using namespace Maverick;

#define SPLINE_TYPE "Akima"
//#define SPLINE_TYPE "Linear"

#define SPLINE_EXTEND_RANGE MaverickUtils::GF1ASpline::ExtendRange::keep_derivative

// convert nlp to ocp solution: lagrange multiplers are converted
std::unique_ptr<OcpSolution>  MidpointOcp2NlpSinglePhase::translateNlp2OcpSolution( Nlp const & nlp ) const {
    MidpointOcpSolution * sol = new MidpointOcpSolution();
    sol->setSolutionAtPhase(_i_phase, *(translateNlp2MidpointOcpSolution(nlp)) );
    return std::unique_ptr<MidpointOcpSolution> (sol);
};

// convert nlp to ocp solution: lagrange multiplers are converted
std::unique_ptr<MidpointOcpSolutionSinglePhase>  MidpointOcp2NlpSinglePhase::translateNlp2MidpointOcpSolution( Nlp const & nlp_input ) const {

    MAVERICK_ASSERT( nlp_input.getNlpSize() == getNlpSize(), "MidpointOcp2NlpSinglePhase::translateNlp2MidpointOcpSolution: wrong nlp size.\n");
    MAVERICK_ASSERT( nlp_input.getNlpConstraintsSize() == getNlpConstraintsSize(), "MidpointOcp2NlpSinglePhase::translateNlp2MidpointOcpSolution: wrong nlp constraints size.\n");

    // convert multipliers from nlp to ocp first
    Nlp nlp_tmp(nlp_input);
    convertNlp2OcpMultipliers(nlp_tmp,
                              false);
    // now the nlp should no more be modified, so we take a const reference
    Nlp const & nlp = nlp_tmp;

    integer const num_mesh_point = _p_mesh->getNumberOfDiscretisationPoints();

    // initialize variables
    vec_1d_real cumulative_target(num_mesh_point);
    cumulative_target[0] = 0;
    vec_1d_real integrand_target(num_mesh_point-1); // evaluated at middle mesh point

    vec_2d_real states_controls(_dim_xu);
    vec_2d_real states_controls_lower_bound_mult(_dim_xu);
    vec_2d_real states_controls_upper_bound_mult(_dim_xu);
    for (size i=0; i<_dim_xu; i++) {
        states_controls[i] = vec_1d_real(num_mesh_point);
        states_controls_lower_bound_mult[i] = vec_1d_real(num_mesh_point);
        states_controls_upper_bound_mult[i] = vec_1d_real(num_mesh_point);
    }

    vec_2d_real algebraic_states_controls(_dim_axu);
    vec_2d_real algebraic_states_controls_lower_bound_mult(_dim_axu);
    vec_2d_real algebraic_states_controls_upper_bound_mult(_dim_axu);
    for (size i=0; i<_dim_axu; i++) {
        algebraic_states_controls[i] = vec_1d_real(num_mesh_point-1);
        algebraic_states_controls_lower_bound_mult[i] = vec_1d_real(num_mesh_point-1);
        algebraic_states_controls_upper_bound_mult[i] = vec_1d_real(num_mesh_point-1);
    }

    vec_2d_real states_constr(_dim_poc);
    vec_2d_real states_constr_mult(_dim_poc);
    for (size i=0; i<_dim_poc; i++) {
        states_constr[i] = vec_1d_real(num_mesh_point);
        states_constr_mult[i] = vec_1d_real(num_mesh_point);
    }

    vec_2d_real path_constr(_dim_pc);
    vec_2d_real path_constr_mult(_dim_pc);
    for (size i=0; i<_dim_pc; i++) {
        path_constr[i] = vec_1d_real(num_mesh_point-1); // evaluated at middle mesh point
        path_constr_mult[i] = vec_1d_real(num_mesh_point-1); // evaluated at middle mesh point
    }

    vec_2d_real int_constr(_dim_ic);
    vec_1d_real int_constr_mult(_dim_ic);
    for (size i=0; i<_dim_ic; i++) {
        int_constr[i] = vec_1d_real(num_mesh_point);
        int_constr[i][0]=0;
    }

    vec_2d_real fo_eqns(_dim_fo);
    vec_2d_real fo_eqns_mult(_dim_fo);
    for (size i=0; i<_dim_fo; i++) {
        fo_eqns[i] = vec_1d_real(num_mesh_point-1); // evaluated at middle mesh point
        fo_eqns_mult[i] = vec_1d_real(num_mesh_point-1); // evaluated at middle mesh point
    }

    vec_2d_real post_proc(_ocp_problem.numberOfPostProcessing(_i_phase));
    for (size i=0; i<post_proc.size(); i++) {
        post_proc[i] = vec_1d_real(num_mesh_point);
    }

    vec_2d_real diff_post_proc(_ocp_problem.numberOfDifferentialPostProcessing(_i_phase));
    for (size i=0; i<diff_post_proc.size(); i++) {
        diff_post_proc[i] = vec_1d_real(num_mesh_point-1); // evaluated at middle mesh point
    }

    vec_2d_real int_post_proc(_ocp_problem.numberOfIntegralPostProcessing(_i_phase));
    for (size i=0; i<int_post_proc.size(); i++) {
        int_post_proc[i] = vec_1d_real(num_mesh_point);
        int_post_proc[i][0]=0;
    }

    vec_1d_real params(_dim_p);
    vec_1d_real params_upper_bounds_mult(_dim_p);
    vec_1d_real params_lower_bounds_mult(_dim_p);

    vec_1d_real boundary_conditions( _dim_bc );
    vec_1d_real bcs_mult( _dim_bc);

    // loop over mesh point to save solution
    real const * current_nlp_y = nlp.getY().data();
    real const * current_nlp_z_u = nlp.getUpperBoundsMultiplier().data();
    real const * current_nlp_z_l = nlp.getLowerBoundsMultiplier().data();
    real const * current_nlp_constr = nlp.getConstraints().data();
    real const * current_nlp_constr_mult = nlp.getConstraintsMultipliers().data();
    //    int counter = 0;
    real const * nlp_params = current_nlp_y + getNlpParamPtrIndex();

    real const * ocp_initial_state_control = current_nlp_y;
    real const * ocp_final_state_control = current_nlp_y + getNlpYPtrIndexForInterval(_p_mesh->getNumberOfIntervals());

    for (integer c_mesh_point=0; c_mesh_point<num_mesh_point; c_mesh_point++) {
        //zeta
        real c_zeta = _p_mesh->getZeta(c_mesh_point);

        // left state
        real const *left_xu = current_nlp_y;

        //states and controls
        for (integer j=0; j<states_controls.size(); j++) {
            states_controls[j][c_mesh_point] = *(current_nlp_y + j );
            states_controls_upper_bound_mult[j][c_mesh_point] = *(current_nlp_z_u + j );
            states_controls_lower_bound_mult[j][c_mesh_point] = *(current_nlp_z_l + j );
        }

        //post processing
        real tmp_post_proc[post_proc.size()];
        _ocp_problem.postProcessing(_i_phase, left_xu, nlp_params, c_zeta, tmp_post_proc);
        for (integer j=0; j<post_proc.size(); j++)
            post_proc[j][c_mesh_point] = tmp_post_proc[j];

        if ( c_mesh_point < num_mesh_point - 1) {

            //algebraic states and controls
            for (integer j=0; j<algebraic_states_controls.size(); j++) {
                algebraic_states_controls[j][c_mesh_point] = *(current_nlp_y + _dim_y + j );
                algebraic_states_controls_upper_bound_mult[j][c_mesh_point] = *(current_nlp_z_u + _dim_y + j );
                algebraic_states_controls_lower_bound_mult[j][c_mesh_point] = *(current_nlp_z_l + _dim_y + j );
            }

            //states constraints
            for (integer j=0; j<states_constr.size(); j++) {
                states_constr[j][c_mesh_point] = *(current_nlp_constr + _dim_fo + _dim_pc + j );
                states_constr_mult[j][c_mesh_point] = *(current_nlp_constr_mult + _dim_fo + _dim_pc + j );
            }

            //right and center states
            real const *right_xu = current_nlp_y + _dim_y + _dim_ay;
            real center_xu[_dim_xu];
            computeTpzCenterWithoutScaling(left_xu, right_xu, center_xu, _dim_xu);

            // algerbaic states
            real const *alg_xu = current_nlp_y + _dim_y;

            real c_center_zeta = _p_mesh->getZetaCenter(c_mesh_point);
            real c_d_zeta = _p_mesh->getDz(c_mesh_point);
            real diff_xu[_dim_xu];
            computeTpzDerivativeWithoutScaling(left_xu, right_xu, diff_xu, 1.0/c_d_zeta, _dim_xu);

            //integrand target
            _ocp_problem.lagrange(_i_phase, center_xu, diff_xu, alg_xu, nlp_params, c_center_zeta, integrand_target[c_mesh_point]);

            //cumulative target
            cumulative_target[c_mesh_point+1] = cumulative_target[c_mesh_point] + integrand_target[c_mesh_point] * c_d_zeta;

            //fo equations
            for (integer j=0; j<fo_eqns.size(); j++) {
                fo_eqns[j][c_mesh_point] = *(current_nlp_constr + j );
                fo_eqns_mult[j][c_mesh_point] = *(current_nlp_constr_mult + j );
            }
#ifdef MAVERICK_DEBUG_SOLUTION
            real tmp_fo_eqns[_dim_fo];
            _ocp_problem.foEqns(_i_phase, center_xu, diff_xu, nlp_params, c_center_zeta, tmp_fo_eqns);
            for (integer tmp=0; tmp<_dim_fo; tmp++) {
                real error = abs(tmp_fo_eqns[tmp] - *(current_nlp_constr + tmp ) );
                MAVERICK_ASSERT( error < 1e-6, "MidpointOcp2NlpSinglePhase::translateNlp2MidpointOcpSolution: wrong evaluation of fo equations. Nlp value: " << *(current_nlp_constr + _dim_poc + tmp) << ", calculated value: "  << tmp_fo_eqns[tmp] << " at mesh point " << c_mesh_point << ".\n")
            }
#endif

            //path constraints
            for (integer j=0; j<path_constr.size(); j++) {
                path_constr[j][c_mesh_point] = *(current_nlp_constr + _dim_fo + j );
                path_constr_mult[j][c_mesh_point] = *(current_nlp_constr_mult + _dim_fo + j );
            }

#ifdef MAVERICK_DEBUG_SOLUTION
            real tmp_path_constr[_dim_pc];
            _ocp_problem.pathConstraints(_i_phase, center_xu, diff_xu, nlp_params, c_center_zeta, tmp_path_constr);
            for (integer tmp=0; tmp<_dim_pc; tmp++) {
                real error = abs(tmp_path_constr[tmp] - *(current_nlp_constr + _dim_fo + tmp ) );
                MAVERICK_ASSERT( error < 1e-4, "MidpointOcp2NlpSinglePhase::translateNlp2MidpointOcpSolution: wrong evaluation of path constraints\n")
            }
#endif
            //integral constraints
            real tmp_int_constr[_dim_ic];
            _ocp_problem.intConstraints(_i_phase, center_xu, diff_xu, alg_xu, nlp_params, c_center_zeta, tmp_int_constr);
            for (integer j=0; j<int_constr.size(); j++)
                int_constr[j][c_mesh_point+1] = int_constr[j][c_mesh_point] + tmp_int_constr[j] * c_d_zeta;

            //differential post processing
            real tmp_diff_post_proc[diff_post_proc.size()];
            _ocp_problem.differentialPostProcessing(_i_phase, center_xu, diff_xu, alg_xu, nlp_params, c_center_zeta, tmp_diff_post_proc);
            for (integer j=0; j<diff_post_proc.size(); j++)
                diff_post_proc[j][c_mesh_point] = tmp_diff_post_proc[j];

            //integral post processing
            real tmp_int_post_proc[int_post_proc.size()];
            _ocp_problem.integralPostProcessing(_i_phase, center_xu, diff_xu, alg_xu, nlp_params, c_center_zeta, tmp_int_post_proc);
            for (integer j=0; j<int_post_proc.size(); j++)
                int_post_proc[j][c_mesh_point+1] = int_post_proc[j][c_mesh_point] + tmp_int_post_proc[j] * c_d_zeta;

        } else {
            //states constraints
            for (integer j=0; j<states_constr.size(); j++) {
                states_constr[j][c_mesh_point] = *(current_nlp_constr + j );
                states_constr_mult[j][c_mesh_point] = *(current_nlp_constr_mult + j );
            }

        }
        if ( c_mesh_point < num_mesh_point - 1) {
            current_nlp_y += _dim_y + _dim_ay;
            current_nlp_z_u += _dim_y + _dim_ay;
            current_nlp_z_l += _dim_y + _dim_ay;

            current_nlp_constr += _dim_q;
            current_nlp_constr_mult += _dim_q;
        }
        else {
            current_nlp_y += _dim_y;
            current_nlp_z_u += _dim_y;
            current_nlp_z_l += _dim_y;

            current_nlp_constr += _dim_poc;
            current_nlp_constr_mult += _dim_poc;
        }
    }
    //now boundary conditions
    copyVectorTo(current_nlp_constr, boundary_conditions.data(), _dim_bc);
    copyVectorTo(current_nlp_constr_mult, bcs_mult.data(), _dim_bc);
    current_nlp_constr += _dim_bc;
    current_nlp_constr_mult += _dim_bc;

    //integral constraints
    copyVectorTo(current_nlp_constr_mult, int_constr_mult.data(), _dim_ic);
    current_nlp_constr += _dim_ic;
    current_nlp_constr_mult += _dim_ic;

    //check the integral constraints
#ifdef MAVERICK_DEBUG_SOLUTION
    real const * const int_constr_ptr = nlp.getConstraints().data() + getNlpConstraintsPtrIndexForIntConstr();
    for (integer tmp=0; tmp<_dim_ic; tmp++) {
        real error = abs( *(int_constr[tmp].end()-1) - (int_constr_ptr[tmp] ) );
        if ( *(int_constr[tmp].end()-1) != 0 )
            error = error / abs((*(int_constr[tmp].end()-1)));
        MAVERICK_ASSERT( error < 1e-3, "MidpointOcp2NlpSinglePhase::translateNlp2MidpointOcpSolution: wrong evaluation of integral constraints. Calculated " << *(int_constr[tmp].end()-1) << ", should be " << int_constr_ptr[tmp] << "\n")
    }
#endif

    //nlp parameters
    copyVectorTo(current_nlp_y, params.data(), _dim_p);
    copyVectorTo(current_nlp_z_u, params_upper_bounds_mult.data(), _dim_p);
    copyVectorTo(current_nlp_z_l, params_lower_bounds_mult.data(), _dim_p);
    current_nlp_y += _dim_p;
    current_nlp_z_u += _dim_p;
    current_nlp_z_l += _dim_p;

    //target
    real mayer_target;
    _ocp_problem.mayer(_i_phase, ocp_initial_state_control, ocp_final_state_control, nlp_params, mayer_target);
    real const target = cumulative_target.back() + mayer_target;

    // some debug checks
    MAVERICK_DEBUG_ASSERT(current_nlp_y == nlp.getY().data() + getNlpSize(), "MidpointOcp2NlpSinglePhase::getMidpointOcpSolution: wrong final pointer to the nlp states.\n")
    MAVERICK_DEBUG_ASSERT(current_nlp_z_u == nlp.getUpperBoundsMultiplier().data() + getNlpSize(), "MidpointOcp2NlpSinglePhase::getMidpointOcpSolution: wrong final pointer to the nlp upper multipliers.\n")
    MAVERICK_DEBUG_ASSERT(current_nlp_z_l == nlp.getLowerBoundsMultiplier().data() + getNlpSize(), "MidpointOcp2NlpSinglePhase::getMidpointOcpSolution: wrong final pointer to the nlp lower bounds multipliers.\n")
    MAVERICK_DEBUG_ASSERT(current_nlp_constr == nlp.getConstraints().data() + getNlpConstraintsSize(), "MidpointOcp2NlpSinglePhase::getMidpointOcpSolution: wrong final pointer to the nlp constraints.\n")
    MAVERICK_DEBUG_ASSERT(current_nlp_constr_mult == nlp.getConstraintsMultipliers().data() + getNlpConstraintsSize(), "MidpointOcp2NlpSinglePhase::getMidpointOcpSolution: wrong final pointer to the nlp constraints multipliers.\n")

    MidpointOcpSolutionSinglePhase * ocp_solution = new MidpointOcpSolutionSinglePhase();
    ocp_solution->setSolution(target,
                             _p_mesh->getDiscretisationPoints(),
                             cumulative_target,
                             integrand_target,
                             states_controls,
                             algebraic_states_controls,
                             params,
                             states_constr,
                             path_constr,
                             int_constr,
                             fo_eqns,
                             boundary_conditions,
                             post_proc,
                             diff_post_proc,
                             int_post_proc,
                             states_controls_upper_bound_mult,
                             states_controls_lower_bound_mult,
                             algebraic_states_controls_upper_bound_mult,
                             algebraic_states_controls_lower_bound_mult,
                             params_upper_bounds_mult,
                             params_lower_bounds_mult,
                             states_constr_mult,
                             path_constr_mult,
                             int_constr_mult,
                             fo_eqns_mult,
                             bcs_mult
                             );

    return std::unique_ptr<MidpointOcpSolutionSinglePhase> (ocp_solution);
}

// convert ocp guess or ocp solution to nlp: lagrange multiplers are converted
Nlp MidpointOcp2NlpSinglePhase::translateOcpGuess2Nlp( OcpGuess const & ocp_guess ) const {

    integer const nlp_size = getNlpSize();

    real * nlp_y = new real[ nlp_size ];
    //initialize to zero for safety
    writeRealToVector(nlp_y, 0, nlp_size);

    real * nlp_z_u = new real[ nlp_size ];
    //initialize to zero for safety
    writeRealToVector(nlp_z_u, 0, nlp_size);

    real * nlp_z_l = new real[ nlp_size ];
    //initialize to zero for safety
    writeRealToVector(nlp_z_l, 0, nlp_size);

    real * current_y = nlp_y;
    real * current_z_u = nlp_z_u;
    real * current_z_l = nlp_z_l;

//    real nlp_constr[ getNlpConstraintsSize() ];
    real * nlp_constr_mult = new real[ getNlpConstraintsSize() ];
    //initialize to zero for safety
    writeRealToVector(nlp_constr_mult, 0, getNlpConstraintsSize() );
//    real * current_nlp_constr = nlp_constr;
    real * current_nlp_constr_mult = nlp_constr_mult;

    // pointer to parameters
    real * const params = nlp_y + getNlpParamPtrIndex() ;
    real * const params_z_u = nlp_z_u + getNlpParamPtrIndex() ;
    real * const params_z_l = nlp_z_l + getNlpParamPtrIndex() ;

    // pointer to boundary conditions
    real * const nlp_bcs_mult = nlp_constr_mult + getNlpConstraintsPtrIndexForBcs();

    // pointer to integral constraints
    real * const nlp_int_constr_mult = nlp_constr_mult + getNlpConstraintsPtrIndexForIntConstr() ;

    real first_zeta = _p_mesh->getDiscretisationPoints().front();
    real last_zeta = _p_mesh->getDiscretisationPoints().back();

    // eval parameter, boundary conditions and integral constraints
    ocp_guess.eval(_i_phase,
                   first_zeta, last_zeta,
                   _dim_p, params, params_z_u, params_z_l,
                   _dim_bc, nlp_bcs_mult,
                   _dim_ic, nlp_int_constr_mult );

    //now write the provided guess at each mesh point
    for (integer c_mesh_point =0; c_mesh_point < _p_mesh->getNumberOfIntervals(); c_mesh_point++ ) {
        real c_zeta = _p_mesh->getZeta(c_mesh_point);
        real c_zeta_center = _p_mesh->getZetaCenter(c_mesh_point);

        real * fo_eqns_mult = current_nlp_constr_mult;
        real * path_constr_mult = current_nlp_constr_mult + _dim_fo;
        real * point_constr_mult = path_constr_mult + _dim_pc;

        // eval quantities at mesh point
        ocp_guess.evalAtMesh(_i_phase,
                             c_zeta,
                             _dim_xu, current_y, current_z_u, current_z_l,
                             0, nullptr, nullptr, nullptr, //algebraic states
                             0, nullptr,
                             _dim_poc, point_constr_mult,
                             0, nullptr);

        // eval quantities at middle of mesh interval
        ocp_guess.evalAtMesh(_i_phase,
                             c_zeta_center,
                             0, nullptr, nullptr, nullptr,
                             _dim_axu, current_y + _dim_y, current_z_u + _dim_y, current_z_l + _dim_y, //algebraic states
                             _dim_fo,  fo_eqns_mult,
                             0, nullptr,
                             _dim_pc, path_constr_mult);

        current_y += _dim_y + _dim_ay;
        current_z_u += _dim_y + _dim_ay;
        current_z_l += _dim_y + _dim_ay;
        current_nlp_constr_mult += _dim_q;
    }

    // now evaluate last constraints and states
    real * point_constr_mult = current_nlp_constr_mult;

    ocp_guess.evalAtMesh(_i_phase,
                         last_zeta,
                         _dim_xu, current_y, current_z_u, current_z_l,
                         0, nullptr, nullptr, nullptr, //algebraic states
                         0, nullptr,
                         _dim_poc, point_constr_mult,
                         0, nullptr);

    current_y += _dim_y;
    current_z_u += _dim_y;
    current_z_l += _dim_y;
    current_nlp_constr_mult += _dim_poc;

    // boundary conditions and integral constraints have already been computed
    current_nlp_constr_mult += _dim_bc;
    current_nlp_constr_mult += _dim_ic;

    // parameters have already been copied
    current_y += _dim_p;
    current_z_u += _dim_p;
    current_z_l += _dim_p;

    MAVERICK_DEBUG_ASSERT( current_y == nlp_y + nlp_size, "MidpointOcp2NlpSinglePhase::translateMidpointOcpSolution2Nlp: wrong final pointer to nlp y. Difference (current minus expected): " << current_y - (nlp_y + nlp_size)  << " \n")
    MAVERICK_DEBUG_ASSERT( current_z_u == nlp_z_u + nlp_size, "MidpointOcp2NlpSinglePhase::translateMidpointOcpSolution2Nlp: wrong final pointer to nlp z upper.\n")
    MAVERICK_DEBUG_ASSERT( current_z_l == nlp_z_l + nlp_size, "MidpointOcp2NlpSinglePhase::translateMidpointOcpSolution2Nlp: wrong final pointer to nlp z lower.\n")
    MAVERICK_DEBUG_ASSERT( current_nlp_constr_mult == nlp_constr_mult + getNlpConstraintsSize(), "MidpointOcp2NlpSinglePhase::translateMidpointOcpSolution2Nlp: wrong final pointer to nlp contraints.\n")
    MAVERICK_DEBUG_ASSERT( nlp_bcs_mult + _dim_bc == nlp_int_constr_mult, "MidpointOcp2NlpSinglePhase::translateMidpointOcpSolution2Nlp: wrong final pointer for boundary conditions multipliers.\n")
    MAVERICK_DEBUG_ASSERT( nlp_int_constr_mult + _dim_ic == nlp_constr_mult + getNlpConstraintsSize(), "MidpointOcp2NlpSinglePhase::translateMidpointOcpSolution2Nlp: wrong final pointer for integral constraints multipliers.\n")

    // write the solution
    Nlp nlp;

    // set constraints to zero because they haven't been calculated
    real *nlp_constr = new real[ getNlpConstraintsSize() ];
    writeRealToVector(nlp_constr, 0, getNlpConstraintsSize());

    nlp.setNlp(nlp_size, nlp_y, nlp_z_u, nlp_z_l,
               getNlpConstraintsSize(), nlp_constr, nlp_constr_mult
//               lagrange_mayer_percentage
               );

    // convert the multipleris from OCP to NLP
    convertNlp2OcpMultipliers(nlp,
                              true);
    delete [] nlp_y;
    delete [] nlp_z_u;
    delete [] nlp_z_l;
    delete [] nlp_constr_mult;
    delete [] nlp_constr;

    return nlp;
}

void MidpointOcp2NlpSinglePhase::scaleNlp(Nlp & nlp,
                                     bool const unscale) const
{
    Nlp const & nlp_input = nlp;
    MAVERICK_DEBUG_ASSERT( nlp_input.getNlpSize() ==  getNlpSize(), "MidpointOcp2NlpSinglePhase::scaleNlp: wrong nlp size.\n")
    MAVERICK_DEBUG_ASSERT( nlp_input.getNlpConstraintsSize() ==  getNlpConstraintsSize(), "MidpointOcp2NlpSinglePhase::scaleNlp: wrong nlp constraints size.\n")

    // create working variables
    real * nlp_y_output                  = new real[getNlpSize()];
    real * nlp_y_upper_bound_mult_output = new real[getNlpSize()];
    real * nlp_y_lower_bound_mult_output = new real[getNlpSize()];
    real * nlp_constraints_output        = new real[getNlpConstraintsSize()];
    real * nlp_constraints_mult_output   = new real[getNlpConstraintsSize()];

    real const * c_nlp_y_input                = nlp_input.getY().data();
    real       * c_nlp_y_output               = nlp_y_output;
    real const * c_nlp_z_upper_input          = nlp_input.getUpperBoundsMultiplier().data();
    real       * c_nlp_z_upper_output         = nlp_y_upper_bound_mult_output;
    real const * c_nlp_z_lower_input          = nlp_input.getLowerBoundsMultiplier().data();
    real       * c_nlp_z_lower_output         = nlp_y_lower_bound_mult_output;
    real const * c_nlp_constr_input           = nlp_input.getConstraints().data();
    real       * c_nlp_constr_output          = nlp_constraints_output;
    real const * c_nlp_constr_mult_input      = nlp_input.getConstraintsMultipliers().data();
    real       * c_nlp_constr_mult_output     = nlp_constraints_mult_output;

    // create working variables for inverse scaling coefficients
    real p_inv_scaling_y[_dim_y];
    copyVectorTo(_p_inv_scaling_y, p_inv_scaling_y, _dim_y);
    real p_inv_scaling_ay[_dim_ay];
    copyVectorTo(_p_inv_scaling_ay, p_inv_scaling_ay, _dim_ay);
    real p_inv_scaling_r[_dim_p];
    copyVectorTo(_p_inv_scaling_r, p_inv_scaling_r, _dim_p);
    real p_inv_scaling_fo_eqns_global[_dim_fo];
    copyVectorTo(_p_inv_scaling_fo_eqns_global, p_inv_scaling_fo_eqns_global, _dim_fo);
    real p_inv_scaling_path_constr_global[_dim_pc];
    copyVectorTo(_p_inv_scaling_path_constr_global, p_inv_scaling_path_constr_global, _dim_pc);
    real p_inv_scaling_point_constr_global[_dim_poc];
    copyVectorTo(_p_inv_scaling_point_constr_global, p_inv_scaling_point_constr_global, _dim_poc);
    real p_inv_scaling_int_constr[_dim_ic];
    copyVectorTo(_p_inv_scaling_int_constr, p_inv_scaling_int_constr, _dim_ic);
    real p_inv_scaling_bcs[_dim_bc];
    copyVectorTo(_p_inv_scaling_bcs, p_inv_scaling_bcs, _dim_bc);
    real inv_scaling_target = _inv_scaling_target;

    // if the UNSCALE is requested, invert the previous created coefficients
    if (unscale) {
        writeInverseVectorTo(p_inv_scaling_y, p_inv_scaling_y, _dim_y);
        writeInverseVectorTo(p_inv_scaling_ay, p_inv_scaling_ay, _dim_ay);
        writeInverseVectorTo(p_inv_scaling_r, p_inv_scaling_r, _dim_p);
        writeInverseVectorTo(p_inv_scaling_fo_eqns_global, p_inv_scaling_fo_eqns_global, _dim_fo);
        writeInverseVectorTo(p_inv_scaling_path_constr_global, p_inv_scaling_path_constr_global, _dim_pc);
        writeInverseVectorTo(p_inv_scaling_point_constr_global, p_inv_scaling_point_constr_global, _dim_poc);
        writeInverseVectorTo(p_inv_scaling_int_constr, p_inv_scaling_int_constr, _dim_ic);
        writeInverseVectorTo(p_inv_scaling_bcs, p_inv_scaling_bcs, _dim_bc);
        inv_scaling_target = 1/inv_scaling_target;
    }

    // create working variables for scaling coefficients
    real p_scaling_y[_dim_y];
    writeInverseVectorTo(p_inv_scaling_y, p_scaling_y, _dim_y);
    real p_scaling_ay[_dim_ay];
    writeInverseVectorTo(p_inv_scaling_ay, p_scaling_ay, _dim_ay);
    real p_scaling_r[_dim_p];
    writeInverseVectorTo(p_inv_scaling_r, p_scaling_r, _dim_p);
    real p_scaling_fo_eqns_global[_dim_fo];
    writeInverseVectorTo(p_inv_scaling_fo_eqns_global, p_scaling_fo_eqns_global, _dim_fo);
    real p_scaling_path_constr_global[_dim_pc];
    writeInverseVectorTo(p_inv_scaling_path_constr_global, p_scaling_path_constr_global, _dim_pc);
    real p_scaling_point_constr_global[_dim_poc];
    writeInverseVectorTo(p_inv_scaling_point_constr_global, p_scaling_point_constr_global, _dim_poc);
    real p_scaling_int_constr[_dim_ic];
    writeInverseVectorTo(p_inv_scaling_int_constr, p_scaling_int_constr, _dim_ic);
    real p_scaling_bcs[_dim_bc];
    writeInverseVectorTo(p_inv_scaling_bcs, p_scaling_bcs, _dim_bc);

    //run over all the mesh points and scale (or UNSCALE) the nlp data
    integer const max_index = _p_mesh->getNumberOfIntervals();
    for (integer c_mesh_point =0; c_mesh_point < _p_mesh->getNumberOfDiscretisationPoints(); c_mesh_point++ ) {
        integer index = c_mesh_point;
        if (index==max_index) // to avoid accessing wrong memroy location at last loop
            index--;

        // scale states
        multiplyAndCopyVectorTo(c_nlp_y_input, c_nlp_y_output, p_inv_scaling_y, _dim_y);
        c_nlp_y_input += _dim_y;
        c_nlp_y_output += _dim_y;

        multiplyAndCopyVectorTo(c_nlp_z_upper_input, c_nlp_z_upper_output, p_scaling_y, _dim_y);
        c_nlp_z_upper_input += _dim_y;
        c_nlp_z_upper_output += _dim_y;

        multiplyAndCopyVectorTo(c_nlp_z_lower_input, c_nlp_z_lower_output, p_scaling_y, _dim_y);
        c_nlp_z_lower_input += _dim_y;
        c_nlp_z_lower_output += _dim_y;

        if ( c_mesh_point != (_p_mesh->getNumberOfDiscretisationPoints()-1) )  {
            real const d_zeta = _p_mesh->getDz(c_mesh_point);

            // scale algebraic states
            multiplyAndCopyVectorTo(c_nlp_y_input, c_nlp_y_output, p_inv_scaling_ay, _dim_ay);
            c_nlp_y_input += _dim_ay;
            c_nlp_y_output += _dim_ay;

            multiplyAndCopyVectorTo(c_nlp_z_upper_input, c_nlp_z_upper_output, p_scaling_ay, _dim_ay);
            c_nlp_z_upper_input += _dim_ay;
            c_nlp_z_upper_output += _dim_ay;

            multiplyAndCopyVectorTo(c_nlp_z_lower_input, c_nlp_z_lower_output, p_scaling_ay, _dim_ay);
            c_nlp_z_lower_input += _dim_ay;
            c_nlp_z_lower_output += _dim_ay;

            // fo eqns
            real p_inv_scaling_fo_eqns[_dim_fo];
            real p_scaling_fo_eqns[_dim_fo];
            if (_multiply_foeqns_by_dz) {
                multiplyAndCopyVectorTo(p_inv_scaling_fo_eqns_global, p_inv_scaling_fo_eqns, d_zeta, _dim_fo);
                multiplyAndCopyVectorTo(p_scaling_fo_eqns_global, p_scaling_fo_eqns, 1.0/d_zeta, _dim_fo);
            } else {
                copyVectorTo(p_inv_scaling_fo_eqns_global, p_inv_scaling_fo_eqns, _dim_fo);
                copyVectorTo(p_scaling_fo_eqns_global, p_scaling_fo_eqns, _dim_fo);
            }

            multiplyAndCopyVectorTo(c_nlp_constr_input, c_nlp_constr_output, p_inv_scaling_fo_eqns, _dim_fo);
            c_nlp_constr_input += _dim_fo;
            c_nlp_constr_output += _dim_fo;

            multiplyAndCopyVectorTo(c_nlp_constr_mult_input, c_nlp_constr_mult_output, p_scaling_fo_eqns, _dim_fo);
            c_nlp_constr_mult_input += _dim_fo;
            c_nlp_constr_mult_output += _dim_fo;

            //path constr
            real p_inv_scaling_path_constr[_dim_pc];
            real p_scaling_path_constr[_dim_pc];
            if (_multiply_path_constr_by_dz) {
                multiplyAndCopyVectorTo(p_inv_scaling_path_constr_global, p_inv_scaling_path_constr, d_zeta, _dim_pc);
                multiplyAndCopyVectorTo(p_scaling_path_constr_global, p_scaling_path_constr, 1.0/d_zeta, _dim_pc);
            } else {
                copyVectorTo(p_inv_scaling_path_constr_global, p_inv_scaling_path_constr, _dim_pc);
                copyVectorTo(p_scaling_path_constr_global, p_scaling_path_constr, _dim_pc);
            }

            multiplyAndCopyVectorTo(c_nlp_constr_input, c_nlp_constr_output, p_inv_scaling_path_constr, _dim_pc);
            c_nlp_constr_input += _dim_pc;
            c_nlp_constr_output += _dim_pc;

            multiplyAndCopyVectorTo(c_nlp_constr_mult_input, c_nlp_constr_mult_output, p_scaling_path_constr, _dim_pc);
            c_nlp_constr_mult_input += _dim_pc;
            c_nlp_constr_mult_output += _dim_pc;
        }

        //point constraints
        real const d_zeta_dual = _p_mesh->getDzDual(c_mesh_point);
        real p_inv_scaling_point_constr[_dim_poc];
        real p_scaling_point_constr[_dim_poc];
        if (_multiply_point_constr_by_dz) {
            multiplyAndCopyVectorTo(p_inv_scaling_point_constr_global, p_inv_scaling_point_constr, d_zeta_dual, _dim_poc);
            multiplyAndCopyVectorTo(p_scaling_point_constr_global, p_scaling_point_constr, 1.0/d_zeta_dual, _dim_poc);
        } else {
            copyVectorTo(p_inv_scaling_point_constr_global, p_inv_scaling_point_constr, _dim_poc);
            copyVectorTo(p_scaling_point_constr_global, p_scaling_point_constr, _dim_poc);
        }

        multiplyAndCopyVectorTo(c_nlp_constr_input, c_nlp_constr_output, p_inv_scaling_point_constr, _dim_poc);
        c_nlp_constr_input += _dim_poc;
        c_nlp_constr_output += _dim_poc;

        multiplyAndCopyVectorTo(c_nlp_constr_mult_input, c_nlp_constr_mult_output, p_scaling_point_constr, _dim_poc);
        c_nlp_constr_mult_input += _dim_poc;
        c_nlp_constr_mult_output += _dim_poc;
    }

    // parameters
    multiplyAndCopyVectorTo(c_nlp_y_input, c_nlp_y_output, p_inv_scaling_r, _dim_p);
    c_nlp_y_input += _dim_p;
    c_nlp_y_output += _dim_p;

    multiplyAndCopyVectorTo(c_nlp_z_upper_input, c_nlp_z_upper_output, p_scaling_r, _dim_p);
    c_nlp_z_upper_input += _dim_p;
    c_nlp_z_upper_output += _dim_p;

    multiplyAndCopyVectorTo(c_nlp_z_lower_input, c_nlp_z_lower_output, p_scaling_r, _dim_p);
    c_nlp_z_lower_input += _dim_p;
    c_nlp_z_lower_output += _dim_p;

    // boundary conditions
    multiplyAndCopyVectorTo(c_nlp_constr_input, c_nlp_constr_output, p_inv_scaling_bcs, _dim_bc);
    c_nlp_constr_input += _dim_bc;
    c_nlp_constr_output += _dim_bc;

    multiplyAndCopyVectorTo(c_nlp_constr_mult_input, c_nlp_constr_mult_output, p_scaling_bcs, _dim_bc);
    c_nlp_constr_mult_input += _dim_bc;
    c_nlp_constr_mult_output += _dim_bc;

    // integral constraints
    multiplyAndCopyVectorTo(c_nlp_constr_input, c_nlp_constr_output, p_inv_scaling_int_constr, _dim_ic);
    c_nlp_constr_input += _dim_ic;
    c_nlp_constr_output += _dim_ic;

    multiplyAndCopyVectorTo(c_nlp_constr_mult_input, c_nlp_constr_mult_output, p_scaling_int_constr, _dim_ic);
    c_nlp_constr_mult_input += _dim_ic;
    c_nlp_constr_mult_output += _dim_ic;

    //now scale the multipliers accordingly to the scale of the target
    if (inv_scaling_target != 1) {
        for (integer i=0; i<getNlpSize(); i++)
            nlp_y_upper_bound_mult_output[i] = nlp_y_upper_bound_mult_output[i] * inv_scaling_target;

        for (integer i=0; i<getNlpSize(); i++)
            nlp_y_lower_bound_mult_output[i] = nlp_y_lower_bound_mult_output[i] * inv_scaling_target;

        for (integer i=0; i<getNlpConstraintsSize(); i++)
            nlp_constraints_mult_output[i] = nlp_constraints_mult_output[i] * inv_scaling_target;
    }


    MAVERICK_DEBUG_ASSERT( c_nlp_y_output == nlp_y_output + getNlpSize(), "MidpointOcp2NlpSinglePhase::scaleNlp: wrong final y pointer.\n")

    MAVERICK_DEBUG_ASSERT( c_nlp_z_lower_output == nlp_y_lower_bound_mult_output + getNlpSize(), "MidpointOcp2NlpSinglePhase::scaleNlp: wrong final lower bounds pointer.\n")

    MAVERICK_DEBUG_ASSERT( c_nlp_z_upper_output == nlp_y_upper_bound_mult_output + getNlpSize(), "MidpointOcp2NlpSinglePhase::scaleNlp: wrong final upper bounds pointer.\n")

    MAVERICK_DEBUG_ASSERT( c_nlp_constr_mult_output == nlp_constraints_mult_output + getNlpConstraintsSize(), "MidpointOcp2NlpSinglePhase::scaleNlpMultipliers: wrong final constraints pointer.\n")

    Nlp & nlp_output = nlp;

    nlp_output.setNlp(getNlpSize(), nlp_y_output, nlp_y_upper_bound_mult_output, nlp_y_lower_bound_mult_output,
                      getNlpConstraintsSize(), nlp_constraints_output, nlp_constraints_mult_output);
    //                      lagrange_mayer_percentage);
    delete [] nlp_y_output;
    delete [] nlp_y_upper_bound_mult_output;
    delete [] nlp_y_lower_bound_mult_output;
    delete [] nlp_constraints_output;
    delete [] nlp_constraints_mult_output;

}

// translates the multiplers of the NLP to OCP ones. The Y and CONSTRAINTS data of the output nlp is copied form the input one
// if the inverse flag is true, then the inverse scaling is performed (i.e. ocp to nlp)
// the two nlp (input and output) can also be the same nlp
void MidpointOcp2NlpSinglePhase::convertNlp2OcpMultipliers(Nlp & nlp,
                                                      bool const inverse) const
{
    Nlp const & nlp_input = nlp;

    MAVERICK_DEBUG_ASSERT( nlp_input.getNlpSize() ==  getNlpSize(), "MidpointOcp2NlpSinglePhase::convertNlpMultipliers: wrong nlp size.\n")
    MAVERICK_DEBUG_ASSERT( nlp_input.getNlpConstraintsSize() ==  getNlpConstraintsSize(), "MidpointOcp2NlpSinglePhase::convertNlpMultipliers: wrong nlp constraints size.\n")

    // create working variables
    real * nlp_y_upper_bound_mult_output = new real[getNlpSize()];
    real * nlp_y_lower_bound_mult_output = new real[getNlpSize()];
    real * nlp_constraints_mult_output   = new real[getNlpConstraintsSize()];

    real const * c_nlp_z_upper_mult_input          = nlp_input.getUpperBoundsMultiplier().data();
    real       * c_nlp_z_upper_mult_output         = nlp_y_upper_bound_mult_output;
    real const * c_nlp_z_lower_mult_input          = nlp_input.getLowerBoundsMultiplier().data();
    real       * c_nlp_z_lower_mult_output         = nlp_y_lower_bound_mult_output;
    real const * c_nlp_constr_mult_input           = nlp_input.getConstraintsMultipliers().data();
    real       * c_nlp_constr_mult_output          = nlp_constraints_mult_output;

    //run over all the mesh points and scale (or UNSCALE) the nlp data
    integer const num_mesh_intervals = _p_mesh->getNumberOfIntervals();
    for (integer c_mesh_point = 0; c_mesh_point < num_mesh_intervals; c_mesh_point++ ) {

        real c_dz = _p_mesh->getDz(c_mesh_point);

        //real delta = _p_mesh->getDz(c_mesh_point); // mesh interval DUAL amplitude

        if (inverse) {  // in this case multipliers must be scaled inversely, so we take the inverse of c_dz
            c_dz = 1 / c_dz;
        //    delta = 1 / delta;
        }

        real const delta_inv = 1 / c_dz;

        // scale states
        multiplyAndCopyVectorTo(c_nlp_z_upper_mult_input, c_nlp_z_upper_mult_output, delta_inv, _dim_y);
        c_nlp_z_upper_mult_input += _dim_y;
        c_nlp_z_upper_mult_output += _dim_y;

        multiplyAndCopyVectorTo(c_nlp_z_lower_mult_input, c_nlp_z_lower_mult_output, delta_inv, _dim_y);
        c_nlp_z_lower_mult_input += _dim_y;
        c_nlp_z_lower_mult_output += _dim_y;

        // scale algebraic states
        multiplyAndCopyVectorTo(c_nlp_z_upper_mult_input, c_nlp_z_upper_mult_output, delta_inv, _dim_ay);
        c_nlp_z_upper_mult_input += _dim_ay;
        c_nlp_z_upper_mult_output += _dim_ay;

        multiplyAndCopyVectorTo(c_nlp_z_lower_mult_input, c_nlp_z_lower_mult_output, delta_inv, _dim_ay);
        c_nlp_z_lower_mult_input += _dim_ay;
        c_nlp_z_lower_mult_output += _dim_ay;

        // fo eqns
        multiplyAndCopyVectorTo(c_nlp_constr_mult_input, c_nlp_constr_mult_output, delta_inv, _dim_fo);
        c_nlp_constr_mult_input += _dim_fo;
        c_nlp_constr_mult_output += _dim_fo;

        //path constr
        multiplyAndCopyVectorTo(c_nlp_constr_mult_input, c_nlp_constr_mult_output, delta_inv, _dim_pc);
        c_nlp_constr_mult_input += _dim_pc;
        c_nlp_constr_mult_output += _dim_pc;

        //point constraints
        multiplyAndCopyVectorTo(c_nlp_constr_mult_input, c_nlp_constr_mult_output, delta_inv, _dim_poc);
        c_nlp_constr_mult_input += _dim_poc;
        c_nlp_constr_mult_output += _dim_poc;
    }

    real delta = _p_mesh->getDz(num_mesh_intervals-1); // dual amplitude
    if (inverse)  // in this case multipliers must be scaled inversely, so we take the inverse of c_dz
        delta = 1 / delta;
    real const delta_inv = 1 / delta;

    // last states
    multiplyAndCopyVectorTo(c_nlp_z_upper_mult_input, c_nlp_z_upper_mult_output, delta_inv, _dim_y);
    c_nlp_z_upper_mult_input += _dim_y;
    c_nlp_z_upper_mult_output += _dim_y;

    multiplyAndCopyVectorTo(c_nlp_z_lower_mult_input, c_nlp_z_lower_mult_output, delta_inv, _dim_y);
    c_nlp_z_lower_mult_input += _dim_y;
    c_nlp_z_lower_mult_output += _dim_y;

    //last point constraints
    multiplyAndCopyVectorTo(c_nlp_constr_mult_input, c_nlp_constr_mult_output, delta_inv, _dim_poc);
    c_nlp_constr_mult_input += _dim_poc;
    c_nlp_constr_mult_output += _dim_poc;

    // parameters
    copyVectorTo(c_nlp_z_upper_mult_input, c_nlp_z_upper_mult_output, _dim_p);
    c_nlp_z_upper_mult_input += _dim_p;
    c_nlp_z_upper_mult_output += _dim_p;

    copyVectorTo(c_nlp_z_lower_mult_input, c_nlp_z_lower_mult_output, _dim_p);
    c_nlp_z_lower_mult_input += _dim_p;
    c_nlp_z_lower_mult_output += _dim_p;

    // boundary conditions
    copyVectorTo(c_nlp_constr_mult_input, c_nlp_constr_mult_output, _dim_bc);
    c_nlp_constr_mult_input += _dim_bc;
    c_nlp_constr_mult_output += _dim_bc;

    // integral constraints
    copyVectorTo(c_nlp_constr_mult_input, c_nlp_constr_mult_output, _dim_ic);
    c_nlp_constr_mult_input += _dim_ic;
    c_nlp_constr_mult_output += _dim_ic;

    MAVERICK_DEBUG_ASSERT( c_nlp_z_lower_mult_output == nlp_y_lower_bound_mult_output + getNlpSize(), "MidpointOcp2NlpSinglePhase::convertNlp2OcpMultipliers: wrong final lower bounds pointer.\n")

    MAVERICK_DEBUG_ASSERT( c_nlp_z_upper_mult_output == nlp_y_upper_bound_mult_output + getNlpSize(), "MidpointOcp2NlpSinglePhase::convertNlp2OcpMultipliers: wrong final upper bounds pointer.\n")

    MAVERICK_DEBUG_ASSERT( c_nlp_constr_mult_output == nlp_constraints_mult_output + getNlpConstraintsSize(), "MidpointOcp2NlpSinglePhase::convertNlp2OcpMultipliers: wrong final constraints pointer.\n")

    Nlp & nlp_output = nlp;
    nlp_output.setNlp(getNlpSize(), nlp_input.getY().data(), nlp_y_upper_bound_mult_output, nlp_y_lower_bound_mult_output,
                      getNlpConstraintsSize(), nlp_input.getConstraints().data(), nlp_constraints_mult_output
//                      nlp_input.getLagrangeMayerPercentage()
                      );

    delete [] nlp_y_upper_bound_mult_output;
    delete [] nlp_y_lower_bound_mult_output;
    delete [] nlp_constraints_mult_output;

}
