#include "MidpointOcpSolutionSinglePhase.hh"
#include "MaverickCore/MaverickPrivateDefs.hh"
#include "MaverickCore/MaverickFunctions.hh"
#include "MaverickCore/MaverickSingleton.hh"
#include "MaverickUtils/GenericFunction/GF1ASpline.hh"
#include "MaverickUtils/GenericFunction/GF1APolyFive.hh"
#include <iomanip>

#define GF1A_TYPE MaverickUtils::GF1ASpline

#define SPLINE_TYPE "Akima"
//#define SPLINE_TYPE "Linear"

//#define SPLINE_CHECK_RANGE true
#define SPLINE_CHECK_RANGE false

#define SPLINE_EXTEND_RANGE MaverickUtils::GF1ASpline::ExtendRange::keep_derivative

using namespace std;

namespace Maverick {
    
    MidpointOcpSolutionSinglePhase::MidpointOcpSolutionSinglePhase() {}

    MidpointOcpSolutionSinglePhase::MidpointOcpSolutionSinglePhase( MidpointOcpSolutionSinglePhase const & ocp_solution ) {
        setSolution(ocp_solution.getTarget(),
                    ocp_solution.getDiscretisationPoints(),
                    ocp_solution.getCumulativeTarget(),
                    ocp_solution.getIntegrandTarget(),
                    ocp_solution.getStatesControls(),
                    ocp_solution.getAlgebraicStatesControls(),
                    ocp_solution.getParameters(),
                    ocp_solution.getPointConstraints(),
                    ocp_solution.getPathConstraints(),
                    ocp_solution.getIntegralConstraints(),
                    ocp_solution.getFoEquations(),
                    ocp_solution.getBoundaryConditions(),
                    ocp_solution.getPostProcessing(),
                    ocp_solution.getDifferentialPostProcessing(),
                    ocp_solution.getIntegralPostProcessing(),
                    ocp_solution.getStatesControlsUpperBoundsMultipliers(),
                    ocp_solution.getStatesControlsLowerBoundsMultipliers(),
                    ocp_solution.getAlgebraicStatesControlsUpperBoundsMultipliers(),
                    ocp_solution.getAlgebraicStatesControlsLowerBoundsMultipliers(),
                    ocp_solution.getParametersUpperBoundsMultipliers(),
                    ocp_solution.getParametersLowerBoundsMultipliers(),
                    ocp_solution.getPointConstraintsMultipliers(),
                    ocp_solution.getPathConstraintsMultipliers(),
                    ocp_solution.getIntConstraintsMultipliers(),
                    ocp_solution.getFoEqnsMultipliers(),
                    ocp_solution.getBoundaryConditionsMultipliers() );
    }

    MidpointOcpSolutionSinglePhase& MidpointOcpSolutionSinglePhase::operator=(const MidpointOcpSolutionSinglePhase & ocp_solution) {
        setSolution(ocp_solution.getTarget(),
                    ocp_solution.getDiscretisationPoints(),
                    ocp_solution.getCumulativeTarget(),
                    ocp_solution.getIntegrandTarget(),
                    ocp_solution.getStatesControls(),
                    ocp_solution.getAlgebraicStatesControls(),
                    ocp_solution.getParameters(),
                    ocp_solution.getPointConstraints(),
                    ocp_solution.getPathConstraints(),
                    ocp_solution.getIntegralConstraints(),
                    ocp_solution.getFoEquations(),
                    ocp_solution.getBoundaryConditions(),
                    ocp_solution.getPostProcessing(),
                    ocp_solution.getDifferentialPostProcessing(),
                    ocp_solution.getIntegralPostProcessing(),
                    ocp_solution.getStatesControlsUpperBoundsMultipliers(),
                    ocp_solution.getStatesControlsLowerBoundsMultipliers(),
                    ocp_solution.getAlgebraicStatesControlsUpperBoundsMultipliers(),
                    ocp_solution.getAlgebraicStatesControlsLowerBoundsMultipliers(),
                    ocp_solution.getParametersUpperBoundsMultipliers(),
                    ocp_solution.getParametersLowerBoundsMultipliers(),
                    ocp_solution.getPointConstraintsMultipliers(),
                    ocp_solution.getPathConstraintsMultipliers(),
                    ocp_solution.getIntConstraintsMultipliers(),
                    ocp_solution.getFoEqnsMultipliers(),
                    ocp_solution.getBoundaryConditionsMultipliers() );
        return *this;
    }


    MidpointOcpSolutionSinglePhase::~MidpointOcpSolutionSinglePhase() {
        clearPointers();
    }

    void MidpointOcpSolutionSinglePhase::clearPointers() {
        if ( _gf1a_cumulative_target != nullptr )
            delete _gf1a_cumulative_target;
        _gf1a_cumulative_target = nullptr;

        if ( _gf1a_integrand_target != nullptr )
            delete _gf1a_integrand_target;
        _gf1a_integrand_target = nullptr;

        for (size i=0; i<_gf1a_states_controls.size(); i++) {
            MaverickUtils::GenericFunction1AInterface * tmp_gf1a = _gf1a_states_controls[i];
            if ( tmp_gf1a!= nullptr )
                delete tmp_gf1a;
            _gf1a_states_controls[i] = nullptr;
        }
        for (size i=0; i<_gf1a_states_controls_upper_bounds_multipliers.size(); i++) {
            MaverickUtils::GenericFunction1AInterface * tmp_gf1a = _gf1a_states_controls_upper_bounds_multipliers[i];
            if ( tmp_gf1a!= nullptr )
                delete tmp_gf1a;
            _gf1a_states_controls_upper_bounds_multipliers[i] = nullptr;
        }
        for (size i=0; i<_gf1a_states_controls_lower_bounds_multipliers.size(); i++) {
            MaverickUtils::GenericFunction1AInterface * tmp_gf1a = _gf1a_states_controls_lower_bounds_multipliers[i];
            if ( tmp_gf1a!= nullptr )
                delete tmp_gf1a;
            _gf1a_states_controls_lower_bounds_multipliers[i] = nullptr;
        }
        for (size i=0; i<_gf1a_algebraic_states_controls.size(); i++) {
            MaverickUtils::GenericFunction1AInterface * tmp_gf1a = _gf1a_algebraic_states_controls[i];
            if ( tmp_gf1a!= nullptr )
                delete tmp_gf1a;
            _gf1a_algebraic_states_controls[i] = nullptr;
        }
        for (size i=0; i<_gf1a_algebraic_states_controls_upper_bounds_multipliers.size(); i++) {
            MaverickUtils::GenericFunction1AInterface * tmp_gf1a = _gf1a_algebraic_states_controls_upper_bounds_multipliers[i];
            if ( tmp_gf1a!= nullptr )
                delete tmp_gf1a;
            _gf1a_algebraic_states_controls_upper_bounds_multipliers[i] = nullptr;
        }
        for (size i=0; i<_gf1a_algebraic_states_controls_lower_bounds_multipliers.size(); i++) {
            MaverickUtils::GenericFunction1AInterface * tmp_gf1a = _gf1a_algebraic_states_controls_lower_bounds_multipliers[i];
            if ( tmp_gf1a!= nullptr )
                delete tmp_gf1a;
            _gf1a_algebraic_states_controls_lower_bounds_multipliers[i] = nullptr;
        }
        for (size i=0; i<_gf1a_point_constraints.size(); i++) {
            MaverickUtils::GenericFunction1AInterface * tmp_gf1a = _gf1a_point_constraints[i];
            if ( tmp_gf1a!= nullptr )
                delete tmp_gf1a;
            _gf1a_point_constraints[i] = nullptr;
        }
        for (size i=0; i<_gf1a_point_constraints_multipliers.size(); i++) {
            MaverickUtils::GenericFunction1AInterface * tmp_gf1a = _gf1a_point_constraints_multipliers[i];
            if ( tmp_gf1a!= nullptr )
                delete tmp_gf1a;
            _gf1a_point_constraints_multipliers[i] = nullptr;
        }
        for (size i=0; i<_gf1a_path_constr.size(); i++) {
            MaverickUtils::GenericFunction1AInterface * tmp_gf1a = _gf1a_path_constr[i];
            if ( tmp_gf1a!= nullptr )
                delete tmp_gf1a;
            _gf1a_path_constr[i] = nullptr;
        }
        for (size i=0; i<_gf1a_path_constr_multipliers.size(); i++) {
            MaverickUtils::GenericFunction1AInterface * tmp_gf1a = _gf1a_path_constr_multipliers[i];
            if ( tmp_gf1a!= nullptr )
                delete tmp_gf1a;
            _gf1a_path_constr_multipliers[i] = nullptr;
        }
        for (size i=0; i<_gf1a_int_constr.size(); i++) {
            MaverickUtils::GenericFunction1AInterface * tmp_gf1a = _gf1a_int_constr[i];
            if ( tmp_gf1a!= nullptr )
                delete tmp_gf1a;
            _gf1a_int_constr[i] = nullptr;
        }
        for (size i=0; i<_gf1a_fo_eqns.size(); i++) {
            MaverickUtils::GenericFunction1AInterface * tmp_gf1a = _gf1a_fo_eqns[i];
            if ( tmp_gf1a!= nullptr )
                delete tmp_gf1a;
            _gf1a_fo_eqns[i] = nullptr;
        }
        for (size i=0; i<_gf1a_fo_eqns_multipliers.size(); i++) {
            MaverickUtils::GenericFunction1AInterface * tmp_gf1a = _gf1a_fo_eqns_multipliers[i];
            if ( tmp_gf1a!= nullptr )
                delete tmp_gf1a;
            _gf1a_fo_eqns_multipliers[i] = nullptr;
        }
        for (size i=0; i<_gf1a_post_processing.size(); i++) {
            MaverickUtils::GenericFunction1AInterface * tmp_gf1a = _gf1a_post_processing[i];
            if ( tmp_gf1a!= nullptr )
                delete tmp_gf1a;
            _gf1a_post_processing[i] = nullptr;
        }
        for (size i=0; i<_gf1a_differential_post_processing.size(); i++) {
            MaverickUtils::GenericFunction1AInterface * tmp_gf1a = _gf1a_differential_post_processing[i];
            if ( tmp_gf1a!= nullptr )
                delete tmp_gf1a;
            _gf1a_differential_post_processing[i] = nullptr;
        }
        for (size i=0; i<_gf1a_integral_post_processing.size(); i++) {
            MaverickUtils::GenericFunction1AInterface * tmp_gf1a = _gf1a_integral_post_processing[i];
            if ( tmp_gf1a!= nullptr )
                delete tmp_gf1a;
            _gf1a_integral_post_processing[i] = nullptr;
        }
    }

    // get a copy
    unique_ptr<OcpSolutionSinglePhase> MidpointOcpSolutionSinglePhase::copy() const {
        return std::unique_ptr<OcpSolutionSinglePhase>( new MidpointOcpSolutionSinglePhase(*this) );
    }

    //setter

    void MidpointOcpSolutionSinglePhase::setSolution(real        const   target,
                                             vec_1d_real const & zeta,
                                             vec_1d_real const & cumulative_target,
                                             vec_1d_real const & integrand_target,
                                             vec_2d_real const & states_controls,
                                             vec_2d_real const & algebraic_states_controls,
                                             vec_1d_real const & params,
                                             vec_2d_real const & point_constr,
                                             vec_2d_real const & path_constr,
                                             vec_2d_real const & int_constr,
                                             vec_2d_real const & fo_eqns,
                                             vec_1d_real const & boundary_conditions,
                                             vec_2d_real const & post_processing,
                                             vec_2d_real const & differential_post_processing,
                                             vec_2d_real const & integral_post_processing) {

        vec_1d_real vec_1d_points_zeros(zeta.size(), 0);
        vec_1d_real vec_1d_intervals_zeros(zeta.size()-1, 0);
        vec_2d_real states_controls_bounds_mult_zeros(states_controls.size(), vec_1d_points_zeros);
        vec_2d_real algebraic_states_controls_bounds_mult_zeros(algebraic_states_controls.size(), vec_1d_intervals_zeros);
        vec_1d_real params_bounds_mult_zeros(params.size(), 0);
        vec_2d_real point_constr_mult_zeros(point_constr.size(), vec_1d_points_zeros);
        vec_2d_real path_constr_mult_zeros(path_constr.size(), vec_1d_intervals_zeros);
        vec_1d_real int_constr_mult_zeros(int_constr.size() , 0);
        vec_2d_real fo_eqns_mult_zeros(fo_eqns.size(), vec_1d_intervals_zeros);
        vec_1d_real bcs_mult_zero = vec_1d_real(boundary_conditions.size(), 0);

        setSolution(target,
                    zeta,
                    cumulative_target,
                    integrand_target,
                    states_controls,
                    algebraic_states_controls,
                    params,
                    point_constr,
                    path_constr,
                    int_constr,
                    fo_eqns,
                    boundary_conditions,
                    post_processing,
                    differential_post_processing,
                    integral_post_processing,

                    states_controls_bounds_mult_zeros,
                    states_controls_bounds_mult_zeros,
                    algebraic_states_controls_bounds_mult_zeros,
                    algebraic_states_controls_bounds_mult_zeros,
                    params_bounds_mult_zeros,
                    params_bounds_mult_zeros,
                    point_constr_mult_zeros,
                    path_constr_mult_zeros,
                    int_constr_mult_zeros,
                    fo_eqns_mult_zeros,
                    bcs_mult_zero
                    );
    }

    void MidpointOcpSolutionSinglePhase::setSolution(real        const   target,
                                             vec_1d_real const & zeta,
                                             vec_1d_real const & cumulative_target,
                                             vec_1d_real const & integrand_target,
                                             vec_2d_real const & states_controls,
                                             vec_2d_real const & algebraic_states_controls,
                                             vec_1d_real const & params,
                                             vec_2d_real const & point_constr,
                                             vec_2d_real const & path_constr,
                                             vec_2d_real const & int_constr,
                                             vec_2d_real const & fo_eqns,
                                             vec_1d_real const & boundary_conditions,
                                             vec_2d_real const & post_processing,
                                             vec_2d_real const & differential_post_processing,
                                             vec_2d_real const & integral_post_processing,
                                             vec_2d_real const & states_controls_upper_bounds_multipliers,
                                             vec_2d_real const & states_controls_lower_bounds_multipliers,
                                             vec_2d_real const & algebraic_states_controls_upper_bounds_multipliers,
                                             vec_2d_real const & algebraic_states_controls_lower_bounds_multipliers,
                                             vec_1d_real const & parameters_upper_bounds_multipliers,
                                             vec_1d_real const & parameters_lower_bounds_multipliers,
                                             vec_2d_real const & point_constraints_multipliers,
                                             vec_2d_real const & path_constr_multipliers,
                                             vec_1d_real const & int_constr_multipliers,
                                             vec_2d_real const & fo_eqns_multipliers,
                                             vec_1d_real const & bcs_multipliers ) {

        _target = target;
        _zeta = zeta;
        _cumulative_target = cumulative_target;
        _integrand_target = integrand_target;
        _states_controls = states_controls;
        _algebraic_states_controls = algebraic_states_controls;
        _parameters = params;
        _point_constraints = point_constr;
        _path_constr = path_constr;
        _int_constr = int_constr;
        _fo_eqns = fo_eqns;
        _boundary_conditions = boundary_conditions;
        _post_processing = post_processing;
        _differential_post_processing = differential_post_processing;
        _integral_post_processing = integral_post_processing;

        _states_controls_upper_bounds_multipliers = states_controls_upper_bounds_multipliers;
        _states_controls_lower_bounds_multipliers = states_controls_lower_bounds_multipliers;
        _algebraic_states_controls_upper_bounds_multipliers = algebraic_states_controls_upper_bounds_multipliers;
        _algebraic_states_controls_lower_bounds_multipliers = algebraic_states_controls_lower_bounds_multipliers;
        _parameters_upper_bounds_multipliers = parameters_upper_bounds_multipliers;
        _parameters_lower_bounds_multipliers = parameters_lower_bounds_multipliers;
        _point_constraints_multipliers = point_constraints_multipliers;
        _path_constr_multipliers = path_constr_multipliers;
        _int_constr_multipliers = int_constr_multipliers;
        _fo_eqns_multipliers = fo_eqns_multipliers;
        _bcs_multipliers = bcs_multipliers;

        if ( !areVectorsConsistent() ) {
            MaverickSingleton::getInstance().Log( InfoLevel::info_level_warning, "MidpointOcpSolutionSinglePhase::setSolution: solution has been set with non consistent vectors length. Solution will be cleared.\n");
            clear();
            return;
        }

        buildGf1a();
    }

    bool MidpointOcpSolutionSinglePhase::areVectorsConsistent() const {

        if ( ( _states_controls_upper_bounds_multipliers.size() != _states_controls.size() ) ||
            ( _states_controls_lower_bounds_multipliers.size() != _states_controls.size() ) ||
            ( _algebraic_states_controls_upper_bounds_multipliers.size() != _algebraic_states_controls.size() ) ||
            ( _algebraic_states_controls_lower_bounds_multipliers.size() != _algebraic_states_controls.size() ) ||
            ( _parameters_upper_bounds_multipliers.size() != _parameters.size() )           ||
            ( _parameters_lower_bounds_multipliers.size() != _parameters.size() )           ||
            ( _point_constraints_multipliers.size() != _point_constraints.size() )          ||
            ( _path_constr_multipliers.size() != _path_constr.size() )                      ||
            ( _int_constr_multipliers.size() != _int_constr.size() )                        ||
            ( _fo_eqns_multipliers.size() != _fo_eqns.size() )                              ||
            ( _bcs_multipliers.size() != _boundary_conditions.size() )
            )
            return false;

        size_t point_size = _zeta.size();
        size_t interval_size = point_size - 1;
        if (point_size == 0)
            interval_size = 0;

        if (point_size - _cumulative_target.size() != 0) return false;
        if (interval_size - _integrand_target.size() != 0) return false;

        for (integer i=0; i< _states_controls.size(); i++)
            if ( point_size - _states_controls[i].size() != 0 ) return false;

        for (integer i=0; i< _algebraic_states_controls.size(); i++)
            if ( interval_size - _algebraic_states_controls[i].size() != 0 ) return false;

        for (integer i=0; i< _point_constraints.size(); i++)
            if ( point_size - _point_constraints[i].size() != 0 ) return false;

        for (integer i=0; i< _path_constr.size(); i++)
            if ( interval_size - _path_constr[i].size() != 0 ) return false;

        for (integer i=0; i< _int_constr.size(); i++)
            if ( point_size - _int_constr[i].size() != 0 ) return false;

        for (integer i=0; i< _fo_eqns.size(); i++)
            if ( interval_size - _fo_eqns[i].size() != 0 ) return false;

        for (integer i=0; i< _post_processing.size(); i++)
            if ( point_size - _post_processing[i].size() != 0 ) return false;

        for (integer i=0; i< _differential_post_processing.size(); i++)
            if ( interval_size - _differential_post_processing[i].size() != 0 ) return false;

        for (integer i=0; i< _integral_post_processing.size(); i++)
            if ( point_size - _integral_post_processing[i].size() != 0 ) return false;

        for (size i=0; i< _states_controls_upper_bounds_multipliers.size(); i++)
            if ( point_size - _states_controls_upper_bounds_multipliers[i].size() != 0 ) return false;

        for (size i=0; i< _states_controls_lower_bounds_multipliers.size(); i++)
            if ( point_size - _states_controls_lower_bounds_multipliers[i].size() != 0 ) return false;

        for (size i=0; i< _algebraic_states_controls_upper_bounds_multipliers.size(); i++)
            if ( interval_size - _algebraic_states_controls_upper_bounds_multipliers[i].size() != 0 ) return false;

        for (size i=0; i< _algebraic_states_controls_lower_bounds_multipliers.size(); i++)
            if ( interval_size - _algebraic_states_controls_lower_bounds_multipliers[i].size() != 0 ) return false;

        for (size i=0; i< _point_constraints_multipliers.size(); i++)
            if ( point_size - _point_constraints_multipliers[i].size() != 0 ) return false;

        for (size i=0; i< _path_constr_multipliers.size(); i++)
            if ( interval_size - _path_constr_multipliers[i].size() != 0 ) return false;

        for (size i=0; i< _fo_eqns_multipliers.size(); i++)
            if ( interval_size - _fo_eqns_multipliers[i].size() != 0 ) return false;

        return true;
    }

    void MidpointOcpSolutionSinglePhase::buildGf1a() {
        clearPointers();

        if (_zeta.size() == 0) { // in this case setup a null (zero) solution
            GC::GenericContainer gc;
            for (size i=0; i<_states_controls.size(); i++) {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                _gf1a_states_controls.push_back( tmp_gf1a );
            }

            for (size i=0; i<_states_controls_upper_bounds_multipliers.size(); i++) {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                _gf1a_states_controls_upper_bounds_multipliers.push_back( tmp_gf1a );
            }
            for (size i=0; i<_states_controls_lower_bounds_multipliers.size(); i++) {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                _gf1a_states_controls_lower_bounds_multipliers.push_back( tmp_gf1a );
            }

            for (size i=0; i<_algebraic_states_controls.size(); i++) {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                _gf1a_algebraic_states_controls.push_back( tmp_gf1a );
            }

            for (size i=0; i<_algebraic_states_controls_upper_bounds_multipliers.size(); i++) {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                _gf1a_algebraic_states_controls_upper_bounds_multipliers.push_back( tmp_gf1a );
            }
            for (size i=0; i<_algebraic_states_controls_lower_bounds_multipliers.size(); i++) {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                _gf1a_algebraic_states_controls_lower_bounds_multipliers.push_back( tmp_gf1a );
            }

            for (size i=0; i<_point_constraints.size(); i++) {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                _gf1a_point_constraints.push_back( tmp_gf1a );
            }

            for (size i=0; i<_point_constraints_multipliers.size(); i++) {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                _gf1a_point_constraints_multipliers.push_back( tmp_gf1a );
            }

            for (size i=0; i<_path_constr.size(); i++) {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                _gf1a_path_constr.push_back( tmp_gf1a );
            }

            for (size i=0; i<_path_constr_multipliers.size(); i++) {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                _gf1a_path_constr_multipliers.push_back( tmp_gf1a );
            }

            for (size i=0; i<_int_constr.size(); i++) {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                _gf1a_int_constr.push_back( tmp_gf1a );
            }

            for (size i=0; i<_fo_eqns.size(); i++) {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                _gf1a_fo_eqns.push_back( tmp_gf1a );
            }

            for (size i=0; i<_fo_eqns_multipliers.size(); i++) {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                _gf1a_fo_eqns_multipliers.push_back( tmp_gf1a );
            }

            for (size i=0; i<_post_processing.size(); i++) {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                _gf1a_post_processing.push_back( tmp_gf1a );
            }

            for (size i=0; i<_differential_post_processing.size(); i++) {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                _gf1a_differential_post_processing.push_back( tmp_gf1a );
            }

            for (size i=0; i<_integral_post_processing.size(); i++) {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                _gf1a_integral_post_processing.push_back( tmp_gf1a );
            }

            {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                _gf1a_integrand_target = tmp_gf1a ;
            }

            {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                _gf1a_cumulative_target = tmp_gf1a ;
            }
        } else if (_zeta.size() == 1) {
            // in this case setup a constant solution
            GC::GenericContainer gc;
            for (size i=0; i<_states_controls.size(); i++) {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                tmp_gf1a->setA0(_states_controls[i][0]);
                _gf1a_states_controls.push_back( tmp_gf1a );
            }

            for (size i=0; i<_states_controls_upper_bounds_multipliers.size(); i++) {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                tmp_gf1a->setA0(_states_controls_upper_bounds_multipliers[i][0]);
                _gf1a_states_controls_upper_bounds_multipliers.push_back( tmp_gf1a );
            }
            for (size i=0; i<_states_controls_lower_bounds_multipliers.size(); i++) {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                tmp_gf1a->setA0(_states_controls_lower_bounds_multipliers[i][0]);
                _gf1a_states_controls_lower_bounds_multipliers.push_back( tmp_gf1a );
            }

            for (size i=0; i<_algebraic_states_controls.size(); i++) {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                tmp_gf1a->setA0(_states_controls[i][0]);
                _gf1a_algebraic_states_controls.push_back( tmp_gf1a );
            }

            for (size i=0; i<_algebraic_states_controls_upper_bounds_multipliers.size(); i++) {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                tmp_gf1a->setA0(_algebraic_states_controls_upper_bounds_multipliers[i][0]);
                _gf1a_algebraic_states_controls_upper_bounds_multipliers.push_back( tmp_gf1a );
            }
            for (size i=0; i<_algebraic_states_controls_lower_bounds_multipliers.size(); i++) {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                tmp_gf1a->setA0(_algebraic_states_controls_lower_bounds_multipliers[i][0]);
                _gf1a_algebraic_states_controls_lower_bounds_multipliers.push_back( tmp_gf1a );
            }

            for (size i=0; i<_point_constraints.size(); i++) {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                tmp_gf1a->setA0(_point_constraints[i][0]);
                _gf1a_point_constraints.push_back( tmp_gf1a );
            }

            for (size i=0; i<_point_constraints_multipliers.size(); i++) {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                tmp_gf1a->setA0(_point_constraints_multipliers[i][0]);
                _gf1a_point_constraints_multipliers.push_back( tmp_gf1a );
            }

            for (size i=0; i<_path_constr.size(); i++) {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                tmp_gf1a->setA0(_path_constr[i][0]);
                _gf1a_path_constr.push_back( tmp_gf1a );
            }

            for (size i=0; i<_path_constr_multipliers.size(); i++) {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                tmp_gf1a->setA0(_path_constr_multipliers[i][0]);
                _gf1a_path_constr_multipliers.push_back( tmp_gf1a );
            }

            for (size i=0; i<_int_constr.size(); i++) {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                tmp_gf1a->setA0(_int_constr[i][0]);
                _gf1a_int_constr.push_back( tmp_gf1a );
            }

            for (size i=0; i<_fo_eqns.size(); i++) {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                tmp_gf1a->setA0(_fo_eqns_multipliers[i][0]);
                _gf1a_fo_eqns.push_back( tmp_gf1a );
            }

            for (size i=0; i<_fo_eqns_multipliers.size(); i++) {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                tmp_gf1a->setA0(_fo_eqns_multipliers[i][0]);
                _gf1a_fo_eqns_multipliers.push_back( tmp_gf1a );
            }

            for (size i=0; i<_post_processing.size(); i++) {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                tmp_gf1a->setA0(_post_processing[i][0]);
                _gf1a_post_processing.push_back( tmp_gf1a );
            }

            for (size i=0; i<_differential_post_processing.size(); i++) {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                tmp_gf1a->setA0(_differential_post_processing[i][0]);
                _gf1a_differential_post_processing.push_back( tmp_gf1a );
            }

            for (size i=0; i<_integral_post_processing.size(); i++) {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                tmp_gf1a->setA0(_integral_post_processing[i][0]);
                _gf1a_integral_post_processing.push_back( tmp_gf1a );
            }

            {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                tmp_gf1a->setA0(_integrand_target[0]);
                _gf1a_integrand_target = tmp_gf1a ;
            }

            {
                MaverickUtils::GF1APolyFive * tmp_gf1a = new MaverickUtils::GF1APolyFive();
                tmp_gf1a->setup(gc);
                tmp_gf1a->setA0(_cumulative_target[0]);
                _gf1a_cumulative_target = tmp_gf1a ;
            }
        } else { // in this case build the splines
            vec_1d_real zeta_midpoint = extractMidpoints(_zeta);

            for (size i=0; i<_states_controls.size(); i++) {
                GF1A_TYPE * tmp_gf1a = new GF1A_TYPE();
                tmp_gf1a->setCheckRange(SPLINE_CHECK_RANGE);
                tmp_gf1a->setup(SPLINE_TYPE, _zeta, _states_controls[i], SPLINE_EXTEND_RANGE);
                _gf1a_states_controls.push_back( tmp_gf1a );
            }

            for (size i=0; i<_states_controls_upper_bounds_multipliers.size(); i++) {
                GF1A_TYPE * tmp_gf1a = new GF1A_TYPE();
                tmp_gf1a->setCheckRange(SPLINE_CHECK_RANGE);
                tmp_gf1a->setup(SPLINE_TYPE, _zeta, _states_controls_upper_bounds_multipliers[i], SPLINE_EXTEND_RANGE);
                _gf1a_states_controls_upper_bounds_multipliers.push_back( tmp_gf1a );
            }
            for (size i=0; i<_states_controls_lower_bounds_multipliers.size(); i++) {
                GF1A_TYPE * tmp_gf1a = new GF1A_TYPE();
                tmp_gf1a->setCheckRange(SPLINE_CHECK_RANGE);
                tmp_gf1a->setup(SPLINE_TYPE, _zeta, _states_controls_lower_bounds_multipliers[i], SPLINE_EXTEND_RANGE);
                _gf1a_states_controls_lower_bounds_multipliers.push_back( tmp_gf1a );
            }

            for (size i=0; i<_algebraic_states_controls.size(); i++) {
                GF1A_TYPE * tmp_gf1a = new GF1A_TYPE();
                tmp_gf1a->setCheckRange(SPLINE_CHECK_RANGE);
                tmp_gf1a->setup(SPLINE_TYPE, zeta_midpoint, _algebraic_states_controls[i], SPLINE_EXTEND_RANGE);
                _gf1a_algebraic_states_controls.push_back( tmp_gf1a );
            }

            for (size i=0; i<_algebraic_states_controls_upper_bounds_multipliers.size(); i++) {
                GF1A_TYPE * tmp_gf1a = new GF1A_TYPE();
                tmp_gf1a->setCheckRange(SPLINE_CHECK_RANGE);
                tmp_gf1a->setup(SPLINE_TYPE, zeta_midpoint, _algebraic_states_controls_upper_bounds_multipliers[i], SPLINE_EXTEND_RANGE);
                _gf1a_algebraic_states_controls_upper_bounds_multipliers.push_back( tmp_gf1a );
            }
            for (size i=0; i<_algebraic_states_controls_lower_bounds_multipliers.size(); i++) {
                GF1A_TYPE * tmp_gf1a = new GF1A_TYPE();
                tmp_gf1a->setCheckRange(SPLINE_CHECK_RANGE);
                tmp_gf1a->setup(SPLINE_TYPE, zeta_midpoint, _algebraic_states_controls_lower_bounds_multipliers[i], SPLINE_EXTEND_RANGE);
                _gf1a_algebraic_states_controls_lower_bounds_multipliers.push_back( tmp_gf1a );
            }

            for (size i=0; i<_point_constraints.size(); i++) {
                GF1A_TYPE * tmp_gf1a = new GF1A_TYPE();
                tmp_gf1a->setCheckRange(SPLINE_CHECK_RANGE);
                tmp_gf1a->setup(SPLINE_TYPE, _zeta, _point_constraints[i], SPLINE_EXTEND_RANGE);
                _gf1a_point_constraints.push_back( tmp_gf1a );
            }

            for (size i=0; i<_point_constraints_multipliers.size(); i++) {
                GF1A_TYPE * tmp_gf1a = new GF1A_TYPE();
                tmp_gf1a->setCheckRange(SPLINE_CHECK_RANGE);
                tmp_gf1a->setup(SPLINE_TYPE, _zeta, _point_constraints_multipliers[i], SPLINE_EXTEND_RANGE);
                _gf1a_point_constraints_multipliers.push_back( tmp_gf1a );
            }

            for (size i=0; i<_path_constr.size(); i++) {
                GF1A_TYPE * tmp_gf1a = new GF1A_TYPE();
                tmp_gf1a->setCheckRange(SPLINE_CHECK_RANGE);
                tmp_gf1a->setup(SPLINE_TYPE, zeta_midpoint, _path_constr[i], SPLINE_EXTEND_RANGE);
                _gf1a_path_constr.push_back( tmp_gf1a );
            }

            for (size i=0; i<_path_constr_multipliers.size(); i++) {
                GF1A_TYPE * tmp_gf1a = new GF1A_TYPE();
                tmp_gf1a->setCheckRange(SPLINE_CHECK_RANGE);
                tmp_gf1a->setup(SPLINE_TYPE, zeta_midpoint, _path_constr_multipliers[i] , SPLINE_EXTEND_RANGE);
                _gf1a_path_constr_multipliers.push_back( tmp_gf1a );
            }

            for (size i=0; i<_int_constr.size(); i++) {
                GF1A_TYPE * tmp_gf1a = new GF1A_TYPE();
                tmp_gf1a->setCheckRange(SPLINE_CHECK_RANGE);
                tmp_gf1a->setup(SPLINE_TYPE, _zeta, _int_constr[i], SPLINE_EXTEND_RANGE);
                _gf1a_int_constr.push_back( tmp_gf1a );
            }

            for (size i=0; i<_fo_eqns.size(); i++) {
                GF1A_TYPE * tmp_gf1a = new GF1A_TYPE();
                tmp_gf1a->setCheckRange(SPLINE_CHECK_RANGE);
                tmp_gf1a->setup(SPLINE_TYPE, zeta_midpoint, _fo_eqns[i], SPLINE_EXTEND_RANGE);
                _gf1a_fo_eqns.push_back( tmp_gf1a );
            }

            for (size i=0; i<_fo_eqns_multipliers.size(); i++) {
                GF1A_TYPE * tmp_gf1a = new GF1A_TYPE();
                tmp_gf1a->setCheckRange(SPLINE_CHECK_RANGE);
                tmp_gf1a->setup(SPLINE_TYPE, zeta_midpoint, _fo_eqns_multipliers[i], SPLINE_EXTEND_RANGE);
                _gf1a_fo_eqns_multipliers.push_back( tmp_gf1a );
            }

            for (size i=0; i<_post_processing.size(); i++) {
                GF1A_TYPE * tmp_gf1a = new GF1A_TYPE();
                tmp_gf1a->setCheckRange(SPLINE_CHECK_RANGE);
                tmp_gf1a->setup(SPLINE_TYPE, _zeta, _post_processing[i], SPLINE_EXTEND_RANGE);
                _gf1a_post_processing.push_back( tmp_gf1a );
            }

            for (size i=0; i<_differential_post_processing.size(); i++) {
                GF1A_TYPE * tmp_gf1a = new GF1A_TYPE();
                tmp_gf1a->setCheckRange(SPLINE_CHECK_RANGE);
                tmp_gf1a->setup(SPLINE_TYPE, zeta_midpoint, _differential_post_processing[i], SPLINE_EXTEND_RANGE);
                _gf1a_differential_post_processing.push_back( tmp_gf1a );
            }

            for (size i=0; i<_integral_post_processing.size(); i++) {
                GF1A_TYPE * tmp_gf1a = new GF1A_TYPE();
                tmp_gf1a->setCheckRange(SPLINE_CHECK_RANGE);
                tmp_gf1a->setup(SPLINE_TYPE, _zeta, _integral_post_processing[i], SPLINE_EXTEND_RANGE);
                _gf1a_integral_post_processing.push_back( tmp_gf1a );
            }

            {
                GF1A_TYPE * tmp_gf1a = new GF1A_TYPE();
                tmp_gf1a->setCheckRange(SPLINE_CHECK_RANGE);
                tmp_gf1a->setup(SPLINE_TYPE, zeta_midpoint, _integrand_target, SPLINE_EXTEND_RANGE);
                _gf1a_integrand_target = tmp_gf1a ;
            }

            {
                GF1A_TYPE * tmp_gf1a = new GF1A_TYPE();
                tmp_gf1a->setCheckRange(SPLINE_CHECK_RANGE);
                tmp_gf1a->setup(SPLINE_TYPE, _zeta, _cumulative_target, SPLINE_EXTEND_RANGE);
                _gf1a_cumulative_target = tmp_gf1a ;
            }

        }
    }

    void MidpointOcpSolutionSinglePhase::clear() {
        _target = 0;
        _cumulative_target = {};
        _integrand_target = {};
        _states_controls = {};
        _algebraic_states_controls = {};
        _point_constraints = {};
        _path_constr = {};
        _int_constr = {};
        _fo_eqns = {};
        _parameters = {};
        _zeta = {};
        _boundary_conditions = {};
        _post_processing = {};
        _differential_post_processing = {};
        _integral_post_processing = {};

        _states_controls_upper_bounds_multipliers = {};
        _states_controls_lower_bounds_multipliers = {};
        _algebraic_states_controls_upper_bounds_multipliers = {};
        _algebraic_states_controls_lower_bounds_multipliers = {};
        _parameters_upper_bounds_multipliers = {};
        _parameters_lower_bounds_multipliers = {};
        _point_constraints_multipliers = {};
        _path_constr_multipliers = {};
        _int_constr_multipliers = {};
        _fo_eqns_multipliers = {};
        _bcs_multipliers = {};
    }

    void MidpointOcpSolutionSinglePhase::writeMeshVarsToStream( ostream & body, ostream & header ) const {
        writeMeshVarsToStream( body, header, false, 0 );
    }

    void MidpointOcpSolutionSinglePhase::writeMeshVarsToStream( ostream & body, ostream & header, bool const add_phase_index, integer const phase_index ) const {
        //write solution to output in the following sequence:

        //header
        if (add_phase_index)
            header << "phase" << StreamChars::separator;
        header << "zeta" << StreamChars::separator;

        for (integer i=0; i<_states_controls.size(); i++)
            header << "state_control" << i << StreamChars::separator;

        for (integer i=0; i<_algebraic_states_controls.size(); i++)
            header << "algebraic_state_control" << i << StreamChars::separator;

        for (integer i=0; i<_parameters.size(); i++)
            header << "parameter" << i << StreamChars::separator;

        for (integer i=0; i<_post_processing.size(); i++)
            header << "post_proc" << i << StreamChars::separator;

        for (integer i=0; i<_differential_post_processing.size(); i++)
            header << "diff_post_proc" << i << StreamChars::separator;

        for (integer i=0; i<_integral_post_processing.size(); i++)
            header << "int_post_proc" << i << StreamChars::separator;

        // multipliers
        for (integer i=0; i<_states_controls_upper_bounds_multipliers.size(); i++)
            header << "lambda_states_controls_upper" << i << StreamChars::separator;

        for (integer i=0; i<_states_controls_lower_bounds_multipliers.size(); i++)
            header << "lambda_states_controls_lower" << i << StreamChars::separator;

        for (integer i=0; i<_algebraic_states_controls_upper_bounds_multipliers.size(); i++)
            header << "lambda_algebraic_states_controls_upper" << i << StreamChars::separator;

        for (integer i=0; i<_algebraic_states_controls_lower_bounds_multipliers.size(); i++)
            header << "lambda_algebraic_states_controls_lower" << i << StreamChars::separator;

        for (integer i=0; i<_point_constraints_multipliers.size(); i++)
            header << "lambda_states_constraints" << i << StreamChars::separator;

        for (integer i=0; i<_path_constr_multipliers.size(); i++)
            header << "lambda_differental_constraints" << i << StreamChars::separator;

        for (integer i=0; i<_fo_eqns_multipliers.size(); i++)
            header << "lambda_fo_eq" << i << StreamChars::separator;

        //target
        header << "integrand_target"  << StreamChars::separator
        << "cumulative_target" << StreamChars::new_line;

        //body
        for (integer mesh_point = 0; mesh_point < _zeta.size(); mesh_point++) {
            if (add_phase_index)
                body << phase_index << StreamChars::separator;

            real const zeta = _zeta[mesh_point];
            body << zeta << StreamChars::separator;

            body << std::scientific << std::setprecision(15);

            for (integer i=0; i<_states_controls.size(); i++)
                body << _states_controls[i][mesh_point] << StreamChars::separator;

            for (integer i=0; i<_algebraic_states_controls.size(); i++)
                body << _gf1a_algebraic_states_controls[i]->funcEval(zeta) << StreamChars::separator;

            for (integer i=0; i<_parameters.size(); i++)
                body << _parameters[i] << StreamChars::separator;

            for (integer i=0; i<_post_processing.size(); i++)
                body << _post_processing[i][mesh_point] << StreamChars::separator;

            for (integer i=0; i<_differential_post_processing.size(); i++)
                body << _gf1a_differential_post_processing[i]->funcEval(zeta) << StreamChars::separator;

            for (integer i=0; i<_integral_post_processing.size(); i++)
                body << _integral_post_processing[i][mesh_point] << StreamChars::separator;

            // multipliers
            for (integer i=0; i<_states_controls_upper_bounds_multipliers.size(); i++)
                body << _states_controls_upper_bounds_multipliers[i][mesh_point] << StreamChars::separator;

            for (integer i=0; i<_states_controls_lower_bounds_multipliers.size(); i++)
                body << _states_controls_lower_bounds_multipliers[i][mesh_point] << StreamChars::separator;

            for (integer i=0; i<_algebraic_states_controls_upper_bounds_multipliers.size(); i++)
                body << _gf1a_algebraic_states_controls_upper_bounds_multipliers[i]->funcEval(zeta) << StreamChars::separator;

            for (integer i=0; i<_algebraic_states_controls_lower_bounds_multipliers.size(); i++)
                body << _gf1a_algebraic_states_controls_lower_bounds_multipliers[i]->funcEval(zeta) << StreamChars::separator;

            for (integer i=0; i<_point_constraints_multipliers.size(); i++)
                body << _point_constraints_multipliers[i][mesh_point] << StreamChars::separator;

            for (integer i=0; i<_path_constr_multipliers.size(); i++)
                body << _gf1a_path_constr_multipliers[i]->funcEval(zeta) << StreamChars::separator;

            for (integer i=0; i<_fo_eqns_multipliers.size(); i++)
                body << _gf1a_fo_eqns_multipliers[i]->funcEval(zeta) << StreamChars::separator;

            //target
            body << _gf1a_integrand_target->funcEval(zeta) << StreamChars::separator
            << _cumulative_target[mesh_point] << StreamChars::new_line;
        }
    }

    void MidpointOcpSolutionSinglePhase::evalAtMesh(real      zeta,

                                            integer const num_states_controls,  real * states_controls,        real * states_controls_upper_bounds_mult,  real * states_controls_lower_bounds_mult,
                                            integer const num_alg_states_controls,  real * algebraic_states_controls,       real * algebraic_states_controls_upper_bounds_mult,  real * algebraic_states_controls_lower_bounds_mult,
                                            integer const num_fo_eqns,          real * fo_eqns,                real * fo_eqns_mult,
                                            integer const num_point_constr,     real * states_constr,          real * point_constr_mult,
                                            integer const num_path_constr,      real * path_constr,            real * path_constr_mult,
                                            integer const num_int_constr,       real * int_constr,

                                            integer const num_post_proc,        real * post_proc,
                                            integer const num_diff_post_proc,   real * diff_post_proc,
                                            integer const num_int_post_proc,    real * int_post_proc
                                            ) const {

        if ( (states_controls != nullptr) || (states_controls_upper_bounds_mult != nullptr) || (states_controls_lower_bounds_mult != nullptr) )
            MAVERICK_ASSERT( num_states_controls == _states_controls.size(), "MidpointOcpSolutionSinglePhase::evalAtMesh: wrong states and controls size.\n")

        if ( (algebraic_states_controls != nullptr) || (algebraic_states_controls_upper_bounds_mult != nullptr) || (algebraic_states_controls_lower_bounds_mult != nullptr) )
            MAVERICK_ASSERT( num_alg_states_controls == _algebraic_states_controls.size(), "MidpointOcpSolutionSinglePhase::evalAtMesh: wrong algerbaic states and controls size.\n")

        if ( int_constr != nullptr)
            MAVERICK_ASSERT( num_int_constr == _int_constr.size(), "OcpCompleteSolution::evalAtMesh: wrong integral constraints size.\n")

        if ( post_proc != nullptr )
            MAVERICK_ASSERT( num_post_proc == _post_processing.size(), "OcpCompleteSolution::evalAtMesh: wrong post processing size.\n")

        if ( diff_post_proc != nullptr )
            MAVERICK_ASSERT( num_diff_post_proc == _differential_post_processing.size(), "OcpCompleteSolution::evalAtMesh: wrong differential post processing size.\n")

        if ( int_post_proc != nullptr )
            MAVERICK_ASSERT( num_int_post_proc == _integral_post_processing.size(), "OcpCompleteSolution::evalAtMesh: wrong integral post processing size.\n")

        if ( (states_constr != nullptr) || (point_constr_mult != nullptr) )
            MAVERICK_ASSERT( num_point_constr == _point_constraints.size(), "OcpCompleteSolution::evalAtMesh: wrong point constraints size.\n")

        if ( (path_constr != nullptr) || (path_constr_mult != nullptr) )
            MAVERICK_ASSERT( num_path_constr == _path_constr.size(), "OcpCompleteSolution::evalAtMesh: wrong path constraints size.\n")

        if ( (fo_eqns != nullptr) || (fo_eqns_mult != nullptr) )
            MAVERICK_ASSERT( num_fo_eqns == _fo_eqns_multipliers.size(), "OcpCompleteSolution::evalAtMesh: wrong f.o. equations size.\n")


        if (states_controls != nullptr)
            for (size i=0; i<_states_controls.size(); i++)
                states_controls[i] = _gf1a_states_controls[i]->funcEval(zeta);

        if (states_controls_upper_bounds_mult != nullptr)
            for (size i=0; i<_states_controls.size(); i++)
                states_controls_upper_bounds_mult[i] = _gf1a_states_controls_upper_bounds_multipliers[i]->funcEval(zeta);

        if (states_controls_lower_bounds_mult != nullptr)
            for (size i=0; i<_states_controls.size(); i++)
                states_controls_lower_bounds_mult[i] = _gf1a_states_controls_lower_bounds_multipliers[i]->funcEval(zeta);

        if (algebraic_states_controls != nullptr)
            for (size i=0; i<_algebraic_states_controls.size(); i++)
                algebraic_states_controls[i] = _gf1a_algebraic_states_controls[i]->funcEval(zeta);

        if (algebraic_states_controls_upper_bounds_mult != nullptr)
            for (size i=0; i<_algebraic_states_controls.size(); i++)
                algebraic_states_controls_upper_bounds_mult[i] = _gf1a_algebraic_states_controls_upper_bounds_multipliers[i]->funcEval(zeta);

        if (algebraic_states_controls_lower_bounds_mult != nullptr)
            for (size i=0; i<_algebraic_states_controls.size(); i++)
                algebraic_states_controls_lower_bounds_mult[i] = _gf1a_algebraic_states_controls_lower_bounds_multipliers[i]->funcEval(zeta);

        if (fo_eqns != nullptr)
            for (size i=0; i<_fo_eqns.size(); i++)
                fo_eqns[i] = _gf1a_fo_eqns[i]->funcEval(zeta);

        if (fo_eqns_mult != nullptr)
            for (size i=0; i<_fo_eqns_multipliers.size(); i++)
                fo_eqns_mult[i] = _gf1a_fo_eqns_multipliers[i]->funcEval(zeta);

        if (states_constr != nullptr)
            for (size i=0; i<_point_constraints.size(); i++)
                states_constr[i] = _gf1a_point_constraints[i]->funcEval(zeta);

        if (point_constr_mult != nullptr)
            for (size i=0; i<_point_constraints_multipliers.size(); i++)
                point_constr_mult[i] = _gf1a_point_constraints_multipliers[i]->funcEval(zeta);

        if (path_constr != nullptr)
            for (size i=0; i<_path_constr.size(); i++)
                path_constr[i] = _gf1a_path_constr[i]->funcEval(zeta);

        if (path_constr_mult != nullptr)
            for (size i=0; i<_path_constr_multipliers.size(); i++)
                path_constr_mult[i] = _gf1a_path_constr_multipliers[i]->funcEval(zeta);

        if (int_constr != nullptr)
            for (size i=0; i<_int_constr.size(); i++)
                int_constr[i] = _gf1a_int_constr[i]->funcEval(zeta);

        if (post_proc != nullptr)
            for (size i=0; i<_post_processing.size(); i++)
                post_proc[i] = _gf1a_post_processing[i]->funcEval(zeta);

        if (diff_post_proc != nullptr)
            for (size i=0; i<_differential_post_processing.size(); i++)
                diff_post_proc[i] = _gf1a_differential_post_processing[i]->funcEval(zeta);

        if (int_post_proc != nullptr)
            for (size i=0; i<_integral_post_processing.size(); i++)
                int_post_proc[i] = _gf1a_integral_post_processing[i]->funcEval(zeta);

    }

    void MidpointOcpSolutionSinglePhase::eval(real const initial_zeta, real const final_zeta,

                                      integer const num_parameters,             real * parameters,             real * params_upper_bounds_mult,   real * params_lower_bounds_mult,
                                      integer const num_boundary_conditions,    real * boundary_conditions,    real * boundary_conditions_mult,
                                      integer const num_int_constr,             real * int_constr_at_end,      real * int_constr_mult,
                                      integer const num_int_post_proc,          real * int_post_proc_at_end
                                      ) const {

        if ( (parameters != nullptr) || (params_upper_bounds_mult != nullptr) || (params_lower_bounds_mult != nullptr) )
            MAVERICK_ASSERT( num_parameters == _parameters.size(), "MidpointOcpSolutionSinglePhase::evalParams wrong parameter size.\n")

        if ( (int_constr_at_end != nullptr) || (int_constr_mult != nullptr) )
            MAVERICK_ASSERT( num_int_constr == _int_constr_multipliers.size(), "OcpCompleteSolution::evalParams: wrong integral constraints size.\n")

        if ( (boundary_conditions != nullptr) || (boundary_conditions_mult != nullptr) )
            MAVERICK_ASSERT( num_boundary_conditions == _bcs_multipliers.size(), "OcpCompleteSolution::evalParams: wrong boundary conditions size.\n")

        if ( int_post_proc_at_end != nullptr )
            MAVERICK_ASSERT( num_int_post_proc == _integral_post_processing.size(), "OcpCompleteSolution::eval: wrong integral post processing size.\n")

        if (parameters != nullptr)
            copyVectorTo( _parameters.data(), parameters, num_parameters);

        if (params_upper_bounds_mult != nullptr)
            copyVectorTo( _parameters_upper_bounds_multipliers.data(), params_upper_bounds_mult, num_parameters);

        if (params_lower_bounds_mult != nullptr)
            copyVectorTo( _parameters_lower_bounds_multipliers.data(), params_lower_bounds_mult, num_parameters);

        if (boundary_conditions != nullptr)
            copyVectorTo( _boundary_conditions.data(), boundary_conditions, num_boundary_conditions);

        if (boundary_conditions_mult != nullptr)
            copyVectorTo( _bcs_multipliers.data(), boundary_conditions_mult, num_boundary_conditions);

        if (int_constr_at_end != nullptr)
            for (size i=0; i<num_int_constr; i++)
                int_constr_at_end[i] = *(_int_constr[i].end()-1);

        if (int_constr_mult != nullptr)
            copyVectorTo( _int_constr_multipliers.data(), int_constr_mult, num_int_constr);

        if (int_post_proc_at_end != nullptr)
            for (size i=0; i<num_int_post_proc; i++)
                int_post_proc_at_end[i] = *(_integral_post_processing[i].end()-1);
    }

    // getter
    real MidpointOcpSolutionSinglePhase::getTarget() const {    return _target; }
    vec_1d_real const & MidpointOcpSolutionSinglePhase::getCumulativeTarget() const {    return _cumulative_target; }
    vec_1d_real const & MidpointOcpSolutionSinglePhase::getIntegrandTarget() const {    return _integrand_target; }
    vec_2d_real const & MidpointOcpSolutionSinglePhase::getStatesControls() const {    return _states_controls; }
    vec_2d_real const & MidpointOcpSolutionSinglePhase::getAlgebraicStatesControls() const {    return _algebraic_states_controls; }
    vec_2d_real const & MidpointOcpSolutionSinglePhase::getPointConstraints() const {    return _point_constraints; }
    vec_2d_real const & MidpointOcpSolutionSinglePhase::getPathConstraints() const {    return _path_constr; }
    vec_2d_real const & MidpointOcpSolutionSinglePhase::getIntegralConstraints() const {    return _int_constr; }
    vec_2d_real const & MidpointOcpSolutionSinglePhase::getFoEquations() const {    return _fo_eqns; }
    vec_1d_real const & MidpointOcpSolutionSinglePhase::getParameters() const {    return _parameters; }
    vec_1d_real const & MidpointOcpSolutionSinglePhase::getDiscretisationPoints() const {    return _zeta; }
    vec_1d_real const & MidpointOcpSolutionSinglePhase::getBoundaryConditions() const {    return _boundary_conditions; }
    vec_2d_real const & MidpointOcpSolutionSinglePhase::getPostProcessing() const { return _post_processing; }
    vec_2d_real const & MidpointOcpSolutionSinglePhase::getDifferentialPostProcessing() const { return _differential_post_processing; }
    vec_2d_real const & MidpointOcpSolutionSinglePhase::getIntegralPostProcessing() const { return _integral_post_processing; }
    vec_2d_real const & MidpointOcpSolutionSinglePhase::getStatesControlsUpperBoundsMultipliers() const {return _states_controls_upper_bounds_multipliers; }
    vec_2d_real const & MidpointOcpSolutionSinglePhase::getStatesControlsLowerBoundsMultipliers() const {return _states_controls_lower_bounds_multipliers; }
    vec_2d_real const & MidpointOcpSolutionSinglePhase::getAlgebraicStatesControlsUpperBoundsMultipliers() const {return _algebraic_states_controls_upper_bounds_multipliers; }
    vec_2d_real const & MidpointOcpSolutionSinglePhase::getAlgebraicStatesControlsLowerBoundsMultipliers() const {return _algebraic_states_controls_lower_bounds_multipliers; }
    vec_1d_real const & MidpointOcpSolutionSinglePhase::getParametersUpperBoundsMultipliers() const {return _parameters_upper_bounds_multipliers; }
    vec_1d_real const & MidpointOcpSolutionSinglePhase::getParametersLowerBoundsMultipliers() const {return _parameters_lower_bounds_multipliers; }
    vec_2d_real const & MidpointOcpSolutionSinglePhase::getPointConstraintsMultipliers() const {return _point_constraints_multipliers; }
    vec_2d_real const & MidpointOcpSolutionSinglePhase::getPathConstraintsMultipliers() const {return _path_constr_multipliers; }
    vec_1d_real const & MidpointOcpSolutionSinglePhase::getIntConstraintsMultipliers() const {return _int_constr_multipliers; }
    vec_2d_real const & MidpointOcpSolutionSinglePhase::getFoEqnsMultipliers() const {return _fo_eqns_multipliers; }
    vec_1d_real const & MidpointOcpSolutionSinglePhase::getBoundaryConditionsMultipliers() const {return _bcs_multipliers; }

    void MidpointOcpSolutionSinglePhase::writeContentToGC(GC::GenericContainer & out_gc, MaverickOcp const * const p_ocp, integer const i_phase) const {
        out_gc.clear();
        GC::GenericContainer & gc_midpoint = out_gc["midpoint_vars"]; // container to store midpoint original variables

        out_gc["zeta"].set_vec_real(_zeta);
        vec_1d_real zeta_center;
        zeta_center.reserve(_zeta.size());
        for (size index = 0; index < _zeta.size()-1; index++) {
            zeta_center.push_back( (_zeta[index] + _zeta[index+1] ) / 2.0 );
        }
        gc_midpoint["zeta"].set_vec_real(zeta_center);

        out_gc["cumulative_lagrange_target"].set_vec_real(_cumulative_target);

        vec_1d_real tmp_vec;
        tmp_vec.reserve(_zeta.size());
        for (integer i=0; i<_zeta.size(); i++)
            tmp_vec.push_back(0);

        if (_gf1a_integrand_target != nullptr) { // if the solution has been setup
            for (integer i=0; i<_zeta.size(); i++)
                tmp_vec[i] = _gf1a_integrand_target->funcEval(_zeta[i]);
        } else {
            tmp_vec = {};
        }
        gc_midpoint["integrand_lagrange_target"].set_vec_real(_integrand_target);
        out_gc["integrand_lagrange_target"].set_vec_real(tmp_vec);

        out_gc["target"].set_real( getTarget() );

        if (p_ocp != nullptr ) {
            int const nx = p_ocp->numberOfStates(i_phase);
            for (integer i=0; i<nx; i++) {
                out_gc[p_ocp->stateName(i_phase, i)].set_vec_real(_states_controls[i]);
                out_gc["lambda_"+p_ocp->stateName(i_phase, i)+"_upper"].set_vec_real(_states_controls_upper_bounds_multipliers[i]);
                out_gc["lambda_"+p_ocp->stateName(i_phase, i)+"_lower"].set_vec_real(_states_controls_lower_bounds_multipliers[i]);
            }
            for (integer i=0; i<p_ocp->numberOfControls(i_phase); i++) {
                out_gc[p_ocp->controlName(i_phase, i)].set_vec_real(_states_controls[nx+i]);
                out_gc["lambda_"+p_ocp->controlName(i_phase, i)+"_upper"].set_vec_real(_states_controls_upper_bounds_multipliers[nx+i]);
                out_gc["lambda_"+p_ocp->controlName(i_phase, i)+"_lower"].set_vec_real(_states_controls_lower_bounds_multipliers[nx+i]);
            }
            int const nax = p_ocp->numberOfAlgebraicStates(i_phase);
            for (integer i=0; i<nax; i++) {
                for (integer j=0; j<_zeta.size(); j++)
                    tmp_vec[j] = _gf1a_algebraic_states_controls[i]->funcEval(_zeta[j]);
                out_gc[p_ocp->algebraicStateName(i_phase, i)].set_vec_real(tmp_vec);
                gc_midpoint[p_ocp->algebraicStateName(i_phase, i)].set_vec_real(_algebraic_states_controls[i]);

                for (integer j=0; j<_zeta.size(); j++)
                    tmp_vec[j] = _gf1a_states_controls_upper_bounds_multipliers[i]->funcEval(_zeta[j]);
                out_gc["lambda_"+p_ocp->algebraicStateName(i_phase, i)+"_upper"].set_vec_real(tmp_vec);
                gc_midpoint["lambda_"+p_ocp->algebraicStateName(i_phase, i)+"_upper"].set_vec_real(_algebraic_states_controls_upper_bounds_multipliers[i]);

                for (integer j=0; j<_zeta.size(); j++)
                    tmp_vec[j] = _gf1a_states_controls_lower_bounds_multipliers[i]->funcEval(_zeta[j]);
                out_gc["lambda_"+p_ocp->algebraicStateName(i_phase, i)+"_lower"].set_vec_real(tmp_vec);
                gc_midpoint["lambda_"+p_ocp->algebraicStateName(i_phase, i)+"_lower"].set_vec_real(_algebraic_states_controls_lower_bounds_multipliers[i]);
            }
            for (integer i=0; i<p_ocp->numberOfAlgebraicControls(i_phase); i++) {
                for (integer j=0; j<_zeta.size(); j++)
                    tmp_vec[j] = _gf1a_algebraic_states_controls[nax+i]->funcEval(_zeta[j]);
                out_gc[p_ocp->algebraicControlName(i_phase, i)].set_vec_real(tmp_vec);
                gc_midpoint[p_ocp->algebraicControlName(i_phase, i)].set_vec_real(_algebraic_states_controls[nax+i]);

                for (integer j=0; j<_zeta.size(); j++)
                    tmp_vec[j] = _gf1a_algebraic_states_controls_upper_bounds_multipliers[nax+i]->funcEval(_zeta[j]);
                out_gc["lambda_"+p_ocp->algebraicControlName(i_phase, i)+"_upper"].set_vec_real(tmp_vec);
                gc_midpoint["lambda_"+p_ocp->algebraicControlName(i_phase, i)+"_upper"].set_vec_real(_algebraic_states_controls_upper_bounds_multipliers[nax+i]);

                for (integer j=0; j<_zeta.size(); j++)
                    tmp_vec[j] = _gf1a_algebraic_states_controls_lower_bounds_multipliers[nax+i]->funcEval(_zeta[j]);
                out_gc["lambda_"+p_ocp->algebraicControlName(i_phase, i)+"_lower"].set_vec_real(tmp_vec);
                gc_midpoint["lambda_"+p_ocp->algebraicControlName(i_phase, i)+"_lower"].set_vec_real(_algebraic_states_controls_lower_bounds_multipliers[nax+i]);
            }
            for (integer i=0; i<_parameters.size(); i++) {
                out_gc[p_ocp->parameterName(i_phase, i)].set_real(_parameters[i]);
                out_gc["lambda_"+p_ocp->parameterName(i_phase, i)+"_upper"].set_real(_parameters_upper_bounds_multipliers[i]);
                out_gc["lambda_"+p_ocp->parameterName(i_phase, i)+"_lower"].set_real(_parameters_lower_bounds_multipliers[i]);
            }
            for (integer i=0; i<_point_constraints.size(); i++) {
                out_gc[p_ocp->pointConstraintName(i_phase, i)].set_vec_real(_point_constraints[i]);
                out_gc["lambda_"+p_ocp->pointConstraintName(i_phase, i)].set_vec_real(_point_constraints_multipliers[i]);
            }
            for (integer i=0; i<_path_constr.size(); i++) {
                for (integer j=0; j<_zeta.size(); j++)
                    tmp_vec[j] = _gf1a_path_constr[i]->funcEval(_zeta[j]);
                out_gc[p_ocp->pathConstraintName(i_phase, i)].set_vec_real(tmp_vec);
                gc_midpoint[p_ocp->pathConstraintName(i_phase, i)].set_vec_real(_path_constr[i]);

                for (integer j=0; j<_zeta.size(); j++)
                    tmp_vec[j] = _gf1a_path_constr_multipliers[i]->funcEval(_zeta[j]);
                out_gc["lambda_"+p_ocp->pathConstraintName(i_phase, i)].set_vec_real(tmp_vec);
                gc_midpoint["lambda_"+p_ocp->pathConstraintName(i_phase, i)].set_vec_real(_path_constr_multipliers[i]);
            }
            for (integer i=0; i<_int_constr.size(); i++) {
                out_gc[p_ocp->intConstraintName(i_phase, i)].set_vec_real(_int_constr[i]);
                out_gc["lambda_"+p_ocp->intConstraintName(i_phase, i)].set_real(_int_constr_multipliers[i]);
            }
            for (integer i=0; i<_fo_eqns.size(); i++) {
                for (integer j=0; j<_zeta.size(); j++)
                    tmp_vec[j] = _gf1a_fo_eqns[i]->funcEval(_zeta[j]);
                out_gc["equation"+std::to_string(i)].set_vec_real(tmp_vec);
                gc_midpoint["equation"+std::to_string(i)].set_vec_real(_fo_eqns[i]);

                for (integer j=0; j<_zeta.size(); j++)
                    tmp_vec[j] = _gf1a_fo_eqns_multipliers[i]->funcEval(_zeta[j]);
                out_gc["lambda_equation"+std::to_string(i)].set_vec_real(tmp_vec);
                gc_midpoint["lambda_equation"+std::to_string(i)].set_vec_real(_fo_eqns_multipliers[i]);
            }
            for (integer i=0; i<_boundary_conditions.size(); i++) {
                out_gc[p_ocp->boundaryConditionName(i_phase, i)].set_real(_boundary_conditions[i]);
                out_gc["lambda_"+p_ocp->boundaryConditionName(i_phase, i)].set_real(_bcs_multipliers[i]);
            }
            for (integer i=0; i<_post_processing.size(); i++) {
                out_gc[p_ocp->postProcessingName(i_phase, i)].set_vec_real(_post_processing[i]);
            }
            for (integer i=0; i<_differential_post_processing.size(); i++) {
                for (integer j=0; j<_zeta.size(); j++)
                    tmp_vec[j] = _gf1a_differential_post_processing[i]->funcEval(_zeta[j]);
                out_gc[p_ocp->differentialPostProcessingName(i_phase, i)].set_vec_real(tmp_vec);
                gc_midpoint[p_ocp->differentialPostProcessingName(i_phase, i)].set_vec_real(_differential_post_processing[i]);
            }
            for (integer i=0; i<_integral_post_processing.size(); i++) {
                out_gc[p_ocp->integralPostProcessingName(i_phase, i)].set_vec_real(_integral_post_processing[i]);
            }
        } else {
            out_gc["states_controls"].set_vector((integer) _states_controls.size());
            out_gc["lambda_states_controls_upper"].set_vector((integer) _states_controls.size());
            out_gc["lambda_states_controls_lower"].set_vector((integer) _states_controls.size());
            for (integer i=0; i<_states_controls.size(); i++) {
                out_gc["states_controls"][i].set_vec_real(_states_controls[i]);
                out_gc["lambda_states_controls_upper"][i].set_vec_real(_states_controls_upper_bounds_multipliers[i]);
                out_gc["lambda_states_controls_lower"][i].set_vec_real(_states_controls_lower_bounds_multipliers[i]);
            }
            out_gc["algebraic_states_controls"].set_vector((integer) _algebraic_states_controls.size());
            out_gc["lambda_algebraic_states_controls_upper"].set_vector((integer) _algebraic_states_controls.size());
            out_gc["lambda_algebraic_states_controls_lower"].set_vector((integer) _algebraic_states_controls.size());
            for (integer i=0; i<_algebraic_states_controls.size(); i++) {
                for (integer j=0; j<_zeta.size(); j++)
                    tmp_vec[j] = _gf1a_algebraic_states_controls[i]->funcEval(_zeta[j]);
                out_gc["algebraic_states_controls"][i].set_vec_real(tmp_vec);

                for (integer j=0; j<_zeta.size(); j++)
                    tmp_vec[j] = _gf1a_algebraic_states_controls_upper_bounds_multipliers[i]->funcEval(_zeta[j]);
                out_gc["lambda_algebraic_states_controls_upper"][i].set_vec_real(tmp_vec);

                for (integer j=0; j<_zeta.size(); j++)
                    tmp_vec[j] = _gf1a_algebraic_states_controls_lower_bounds_multipliers[i]->funcEval(_zeta[j]);
                out_gc["lambda_algebraic_states_controls_lower"][i].set_vec_real(tmp_vec);
            }

            gc_midpoint["algebraic_states_controls"].set_vector((integer) _algebraic_states_controls.size());
            gc_midpoint["lambda_algebraic_states_controls_upper"].set_vector((integer) _algebraic_states_controls.size());
            gc_midpoint["lambda_algebraic_states_controls_lower"].set_vector((integer) _algebraic_states_controls.size());
            for (integer i=0; i<_algebraic_states_controls.size(); i++) {
                gc_midpoint["algebraic_states_controls"][i].set_vec_real(_algebraic_states_controls[i]);
                gc_midpoint["lambda_algebraic_states_controls_upper"][i].set_vec_real(_algebraic_states_controls_upper_bounds_multipliers[i]);
                gc_midpoint["lambda_algebraic_states_controls_lower"][i].set_vec_real(_algebraic_states_controls_lower_bounds_multipliers[i]);
            }

            out_gc["parameters"].set_vector((integer) _parameters.size());
            out_gc["lambda_parameters_upper"].set_vector((integer) _parameters.size());
            out_gc["lambda_parameters_lower"].set_vector((integer) _parameters.size());
            for (integer i=0; i<_parameters.size(); i++) {
                out_gc["parameters"][i].set_real(_parameters[i]);
                out_gc["lambda_parameters_upper"][i].set_real(_parameters_upper_bounds_multipliers[i]);
                out_gc["lambda_parameters_lower"][i].set_real(_parameters_lower_bounds_multipliers[i]);
            }
            out_gc["point_constraints"].set_vector((integer) _point_constraints.size());
            out_gc["lambda_point_constraints"].set_vector((integer) _point_constraints.size());
            for (integer i=0; i<_point_constraints.size(); i++) {
                out_gc["point_constraints"][i].set_vec_real(_point_constraints[i]);
                out_gc["lambda_point_constraints"][i].set_vec_real(_point_constraints_multipliers[i]);
            }
            out_gc["path_constraints"].set_vector((u_integer) _path_constr.size());
            out_gc["lambda_path_constraints"].set_vector((u_integer) _path_constr.size());
            for (integer i=0; i<_path_constr.size(); i++) {
                for (integer j=0; j<_zeta.size(); j++)
                    tmp_vec[j] = _gf1a_path_constr[i]->funcEval(_zeta[j]);
                out_gc["path_constraints"][i].set_vec_real(tmp_vec);

                for (integer j=0; j<_zeta.size(); j++)
                    tmp_vec[j] = _gf1a_path_constr_multipliers[i]->funcEval(_zeta[j]);
                out_gc["lambda_path_constraints"][i].set_vec_real(tmp_vec);
            }
            gc_midpoint["path_constraints"].set_vector((u_integer) _path_constr.size());
            gc_midpoint["lambda_path_constraints"].set_vector((u_integer) _path_constr.size());
            for (integer i=0; i<_path_constr.size(); i++) {
                gc_midpoint["path_constraints"][i].set_vec_real(_path_constr[i]);
                gc_midpoint["lambda_path_constraints"][i].set_vec_real(_path_constr_multipliers[i]);
            }
            out_gc["integral_constraints"].set_vector((integer) _int_constr.size());
            out_gc["lambda_integral_constraints"].set_vector((integer) _int_constr.size());
            for (integer i=0; i<_int_constr.size(); i++) {
                out_gc["integral_constraints"][i].set_vec_real(_int_constr[i]);
                out_gc["lambda_integral_constraints"][i].set_real(_int_constr_multipliers[i]);
            }
            out_gc["equations"].set_vector((integer) _fo_eqns.size());
            out_gc["lambda_equations"].set_vector((integer) _fo_eqns.size());
            for (integer i=0; i<_fo_eqns.size(); i++) {
                for (integer j=0; j<_zeta.size(); j++)
                    tmp_vec[j] = _gf1a_fo_eqns[i]->funcEval(_zeta[j]);
                out_gc["equations"][i].set_vec_real(tmp_vec);

                for (integer j=0; j<_zeta.size(); j++)
                    tmp_vec[j] = _gf1a_fo_eqns_multipliers[i]->funcEval(_zeta[j]);
                out_gc["lambda_equations"][i].set_vec_real(tmp_vec);
            }
            gc_midpoint["equations"].set_vector((integer) _fo_eqns.size());
            gc_midpoint["lambda_equations"].set_vector((integer) _fo_eqns.size());
            for (integer i=0; i<_fo_eqns.size(); i++) {
                gc_midpoint["equations"][i].set_vec_real(_fo_eqns[i]);
                gc_midpoint["lambda_equations"][i].set_vec_real(_fo_eqns_multipliers[i]);
            }
            out_gc["boundary_conditions"].set_vector((integer) _boundary_conditions.size());
            out_gc["lambda_boundary_conditions"].set_vector((integer) _boundary_conditions.size());
            for (integer i=0; i<_boundary_conditions.size(); i++) {
                out_gc["boundary_conditions"][i].set_real(_boundary_conditions[i]);
                out_gc["lambda_boundary_conditions"][i].set_real(_bcs_multipliers[i]);
            }
            out_gc["post_processing"].set_vector((integer) _post_processing.size());
            for (integer i=0; i<_post_processing.size(); i++) {
                out_gc["post_processing"][i].set_vec_real(_post_processing[i]);
            }
            out_gc["differential_processing"].set_vector((integer) _differential_post_processing.size());
            for (integer i=0; i<_differential_post_processing.size(); i++) {
                for (integer j=0; j<_zeta.size(); j++)
                    tmp_vec[j] = _gf1a_differential_post_processing[i]->funcEval(_zeta[j]);
                out_gc["differential_processing"][i].set_vec_real(tmp_vec);
            }
            gc_midpoint["differential_processing"].set_vector((integer) _differential_post_processing.size());
            for (integer i=0; i<_differential_post_processing.size(); i++) {
                gc_midpoint["differential_processing"][i].set_vec_real(_differential_post_processing[i]);
            }
            out_gc["integral_post_processing"].set_vector((integer) _integral_post_processing.size());
            for (integer i=0; i<_integral_post_processing.size(); i++) {
                out_gc["integral_post_processing"][i].set_vec_real(_integral_post_processing[i]);
            }
        }
    }
    
    static std::vector<real> resampleVariableAtDifferentMesh(std::vector<real> const & given_mesh, std::vector<real> const & variable, std::vector<real> const & new_mesh) {
        MaverickUtils::GF1ASpline gf1a;
        gf1a.setup("Linear", given_mesh, variable, MaverickUtils::GF1ASpline::ExtendRange::keep_derivative); // "Akima", "Linear"
        
        std::vector<real> out;
        for (real x : new_mesh)
            out.push_back(gf1a.funcEval(x));
        
        return out;
    }
    
    std::unique_ptr<MidpointOcpSolutionSinglePhase> MidpointOcpSolutionSinglePhase::convertFromRealTable(real_table const & table, MaverickOcp const & ocp_problem, integer const i_phase, std::vector<std::string> & found_variables) {
        
        MidpointOcpSolutionSinglePhase * sol = new MidpointOcpSolutionSinglePhase();
        
        vec_1d_real const & zeta = table.at("zeta");
        vec_1d_real zeta_center;
        for (vec_1d_real::const_iterator it = zeta.begin(); it!=zeta.end()-1; it++) {
            zeta_center.push_back(  ( (*it) + (*(it+1)) )/2  );
        }
        // zeta_center.push_back( zeta_center.back() ); // copy last element
        
        vec_1d_real zeros_zeta_1d = vec_1d_real(zeta.size(), 0);
        vec_1d_real zeros_zeta_center_1d = vec_1d_real(zeta_center.size(), 0);
        
        vec_2d_real point_constr = vec_2d_real( ocp_problem.numberOfPointConstraints(i_phase),  zeros_zeta_1d  );
        vec_2d_real path_constr = vec_2d_real( ocp_problem.numberOfPathConstraints(i_phase),  zeros_zeta_center_1d  );
        vec_2d_real int_constr = vec_2d_real( ocp_problem.numberOfIntConstraints(i_phase),  zeros_zeta_1d  );
        vec_2d_real fo_eqns = vec_2d_real( ocp_problem.numberOfStates(i_phase),  zeros_zeta_center_1d  );
        vec_1d_real boundary_conditions = vec_1d_real( ocp_problem.numberOfBoundaryConditions(i_phase), 0);
        vec_2d_real post_processing = vec_2d_real( ocp_problem.numberOfPostProcessing(i_phase),  zeros_zeta_1d  );
        vec_2d_real differential_post_processing = vec_2d_real( ocp_problem.numberOfDifferentialPostProcessing(i_phase),  zeros_zeta_center_1d  );
        vec_2d_real integral_post_processing = vec_2d_real( ocp_problem.numberOfIntegralPostProcessing(i_phase),  zeros_zeta_1d  );
        
        vec_2d_real states_controls = vec_2d_real( ocp_problem.numberOfStates(i_phase)+ocp_problem.numberOfControls(i_phase),  zeros_zeta_center_1d  );
        vec_2d_real algebraic_states_controls = vec_2d_real( ocp_problem.numberOfAlgebraicStates(i_phase)+ocp_problem.numberOfAlgebraicControls(i_phase),  zeros_zeta_1d  );
        vec_1d_real params = vec_1d_real( ocp_problem.numberOfParameters(i_phase), 0);
        
        // now check for states or controls declared in the table
        integer i_s;
        for (i_s = 0; i_s < ocp_problem.numberOfStates(i_phase); i_s++) {
            try {
                string name = ocp_problem.stateName(i_phase, i_s);
                vec_1d_real const & vec = table.at(name);
                states_controls[ i_s ] = vec;
                found_variables.push_back(name);
            } catch (...) {}
        }
        for (integer i_c = 0; i_c < ocp_problem.numberOfControls(i_phase); i_c++) {
            try {
                string name = ocp_problem.controlName(i_phase, i_c);
                vec_1d_real const & vec = table.at(name);
                states_controls[ i_c + i_s ] = vec;
                found_variables.push_back(name);
            } catch (...) {}
        }
        
        for (i_s = 0; i_s < ocp_problem.numberOfAlgebraicStates(i_phase); i_s++) {
            try {
                string name = ocp_problem.algebraicStateName(i_phase, i_s);
                vec_1d_real const & vec = table.at(name);
                algebraic_states_controls[ i_s ] = resampleVariableAtDifferentMesh(zeta, vec, zeta_center);
                found_variables.push_back(name);
            } catch (...) {}
        }
        for (integer i_c = 0; i_c < ocp_problem.numberOfAlgebraicControls(i_phase); i_c++) {
            try {
                string name = ocp_problem.algebraicControlName(i_phase, i_c);
                vec_1d_real const & vec = table.at(name);
                algebraic_states_controls[ i_c + i_s ] = resampleVariableAtDifferentMesh(zeta, vec, zeta_center);
                found_variables.push_back(name);
            } catch (...) {}
        }
        
        //TODO: find multiplier in guess
        
        sol->setSolution(0, //target
                         zeta,
                         zeros_zeta_1d, zeros_zeta_center_1d, //cumulative and integrand target
                         states_controls, algebraic_states_controls, params,
                         point_constr, path_constr, int_constr, fo_eqns,
                         boundary_conditions,
                         post_processing, differential_post_processing, integral_post_processing);
        
        return std::unique_ptr<MidpointOcpSolutionSinglePhase>(sol);
    }
}
