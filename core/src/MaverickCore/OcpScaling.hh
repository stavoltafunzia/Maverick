/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef MAVERICK_OCP_SCALING_HH
#define MAVERICK_OCP_SCALING_HH

#include "MaverickCore/MaverickDefinitions.hh"
#include "MaverickGC/GenericContainer.hh"

namespace Maverick {

    class MaverickOcp;

    struct OcpScalingOptions {

        OcpScalingOptions();

        bool multiply_lagrange_by_n      = false;
        bool multiply_int_constr_by_n    = false;
        bool multiply_foeqns_by_dz       = false;
        bool multiply_foeqns_by_n        = false;
        bool divide_foeqns_by_z          = false;
        bool multiply_path_constr_by_dz  = false;
        bool multiply_path_constr_by_n   = false;
        bool divide_path_constr_by_z     = false;
        bool multiply_point_constr_by_dz = false;
        bool multiply_point_constr_by_n  = false;
        bool divide_point_constr_by_z    = false;
        bool divide_mayer_by_n           = false;
        bool divide_bcs_by_n             = false;
    };

    class OcpScaling {

    public:

        OcpScaling();

        OcpScaling( OcpScaling const & ocp_scaling );

        OcpScaling const & operator=(OcpScaling const & ocp_scaling);

        void setupForOcp( GC::GenericContainer const & gc, MaverickOcp const & ocp_problem, vec_2d_real const & zeta );

        void setupForOcpAsNoScaling( MaverickOcp const & ocp_problem );

        void writeScalingsToStream( std::ostream & out ) const;

        OcpScalingOptions const & getScalingOptions() const;

        real                getTargetScaling(integer const i_phase)                  const;
        vec_1d_real const & getStatesControlScaling(integer const i_phase)           const;
        vec_1d_real const & getAlgebraicStatesControlScaling(integer const i_phase)  const;
        vec_1d_real const & getParamsScaling(integer const i_phase)                  const;
        vec_1d_real const & getFoEqnsScaling(integer const i_phase)                  const;
        vec_1d_real const & getPointConstraintsScaling(integer const i_phase)        const;
        vec_1d_real const & getPathConstraintsScaling(integer const i_phase)         const;
        vec_1d_real const & getIntConstraintsScaling(integer const i_phase)          const;
        vec_1d_real const & getBoundaryConditionsScaling(integer const i_phase)      const;
        vec_1d_real const & getEventConstraintsScaling(integer const i_phase)        const;

    protected:

        void setupForOcpAndOnePhase( GC::GenericContainer const & gc, MaverickOcp const & ocp_problem, vec_2d_real const & zeta );

        bool checkIfStringMatchesAutomaticBounds(std::string const & str);

        void clear();

        void setupOptions( GC::GenericContainer const & gc );

        vec_1d_real _target                    = {};
        vec_2d_real _states_control            = {};
        vec_2d_real _algebraic_states_controls = {};
        vec_2d_real _params                    = {};
        vec_2d_real _fo_equations              = {};
        vec_2d_real _point_constraints         = {};
        vec_2d_real _path_constraints          = {};
        vec_2d_real _int_constraints           = {};
        vec_2d_real _boundary_conditions       = {};
        vec_2d_real _event_constraints         = {};

        OcpScalingOptions _options;

    };
}

#endif
