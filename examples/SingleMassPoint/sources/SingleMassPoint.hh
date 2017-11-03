#ifndef SingleMassPoint_HH
#define SingleMassPoint_HH

#include "MaverickCore/Maverick.hh"
#include "MaverickUtils/GenericFunction/GenericFunction1AInterface.hh"


#define NUM_MODEL_PARAMS 14
#define MOD_PAR_INDEX_g 0
#define MOD_PAR_INDEX_m 1
#define MOD_PAR_INDEX_DX 2
#define MOD_PAR_INDEX_DY 3
#define MOD_PAR_INDEX_Klambda 4
#define MOD_PAR_INDEX_Pmax 5
#define MOD_PAR_INDEX_Fx_max 6
#define MOD_PAR_INDEX_Omega_max 7
#define MOD_PAR_INDEX_V0 8
#define MOD_PAR_INDEX_road_width 9
#define MOD_PAR_INDEX_w_reg 10
#define MOD_PAR_INDEX_Vmax 11
#define MOD_PAR_INDEX_Pmin 12
#define MOD_PAR_INDEX_Vf 13


namespace SingleMassPointNamespace {

    using namespace Maverick;

    class SingleMassPoint : public MaverickOcp {

    public:

        SingleMassPoint();

        std::vector<integer> _dim_pp  ;
        std::vector<integer> _dim_ipp ;

        std::vector<std::vector<std::string>> _post_processing_names ;
        std::vector<std::vector<std::string>> _differential_post_processing_names ;
        std::vector<std::vector<std::string>> _integral_post_processing_names ;

        // model parameters
        bool _is_model_param_set[NUM_MODEL_PARAMS];

        //external objects
        std::unique_ptr<MaverickUtils::GenericFunction1AInterface> _p_RoadXR = nullptr;
        std::unique_ptr<MaverickUtils::GenericFunction1AInterface> _p_RoadYL = nullptr;
        std::unique_ptr<MaverickUtils::GenericFunction1AInterface> _p_RoadHeading = nullptr;
        std::unique_ptr<MaverickUtils::GenericFunction1AInterface> _p_RoadXL = nullptr;
        std::unique_ptr<MaverickUtils::GenericFunction1AInterface> _p_RoadLeftWidth = nullptr;
        std::unique_ptr<MaverickUtils::GenericFunction1AInterface> _p_RoadX = nullptr;
        std::unique_ptr<MaverickUtils::GenericFunction1AInterface> _p_RoadRightWidth = nullptr;
        std::unique_ptr<MaverickUtils::GenericFunction1AInterface> _p_RoadY = nullptr;
        std::unique_ptr<MaverickUtils::GenericFunction1AInterface> _p_RoadYR = nullptr;
        std::unique_ptr<MaverickUtils::GenericFunction1AInterface> _p_RoadCurvature = nullptr;


        // setup
        virtual void derivedSetup(GC::GenericContainer const & gc);

        // info
        virtual void printDerivedInfo(std::ostream & out, InfoLevel info_level) const;

        // OCP interface

    //   +-------------------------------------+
    //   |  _                           _      |
    //   | | |__   ___  _   _ _ __   __| |___  |
    //   | | '_ \ / _ \| | | | '_ \ / _` / __| |
    //   | | |_) | (_) | |_| | | | | (_| \__ \ |
    //   | |_.__/ \___/ \__,_|_| |_|\__,_|___/ |
    //   |                                     |
    //   +-------------------------------------+

        virtual integer getStatesControlsBounds(integer const i_phase,
                                                real    const __zeta,
                                                real          lower[],
                                                real          upper[] ) const;

        virtual integer getAlgebraicStatesControlsBounds(integer const i_phase,
                                                real    const __zeta,
                                                real          lower[],
                                                real          upper[] ) const;

        virtual integer getParametersBounds(integer const i_phase,
                                            real          lower[],
                                            real          upper[] ) const;

        virtual integer getPointConstraintsBounds(integer const i_phase,
                                                  real    const __zeta,
                                                  real          lower[],
                                                  real          upper[] ) const;

        virtual integer getPathConstraintsBounds(integer const i_phase,
                                                 real    const __zeta,
                                                 real          lower[],
                                                 real          upper[] ) const;

        virtual integer getIntConstraintsBounds(integer const i_phase,
                                                real    const __zeta_i,
                                                real    const __zeta_f,
                                                real          lower[],
                                                real          upper[] ) const;

        virtual integer getBoundaryConditionsBounds(integer const i_phase,
                                                    real    const __zeta_i,
                                                    real    const __zeta_f,
                                                    real          lower[],
                                                    real          upper[] ) const;

        virtual integer getEventConstraintsBounds(integer const i_phase,
                                                  real    const __zeta_l,
                                                  real    const __zeta_r,
                                                  real          lower[],
                                                  real          upper[] ) const;

        // +----------------------------+
        // |                            |
        // |   __ _ _   _  ___ ___ ___  |
        // |  / _` | | | |/ _ Y __/ __| |
        // | | (_| | |_| |  __|__ \__ \ |
        // |  \__, |\__,_|\___|___/___/ |
        // |  |___/                     |
        // +----------------------------+

        virtual void evalAtMesh(integer const i_phase,
                        real    const zeta,

                        integer const num_states_controls,  real * states_controls,       real * states_controls_upper_bounds_mult,  real * states_controls_lower_bounds_mult,
                        integer const num_algebraic_states_controls,  real * algebraic_states_controls,       real * algebraic_states_controls_upper_bounds_mult,  real * algebraic_states_controls_lower_bounds_mult,
                        integer const num_fo_eqns,          real * fo_eqns_mult,
                        integer const num_point_constr,     real * point_constr_mult,
                        integer const num_diff_constr,      real * diff_constr_mult
                        ) const;

        virtual void eval(integer const i_phase,
                  real const initial_zeta,   real const final_zeta,
                  
                  integer const num_parameters,          real * parameters,            real * params_upper_bounds_mult,    real * params_lower_bounds_mult,
                  integer const num_boundary_conditions, real * boundary_conditions_mult,
                  integer const num_int_constr,          real * int_constr_mult
                  ) const;


    //   +-----------------------------------+
    //   |                                   |
    //   |  _ __ ___   __ _ _   _  ___ _ __  |
    //   | | '_ ` _ \ / _` | | | |/ _ \ '__| |
    //   | | | | | | | (_| | |_| |  __/ |    |
    //   | |_| |_| |_|\__,_|\__, |\___|_|    |
    //   |                  |___/            |
    //   +-----------------------------------+

    virtual integer mayer ( integer const i_phase,
                           real const initial_state_control[],
                           real const final_state_control[],
                           real const parameters[],
                           real       &value ) const;

    virtual integer mayerJac ( integer const i_phase,
                              real const initial_state_control[],
                              real const final_state_control[],
                              real const parameters[],
                              real       jac_xu_init[],
                              real       jac_xu_fin[],
                              real       jac_p[] ) const;

    virtual integer mayerJacXuInitNnz ( integer const i_phase ) const;
    virtual void mayerJacXuInitPattern ( integer const i_phase, integer cols[] ) const;

    virtual integer mayerJacXuFinNnz ( integer const i_phase ) const;
    virtual void mayerJacXuFinPattern ( integer const i_phase, integer cols[] ) const;

    virtual integer mayerJacPNnz ( integer const i_phase ) const;
    virtual void mayerJacPPattern ( integer const i_phase, integer cols[] ) const;

    virtual integer mayerHess ( integer const i_phase,
                               real const initial_state_control[],
                               real const final_state_control[],
                               real const parameters[],
                               real       lambda_0,
                               real       hess_xu_init_xu_init[],
                               real       hess_xu_init_xu_fin[],
                               real       hess_xu_init_p[],
                               real       hess_xu_fin_xu_fin[],
                               real       hess_xu_fin_p[],
                               real       hess_p_p[] ) const;

    virtual integer mayerHessXuInitXuInitNnz ( integer const i_phase ) const;
    virtual void mayerHessXuInitXuInitPattern ( integer const i_phase,
                                               integer rows[],
                                               integer cols[] ) const;

    virtual integer mayerHessXuInitXuFinNnz ( integer const i_phase ) const;
    virtual void mayerHessXuInitXuFinPattern ( integer const i_phase,
                                              integer rows[],
                                              integer cols[] ) const;

    virtual integer mayerHessXuInitPNnz ( integer const i_phase ) const;
    virtual void mayerHessXuInitPPattern ( integer const i_phase,
                                          integer       rows[],
                                          integer       cols[] ) const;

    virtual integer mayerHessXuFinXuFinNnz ( integer const i_phase ) const;
    virtual void mayerHessXuFinXuFinPattern ( integer const i_phase,
                                             integer rows[],
                                             integer cols[] ) const;

    virtual integer mayerHessXuFinPNnz ( integer const i_phase ) const;
    virtual void mayerHessXuFinPPattern ( integer const i_phase,
                                         integer       rows[],
                                         integer       cols[] ) const;

    virtual integer mayerHessPPNnz ( integer const i_phase ) const;
    virtual void mayerHessPPPattern ( integer const i_phase,
                                     integer rows[],
                                     integer cols[] ) const;

    //   +--------------------------------------------+
    //   |  _                                         |
    //   | | | __ _  __ _ _ __ __ _ _ __   __ _  ___  |
    //   | | |/ _` |/ _` | '__/ _` | '_ \ / _` |/ _ \ |
    //   | | | (_| | (_| | | | (_| | | | | (_| |  __/ |
    //   | |_|\__,_|\__, |_|  \__,_|_| |_|\__, |\___| |
    //   |          |___/                 |___/       |
    //   +--------------------------------------------+

    virtual integer lagrange ( integer const i_phase,
                              real    const state_control[],
                              real    const state_control_derivative[],
                              real    const algebraic_state_control[],
                              real    const parameters[],
                              real           __zeta,
                              real          &value ) const;

    virtual integer lagrangeJac ( integer const i_phase,
                                 real    const state_control[],
                                 real    const state_control_derivative[],
                                 real    const algebraic_state_control[],
                                 real    const parameters[],
                                 real           __zeta,
                                 real          jac_xu[],
                                 real          jac_dxu[],
                                 real          jac_axu[],
                                 real          jac_p[] ) const;

    virtual integer lagrangeJacXuNnz ( integer const i_phase ) const;
    virtual void lagrangeJacXuPattern ( integer const i_phase, integer cols[] ) const;

    virtual integer lagrangeJacDxuNnz ( integer const i_phase ) const;
    virtual void lagrangeJacDxuPattern ( integer const i_phase, integer cols[] ) const;

    virtual integer lagrangeJacAxuNnz ( integer const i_phase ) const;
    virtual void lagrangeJacAxuPattern ( integer const i_phase, integer cols[] ) const;

    virtual integer lagrangeJacPNnz ( integer const i_phase ) const;
    virtual void lagrangeJacPPattern ( integer const i_phase, integer cols[] ) const;

    virtual integer lagrangeHess ( integer const i_phase,
                                  real    const state_control[],
                                  real    const state_control_derivative[],
                                  real    const algebraic_state_control[],
                                  real    const parameters[],
                                  real           __zeta,
                                  real           lambda_0,
                                  real          hess_xu_xu[],
                                  real          hess_xu_dxu[],
                                  real          hess_xu_axu[],
                                  real          hess_xu_p[],
                                  real          hess_dxu_dxu[],
                                  real          hess_dxu_axu[],
                                  real          hess_dxu_p[],
                                  real          hess_axu_axu[],
                                  real          hess_axu_p[],
                                  real          hess_p_p[] ) const;

    virtual integer lagrangeHessXuXuNnz ( integer const i_phase ) const;
    virtual void lagrangeHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

    virtual integer lagrangeHessXuDxuNnz ( integer const i_phase ) const;
    virtual void lagrangeHessXuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

    virtual integer lagrangeHessXuAxuNnz ( integer const i_phase ) const;
    virtual void lagrangeHessXuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

    virtual integer lagrangeHessXuPNnz ( integer const i_phase ) const;
    virtual void lagrangeHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

    virtual integer lagrangeHessDxuDxuNnz ( integer const i_phase ) const;
    virtual void lagrangeHessDxuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

    virtual integer lagrangeHessDxuAxuNnz ( integer const i_phase ) const;
    virtual void lagrangeHessDxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

    virtual integer lagrangeHessDxuPNnz ( integer const i_phase ) const;
    virtual void lagrangeHessDxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

    virtual integer lagrangeHessAxuAxuNnz ( integer const i_phase ) const;
    virtual void lagrangeHessAxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

    virtual integer lagrangeHessAxuPNnz ( integer const i_phase ) const;
    virtual void lagrangeHessAxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

    virtual integer lagrangeHessPPNnz ( integer const i_phase ) const;
    virtual void lagrangeHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

    //   +-------------------------------------------------------------+
    //   |   __                                 _   _                  |
    //   |  / _| ___      ___  __ _ _   _  __ _| |_(_) ___  _ __  ___  |
    //   | | |_ / _ \    / _ \/ _` | | | |/ _` | __| |/ _ \| '_ \/ __| |
    //   | |  _| (_) |  |  __/ (_| | |_| | (_| | |_| | (_) | | | \__ \ |
    //   | |_|(_)___(_)  \___|\__, |\__,_|\__,_|\__|_|\___/|_| |_|___/ |
    //   |                       |_|                                   |
    //   +-------------------------------------------------------------+

    virtual integer foEqns (integer const i_phase,
                            real    const state_control[],
                            real    const state_control_derivative[],
                            real    const algebraic_state_control[],
                            real    const parameters[],
                            real           __zeta,
                            real          values[] ) const;

    virtual integer foEqnsJac (integer const i_phase,
                               real    const state_control[],
                               real    const state_control_derivative[],
                               real    const algebraic_state_control[],
                               real    const parameters[],
                               real       __zeta,
                               real       j_xu[],
                               real       j_dxu[],
                               real       j_axu[],
                               real       j_p[] ) const;

     virtual integer foEqnsJacXuNnz ( integer const i_phase ) const;
     virtual void foEqnsJacXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer foEqnsJacDxuNnz ( integer const i_phase ) const;
     virtual void foEqnsJacDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer foEqnsJacAxuNnz ( integer const i_phase ) const;
     virtual void foEqnsJacAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer foEqnsJacPNnz ( integer const i_phase ) const;
     virtual void foEqnsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer foEqnsHess ( integer const i_phase,
                                   real    const state_control[],
                                   real    const state_control_derivative[],
                                   real    const algebraic_state_control[],
                                   real    const parameters[],
                                   real          __zeta,
                                   real    const lambda[],
                                   real          hess_xu_xu[],
                                   real          hess_xu_dxu[],
                                   real          hess_xu_axu[],
                                   real          hess_xu_p[],
                                   real          hess_dxu_dxu[],
                                   real          hess_dxu_axu[],
                                   real          hess_dxu_p[],
                                   real          hess_axu_axu[],
                                   real          hess_axu_p[],
                                   real          hess_p_p[] ) const;

     virtual integer foEqnsHessXuXuNnz ( integer const i_phase ) const;
     virtual void foEqnsHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer foEqnsHessXuDxuNnz ( integer const i_phase ) const;
     virtual void foEqnsHessXuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer foEqnsHessXuAxuNnz ( integer const i_phase ) const;
     virtual void foEqnsHessXuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer foEqnsHessXuPNnz ( integer const i_phase ) const;
     virtual void foEqnsHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer foEqnsHessDxuDxuNnz ( integer const i_phase ) const;
     virtual void foEqnsHessDxuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer foEqnsHessDxuAxuNnz ( integer const i_phase ) const;
     virtual void foEqnsHessDxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer foEqnsHessDxuPNnz ( integer const i_phase ) const;
     virtual void foEqnsHessDxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer foEqnsHessAxuAxuNnz ( integer const i_phase ) const;
     virtual void foEqnsHessAxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer foEqnsHessAxuPNnz ( integer const i_phase ) const;
     virtual void foEqnsHessAxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer foEqnsHessPPNnz ( integer const i_phase ) const;
     virtual void foEqnsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

    // +-----------------------------------------------------------------------+
    // |      _ _  __  __                       _             _       _        |
    // |   __| (_)/ _|/ _|   ___ ___  _ __  ___| |_ _ __ __ _(_)_ __ | |_ ___  |
    // |  / _` | | |_| |_   / __/ _ \| '_ \/ __| __| '__/ _` | | '_ \| __/ __| |
    // | | (_| | |  _|  _| | (_| (_) | | | \__ \ |_| | | (_| | | | | | |_\__ \ |
    // |  \__,_|_|_| |_|    \___\___/|_| |_|___/\__|_|  \__,_|_|_| |_|\__|___/ |
    // |                                                                       |
    // +-----------------------------------------------------------------------+


    virtual integer pathConstraints (integer const i_phase,
                            real    const state_control[],
                            real    const state_control_derivative[],
                            real    const algebraic_state_control[],
                            real    const parameters[],
                            real           __zeta,
                            real          values[] ) const;

    virtual integer pathConstraintsJac (integer const i_phase,
                               real    const state_control[],
                               real    const state_control_derivative[],
                               real    const algebraic_state_control[],
                               real    const parameters[],
                               real       __zeta,
                               real       j_xu[],
                               real       j_dxu[],
                               real       j_axu[],
                               real       j_p[] ) const;

     virtual integer pathConstraintsJacXuNnz ( integer const i_phase ) const;
     virtual void pathConstraintsJacXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer pathConstraintsJacDxuNnz ( integer const i_phase ) const;
     virtual void pathConstraintsJacDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer pathConstraintsJacAxuNnz ( integer const i_phase ) const;
     virtual void pathConstraintsJacAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer pathConstraintsJacPNnz ( integer const i_phase ) const;
     virtual void pathConstraintsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer pathConstraintsHess ( integer const i_phase,
                                   real    const state_control[],
                                   real    const state_control_derivative[],
                                   real    const algebraic_state_control[],
                                   real    const parameters[],
                                   real           __zeta,
                                   real    const lambda[],
                                   real          hess_xu_xu[],
                                   real          hess_xu_dxu[],
                                   real          hess_xu_axu[],
                                   real          hess_xu_p[],
                                   real          hess_dxu_dxu[],
                                   real          hess_dxu_axu[],
                                   real          hess_dxu_p[],
                                   real          hess_axu_axu[],
                                   real          hess_axu_p[],
                                   real          hess_p_p[] ) const;

     virtual integer pathConstraintsHessXuXuNnz ( integer const i_phase ) const;
     virtual void pathConstraintsHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer pathConstraintsHessXuDxuNnz ( integer const i_phase ) const;
     virtual void pathConstraintsHessXuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer pathConstraintsHessXuAxuNnz ( integer const i_phase ) const;
     virtual void pathConstraintsHessXuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer pathConstraintsHessXuPNnz ( integer const i_phase ) const;
     virtual void pathConstraintsHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer pathConstraintsHessDxuDxuNnz ( integer const i_phase ) const;
     virtual void pathConstraintsHessDxuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer pathConstraintsHessDxuAxuNnz ( integer const i_phase ) const;
     virtual void pathConstraintsHessDxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer pathConstraintsHessDxuPNnz ( integer const i_phase ) const;
     virtual void pathConstraintsHessDxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer pathConstraintsHessAxuAxuNnz ( integer const i_phase ) const;
     virtual void pathConstraintsHessAxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer pathConstraintsHessAxuPNnz ( integer const i_phase ) const;
     virtual void pathConstraintsHessAxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer pathConstraintsHessPPNnz ( integer const i_phase ) const;
     virtual void pathConstraintsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

    // +----------------------------------------------------------------------------+
    // |      _        _                             _             _       _        |
    // |  ___| |_ __ _| |_ ___    ___ ___  _ __  ___| |_ _ __ __ _(_)_ __ | |_ ___  |
    // | / __| __/ _` | __/ _ \  / __/ _ \| '_ \/ __| __| '__/ _` | | '_ \| __/ __| |
    // | \__ \ || (_| | ||  __/ | (_| (_) | | | \__ \ |_| | | (_| | | | | | |_\__ \ |
    // | |___/\__\__,_|\__\___|  \___\___/|_| |_|___/\__|_|  \__,_|_|_| |_|\__|___/ |
    // |                                                                            |
    // +----------------------------------------------------------------------------+


    virtual integer pointConstraints ( integer const i_phase,
                                 real    const state_control[],
                                 real    const parameters[],
                                 real           __zeta,
                                 real          values[] ) const;

    virtual integer pointConstraintsJac (integer const i_phase,
                                        real const state_control[],
                                        real const parameters[],
                                        real       __zeta,
                                        real       j_xu[],
                                        real       j_p[] ) const;

    virtual integer pointConstraintsJacXuNnz ( integer const i_phase ) const;
    virtual void pointConstraintsJacXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

    virtual integer pointConstraintsJacPNnz ( integer const i_phase ) const;
    virtual void pointConstraintsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

    virtual integer pointConstraintsHess (integer const i_phase,
                                     real    const state_control[],
                                     real    const parameters[],
                                     real           __zeta,
                                     real    const lambda[],
                                     real          hess_xu_xu[],
                                     real          hess_xu_p[],
                                     real          hess_p_p[] ) const;
    virtual integer pointConstraintsHessXuXuNnz ( integer const i_phase ) const;
    virtual void pointConstraintsHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

    virtual integer pointConstraintsHessPPNnz ( integer const i_phase ) const;
    virtual void pointConstraintsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

    virtual integer pointConstraintsHessXuPNnz ( integer const i_phase ) const;
    virtual void pointConstraintsHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

    // +-----------------------------------------------------------------------------------------+
    // |  _       _                       _                       _             _       _        |
    // | (_)_ __ | |_ ___  __ _ _ __ __ _| |   ___ ___  _ __  ___| |_ _ __ __ _(_)_ __ | |_ ___  |
    // | | | '_ \| __/ _ \/ _` | '__/ _` | |  / __/ _ \| '_ \/ __| __| '__/ _` | | '_ \| __/ __| |
    // | | | | | | ||  __/ (_| | | | (_| | | | (_| (_) | | | \__ \ |_| | | (_| | | | | | |_\__ \ |
    // | |_|_| |_|\__\___|\__, |_|  \__,_|_|  \___\___/|_| |_|___/\__|_|  \__,_|_|_| |_|\__|___/ |
    // |                  |___/                                                                  |
    // +-----------------------------------------------------------------------------------------+


    virtual integer intConstraints (integer const i_phase,
                            real    const state_control[],
                            real    const state_control_derivative[],
                            real    const algebraic_state_control[],
                            real    const parameters[],
                            real           __zeta,
                            real          values[] ) const;

    virtual integer intConstraintsJac (integer const i_phase,
                               real    const state_control[],
                               real    const state_control_derivative[],
                               real    const algebraic_state_control[],
                               real    const parameters[],
                               real       __zeta,
                               real       j_xu[],
                               real       j_dxu[],
                               real       j_axu[],
                               real       j_p[] ) const;

     virtual integer intConstraintsJacXuNnz ( integer const i_phase ) const;
     virtual void intConstraintsJacXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer intConstraintsJacDxuNnz ( integer const i_phase ) const;
     virtual void intConstraintsJacDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer intConstraintsJacAxuNnz ( integer const i_phase ) const;
     virtual void intConstraintsJacAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer intConstraintsJacPNnz ( integer const i_phase ) const;
     virtual void intConstraintsJacPPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer intConstraintsHess ( integer const i_phase,
                                   real    const state_control[],
                                   real    const state_control_derivative[],
                                   real    const algebraic_state_control[],
                                   real    const parameters[],
                                   real           __zeta,
                                   real    const lambda[],
                                   real          hess_xu_xu[],
                                   real          hess_xu_dxu[],
                                   real          hess_xu_axu[],
                                   real          hess_xu_p[],
                                   real          hess_dxu_dxu[],
                                   real          hess_dxu_axu[],
                                   real          hess_dxu_p[],
                                   real          hess_axu_axu[],
                                   real          hess_axu_p[],
                                   real          hess_p_p[] ) const;

     virtual integer intConstraintsHessXuXuNnz ( integer const i_phase ) const;
     virtual void intConstraintsHessXuXuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer intConstraintsHessXuDxuNnz ( integer const i_phase ) const;
     virtual void intConstraintsHessXuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer intConstraintsHessXuAxuNnz ( integer const i_phase ) const;
     virtual void intConstraintsHessXuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer intConstraintsHessXuPNnz ( integer const i_phase ) const;
     virtual void intConstraintsHessXuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer intConstraintsHessDxuDxuNnz ( integer const i_phase ) const;
     virtual void intConstraintsHessDxuDxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer intConstraintsHessDxuAxuNnz ( integer const i_phase ) const;
     virtual void intConstraintsHessDxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer intConstraintsHessDxuPNnz ( integer const i_phase ) const;
     virtual void intConstraintsHessDxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer intConstraintsHessAxuAxuNnz ( integer const i_phase ) const;
     virtual void intConstraintsHessAxuAxuPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer intConstraintsHessAxuPNnz ( integer const i_phase ) const;
     virtual void intConstraintsHessAxuPPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

     virtual integer intConstraintsHessPPNnz ( integer const i_phase ) const;
     virtual void intConstraintsHessPPPattern ( integer const i_phase, integer rows[], integer cols[] ) const;

    //   +--------------------------------------------------------------------------------------------------+
    //   |  _                           _                                        _ _ _   _                  |
    //   | | |__   ___  _   _ _ __   __| | __ _ _ __ _   _    ___ ___  _ __   __| (_) |_(_) ___  _ __  ___  |
    //   | | '_ \ / _ \| | | | '_ \ / _` |/ _` | '__| | | |  / __/ _ \| '_ \ / _` | | __| |/ _ \| '_ \/ __| |
    //   | | |_) | (_) | |_| | | | | (_| | (_| | |  | |_| | | (_| (_) | | | | (_| | | |_| | (_) | | | \__ \ |
    //   | |_.__/ \___/ \__,_|_| |_|\__,_|\__,_|_|   \__, |  \___\___/|_| |_|\__,_|_|\__|_|\___/|_| |_|___/ |
    //   |                                           |___/                                                  |
    //   +--------------------------------------------------------------------------------------------------+

    virtual integer boundaryConditions ( integer const i_phase,
                          real const initial_state_control[],
                          real const final_state_control[],
                          real const parameters[],
                          real        __zeta_i,
                          real        __zeta_f,
                          real       values[] ) const;

    virtual integer boundaryConditionsJac ( integer const i_phase,
                                 real const initial_state_control[],
                                 real const final_state_control[],
                                 real const parameters[],
                                 real        __zeta_i,
                                 real        __zeta_f,
                                 real       jac_xu_init[],
                                 real       jac_xu_fin[],
                                 real       jac_p[] ) const;

    virtual integer boundaryConditionsJacXuInitNnz ( integer const i_phase ) const;
    virtual void boundaryConditionsJacXuInitPattern ( integer const i_phase,
                                        integer rows[],
                                        integer cols[] ) const;

    virtual integer boundaryConditionsJacXuFinNnz ( integer const i_phase ) const;
    virtual void boundaryConditionsJacXuFinPattern ( integer const i_phase,
                                        integer rows[],
                                        integer cols[] ) const;

    virtual integer boundaryConditionsJacPNnz ( integer const i_phase ) const;
    virtual void boundaryConditionsJacPPattern ( integer const i_phase,
                                   integer       rows[],
                                   integer       cols[] ) const;

    virtual integer boundaryConditionsHess ( integer const i_phase,
                                      real const initial_state_control[],
                                      real const final_state_control[],
                                      real const parameters[],
                                      real        __zeta_i,
                                      real        __zeta_f,
                                      real const lambda[],
                                      real       hess_xu_init_xu_init[],
                                      real       hess_xu_init_xu_fin[],
                                      real       hess_xu_init_p[],
                                      real       hess_xu_fin_xu_fin[],
                                      real       hess_xu_fin_p[],
                                      real       hess_p_p[] ) const;

    virtual integer boundaryConditionsHessXuInitXuInitNnz ( integer const i_phase ) const;
    virtual void boundaryConditionsHessXuInitXuInitPattern ( integer const i_phase,
                                              integer rows[],
                                              integer cols[] ) const;

    virtual integer boundaryConditionsHessXuInitXuFinNnz ( integer const i_phase ) const;
    virtual void boundaryConditionsHessXuInitXuFinPattern ( integer const i_phase,
                                              integer rows[],
                                              integer cols[] ) const;

    virtual integer boundaryConditionsHessXuInitPNnz ( integer const i_phase ) const;
    virtual void boundaryConditionsHessXuInitPPattern ( integer const i_phase,
                                         integer       rows[],
                                         integer       cols[] ) const;

    virtual integer boundaryConditionsHessXuFinXuFinNnz ( integer const i_phase ) const;
    virtual void boundaryConditionsHessXuFinXuFinPattern ( integer const i_phase,
                                              integer rows[],
                                              integer cols[] ) const;

    virtual integer boundaryConditionsHessXuFinPNnz ( integer const i_phase ) const;
    virtual void boundaryConditionsHessXuFinPPattern ( integer const i_phase,
                                      integer       rows[],
                                      integer       cols[] ) const;

    virtual integer boundaryConditionsHessPPNnz ( integer const i_phase ) const;
    virtual void boundaryConditionsHessPPPattern ( integer const i_phase,
                                        integer rows[],
                                        integer cols[] ) const;

    // +--------------------------------------------------------------------------------+
    // |                       _                         _             _       _        |
    // |   _____   _____ _ __ | |_    ___ ___  _ __  ___| |_ _ __ __ _(_)_ __ | |_ ___  |
    // |  / _ \ \ / / _ \ '_ \| __|  / __/ _ \| '_ \/ __| __| '__/ _` | | '_ \| __/ __| |
    // | |  __/\ V /  __/ | | | |_  | (_| (_) | | | \__ \ |_| | | (_| | | | | | |_\__ \ |
    // |  \___| \_/ \___|_| |_|\__|  \___\___/|_| |_|___/\__|_|  \__,_|_|_| |_|\__|___/ |
    // |                                                                                |
    // +--------------------------------------------------------------------------------+


    virtual integer eventConstraints ( integer const i_phase,
                          real const left_state_control[],
                          real const right_state_control[],
                          real const parameters[],
                          real const __zeta_l,
                          real const __zeta_r,
                          real       values[] ) const;

    virtual integer eventConstraintsJac ( integer const i_phase,
                                 real const left_state_control[],
                                 real const right_state_control[],
                                 real const parameters[],
                                 real const __zeta_l,
                                 real const __zeta_r,
                                 real       jac_xu_init[],
                                 real       jac_xu_fin[],
                                 real       jac_p[] ) const;

    virtual integer eventConstraintsJacXuInitNnz ( integer const i_phase ) const;
    virtual void eventConstraintsJacXuInitPattern ( integer const i_phase,
                                        integer rows[],
                                        integer cols[] ) const;

    virtual integer eventConstraintsJacXuFinNnz ( integer const i_phase ) const;
    virtual void eventConstraintsJacXuFinPattern ( integer const i_phase,
                                        integer rows[],
                                        integer cols[] ) const;

    virtual integer eventConstraintsJacPNnz ( integer const i_phase ) const;
    virtual void eventConstraintsJacPPattern ( integer const i_phase,
                                   integer       rows[],
                                   integer       cols[] ) const;

    virtual integer eventConstraintsHess ( integer const i_phase,
                                      real const left_state_control[],
                                      real const right_state_control[],
                                      real const parameters[],
                                      real const __zeta_l,
                                      real const __zeta_r,
                                      real const lambda[],
                                      real       hess_xu_init_xu_init[],
                                      real       hess_xu_init_xu_fin[],
                                      real       hess_xu_init_p[],
                                      real       hess_xu_fin_xu_fin[],
                                      real       hess_xu_fin_p[],
                                      real       hess_p_p[] ) const;

    virtual integer eventConstraintsHessXuInitXuInitNnz ( integer const i_phase ) const;
    virtual void eventConstraintsHessXuInitXuInitPattern ( integer const i_phase,
                                              integer rows[],
                                              integer cols[] ) const;

    virtual integer eventConstraintsHessXuInitXuFinNnz ( integer const i_phase ) const;
    virtual void eventConstraintsHessXuInitXuFinPattern ( integer const i_phase,
                                              integer rows[],
                                              integer cols[] ) const;

    virtual integer eventConstraintsHessXuInitPNnz ( integer const i_phase ) const;
    virtual void eventConstraintsHessXuInitPPattern ( integer const i_phase,
                                         integer       rows[],
                                         integer       cols[] ) const;

    virtual integer eventConstraintsHessXuFinXuFinNnz ( integer const i_phase ) const;
    virtual void eventConstraintsHessXuFinXuFinPattern ( integer const i_phase,
                                              integer rows[],
                                              integer cols[] ) const;

    virtual integer eventConstraintsHessXuFinPNnz ( integer const i_phase ) const;
    virtual void eventConstraintsHessXuFinPPattern ( integer const i_phase,
                                      integer       rows[],
                                      integer       cols[] ) const;

    virtual integer eventConstraintsHessPPNnz ( integer const i_phase ) const;
    virtual void eventConstraintsHessPPPattern ( integer const i_phase,
                                        integer rows[],
                                        integer cols[] ) const;

    // +------------------------------------------------------------------------+
    // |                  _                                      _              |
    // |  _ __   ___  ___| |_   _ __  _ __ ___   ___ ___ ___ ___(_)_ __   __ _  |
    // | | '_ \ / _ \/ __| __| | '_ \| '__/ _ \ / __/ _ Y __/ __| | '_ \ / _` | |
    // | | |_) | (_) \__ \ |_  | |_) | | | (_) | (_|  __|__ \__ \ | | | | (_| | |
    // | | .__/ \___/|___/\__| | .__/|_|  \___/ \___\___|___/___/_|_| |_|\__, | |
    // | |_|                   |_|                                       |___/  |
    // +------------------------------------------------------------------------+

    virtual integer numberOfPostProcessing( integer const i_phase ) const;
    virtual integer numberOfDifferentialPostProcessing( integer const i_phase ) const;
    virtual integer numberOfIntegralPostProcessing( integer const i_phase ) const; 
    virtual std::string postProcessingName( integer const i_phase, integer const i ) const;
    virtual std::string differentialPostProcessingName( integer const i_phase, integer const i ) const;
    virtual std::string integralPostProcessingName( integer const i_phase, integer const i ) const;

    virtual void postProcessing(integer const i_phase,
                                   real    const state_control[],
                                   real    const parameters[],
                                   real          __zeta,
                                   real          values[] ) const;

    virtual void differentialPostProcessing(integer const i_phase,
                                   real    const state_control[],
                                   real    const state_control_derivative[],
                                   real    const algebraic_state_control[],
                                   real    const parameters[],
                                   real          __zeta,
                                   real          values[] ) const;

    virtual void integralPostProcessing(integer const i_phase,
                                           real    const state_control[],
                                           real    const state_control_derivative[],
                                           real    const algebraic_state_control[],
                                           real    const parameters[],
                                           real          __zeta,
                                           real          values[] ) const;

  } ;
}

#endif
