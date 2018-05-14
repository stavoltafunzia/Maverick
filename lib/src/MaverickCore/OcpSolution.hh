/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef OCP_SOLUTION_HH
#define OCP_SOLUTION_HH

#include "MaverickCore/OcpSolutionSinglePhase.hh"
#include "MaverickCore/OcpGuess.hh"
#include "MaverickCore/Mesh.hh"
#include <iostream>

namespace Maverick {

  class OcpSolution : public OcpGuess {

  public:

    OcpSolution();

    // destructor
    virtual ~OcpSolution() {}

    virtual Mesh::DiscretisationType discretisationType() const = 0;

    // full evaluation
    virtual void evalAtMesh(integer const i_phase,
                            real zeta,

                            integer const num_states_controls, real *states_controls,
                            real *states_controls_upper_bounds_mult, real *states_controls_lower_bounds_mult,
                            integer const num_alg_states_controls, real *algebraic_states_controls,
                            real *algebraic_states_controls_upper_bounds_mult,
                            real *algebraic_states_controls_lower_bounds_mult,
                            integer const num_fo_eqns, real *fo_eqns, real *fo_eqns_mult,
                            integer const num_point_constr, real *states_constr, real *point_constr_mult,
                            integer const num_path_constr, real *path_constr, real *path_constr_mult,
                            integer const num_int_constr, real *int_constr,

                            integer const num_post_proc, real *post_proc,
                            integer const num_diff_post_proc, real *diff_post_proc,
                            integer const num_int_post_proc, real *int_post_proc
    ) const = 0;

    virtual void eval(integer const i_phase,
                      real const initial_zeta, real const final_zeta,

                      integer const num_parameters, real *parameters, real *params_upper_bounds_mult,
                      real *params_lower_bounds_mult,
                      integer const num_boundary_conditions, real *boundary_conditions, real *boundary_conditions_mult,
                      integer const num_int_constr, real *int_constr_at_end, real *int_constr_mult,
                      integer const num_int_post_proc, real *int_post_proc_at_end
    ) const = 0;

    // get number of phases
    virtual integer getNumberOfPhases() const = 0;

    // get the overall target
    virtual real getTarget() const = 0;

    virtual void clear() = 0;

    // output

    /* write content to GenericContainer
      If the MaverickOcp pointer is null, states, controls, equations, etc will be saved
      in vectors (i.e. state vector at i-th element has state[i]).
      If the MaverickOcp pointer is NOT null, states, controls, equations, etc will be saved
      with their specific names read from the MaverickOcp object. */
    virtual void writeContentToGC(GC::GenericContainer &out_gc, MaverickOcp const *const p_ocp) const = 0;

    virtual void writeAllMeshVarsToStream(std::ostream &out) const = 0;

    virtual void writeOnePhaseMeshVarsToStream(integer const i_phase, std::ostream &out) const = 0;

    // get a copy
    virtual std::unique_ptr<OcpSolution> copy() const = 0;

    // opertators

    virtual OcpSolutionSinglePhase const &operator[](integer const i_phase) const = 0;

  };
}

#endif
