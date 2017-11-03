#include "NlpSolution.hh"

using namespace Maverick;

NlpSolution::NlpSolution() {
    clear();
}

NlpSolution::NlpSolution( NlpSolution const & nlp_solution ) {
    setSolution(nlp_solution.getNlpSize(), nlp_solution.getY().data(), nlp_solution.getUpperBoundsMultiplier().data(), nlp_solution.getLowerBoundsMultiplier().data(),
                nlp_solution.getNlpConstraintsSize(), nlp_solution.getConstraints().data(), nlp_solution.getConstraintsMultipliers().data(),
//                nlp_solution.getLagrangeMayerPercentage(),
                nlp_solution.getSolverReturnStatus());
}

NlpSolution::NlpSolution( Nlp const & nlp, integer const solver_return_status ) {
    setSolution(nlp.getNlpSize(), nlp.getY().data(), nlp.getUpperBoundsMultiplier().data(), nlp.getLowerBoundsMultiplier().data(),
                nlp.getNlpConstraintsSize(), nlp.getConstraints().data(), nlp.getConstraintsMultipliers().data(),
//                nlp.getLagrangeMayerPercentage(),
                solver_return_status);
}

void NlpSolution::setSolution(size const n_y, real const y[], real const upper_bounds_multipliers[], real const lower_bounds_multipliers[],
                              size const n_c, real const constraints[], real const constraints_multiplier[],
//                              real const lagrange_mayer_percentage,
                              integer const solver_return_status) {
    Nlp::setNlp(n_y, y, upper_bounds_multipliers, lower_bounds_multipliers,
                n_c, constraints, constraints_multiplier);//, lagrange_mayer_percentage);
    setSolverReturnStatus( solver_return_status );
}

integer NlpSolution::getSolverReturnStatus() const {
    return _solver_return_status;
}

void NlpSolution::setSolverReturnStatus( integer const status ) {
    _solver_return_status = status;
}

void NlpSolution::clear() {
    Nlp::clear();
    _solver_return_status = 0;
}
