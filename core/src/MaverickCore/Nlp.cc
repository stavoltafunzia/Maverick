#include "Nlp.hh"
#include "MaverickFunctions.hh"

using namespace Maverick;
using namespace std;

Nlp::Nlp() {
    clear();
}

void Nlp::setNlp(size const n_y, real const y[], real const upper_bounds_multipliers[], real const lower_bounds_multipliers[],
                 size const n_c, real const constraints[], real const constraints_multipliers[]
//                 real const lagrange_mayer_percentage
                 ) {

    setYAndBoundsMultiplier( n_y, y, upper_bounds_multipliers, lower_bounds_multipliers );
    setConstraintsAndMultipliers( n_c, constraints, constraints_multipliers );
//    setLagrangeMayerPercentage(lagrange_mayer_percentage);
    
}

void Nlp::setYAndBoundsMultiplier(size const n_y,
                                  real const y[],
                                  real const upper_bounds_multipliers[],
                                  real const lower_bounds_multipliers[]
                                    ) {
    _y = vector<real>(y, y + n_y);
    _z_l = vector<real>(lower_bounds_multipliers, lower_bounds_multipliers + n_y);
    _z_u = vector<real>(upper_bounds_multipliers, upper_bounds_multipliers + n_y);
}

void Nlp::setConstraintsAndMultipliers( size const n_c, real const constraints[], real const constraints_multipliers[] ) {
    _constraints = vector<real>(constraints, constraints + n_c);
    _constraints_multipliers = vector<real>(constraints_multipliers, constraints_multipliers + n_c);
}

//void Nlp::setLagrangeMayerPercentage( real const lagrange_mayer_percentage ) {
//    _lagrange_mayer_percentage = lagrange_mayer_percentage;
//}

void Nlp::clear() {
    _y = {};
    _z_l = {};
    _z_u = {};
    _constraints = {};
    _constraints_multipliers = {};
    _lagrange_mayer_percentage = 0;
}
