/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef MAVERICK_NLP_HH
#define MAVERICK_NLP_HH

#include "MaverickDefinitions.hh"

namespace Maverick {

  class Nlp {

  public:

    Nlp();

    // setter

    void
    setNlp(size const n_y, real const y[], real const upper_bounds_multipliers[], real const lower_bounds_multipliers[],
           size const n_c, real const constraints[], real const constraints_multipliers[]);

    void setYAndBoundsMultiplier(size const n_y,
                                 real const y[],
                                 real const upper_bounds_multipliers[],
                                 real const lower_bounds_multipliers[]);

    void setConstraintsAndMultipliers(size const n_c, real const constraints[], real const constraints_multipliers[]);

    //getter

    inline size getNlpSize() const { return _y.size(); }

    inline size getNlpConstraintsSize() const { return _constraints.size(); }

    vec_1d_real const &getY() const { return _y; }

    vec_1d_real const &getLowerBoundsMultiplier() const { return _z_l; }

    vec_1d_real const &getUpperBoundsMultiplier() const { return _z_u; }

    vec_1d_real const &getConstraints() const { return _constraints; }

    vec_1d_real const &getConstraintsMultipliers() const { return _constraints_multipliers; }

    // clear

    virtual void clear();

  protected:

    vec_1d_real _y;
    vec_1d_real _z_l;
    vec_1d_real _z_u;
    vec_1d_real _constraints;
    vec_1d_real _constraints_multipliers;

  };
}

#endif
