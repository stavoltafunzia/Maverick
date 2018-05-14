/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef MAVERICK_MESH_SINGLE_PHASE_HH
#define MAVERICK_MESH_SINGLE_PHASE_HH

#include "MaverickGC/GenericContainer.hh"
#include "MaverickCore/MaverickDefinitions.hh"
#include <memory>

namespace Maverick {

  class MeshSinglePhase {

  public:

    MeshSinglePhase();

    virtual ~MeshSinglePhase();

    virtual real getInitialZeta() const = 0;

    virtual real getFinalZeta() const = 0;

    virtual integer getNumberOfDiscretisationPoints() const = 0;

    virtual vec_1d_real const &getDiscretisationPoints() const = 0;

    virtual void setup(GC::GenericContainer const &mesh_data) = 0;

    virtual std::unique_ptr<MeshSinglePhase> copy() const = 0;

    virtual void writeContentToGC(GC::GenericContainer &out_gc) const = 0;

  };
}

#endif
