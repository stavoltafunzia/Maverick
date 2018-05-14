/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef MAVERICK_MIDPOINT_MESH_SINGLE_PHASE_HH
#define MAVERICK_MIDPOINT_MESH_SINGLE_PHASE_HH

#include "MaverickCore/MeshSinglePhase.hh"

namespace Maverick {

  class MidpointMeshSinglePhase : public MeshSinglePhase {

    class Segment {
    public:

      enum SegmentType {
        NumPoints = 0,
        GridSize
      };

      Segment(real const length, real const grid_size);

      Segment(real const length, integer const num_points);

      real const _length = 0;

      real const _num_intervals = 0;

      real const _grid_size = 0;

      SegmentType const _segment_type;

    private:

      Segment();

    };

  public:

    MidpointMeshSinglePhase();

    MidpointMeshSinglePhase(MidpointMeshSinglePhase const &mesh);

    ~MidpointMeshSinglePhase();

    // MeshSinglePhase interface

    virtual real getInitialZeta() const;

    virtual real getFinalZeta() const;

    virtual integer getNumberOfDiscretisationPoints() const;

    virtual vec_1d_real const &getDiscretisationPoints() const;

    virtual void setup(GC::GenericContainer const &mesh_data);

    virtual std::unique_ptr<MeshSinglePhase> copy() const;

    virtual void writeContentToGC(GC::GenericContainer &out_gc) const;

    // additional methods

    void setDiscretisationPoints(vec_1d_real const &zeta);

    inline integer getNumberOfIntervals() const { return getNumberOfDiscretisationPoints() - 1; }

    real getZeta(integer const mesh_point_index) const;

    real getZetaLeft(integer const mesh_interval_index) const;

    real getZetaRight(integer const mesh_interval_index) const;

    real getZetaCenter(integer const mesh_interval_index) const;

    real getDz(integer const mesh_interval_index) const;

    real getDzDual(integer const mesh_interval_index) const;

    void addSegment(Segment const segment);

    void setMeshBegin(real zeta_0);

    real getExpectedMeshBegin() const;

    // operators

    MidpointMeshSinglePhase &operator=(MidpointMeshSinglePhase const &mesh);

    MidpointMeshSinglePhase &operator<<(Segment const &segment);

    MidpointMeshSinglePhase &operator<<(MidpointMeshSinglePhase const &mesh);

  protected:

    bool isVectorIncreasingValues(real const values[], integer const length) const;

    vec_1d_real _zeta = {};

  };
}

#endif
