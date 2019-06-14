#include "RK1MeshSinglePhase.hh"
#include "MaverickCore/MaverickFunctions.hh"
#include "MaverickCore/MaverickPrivateDefinitions.hh"

#define GCMESH_NUM_INT "num_intervals"
#define GCMESH_NUM_PHASES "num_phases"
#define GCMESH_NUM_ZETA_VEC "zeta_vec"
#define GCMESH_NUM_INT "num_intervals"

#define _number_of_mesh_points _zeta.size()
using namespace std;

namespace Maverick {

  RK1MeshSinglePhase::RK1MeshSinglePhase(real const default_alpha) {
    _default_alpha = default_alpha;
    _alpha = _default_alpha;
  }

  RK1MeshSinglePhase::RK1MeshSinglePhase(RK1MeshSinglePhase const &mesh) {
    _zeta = mesh.getDiscretisationPoints();
    _alpha = mesh._alpha;
    _default_alpha = mesh._default_alpha;
  }

  RK1MeshSinglePhase::RK1MeshSinglePhase(RK1MeshSinglePhase && mesh) {
    _alpha = mesh._alpha;
    _zeta = std::move(mesh._zeta);
    _default_alpha = mesh._default_alpha;
  }

  RK1MeshSinglePhase::~RK1MeshSinglePhase() {}

  // RK1MeshSinglePhase interface

  real RK1MeshSinglePhase::getInitialZeta() const {
    MAVERICK_SKIPABLE_ASSERT(_number_of_mesh_points > 0,
                             "RK1MeshSinglePhase::getInitialZeta: mesh not initialized yet")
    return _zeta.front();
  }

  real RK1MeshSinglePhase::getFinalZeta() const {
    MAVERICK_SKIPABLE_ASSERT(_number_of_mesh_points > 0,
                             "RK1MeshSinglePhase::getFinalZeta: mesh not initialized yet")
    return _zeta.back();
  }

  integer RK1MeshSinglePhase::getNumberOfDiscretisationPoints() const {
    return (integer) _zeta.size();
  }

  vec_1d_real const &RK1MeshSinglePhase::getDiscretisationPoints() const {
    return _zeta;
  }

  void RK1MeshSinglePhase::setup(GC::GenericContainer const &mesh_data) {
    //first check if zeta points are explicitely declared
    bool are_zeta_points_declared = findVecRealFromGenericContainer(mesh_data, "MeshPoints", _zeta);
    bool are_segments_declared = false;

    if (!are_zeta_points_declared) { //if zeta points are NOT explicitely declared, check for segments

      GC::GenericContainer const *gc_segments = nullptr;
      try {
        gc_segments = &(mesh_data("Segments"));
        are_segments_declared = true;
      } catch (runtime_error &err) {}

      if (are_segments_declared) {
        // first, find if the begin of the mesh is specified
        {
          real mesh_zeta0 = getExpectedMeshBegin();
          findRealFromGenericContainer(mesh_data, "initial_zeta", mesh_zeta0);
          setMeshBegin(mesh_zeta0);
        }
        
        // find the value of alpha
        {
          _alpha = _default_alpha;
          findRealFromGenericContainer(mesh_data, "alpha", _alpha);
        }

        // now build the segments
        unsigned num_el = gc_segments->get_num_elements();
        for (unsigned i_seg = 0; i_seg < num_el; i_seg++) {
          GC::GenericContainer const &segment = (*gc_segments)(i_seg);
          // get length
          real length;
          MAVERICK_ASSERT(findRealFromGenericContainer(segment, "length", length),
                          "length of segment not specified for segment at index " << i_seg << ".\n")

          integer num_intervals;
          bool are_num_intervals_declared = findIntFromGenericContainer(segment, "num_intervals", num_intervals);

          integer num_points;
          bool are_num_points_declared = findIntFromGenericContainer(segment, "num_points", num_points);

          real grid_size;
          bool is_grid_size_declared = findRealFromGenericContainer(segment, "grid_size", grid_size);

          bool are_points_or_intervals_declared = (are_num_intervals_declared || are_num_points_declared);
          MAVERICK_ASSERT(!(are_points_or_intervals_declared && is_grid_size_declared),
                          "both number of intervals (or points) and grid size are specified for segment at index "
                              << i_seg << ". Forbidden.\n")

          MAVERICK_ASSERT((are_points_or_intervals_declared || is_grid_size_declared),
                          "neither number of intervals (or points) nor grid size are specified for segment at index "
                              << i_seg << ". You must supply at least one.\n")

          if (are_points_or_intervals_declared) {
            if (are_num_points_declared && are_num_intervals_declared) {
              MAVERICK_ASSERT(num_intervals == num_points - 1,
                              "both number of intervals and number of points points are specified for segment at index "
                                  << i_seg << ", but num_points != num_intervals+1. Forbidden.\n")
            }
            if (are_num_points_declared)
              num_intervals = num_points - 1;
            MAVERICK_ASSERT(num_intervals > 0,
                            "number of intervals must be greater than zero for segment at index " << i_seg << ".\n")
            addSegment(Segment(length, num_intervals));
          } else {
            MAVERICK_ASSERT(grid_size > 0,
                            "segment grid size must be greater than zero for segment at index " << i_seg << ".\n")
            addSegment(Segment(length, grid_size));
          }
        }
      }
    }

    MAVERICK_ASSERT(are_zeta_points_declared || are_segments_declared,
                    "Mesh: either mesh points or segments must be supplied.\n")
    MAVERICK_ASSERT(isVectorIncreasingValues(_zeta.data(), (integer) _zeta.size()),
                    "Mesh: non monotonically increasing indipendent coordinate.\n")
    MAVERICK_ASSERT(getNumberOfDiscretisationPoints() > 2, "Mesh: number of mesh points must be grater than two.\n")
  }

  unique_ptr<MeshSinglePhase> RK1MeshSinglePhase::copy() const {
    return unique_ptr<MeshSinglePhase>(new RK1MeshSinglePhase(*this));
  }

  void RK1MeshSinglePhase::writeContentToGC(GC::GenericContainer &out_gc) const {
    out_gc["zeta"].set_vec_real(_zeta);
  }

  // additional methods

  void RK1MeshSinglePhase::setDiscretisationPoints(vec_1d_real const &zeta) {
    _zeta = zeta;
  }

  real RK1MeshSinglePhase::getZetaAtIndex(integer const mesh_point_index) const {
    MAVERICK_SKIPABLE_ASSERT(mesh_point_index < _number_of_mesh_points,
                             "RK1MeshSinglePhase::getZeta: number of mesh point " << mesh_point_index
                                                                                       << " greater than "
                                                                                       << _number_of_mesh_points - 1
                                                                                       << ".")
    return _zeta[mesh_point_index];
  }

  real RK1MeshSinglePhase::getZetaLeft(integer const mesh_interval_index) const {
    MAVERICK_SKIPABLE_ASSERT(mesh_interval_index < _number_of_mesh_points - 1,
                             "RK1MeshSinglePhase::getZetaLeft: number of mesh interval " << mesh_interval_index
                                                                                              << " greater than " <<
                                                                                              _number_of_mesh_points - 2
                                                                                              << ".")
    return _zeta[mesh_interval_index];
  }

  real RK1MeshSinglePhase::getZetaRight(integer const mesh_interval_index) const {
    MAVERICK_SKIPABLE_ASSERT(mesh_interval_index < _number_of_mesh_points - 1,
                             "RK1MeshSinglePhase::getZetaRight: number of mesh interval " << mesh_interval_index
                                                                                               << " greater than " <<
                                                                                               _number_of_mesh_points -
                                                                                               2 << ".")
    return _zeta[mesh_interval_index + 1];
  }

  real RK1MeshSinglePhase::getZetaAlpha(integer const mesh_interval_index) const {
    MAVERICK_SKIPABLE_ASSERT(mesh_interval_index < _number_of_mesh_points - 1,
                             "RK1MeshSinglePhase::getZetaCenter: number of mesh interval " << mesh_interval_index
                                                                                                << " greater than " <<
                                                                                                _number_of_mesh_points -
                                                                                                2 << ".")
    return _zeta[mesh_interval_index] * (1 - _alpha) + _zeta[mesh_interval_index + 1] * _alpha;
  }

  real RK1MeshSinglePhase::getDz(integer const mesh_interval_index) const {
    MAVERICK_SKIPABLE_ASSERT(mesh_interval_index < _number_of_mesh_points - 1,
                             "RK1MeshSinglePhase::getDz: number of mesh interval " << mesh_interval_index
                                                                                        << " greater than "
                                                                                        << _number_of_mesh_points - 2
                                                                                        << ".")
    return (_zeta[mesh_interval_index + 1] - _zeta[mesh_interval_index]);
  }

  real RK1MeshSinglePhase::getDzAverageAtIndex(integer const mesh_point_index) const {
    MAVERICK_SKIPABLE_ASSERT(mesh_point_index < _number_of_mesh_points,
                             "RK1MeshSinglePhase::getDzDual: number of mesh point " << mesh_point_index
                                                                                         << " greater than "
                                                                                         << _number_of_mesh_points
                                                                                         << ".")
    if (mesh_point_index == 0)
      return (_zeta[1] - _zeta[0]);
    if (mesh_point_index == _number_of_mesh_points - 1)
      return (_zeta[_number_of_mesh_points - 1] - _zeta[_number_of_mesh_points - 2]);
    return (_zeta[mesh_point_index + 1] - _zeta[mesh_point_index - 1]) / 2.0;
  }

  void RK1MeshSinglePhase::addSegment(Segment const segment) {
    if (segment._segment_type == Segment::SegmentType::NumPoints) {
      integer num_intervals = segment._num_intervals;
      if (_number_of_mesh_points == 0) {
        _zeta.push_back(0);
      }
      real grid_size = segment._length / num_intervals;
      real last_zeta = _zeta.back();
      for (integer i = 0; i < num_intervals - 1; i++)
        _zeta.push_back(_zeta.back() + grid_size);
      _zeta.push_back(last_zeta + segment._length);
    }
    if (segment._segment_type == Segment::SegmentType::GridSize) {
      if (_number_of_mesh_points == 0) {
        _zeta.push_back(0);
      }
      real last_zeta = _zeta.back();
      integer const num_intervals = ceil(segment._length / segment._grid_size);
      for (integer i = 0; i < num_intervals - 1; i++)
        _zeta.push_back(_zeta.back() + segment._grid_size);
      _zeta.push_back(last_zeta + segment._length);
    }
  }

  void RK1MeshSinglePhase::setMeshBegin(real zeta_0) {
    if (_number_of_mesh_points == 0) {
      _zeta.push_back(zeta_0);
    } else {
      _zeta[0] = zeta_0;
    }
  }

  real RK1MeshSinglePhase::getExpectedMeshBegin() const {
    if (_number_of_mesh_points == 0)
      return 0;
    return _zeta[0];
  }
  
  RK1MeshSinglePhase &RK1MeshSinglePhase::operator=(RK1MeshSinglePhase const & mesh) {
    _alpha = mesh._alpha;
    _zeta = mesh._zeta;
    _default_alpha = mesh._default_alpha;
    return *this;
  }
  
  RK1MeshSinglePhase &RK1MeshSinglePhase::operator=(RK1MeshSinglePhase && mesh) {
    _alpha = mesh._alpha;
    _zeta = std::move(mesh._zeta);
    _default_alpha = mesh._default_alpha;
    return *this;
  }
  
  RK1MeshSinglePhase &RK1MeshSinglePhase::operator<<(Segment const &segment) {
    addSegment(segment);
    return *this;
  }

  RK1MeshSinglePhase &RK1MeshSinglePhase::operator<<(RK1MeshSinglePhase const &mesh) {
    vector<real> mesh_2_add = mesh.getDiscretisationPoints();
    MAVERICK_ASSERT(isVectorIncreasingValues(mesh_2_add.data(), (integer) mesh_2_add.size()),
                    "Mesh::operator << : non monotonically increasing zeta.\n")
    real const zeta_offset = *(_zeta.end() - 1) - *(mesh_2_add.end() - 1);
    for (integer i = 1; i < mesh_2_add.size(); i++)
      _zeta.push_back(mesh_2_add[i] + zeta_offset);
    return *this;
  }

  RK1MeshSinglePhase::Segment::Segment(real const length, real const grid_size) : _length(length),
                                                                                       _num_intervals(0),
                                                                                       _grid_size(grid_size),
                                                                                       _segment_type(
                                                                                           RK1MeshSinglePhase::Segment::SegmentType::GridSize) {
    MAVERICK_ASSERT(_length > 0, "RK1MeshSinglePhase::Segment: nonpositive segment length not allowed.\n")
    MAVERICK_ASSERT(_grid_size > 0, "RK1MeshSinglePhase::Segment: nonpositive segment grid size not allowed.\n")
  }

  RK1MeshSinglePhase::Segment::Segment(real const length, integer const num_intervals) : _length(length),
                                                                                              _num_intervals(num_intervals),
                                                                                              _grid_size(0),
                                                                                              _segment_type(
                                                                                                  RK1MeshSinglePhase::Segment::SegmentType::NumPoints) {
    MAVERICK_ASSERT(_length > 0, "RK1MeshSinglePhase::Segment: nonpositive segment length not allowed.\n")
    MAVERICK_ASSERT(_num_intervals > 0,
                    "RK1MeshSinglePhase::Segment: nonpositive segment number of intervals not allowed.\n")
  }

  bool RK1MeshSinglePhase::isVectorIncreasingValues(real const values[], integer const length) const {
    for (integer i = 0; i < length - 1; i++) {
      if (values[i + 1] <= values[i]) return false;
    }
    return true;
  }

};
