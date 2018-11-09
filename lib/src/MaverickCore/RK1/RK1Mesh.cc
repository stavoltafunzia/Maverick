#include "RK1Mesh.hh"
#include "MaverickCore/MaverickSingleton.hh"
#include "MaverickCore/RK1/RK1Ocp2NlpSinglePhase.hh"
#include "MaverickCore/RK1/RK1MeshSolutionRefiner.hh"

using namespace Maverick;
using namespace std;

#define MESH_DEFAULT_MAX_NEW_POINTS 5
#define MESH_DEFAULT_LOG_FACTOR 7.0
#define MESH_DEFAULT_ALPHA 0.5

RK1Mesh::RK1Mesh() : Mesh() {
  _default_alpha = MESH_DEFAULT_ALPHA;
  _max_new_points = MESH_DEFAULT_MAX_NEW_POINTS;
  _new_points_log_factor = MESH_DEFAULT_LOG_FACTOR;
}

RK1Mesh::RK1Mesh(RK1Mesh const &mesh) : Mesh() {
  RK1Mesh::copy(mesh);
}

RK1Mesh::~RK1Mesh() {}

void RK1Mesh::setup(GC::GenericContainer const &gc_mesh) {
  RK1Mesh::clear();
  Mesh::setup(gc_mesh);
  MaverickSingleton const &maverick = MaverickSingleton::getInstance();

  real tmp;
  
  // get default alpha
  if (gc_mesh.get_if_exists("alpha", tmp)) {
    if ((tmp < 0) || (tmp > 1)) {
      _default_alpha = MESH_DEFAULT_ALPHA;
      stringstream ss;
      ss << "RK1Mesh: alpha must be comprised between 0 and 1. Will use the defualt one: "
        << _default_alpha << "\n";
      maverick.Log(InfoLevel::info_level_warning, ss.str());
    } else {
      _default_alpha = tmp;
      stringstream ss;
      ss << "RK1Mesh: found alpha: " << _default_alpha << "\n";
      maverick.Log(InfoLevel::info_level_verbose, ss.str());
    }
  } else {
    _default_alpha = MESH_DEFAULT_ALPHA;
    stringstream ss;
    ss << "RK1Mesh: alpha not declared. Will use the defualt one: " << _default_alpha << "\n";
    maverick.Log(InfoLevel::info_level_verbose, ss.str());
  }
  
  if (gc_mesh.get_if_exists("log_factor", tmp)) {
    if (tmp <= 0) {
      _new_points_log_factor = MESH_DEFAULT_LOG_FACTOR;
      stringstream ss;
      ss << "RK1Mesh: non positive log factor is not allowed. Will use the defualt one: " << std::scientific
         << _new_points_log_factor << "\n";
      maverick.Log(InfoLevel::info_level_warning, ss.str());
    } else {
      _new_points_log_factor = tmp;
    }
  }
  integer tmp_i;
  if (gc_mesh.get_if_exists("max_new_points", tmp_i)) {
    if (tmp_i < 1) {
      _max_new_points = MESH_DEFAULT_MAX_NEW_POINTS;
      stringstream ss;
      ss << "RK1Mesh: non positive number of mesh points is not allowed. Will use the default one: "
         << _max_new_points << "\n";
      maverick.Log(InfoLevel::info_level_warning, ss.str());
    } else {
      _max_new_points = tmp_i;
    }
  }

  // now build each mesh single phase
  integer i_phase = 0;
  while (true) {
    GC::GenericContainer const *gc = nullptr;
    try {
      gc = &(gc_mesh("Phase" + std::to_string(i_phase)));
    } catch (runtime_error &err) {
      break;
    }

    try {
      real previous_mesh_last_zeta = 0;
      if (_meshes.size() > 0)
        previous_mesh_last_zeta = _meshes.back().getFinalZeta();
      _meshes.emplace_back(RK1MeshSinglePhase(_default_alpha));
      _meshes.back().setMeshBegin(previous_mesh_last_zeta);
      _meshes.back().setup(*gc);
    } catch (runtime_error &err) {
      stringstream ss;
      ss << "Maverick mesh: phase " << i_phase << ", " << err.what();
      throw runtime_error(ss.str());
    }
    i_phase++;
  }

  if (i_phase == 0) { // in this case, no mesh has been provided. If the problem is single phase, then the mesh may be declared as root
    try {
      _meshes.push_back(RK1MeshSinglePhase(_default_alpha));
      _meshes.back().setup(gc_mesh);
    } catch (runtime_error &err) {
      stringstream ss;
      ss << "Maverick mesh: phase 0, " << err.what();
      throw runtime_error(ss.str());
    }
  }
}

void RK1Mesh::clear() {
  Mesh::clear();
  _max_new_points = MESH_DEFAULT_MAX_NEW_POINTS;
  _new_points_log_factor = MESH_DEFAULT_LOG_FACTOR;
  _default_alpha = MESH_DEFAULT_ALPHA;
  _meshes.clear();
  _meshes.shrink_to_fit();
}

integer RK1Mesh::getNumberOfPhases() const {
  return (integer) _meshes.size();
}

unique_ptr<Mesh> RK1Mesh::copy() const {
  return unique_ptr<Mesh>(new RK1Mesh(*this));
}

unique_ptr<Ocp2Nlp> RK1Mesh::getDiscretiser(MaverickOcp const &ocp) const {
  if (getNumberOfPhases() > 1) {
    string error = "Currently only single phase problems are supported.";
    throw std::runtime_error(error);
  }
  if (getNumberOfPhases() == 0) {
    string error = "You must setup the mesh first.";
    throw std::runtime_error(error);
  }
  RK1Ocp2NlpSinglePhase *out = new RK1Ocp2NlpSinglePhase(ocp, *this, 0);
  return unique_ptr<Ocp2Nlp>(out);
}

std::unique_ptr<MeshSolutionRefiner>
RK1Mesh::getMeshSolutionRefiner(MaverickOcp const &ocp_problem, OcpScaling const &ocp_scaling) const {
  return unique_ptr<MeshSolutionRefiner>(new RK1MeshSolutionRefiner(ocp_problem, ocp_scaling));
}

// operators

MeshSinglePhase const &RK1Mesh::operator[](integer const i_phase) const {
  return this->operator()(i_phase);
}


// additional methods

void RK1Mesh::copy(RK1Mesh const &mesh) {
  clear();
  Mesh::copy(mesh);
  _max_new_points = mesh._max_new_points;
  _new_points_log_factor = mesh._new_points_log_factor;
  _default_alpha = mesh._default_alpha;
  for (integer i = 0; i < mesh.getNumberOfPhases(); i++)
    _meshes.push_back(mesh(i));
}

void RK1Mesh::setMeshForPhase(integer const i_phase, RK1MeshSinglePhase const &mesh) {
  integer additional_phases = i_phase - getNumberOfPhases() + 1;
  if (additional_phases <= 0) {
    _meshes[i_phase] = RK1MeshSinglePhase(mesh);
  } else {
    for (integer i = 0; i < additional_phases - 1; i++)
      _meshes.push_back(RK1MeshSinglePhase(_default_alpha));
    _meshes.push_back(mesh);
  }
}

// operators

RK1MeshSinglePhase const &RK1Mesh::operator()(integer const i_phase) const {
  MAVERICK_ASSERT(i_phase < getNumberOfPhases(),
                  "RK1Mesh::operator[]: phase out of bounds. Requested phase " << i_phase
                                                                                    << " when phase bounds are 0 - "
                                                                                    << getNumberOfPhases() - 1 << ".\n")
  return _meshes[i_phase];
}

RK1MeshSinglePhase &RK1Mesh::operator()(integer const i_phase) {
  MAVERICK_ASSERT(i_phase < getNumberOfPhases(),
                  "RK1Mesh::operator[]: phase out of bounds. Requested phase " << i_phase
                                                                                    << " when phase bounds are 0 - "
                                                                                    << getNumberOfPhases() - 1 << ".\n")
  return _meshes[i_phase];
}

RK1Mesh &RK1Mesh::operator=(const RK1Mesh &mesh) {
  copy(mesh);
  return *this;
}

RK1Mesh &RK1Mesh::operator<<(const RK1MeshSinglePhase &mesh) {
  _meshes.push_back(mesh);
  return *this;
}
