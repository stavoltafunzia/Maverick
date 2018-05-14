#include "MaverickCore/MeshSolutionRefiner.hh"
#include "MaverickCore/MaverickSingleton.hh"

#define SEP "   "
using namespace std;

namespace Maverick {


  MeshSolutionRefiner::MeshSolutionRefiner(MaverickOcp const &ocp_problem, OcpScaling const &ocp_scaling)
      : _ocp_problem(ocp_problem), _scaling(ocp_scaling) {}

  MeshSolutionRefiner::~MeshSolutionRefiner() {}

  void MeshSolutionRefiner::setThreadsAffinity(threads_affinity const &th_affinity) {
    _th_affinity = th_affinity;
    if (_th_affinity.size() == 0) {
      MaverickSingleton::getInstance().Log(InfoLevel::info_level_warning,
                                           "MeshSolutionRefiner::setThreadsAffinity: cannot set a zero thread number. Will use 1");
      _th_affinity = {{}};
    }
  }

  void MeshSolutionRefiner::setIntegratorType(EquationIntegratorType type) {
    _integrator_type = type;
  }

}
