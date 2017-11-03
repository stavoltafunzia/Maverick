#include "MaverickCore/MeshSolutionRefiner.hh"
#include "MaverickCore/MaverickSingleton.hh"
#include "MaverickCore/OcpScaling.hh"
#include <iomanip>
#include <thread>

#define SEP "   "
using namespace std;

namespace Maverick {


    MeshSolutionRefiner::MeshSolutionRefiner(MaverickOcp const & ocp_problem, OcpScaling const & ocp_scaling) : _ocp_problem(ocp_problem), _scaling(ocp_scaling) {}

    MeshSolutionRefiner::~MeshSolutionRefiner() {}

    void MeshSolutionRefiner::setNumThreads( integer num_threads ) {
        if (num_threads <= 0) {
            MaverickSingleton::getInstance().Log(InfoLevel::info_level_warning, "MeshSolutionRefiner::setNumThreads: cannot set a non positive thread number. Will use 1");
        }
        _num_threads_to_use = num_threads;
    }

    void MeshSolutionRefiner::setIntegratorType( EquationIntegratorType type ) {
        _integrator_type = type;
    }

}
