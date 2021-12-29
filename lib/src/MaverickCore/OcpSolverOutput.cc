#include "OcpSolverOutput.hh"
#include "MaverickFunctions.hh"
#include "MaverickCore/OcpSolverImpl.hh"

using namespace Maverick;
using namespace std;

OcpSolverOutput::OcpSolverOutput() {}

OcpSolverOutput::OcpSolverOutput(OcpSolverOutput const &output) {
  _history = output._history;
  _solution = output._solution->copy();
}

OcpSolverOutput::SingleIterationOutput::SingleIterationOutput() {}

OcpSolverOutput::SingleIterationOutput::SingleIterationOutput(SingleIterationOutput const &input) {
  start_code = input.start_code;
  return_status = input.return_status;
  mesh = input.mesh->copy();
  unscaled_target = input.unscaled_target;
  iterations = input.iterations;
  calculation_ms = input.calculation_ms;
}

OcpSolverOutput::SingleIterationOutput &
OcpSolverOutput::SingleIterationOutput::operator=(OcpSolverOutput::SingleIterationOutput const &input) {
  start_code = input.start_code;
  return_status = input.return_status;
  mesh = input.mesh->copy();
  unscaled_target = input.unscaled_target;
  iterations = input.iterations;
  calculation_ms = input.calculation_ms;
  return *this;
}

// access element at index
OcpSolverOutput::SingleIterationOutput const &OcpSolverOutput::at(integer const index) const {
  MAVERICK_ASSERT(index < getHistoryLength(),
                  "OcpSolverOutput::SingleIterationOutput::at: access index out of bounds\n");
  return _history[index];
}

// get the mesh history
vec_3d_real OcpSolverOutput::getMeshHistory() const {
  vec_3d_real output;
  for (integer i = 0; i < getHistoryLength(); i++) {
    output.push_back(_history[i].mesh->getDiscretisationPoints());
  }
  return output;
}

// get sum of all iterations
integer OcpSolverOutput::getTotalIterations() const {
  integer out = 0;
  for (std::vector<SingleIterationOutput>::const_iterator it = _history.begin();
       it != _history.end(); it++) {
    out += it->iterations;
  }
  return out;
}

// get sum of all cpu msec
unsigned long OcpSolverOutput::getTotalCalculationMs() const {
  unsigned long out = 0;
  for (std::vector<SingleIterationOutput>::const_iterator it = _history.begin();
       it != _history.end(); it++) {
    out += it->calculation_ms;
  }
  return out;
}

// concatenation operator
void OcpSolverOutput::append(SingleIterationOutput const &output, OcpSolution const &ocp_solution) {
  _history.push_back(output);
  _solution = ocp_solution.copy();
}

// assignement operator
OcpSolverOutput const &OcpSolverOutput::operator=(OcpSolverOutput const &output) {
  _history = output._history;
  _solution = output._solution->copy();
  return *this;
}

// concatenation operator
OcpSolverOutput const &OcpSolverOutput::operator<<(OcpSolverOutput const &output) {
  concatenateVectors(_history, output._history);
  _solution = output._solution->copy();
  return *this;
}

// access element operator
OcpSolverOutput::SingleIterationOutput const &OcpSolverOutput::operator[](integer const index) {
  MAVERICK_ASSERT(index < getHistoryLength(),
                  "OcpSolverOutput::SingleIterationOutput::operator []: access index out of bounds\n");
  return _history[index];
}

void OcpSolverOutput::writeContentToGC(GC::GenericContainer &out_gc, MaverickOcp const *const p_ocp) const {
  out_gc.clear();

  // total iterations
  out_gc["total_iterations"].set_real(getTotalIterations());

  // total calculation msec
  out_gc["total_calculation_ms"].set_real(getTotalCalculationMs());

  // history length
  out_gc["history_length"].set_int(getHistoryLength());

  // history detail
  GC::GenericContainer &history_gc = out_gc["history"];
  history_gc.set_vector(getHistoryLength());
  for (integer i = 0; i < getHistoryLength(); i++) {
    writeSingleIterationOutputToGC(i, history_gc[i]);
  }

  // solution
  _solution->writeContentToGC(out_gc["solution"], p_ocp);
}

void OcpSolverOutput::writeSingleIterationOutputToGC(integer const index, GC::GenericContainer &gc) const {
  gc.clear();

  SingleIterationOutput const &it_out = _history[index];

  //start code
  gc["start_code"].set_string(OcpSolverImpl::convertSolverStartCodeToString(it_out.start_code));

  // solver return status
  gc["solver_return_status"].set_string(OcpSolverImpl::convertSolverReturnStatusToString(it_out.return_status));

  // mesh
  it_out.mesh->writeContentToGC(gc["mesh"]);

  // target
  gc["target"].set_real(it_out.unscaled_target);

  // iterations
  gc["iterations"].set_int(it_out.iterations);

  // calculation msec
  // TODO put unsigned long in the GC
  gc["calculation_ms"].set_long(it_out.calculation_ms);

}

integer OcpSolverOutput::getHistoryLength() const {
  return safeCastBetweenTypes<integer, size_t>(_history.size());
}

std::shared_ptr<const OcpSolution> OcpSolverOutput::getSolution() const {
  return _solution;
}
