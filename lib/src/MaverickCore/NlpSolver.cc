#include "NlpSolver.hh"

using namespace Maverick;
using namespace std;


NlpSolver::NlpSolver() {
}

NlpSolver::~NlpSolver() {
}

NlpSolver::NlpSolver(std::shared_ptr<const Ocp2Nlp> ocp_2_nlp) {
  _p_ocp_2_nlp = ocp_2_nlp;
}

void NlpSolver::setOcp2Nlp(std::shared_ptr<const Ocp2Nlp> ocp_2_nlp) {
  _p_ocp_2_nlp = ocp_2_nlp;
  setOcp2NlpDerived(ocp_2_nlp);
}
