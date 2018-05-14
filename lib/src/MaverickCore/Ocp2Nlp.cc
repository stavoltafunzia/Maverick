#include "Ocp2Nlp.hh"
#include "MaverickCore/MaverickFunctions.hh"
#include "MaverickCore/MaverickPrivateDefinitions.hh"

using namespace Maverick;

Ocp2Nlp::Ocp2Nlp(MaverickOcp const &ocp_problem, Mesh const &mesh) : _ocp_problem(ocp_problem), _mesh(mesh) {

  _th_affinity = {{}};

  _int_vec_pointers = {
      // target gradient related
      &_p_nlp_target_j_cols,

      // constraints jacobian pointers
      &_p_nlp_constraints_j_rows,
      &_p_nlp_constraints_j_cols,

      // hessian pointers
      &_p_nlp_hessian_rows,
      &_p_nlp_hessian_cols
  };

  _real_vec_pointers = {};
}

Ocp2Nlp::~Ocp2Nlp() {
  deleteAllDataPointers();
}

void Ocp2Nlp::deleteAllDataPointers() {

  for (integer i = 0; i < _int_vec_pointers.size(); i++) {
    if (*(_int_vec_pointers[i]) != nullptr)
      delete[] *(_int_vec_pointers[i]);
    *_int_vec_pointers[i] = nullptr;
  }

  for (integer i = 0; i < _real_vec_pointers.size(); i++) {
    if (*(_real_vec_pointers[i]) != nullptr)
      delete[] *(_real_vec_pointers[i]);
    *_real_vec_pointers[i] = nullptr;
  }

}

integer
Ocp2Nlp::getNlpTargetGradientPattern(integer col_index[], integer const col_offset, integer const length) const {
  MAVERICK_DEBUG_ASSERT(length == getNlpTargetGradientNnz(),
                        "Ocp2Nlp::getNlpTargetGradientPattern: wrong number of nnz.\n")
  MAVERICK_DEBUG_ASSERT(_p_nlp_target_j_cols != nullptr,
                        "Ocp2Nlp::getNlpTargetGradientPattern: called when _p_nlp_target_j_cols is nullptr.\n")

  sumAndWriteVectorTo(_p_nlp_target_j_cols, col_index, col_offset, length);
  return 0;
}

integer Ocp2Nlp::getNlpConstraintsJacobianPattern(integer row_index[], integer col_index[], integer const row_offset,
                                                  integer const col_offset, integer const length) const {
#ifdef MAVERICK_DEBUG
  MAVERICK_ASSERT( length == getNlpConstraintsJacobianNnz(), "Ocp2Nlp::getNlpConstraintsJacobianPattern: wrong number of nnz.\n")
  MAVERICK_ASSERT( _p_nlp_constraints_j_cols != nullptr, "Ocp2Nlp::getNlpConstraintsJacobianPattern: called when _p_nlp_constraints_j_cols is nullptr.\n")
  MAVERICK_ASSERT( _p_nlp_constraints_j_rows != nullptr, "Ocp2Nlp::getNlpConstraintsJacobianPattern: called when _p_nlp_constraints_j_rows is nullptr.\n")
#endif
  sumAndWriteVectorTo(_p_nlp_constraints_j_rows, row_index, row_offset, length);
  sumAndWriteVectorTo(_p_nlp_constraints_j_cols, col_index, col_offset, length);
  return 0;
}

integer Ocp2Nlp::getNlpHessianPattern(integer row_index[], integer col_index[], integer const row_offset,
                                      integer const col_offset, integer const length) const {
#ifdef MAVERICK_DEBUG
  MAVERICK_ASSERT( length == getNlpHessianNnz(), "Ocp2Nlp::getNlpHessianPattern: wrong number of nnz. Called " << length << ", expected " << getNlpHessianNnz() << "\n")
  MAVERICK_ASSERT( _p_nlp_hessian_rows != nullptr, "Ocp2Nlp::getNlpHessianPattern: called when _p_nlp_constraints_j_cols is nullptr.\n")
  MAVERICK_ASSERT( _p_nlp_hessian_cols != nullptr, "Ocp2Nlp::getNlpHessianPattern: called when _p_nlp_constraints_j_rows is nullptr.\n")
#endif
  sumAndWriteVectorTo(_p_nlp_hessian_rows, row_index, row_offset, length);
  sumAndWriteVectorTo(_p_nlp_hessian_cols, col_index, col_offset, length);
  return 0;
}

void Ocp2Nlp::setMinNumberOfNlpVarsPerThreads(integer const min_nlp) {
  _min_nlp_vars_per_thread = min_nlp;
  calculateWorkForThreads();
}

void Ocp2Nlp::evalNlpConstraints(Nlp &nlp) const {
  MAVERICK_ASSERT(nlp.getNlpSize() == getNlpSize(), "Ocp2Nlp::evalNlpConstraints: wrong nlp size")
  MAVERICK_ASSERT(nlp.getNlpConstraintsSize() == getNlpConstraintsSize(),
                  "Ocp2Nlp::evalNlpConstraints: wrong nlp constraints size")
  integer const n_constr = getNlpConstraintsSize();
  real *constr = new real[n_constr];
  calculateNlpQuantities(nlp.getY().data(), (integer) nlp.getNlpSize(), nullptr, 0,
                         nullptr,
                         nullptr, 0,
                         constr, n_constr,
                         nullptr, 0,
                         nullptr, 0
  );
  nlp.setConstraintsAndMultipliers(n_constr, constr, nlp.getConstraintsMultipliers().data());
  delete[] constr;
}
