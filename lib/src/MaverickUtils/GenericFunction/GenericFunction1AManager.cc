#include "GenericFunction1AManager.hh"
#include "GF1ASpline.hh"
#include "GF1APolyFive.hh"
#include "GF1ASinAtan.hh"
#include "RegPosPartSqrt.hh"
#include "RegNegPartSqrt.hh"
#include "RegAbsValSqrt.hh"
#include "RegSignSinAtan.hh"

#define POLYFIVE "1arg_poly_five"
#define SPLINE "1arg_spline"
// #define FUNCTION_1ARG_ATAN "1arg_atan"
#define SIN_ATAN "1arg_sin_atan"
// #define FUNCTION_1ARG_ID "1arg_identity"
// #define FUNCTION_1ARG_SAFE "1arg_safe"
#define REG_POS_SQRT "1arg_reg_pos_sqrt"
#define REG_NEG_SQRT "1arg_reg_neg_sqrt"
#define REG_ABS_VAL_SQRT "1arg_reg_abs_val_sqrt"
#define REG_SIGN_SIN_ATAN "1arg_reg_sign_sin_atan"
// #define FUNCTION_2ARG_POLYTHREE "2arg_poly_three"
// #define FUNCTION_2ARG_SAFE "2arg_safe"
// #define FUNCTION_3ARG_POLYONE "3arg_poly_one"
// #define FUNCTION_3ARG_POLYTHREE "3arg_poly_three"

using namespace std;

namespace MaverickUtils {
  GenericFunction1AManager::GenericFunction1AManager() : ComponentManagerBase() {
    _available_models_vector = {SPLINE, POLYFIVE, SIN_ATAN, REG_POS_SQRT, REG_NEG_SQRT, REG_ABS_VAL_SQRT,
                                REG_SIGN_SIN_ATAN};
    setManagerType();
  }

  GenericFunction1AManager::GenericFunction1AManager(string const &_name) : ComponentManagerBase(_name) {
    _available_models_vector = {SPLINE, POLYFIVE, SIN_ATAN, REG_POS_SQRT, REG_NEG_SQRT, REG_ABS_VAL_SQRT,
                                REG_SIGN_SIN_ATAN};
    setManagerType();
  }

  void GenericFunction1AManager::setManagerType() {
    _manager_type = "Generic function";
  }

  GenericFunction1AManager::~GenericFunction1AManager() {
    // deletion of pointer is made by the parent class
  }

  void GenericFunction1AManager::setup(GC::GenericContainer const &gc) {
    if (_p_function != nullptr) {
      delete _p_function;
      _p_function = nullptr;
      _component = _p_function;
    }
    ComponentManagerBase::setup(gc);

    string requested_type;
    try { //check if the "type" value is specified in the ruby or lua file
      requested_type = gc("type").get_string();
    } catch (...) {
      throwExceptionTypeNotDeclared();
    }

    if (requested_type.compare(POLYFIVE) == 0) {
      _p_function = new GF1APolyFive();
    }
    if (requested_type.compare(SPLINE) == 0) {
      _p_function = new GF1ASpline();
    }
    if (requested_type.compare(SIN_ATAN) == 0) {
      _p_function = new GF1ASinAtan();
    }
    if (requested_type.compare(REG_POS_SQRT) == 0) {
      _p_function = new RegPosPartSqrt();
    }
    if (requested_type.compare(REG_NEG_SQRT) == 0) {
      _p_function = new RegNegPartSqrt();
    }
    if (requested_type.compare(REG_ABS_VAL_SQRT) == 0) {
      _p_function = new RegAbsValSqrt();
    }
    if (requested_type.compare(REG_SIGN_SIN_ATAN) == 0) {
      _p_function = new RegSignSinAtan();
    }

//        if (requestedType.compare(FUNCTION_1ARG_ATAN)==0) {
//            found=true;
//            functionPointer=new GenericFunction1ArgAtan();
//        }
//        if (requestedType.compare(SIN_ATAN)==0) {
//            found=true;
//            functionPointer=new GenericFunction1ArgSinAtan();
//        }
//        if (requestedType.compare(FUNCTION_1ARG_ID)==0) {
//            found=true;
//            functionPointer=new GenericFunction1ArgID();
//        }
//        if (requestedType.compare(FUNCTION_1ARG_SAFE)==0) {
//            found=true;
//            functionPointer=new GenericFunction1ArgSafe();
//        }
//        if (requestedType.compare(FUNCTION_2ARG_POLYTHREE)==0) {
//            found=true;
//            functionPointer=new GenericFunction2ArgPolyThree();
//        }
//        if (requestedType.compare(FUNCTION_2ARG_SAFE)==0) {
//            found=true;
//            functionPointer=new GenericFunction2ArgSafe();
//        }
//        if (requestedType.compare(FUNCTION_3ARG_POLYONE)==0) {
//            found=true;
//            functionPointer=new GenericFunction3ArgPolyOne();
//        }
//        if (requestedType.compare(FUNCTION_3ARG_POLYTHREE)==0) {
//            found=true;
//            functionPointer=new GenericFunction3ArgPolyThree();
//        }

    if (_p_function != nullptr) { //everything works
      _component = _p_function;
      ComponentManagerBase::childObjectSetup(gc);
    } else {
      std::stringstream ss;
      getErrorMessageModelNotAvailable(requested_type, ss);
      std::runtime_error exc(ss.str());
      throw exc;
    }
  }

};
