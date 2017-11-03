#include "GenericFunction2AManager.hh"
#include "GF2ASpline.hh"
#include "GF2APolyThree.hh"

#define SPLINE "2arg_spline"
#define POLY_THREE "2arg_poly_three"

using namespace std;

namespace MaverickUtils {
    GenericFunction2AManager::GenericFunction2AManager(): ComponentManagerBase() {
        _available_models_vector = {SPLINE, POLY_THREE};
        setManagerType();
    }

    GenericFunction2AManager::GenericFunction2AManager(string const & _name) :ComponentManagerBase(_name) {
        _available_models_vector = {SPLINE, POLY_THREE};
        setManagerType();
    }

    void GenericFunction2AManager::setManagerType() {
        _manager_type = "Generic function";
    }

    GenericFunction2AManager::~GenericFunction2AManager() {
        // deletion of pointer is made by the parent class
    }

    void GenericFunction2AManager::setup( GC::GenericContainer const & gc ) {
        if (_p_function!=nullptr) {
            delete _p_function;
            _p_function = nullptr;
            _component = _p_function;
        }
        ComponentManagerBase::setup(gc);

        string requested_type;
        try { //check if the "type" value is specified in the ruby or lua file
            requested_type=gc("type").get_string();
        } catch (... ) {
            throwExceptionTypeNotDeclared();
        }

        if (requested_type.compare(SPLINE) == 0) {
            _p_function = new GF2ASpline();
        }
        if (requested_type.compare(POLY_THREE) ==  0) {
            _p_function = new GF2APolyThree();
        }

        if (_p_function != nullptr) { //everything works
            _component = _p_function;
            ComponentManagerBase::childObjectSetup(gc);
        } else {
            std::stringstream ss;
            getErrorMessageModelNotAvailable(requested_type, ss);
            std::runtime_error exc (ss.str());
            throw exc;
        }
    }

};
