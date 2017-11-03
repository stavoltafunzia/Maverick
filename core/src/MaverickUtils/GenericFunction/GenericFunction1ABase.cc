#include "GenericFunction1ABase.hh"

namespace MaverickUtils {

    void GenericFunction1ABase::setup( GC::GenericContainer const & gc ) {
        ComponentBase::setup( gc );

        try {
            _x_scale_inv = findValueParamWithPrefixAndName("", "x_scale", gc);
            _x_scale_inv = 1.0/_x_scale_inv;
        }
        catch (...) {
            _x_scale_inv = 1;
        }
        try {
            _y_scale = findValueParamWithPrefixAndName("", "y_scale", gc);
        }
        catch (...) {
            _y_scale = 1;
        }

    }

    void GenericFunction1ABase::printParametersInfo( std::ostream & out ) const {
        out << "x_scale: " << 1.0/_x_scale_inv << ", y_scale: " << _y_scale;
    }
    
    void GenericFunction1ABase::clear() {
        _x_scale_inv = 1;
        _y_scale = 1;
    }
}


