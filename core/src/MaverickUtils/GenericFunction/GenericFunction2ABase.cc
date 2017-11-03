#include "GenericFunction2ABase.hh"

namespace MaverickUtils {

    void GenericFunction2ABase::setup( GC::GenericContainer const & gc ) {
        ComponentBase::setup( gc );

        try {
            _x1_scale_inv = findValueParamWithPrefixAndName("", "x1_scale", gc);
            _x1_scale_inv = 1.0/_x1_scale_inv;
        }
        catch (...) {
            _x1_scale_inv = 1;
        }
        try {
            _x2_scale_inv = findValueParamWithPrefixAndName("", "x2_scale", gc);
            _x2_scale_inv = 1.0/_x2_scale_inv;
        }
        catch (...) {
            _x2_scale_inv = 1;
        }
        try {
            _y_scale = findValueParamWithPrefixAndName("", "y_scale", gc);
        }
        catch (...) {
            _y_scale = 1;
        }

    }

    void GenericFunction2ABase::printParametersInfo( std::ostream & out ) const {
        out << "x1_scale: " << 1.0/_x1_scale_inv << ", x2_scale: " << 1.0/_x2_scale_inv << ", y_scale: " << _y_scale;
    }
}
