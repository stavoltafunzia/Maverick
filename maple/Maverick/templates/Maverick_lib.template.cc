#include "_MAVERICK_ENTER_CLASS_NAME_lib.h"
#include "_MAVERICK_ENTER_CLASS_NAME.hh"

namespace Maverick {
    
    std::unique_ptr<MaverickOcp> getMaverickOcpPointer() {
        return std::unique_ptr<MaverickOcp>(new _MAVERICK_ENTER_CLASS_NAMENamespace::_MAVERICK_ENTER_CLASS_NAME() );
    }
    
}

void * getMaverickOcpPointer() {
    return (void*) new _MAVERICK_ENTER_CLASS_NAMENamespace::_MAVERICK_ENTER_CLASS_NAME();
}

