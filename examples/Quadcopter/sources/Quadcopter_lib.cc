#include "Quadcopter_lib.h"
#include "Quadcopter.hh"

namespace Maverick {
    
    std::unique_ptr<MaverickOcp> getMaverickOcpPointer() {
        return std::unique_ptr<MaverickOcp>(new QuadcopterNamespace::Quadcopter() );
    }
    
}

void * getMaverickOcpPointer() {
    return (void*) new QuadcopterNamespace::Quadcopter();
}

