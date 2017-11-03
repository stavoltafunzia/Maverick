#include "SingleMassPoint_lib.h"
#include "SingleMassPoint.hh"

namespace Maverick {
    
    std::unique_ptr<MaverickOcp> getMaverickOcpPointer() {
        return std::unique_ptr<MaverickOcp>(new SingleMassPointNamespace::SingleMassPoint() );
    }
    
}

void * getMaverickOcpPointer() {
    return (void*) new SingleMassPointNamespace::SingleMassPoint();
}

