#include "SingleMassPointE_lib.h"
#include "SingleMassPointE.hh"

namespace Maverick {
    
    std::unique_ptr<MaverickOcp> getMaverickOcpPointer() {
        return std::unique_ptr<MaverickOcp>(new SingleMassPointENamespace::SingleMassPointE() );
    }
    
}

void * getMaverickOcpPointer() {
    return (void*) new SingleMassPointENamespace::SingleMassPointE();
}

