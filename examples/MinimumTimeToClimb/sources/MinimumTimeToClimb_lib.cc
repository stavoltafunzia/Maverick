#include "MinimumTimeToClimb_lib.h"
#include "MinimumTimeToClimb.hh"

namespace Maverick {
    
    std::unique_ptr<MaverickOcp> getMaverickOcpPointer() {
        return std::unique_ptr<MaverickOcp>(new MinimumTimeToClimbNamespace::MinimumTimeToClimb() );
    }
    
}

void * getMaverickOcpPointer() {
    return (void*) new MinimumTimeToClimbNamespace::MinimumTimeToClimb();
}

