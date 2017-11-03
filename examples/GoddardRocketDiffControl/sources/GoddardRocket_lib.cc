#include "GoddardRocket_lib.h"
#include "GoddardRocket.hh"

namespace Maverick {
    
    std::unique_ptr<MaverickOcp> getMaverickOcpPointer() {
        return std::unique_ptr<MaverickOcp>(new GoddardRocketNamespace::GoddardRocket() );
    }
    
}

void * getMaverickOcpPointer() {
    return (void*) new GoddardRocketNamespace::GoddardRocket();
}

