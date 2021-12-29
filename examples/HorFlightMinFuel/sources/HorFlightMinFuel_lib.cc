#include "HorFlightMinFuel_lib.h"
#include "HorFlightMinFuel.hh"

namespace Maverick {
    
    std::unique_ptr<MaverickOcp> getMaverickOcpPointer() {
        return std::unique_ptr<MaverickOcp>(new HorFlightMinFuelNamespace::HorFlightMinFuel() );
    }
    
}

void * getMaverickOcpPointer() {
    return (void*) new HorFlightMinFuelNamespace::HorFlightMinFuel();
}

