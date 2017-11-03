#ifndef _MAVERICK_ENTER_CLASS_NAME_LIB_HH
#define _MAVERICK_ENTER_CLASS_NAME_LIB_HH

// C++ interface
#ifdef __cplusplus

#include "MaverickCore/MaverickOcp.hh"
#include <memory>

namespace Maverick {
    
    std::unique_ptr<MaverickOcp> getMaverickOcpPointer();
    
}

#endif

// C interface
#ifdef __cplusplus
extern "C" {
#endif
    
    // returns a pointer to an object of type _MAVERICK_ENTER_CLASS_NAME
    // to delete the object, the MaverickCInterface.h provides a function for it
    void * getMaverickOcpPointer();
    
#ifdef __cplusplus
}
#endif
    
#endif
