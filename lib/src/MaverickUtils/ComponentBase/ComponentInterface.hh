/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef COMPONENT_INTERFACE_HH
#define COMPONENT_INTERFACE_HH

#include "MaverickCore/MaverickDefinitions.hh"
#include "MaverickGC/GenericContainer.hh"

namespace MaverickUtils {

  using namespace Maverick;

  class ComponentInterface {

  public:

    virtual ~ComponentInterface() {}

    //Public methods

    //! Method that makes the setup, setting all the parameters
    virtual void setup(GC::GenericContainer const &gc) = 0;

    virtual void printInfo(std::ostream &out, InfoLevel info_level) const = 0;

    virtual std::string getComponentType() const = 0;

  };
}

#endif
