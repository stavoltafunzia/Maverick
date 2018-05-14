/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef COMPONENT_MANAGER_BASE_HH
#define COMPONENT_MANAGER_BASE_HH

#include "ComponentInterface.hh"
#include "ComponentBase.hh"

namespace MaverickUtils {

  class ComponentManagerBase : public ComponentInterface {

  public:

    ComponentManagerBase();

    ComponentManagerBase(const std::string &_name);

    virtual ~ComponentManagerBase();

    void printInfo(std::ostream &stream, InfoLevel info_level) const;

    virtual void setup(GC::GenericContainer const &gc);

    std::vector<std::string> const &getAvailableModels() const;

    void getErrorMessageModelNotAvailable(std::string const &requested_model, std::ostream &out) const;

    virtual std::string getComponentType() const;

  protected:

    std::vector<std::string> _available_models_vector = {};

    std::string _name = "";

    std::string _manager_type = "";

    ComponentInterface *_component = nullptr;

    void throwExceptionTypeNotDeclared() const;

    void childObjectSetup(GC::GenericContainer const &gc);

    void addNameToStream(std::ostream &out) const;

  private:

    virtual void setManagerType() = 0;


  };
}

#endif
