/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef COMPONENT_BASE_HH
#define COMPONENT_BASE_HH

#include "ComponentInterface.hh"
#include "ComponentDefinitions.hh"

namespace MaverickUtils {

  class ComponentBase : public ComponentInterface {

  public:

    //! Default constructor
    ComponentBase();

    //! Destructor
    virtual ~ComponentBase();

    //Public methods

    std::string getComponentType() const;

    void printInfo(std::ostream &out, InfoLevel info_level) const;

  protected:

    std::string _model = "";

    bool _ignore_nim = false;

    /// Exception class that is thrown when a suspension parameter is defined twice with different values in the configuration file
    class DoubledValue : public std::runtime_error {

    public:

      /// Constructor that sets the message of the exception
      DoubledValue(std::string const &error) : std::runtime_error(error) {};

      /// Copy constructor
      DoubledValue(DoubledValue const &dv) : std::runtime_error(dv) {};
    };

    //Protected methods

    /// Method that thorws an exception if an error in the setup method occurs
    void throwSetupErrorException(std::string const &parent_exc_message) const;

    /// Mehtod that looks in the .rb or .lua file for a parameter named 'name' that can be nested in a subgrup of name 'prefix' or can be in the root container with name 'prefix_name'
    real findValueParamWithPrefixAndName(std::string const &prefix, std::string const &name,
                                         GC::GenericContainer const &gc) const;

    std::string findStringParamWithPrefixAndName(std::string const &prefix, std::string const &name,
                                                 GC::GenericContainer const &gc) const;

    integer findIntParamWithPrefixAndName(std::string const &prefix, std::string const &name,
                                          GC::GenericContainer const &gc) const;

    vec_1d_real findVectorParamWithPrefixAndName(std::string const &prefix, std::string const &name,
                                                 GC::GenericContainer const &gc) const;

    /// Mehtod that looks in the .rb or .lua file for a parameter named 'name' that can be nested in a subgrup of name 'prefix' or can be in the root container with name 'prefix_name'. 'name' can be name1 or name2
    real findValueParamWithPrefixAndName(std::string const &prefix, std::string const &name1, std::string const &name2,
                                         GC::GenericContainer const &gc) const;

    /// Mehtod that looks in the .rb or .lua file for a parameter named 'name1', or, eventually, whose inverse (1/x) is declred with a parameter 'name2'
    real findValueParamWithNameAndInverseNameAndDefValue(std::string const &name1, std::string const &name2,
                                                         real const defValue, GC::GenericContainer const &gc) const;

    /// Method that thorws an exception if a non-overridden  method is called
    void throwUnimplementedMethodException(std::string const &method) const;

    virtual void printParametersInfo(std::ostream &out) const = 0;

    virtual void setModelType() = 0;

    void setup(GC::GenericContainer const &gc);

  };
}

#endif
