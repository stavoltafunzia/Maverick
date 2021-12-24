#include "ComponentBase.hh"
#include "MaverickCore/MaverickFunctions.hh"

using namespace std;
using namespace GC;

namespace MaverickUtils {

  using namespace Maverick;

  // Default contructor
  ComponentBase::ComponentBase() {}

  // Destructor
  ComponentBase::~ComponentBase() {}

  //Protected methods
  void ComponentBase::throwSetupErrorException(string const &parent_exc_message) const {
    string message = _model + ": unable to initialize all parameters required by the model.\n";
    message = message.append(parent_exc_message);
    throw std::runtime_error(message);
  }

  real ComponentBase::findValueParamWithPrefixAndName(string const &prefix, string const &name,
                                                      GenericContainer const &gc) const {
    bool foundRoot = false;
    real rootValue;
    try { //check if the parameter is found in the root container
      if (prefix.compare("") != 0)
        foundRoot = findRealFromGenericContainer(gc, prefix + "_" + name, rootValue);
        // rootValue = gc(prefix + "_" + name).get_number();
      else
        foundRoot = findRealFromGenericContainer(gc, name, rootValue);
      // rootValue = gc(name).get_number();
    } catch (...) {}

    bool foundGrouped = false;
    real groupedValue;
    try { //check if the parameter is found in the subgroped container
      // GenericContainer const & sub = gc(prefix);
      foundGrouped = findRealFromGenericContainer(gc(prefix), name, groupedValue);
      // groupedValue = sub(name).get_number();
      // foundGrouped = true;
    } catch (...) {}

    if (foundRoot &&
        foundGrouped) { // if the parameter is found twice, accept the values only if they are equal, otherwise thorw exception
      if (rootValue == groupedValue) return rootValue;
      else {
        string message = _model + ": parameter of name '" + name;
        if (prefix.compare("") != 0) message.append("', group '" + prefix);
        message.append(+"' is defined twice with different values in setup.\n");
        throw DoubledValue(message);
      }
    }

    // if the parameter is found only once return the correct value
    if (foundRoot) return rootValue;
    if (foundGrouped) return groupedValue;

    // if the parameter is not found throw exception
    throw runtime_error(" can't find parameter, or wrong parameter type, for name '" + name + "', group '" + prefix +
                        "' in setup.\n");
  }

  int_type ComponentBase::findIntParamWithPrefixAndName(string const &prefix, string const &name,
                                                        GenericContainer const &gc) const {
    bool foundRoot = false;
    int_type rootValue;
    try { //check if the parameter is found in the root container
      if (prefix.compare("") != 0)
        rootValue = gc(prefix + "_" + name).get_int();
      else
        rootValue = gc(name).get_int();
      foundRoot = true;
    } catch (...) {}

    bool foundGrouped = false;
    int_type groupedValue;
    try { //check if the parameter is found in the subgroped container
      GenericContainer const &sub = gc(prefix);
      groupedValue = sub(name).get_int();
      foundGrouped = true;
    } catch (...) {}

    if (foundRoot &&
        foundGrouped) { // if the parameter is found twice, accept the values only if they are equal, otherwise thorw exception
      if (rootValue == groupedValue) return rootValue;
      else {
        string message = _model + ": parameter of name '" + name;
        if (prefix.compare("") != 0) message.append("', group '" + prefix);
        message.append(+"' is defined twice with different values in setup.\n");
        throw DoubledValue(message);
      }
    }

    // if the parameter is found only once return the correct value
    if (foundRoot) return rootValue;
    if (foundGrouped) return groupedValue;

    // if the parameter is not found throw exception
    throw runtime_error(" can't find parameter, or wrong parameter type, for name '" + name + "', group '" + prefix +
                        "' in setup.\n");
  }

  real ComponentBase::findValueParamWithPrefixAndName(string const &prefix, string const &name1, string const &name2,
                                                      GenericContainer const &gc) const {
    real value = 0;  // init to zero just to suppress a gcc warning
    exception _exc;
    DoubledValue *_dv = 0;

    bool foundFirst = false;
    try { //check if the parameter is found with the first name
      value = findValueParamWithPrefixAndName(prefix, name2, gc);
      foundFirst = true;
    } catch (DoubledValue const &dv) {
      foundFirst = true;
      _dv = new DoubledValue(dv);
    } catch (exception const &exc) { _exc = exc; }

    bool foundSecond = false;
    try { //check if the parameter is found with the second name
      value = findValueParamWithPrefixAndName(prefix, name1, gc);
      foundSecond = true;
    } catch (DoubledValue const &dv) {
      foundSecond = true;
      if (_dv == 0) _dv = new DoubledValue(dv);
    } catch (exception const &exc) { _exc = exc; }

    if (foundFirst && foundSecond) // if the parameter is found with different names throw exception
      throw runtime_error(
          _model + ": parameter of group '" + prefix + "' defined with both names: '" + name1 + "', and '" + name2 +
          "' in setup.\n");

    if (foundFirst ||
        foundSecond) { // if the parameter is found only once, throw an exception if an exception was detected, otherwise return the value
      if (_dv == 0) return value;
      throw *_dv;
    }

    // if the parameter is not found, throw an exception
    throw _exc;
  }

  real ComponentBase::findValueParamWithNameAndInverseNameAndDefValue(string const &name1, string const &name2,
                                                                      real const defValue,
                                                                      GenericContainer const &gc) const {
    real value = defValue;

    bool foundFirst = false;
    try { //check if the parameter is found with the first name
      value = findValueParamWithPrefixAndName("", name1, gc);
      foundFirst = true;
    } catch (...) {}

    bool foundSecond = false;
    try { //check if the parameter is found with the second name
      value = 1 / (findValueParamWithPrefixAndName("", name2, gc));
      foundSecond = true;
    } catch (...) {}

    if (foundFirst && foundSecond) {// if the parameter is found with different names thorw exception
      string errorString =
          _model + ": parameter of name '" + name1 + "' defined with both with its name and its inverse name: '" +
          name2 + "' in the setup. Only one definition is allowed.\n";
      throw runtime_error(errorString);
    }

    return value;
  }

  string ComponentBase::findStringParamWithPrefixAndName(std::string const &prefix, std::string const &name,
                                                         GC::GenericContainer const &gc) const {
    bool foundRoot = false;
    string rootValue;
    try { //check if the parameter is found in the root container
      if (prefix.compare("") != 0)
        rootValue = gc(prefix + "_" + name).get_string();
      else
        rootValue = gc(name).get_string();
      foundRoot = true;
    } catch (...) {}

    bool foundGrouped = false;
    string groupedValue;
    try { //check if the parameter is found in the subgroped container
      GenericContainer const &sub = gc(prefix);
      groupedValue = sub(name).get_string();
      foundGrouped = true;
    } catch (...) {}

    if (foundRoot &&
        foundGrouped) { // if the parameter is found twice, accept the values only if they are equal, otherwise thorw exception
      if (rootValue.compare(groupedValue) == 0)
        return rootValue;
      else {
        string message = _model + ": parameter of name '" + name;
        if (prefix.compare("") != 0) message.append("', group '" + prefix);
        message.append(+"' is defined twice with different values in setup.\n");
        throw DoubledValue(message);
      }
    }

    // if the parameter is found only once return the correct value
    if (foundRoot) return rootValue;
    if (foundGrouped) return groupedValue;

    // if the parameter is not found throw exception
    throw runtime_error(" can't find parameter, or wrong parameter type, for name '" + name + "', group '" + prefix +
                        "' in setup.\n");
  }

  vec_real_type ComponentBase::findVectorParamWithPrefixAndName(string const &prefix, string const &name,
                                                                GenericContainer const &gc) const {
    bool foundRoot = false;
    vec_1d_real rootValue;

    if (prefix.compare("") != 0) //check if the parameter is found in the root container
      foundRoot = findVecRealFromGenericContainer(gc, prefix + "_" + name, rootValue);
    else
      foundRoot = findVecRealFromGenericContainer(gc, name, rootValue);

    vec_1d_real groupedValue;
    bool foundGrouped = false;
    try {
      foundGrouped = findVecRealFromGenericContainer(gc(prefix), name, groupedValue);
    } catch (...) {}

    if (foundRoot &&
        foundGrouped) { // if the parameter is found twice, accept the values only if they are equal, otherwise thorw exception
      if (rootValue == groupedValue) return rootValue;
      else {
        string message = _model + ": vector parameter of name '" + name;
        if (prefix.compare("") != 0) message.append("', group '" + prefix);
        message.append(+"' is defined twice with different values in setup.\n");
        throw DoubledValue(message);
      }
    }

    // if the parameter is found only once return the correct value
    if (foundRoot) return rootValue;
    if (foundGrouped) return groupedValue;

    // if the parameter is not found throw exception
    throw runtime_error(
        "Can't find vector parameter, or wrong parameter type, for name '" + name + "', group '" + prefix +
        "' in setup.\n");
  }

  //Public methods
  void ComponentBase::setup(GenericContainer const &gc) {
    _ignore_nim = false;

    try { //check if comment is declared in file
      _ignore_nim = gc("ignore_nim").get_bool();
    } catch (...) {}
  }

  void ComponentBase::printInfo(std::ostream &stream, InfoLevel info_level) const {
    if (info_level >= info_level_very_verbose) {
      printParametersInfo(stream);
    }
    stream << "\n";
  }

  string ComponentBase::getComponentType() const {
    return _model;
  }

  void ComponentBase::throwUnimplementedMethodException(string const &method) const {
    string message = string("WARNING! Method '").append(method).append("' has been called for object '").append(_model)
        .append("' but it's not implemented. Use a different object.\n");
    throw std::runtime_error(message);
  }

}
