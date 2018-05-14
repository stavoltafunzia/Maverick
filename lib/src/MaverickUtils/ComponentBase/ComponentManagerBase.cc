#include "ComponentManagerBase.hh"

using namespace std;

namespace MaverickUtils {

  ComponentManagerBase::ComponentManagerBase() {
  }

  ComponentManagerBase::ComponentManagerBase(const std::string &name) : _name(name) {}

  ComponentManagerBase::~ComponentManagerBase() {
    if (_component != nullptr)
      delete _component;
    _component = nullptr;
  }

  void ComponentManagerBase::printInfo(ostream &stream, InfoLevel info_level) const {
    if (info_level >= info_level_verbose) {
      addNameToStream(stream);
      if (_component == nullptr) {
        stream << " not initialised yet.\n";
      } else {
        stream << ": " << _component->getComponentType() << ".";
        stringstream tmp;
        _component->printInfo(tmp, info_level);

        if (tmp.tellp() > 1)
          stream << "\n" << tmp.rdbuf();
      }
    }
  }

  void ComponentManagerBase::setup(GC::GenericContainer const &gc) {

  }

  void ComponentManagerBase::childObjectSetup(GC::GenericContainer const &gc) {
    try {
      _component->setup(gc);
    } catch (runtime_error const &exc) {
      string message = exc.what();
      stringstream ss;
      addNameToStream(ss);
      throw runtime_error(ss.str() + message);
    }
  }

  vector<string> const &ComponentManagerBase::getAvailableModels() const {
    return _available_models_vector;
  }

  void ComponentManagerBase::getErrorMessageModelNotAvailable(string const &requested_model, ostream &out) const {
    string errorString = "requested model of type '" + requested_model + "' which is not available. ";
    string availableModels = "Available models are:\n ";

    for (vector<string>::const_iterator it = _available_models_vector.begin();
         it != _available_models_vector.end(); it++) {
      availableModels = availableModels + "- " + *it + "\n";
    }
    errorString = errorString + availableModels;
    addNameToStream(out);
    out << errorString;
  }

  void ComponentManagerBase::addNameToStream(std::ostream &out) const {
    out << _manager_type << " ";
    if (_name.compare("") != 0) {
      out << "named '" << _name << "'";
    } else {
      out << "(unnamed)";
    }
  }

  string ComponentManagerBase::getComponentType() const {
    if (_component != nullptr)
      return _component->getComponentType();
    return "Uninitialized component model";
  }

  void ComponentManagerBase::throwExceptionTypeNotDeclared() const {
    stringstream ss;
    addNameToStream(ss);
    ss << " model type not specified. Please specify the type before calling the setup.";
    throw runtime_error(ss.str());
  }

//    string ComponentManagerBase::getComponentType() const {
//        return "";
//    }
}
