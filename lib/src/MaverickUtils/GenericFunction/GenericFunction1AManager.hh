/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef GENERIC_FUNCTION_1A_MANAGER_HH
#define GENERIC_FUNCTION_1A_MANAGER_HH

#include "GenericFunction1AInterface.hh"
#include "GenericFunction1ABase.hh"
#include "../ComponentBase/ComponentManagerBase.hh"

namespace MaverickUtils {

  class GenericFunction1AManager : public GenericFunction1AInterface, public ComponentManagerBase {

  public:

    GenericFunction1AManager();

    GenericFunction1AManager(std::string const &_name);

    ~GenericFunction1AManager();

    virtual void setup(GC::GenericContainer const &gc);

    inline virtual real funcEval(real const x1) const { return _p_function->funcEval(x1); }

    inline virtual real funcEval_D_1(real const x1) const { return _p_function->funcEval_D_1(x1); }

    inline virtual real funcEval_D_1_1(real const x1) const { return _p_function->funcEval_D_1_1(x1); }

    inline virtual real funcEval_D_1_1_1(real const x1) const { return _p_function->funcEval_D_1_1_1(x1); }

    virtual void printInfo(std::ostream &out, InfoLevel info_level) const {
      ComponentManagerBase::printInfo(out, info_level);
    }

    virtual std::string getComponentType() const { return ComponentManagerBase::getComponentType(); }

  protected:

    GenericFunction1AInterface *_p_function = nullptr;

  private:

    virtual void setManagerType();

  };

}

#endif
