/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef GENERIC_FUNCTION_2A_MANAGER_HH
#define GENERIC_FUNCTION_2A_MANAGER_HH

#include "GenericFunction2AInterface.hh"
#include "GenericFunction2ABase.hh"
#include "../ComponentBase/ComponentManagerBase.hh"

namespace MaverickUtils {

  class GenericFunction2AManager : public GenericFunction2AInterface, public ComponentManagerBase {

  public:

    GenericFunction2AManager();

    GenericFunction2AManager(std::string const &_name);

    ~GenericFunction2AManager();

    virtual void setup(GC::GenericContainer const &gc);

    inline virtual real funcEval(real const x1, real const x2) const { return _p_function->funcEval(x1, x2); }

    inline virtual real funcEval_D_1(real const x1, real const x2) const { return _p_function->funcEval_D_1(x1, x2); }

    inline virtual real funcEval_D_2(real const x1, real const x2) const { return _p_function->funcEval_D_2(x1, x2); }

    inline virtual real funcEval_D_1_1(real const x1, real const x2) const {
      return _p_function->funcEval_D_1_1(x1, x2);
    }

    inline virtual real funcEval_D_1_2(real const x1, real const x2) const {
      return _p_function->funcEval_D_1_2(x1, x2);
    }

    inline virtual real funcEval_D_2_2(real const x1, real const x2) const {
      return _p_function->funcEval_D_2_2(x1, x2);
    }

    virtual void printInfo(std::ostream &out, InfoLevel info_level) const {
      ComponentManagerBase::printInfo(out, info_level);
    }

    virtual std::string getComponentType() const { return ComponentManagerBase::getComponentType(); }

  protected:

    GenericFunction2AInterface *_p_function = nullptr;

  private:

    virtual void setManagerType();

  };

}

#endif
