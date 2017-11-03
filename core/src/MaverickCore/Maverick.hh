/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef MAVERICK_HH
#define MAVERICK_HH

#include "MaverickCore/MaverickOcp.hh"
#include "MaverickCore/MaverickSolver.hh"
#include "MaverickUtils/GenericFunction/GenericFunction1AInterface.hh"
#include "MaverickUtils/GenericFunction/GenericFunction2AInterface.hh"

namespace Maverick {

    std::unique_ptr<Maverick::MaverickSolver> getMaverickSolver(Maverick::MaverickOcp & ocp);

    std::unique_ptr<MaverickUtils::GenericFunction1AInterface> getGenericFunction1A();

    std::unique_ptr<MaverickUtils::GenericFunction1AInterface> getGenericFunction1A( std::string const & name );

    std::unique_ptr<MaverickUtils::GenericFunction2AInterface> getGenericFunction2A();

    std::unique_ptr<MaverickUtils::GenericFunction2AInterface> getGenericFunction2A( std::string const & name );

}

#endif
