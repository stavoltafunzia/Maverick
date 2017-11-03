/******************************************************
*                                                     *
*   This software is released by Nicola Dal Bianco    *
*      under the Gnu General Public License V3        *
*                                                     *
******************************************************/

#ifndef MAVERICK_SINGLETON_HH
#define MAVERICK_SINGLETON_HH

#include "MaverickDefinitions.hh"
#include <dlfcn.h>
#include <mutex>

namespace Maverick {

    class MaverickSingleton {

    public:

        static MaverickSingleton & getInstance();

        ~MaverickSingleton();

        MaverickSingleton(MaverickSingleton const &) = delete;

        void operator=(MaverickSingleton const &)    = delete;

        // interface for threads

        integer getHardwareConcurrencyNumThreads() const;

        //interface for log

        bool Log( InfoLevel const level, std::string const & mess ) const;

        void setInfoLevel( InfoLevel const level );

        InfoLevel getInfoLevel() const;

        // get maverick2ipopt shared lib instance

        void * getMaverickToIpSharedLibHandle();

        void * getMaverickTsSharedLibHandle();

        void printInfo() const;

    protected:

        std::mutex _il_mutex;

        InfoLevel _info_level = info_level_normal;

        void * _p_mavericktoip = nullptr;

        void * _p_maverickts = nullptr;

        void loadMavericktoipSharedLibHandle();

        void loadMavericktsSharedLibHandle();

        void closeMavericktoipSharedLibHandle();

        void closeMavericktsSharedLibHandle();
        
    private:
        
        MaverickSingleton();

    };

}

#endif
