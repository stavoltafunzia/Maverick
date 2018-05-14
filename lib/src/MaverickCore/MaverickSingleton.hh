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

    static MaverickSingleton &getInstance();

    ~MaverickSingleton();

    MaverickSingleton(MaverickSingleton const &) = delete;

    void operator=(MaverickSingleton const &)    = delete;

    void printInfo() const;

    // interface for threads

    u_integer getHardwareConcurrencyNumThreads() const;

    //interface for log

    bool Log(InfoLevel const level, std::string const &mess) const;

    void setInfoLevel(InfoLevel const level);

    InfoLevel getInfoLevel() const;

    // get shared lib instances

    void *getMaverickToIpSharedLibHandle();

    void *getMaverickTsSharedLibHandle();

    void *getCustomSharedLibHandle(std::string const &libname);

  protected:

    std::mutex _il_mutex;

    InfoLevel _info_level = info_level_normal;

    void *_p_mavericktoip = nullptr;

    void *_p_maverickts = nullptr;

    void *_p_custom = nullptr;

    std::string _custom_lib_name;

    void loadMavericktoipSharedLibHandle();

    void loadMavericktsSharedLibHandle();

    void loadCustomSharedLibHandle(std::string const &libname);

    void closeMavericktoipSharedLibHandle();

    void closeMavericktsSharedLibHandle();

    void closeCustomSharedLibHandle();

  private:

    MaverickSingleton();

  };

}

#endif
