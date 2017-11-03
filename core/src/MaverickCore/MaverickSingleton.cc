#include "MaverickSingleton.hh"
#include "MaverickCore/MaverickFunctions.hh"
#include "MaverickCore/MaverickPrivateDefs.hh"
#include <thread>

using namespace Maverick;
using namespace std;

MaverickSingleton::MaverickSingleton() {
//    cout << "Singleton constructed\n";
}

MaverickSingleton::~MaverickSingleton() {
    // printInfo();
    closeMavericktoipSharedLibHandle();
    closeMavericktsSharedLibHandle();
//    cout << "Singleton destructed\n";
}

MaverickSingleton & MaverickSingleton::getInstance() {
    static MaverickSingleton instance; // Guaranteed to be destroyed.
                                       // Instantiated on first use.
    return instance;
}

//void MaverickSingleton::setNumThreadsToUse( u_integer const num_threads ) {
//    _nt_mutex.lock();
//    threads_to_use = num_threads;
//    _nt_mutex.unlock();
//}

//u_integer MaverickSingleton::getNumThreadsToUse() {
//    //WARNING: sarebbe da mettere un lock anche qui, ma questa race condition non accadrà mai. Per tenere il codice + snello omettiamo il lock
//    // _nt_mutex.lock();
//    // u_integer threads_to_use_copy = threads_to_use;
//    // _nt_mutex.unlock();
//    return threads_to_use;
//}

integer MaverickSingleton::getHardwareConcurrencyNumThreads() const {

    integer threads_to_use = std::thread::hardware_concurrency();
    if (threads_to_use == 0)
        threads_to_use = 1;

    return threads_to_use;
}

void MaverickSingleton::setInfoLevel( InfoLevel const level ) {
    _il_mutex.lock();
    _info_level = level;
    _il_mutex.unlock();
}

bool MaverickSingleton::Log( InfoLevel const level, std::string const & mess ) const {
    //WARNING: sarebbe da mettere un lock anche qui, ma questa rece condition non accadrà mai. Per tenere il codice + snello omettiamo il lock
    // _il_mutex.lock();
    if ( _info_level >= level ) {
        // _il_mutex.unlock();
        if ( level == info_level_warning )
            std::cout << "WARNING: ";
        std::cout << mess;
        return true;
    }
    return false;
}

Maverick::InfoLevel MaverickSingleton::getInfoLevel() const {
    //WARNING: sarebbe da mettere un lock anche qui, ma questa rece condition non accadrà mai. Per tenere il codice + snello omettiamo il lock
    // _il_mutex.lock();
    // InfoLevel info_level_copy = info_level;
    // _il_mutex.unlock();
    return _info_level;
}

void MaverickSingleton::loadMavericktoipSharedLibHandle() {
    if (_p_mavericktoip == nullptr) {
        string sl_name = "libmavericktoip." + string(SHARED_LIB_EXTENSION);
#ifndef MAVERICK_DEBUG_VERSION_LIB
        string sl_name_and_path = "/usr/local/maverick/lib/" + sl_name;
#else
        string sl_name_and_path = "/usr/local/maverick_debug/lib/" + sl_name;
#endif

        _p_mavericktoip = dlopen(sl_name_and_path.c_str(), RTLD_LAZY);

        if (!_p_mavericktoip) {
            _p_mavericktoip = dlopen(sl_name.c_str(), RTLD_LAZY);
        }
        if (!_p_mavericktoip) {
            string message = "error while loading shared library: " + sl_name_and_path + ": " + string(dlerror()) + "\n";
            throw std::runtime_error(message);
        }
    }
}

void MaverickSingleton::loadMavericktsSharedLibHandle() {
    if (_p_maverickts == nullptr) {
        string sl_name = "libmaverickts." + string(SHARED_LIB_EXTENSION);
#ifndef MAVERICK_DEBUG_VERSION_LIB
        string sl_name_and_path = "/usr/local/maverick/lib/" + sl_name;
#else
        string sl_name_and_path = "/usr/local/maverick_debug/lib/" + sl_name;
#endif


        _p_maverickts = dlopen(sl_name_and_path.c_str(), RTLD_LAZY);

        if (!_p_maverickts) {
            _p_maverickts = dlopen(sl_name.c_str(), RTLD_LAZY);
        }
        if (!_p_maverickts) {
            string message = "error while loading shared library: " + sl_name_and_path + ": " + string(dlerror()) + "\n";
            throw std::runtime_error(message);
        }
    }
}

void MaverickSingleton::closeMavericktoipSharedLibHandle() {
    if ( _p_mavericktoip )
        dlclose(_p_mavericktoip);
    _p_mavericktoip = nullptr;
}

void * MaverickSingleton::getMaverickToIpSharedLibHandle() {
    loadMavericktoipSharedLibHandle();
    return _p_mavericktoip;
}

void MaverickSingleton::closeMavericktsSharedLibHandle() {
    if ( _p_maverickts )
        dlclose(_p_maverickts);
    _p_maverickts = nullptr;
}

void * MaverickSingleton::getMaverickTsSharedLibHandle() {
    loadMavericktsSharedLibHandle();
    return _p_maverickts;
}

void MaverickSingleton::printInfo() const {
    if ( _info_level >= info_level_none ) {
        stringstream ss;
        printMaverickInfo(ss);
        cout << ss.str();
    }
}
