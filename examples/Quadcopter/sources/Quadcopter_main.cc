#include <iostream>
#include "MaverickCore/Maverick.hh"
#include "Quadcopter.hh"
#include "MaverickGC/GenericContainerLuaInterface.hh"

using namespace std;

int main(int argc, const char * argv[]) {

    try {
        // read lua file
        GC::GenericContainer gc;
        GC::LuaInterpreter lua ;
        if (argc==2) {
            cout << "Reading lua file '" << string(argv[1]) << "' ..." << endl;
            lua.do_file(argv[1]) ;
        } else {
            cout << "Reading lua file 'Quadcopter_data.lua'..." << endl;
            lua.do_file("Quadcopter_data.lua") ;
        }
        lua.global_to_GC("Data",gc) ;


        QuadcopterNamespace::Quadcopter ocp;

        unique_ptr<Maverick::MaverickOcpSolver> solver = Maverick::getMaverickOcpSolver(ocp);

        solver->solve(gc);

        std::cout << "ALL DONE!\n";

    } catch ( std::runtime_error & exc) {
        cout << exc.what();
    }

    return 0;
}
