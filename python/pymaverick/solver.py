import os
from subprocess import call
from enum import IntEnum
import pymaverick.core as mvc


class CompileLevel(IntEnum):
    compile_never = 0
    compile_if_modified = 1
    compile_always = 2


class Solver:

    # initialize object: first argument
    def __init__(self, ocp_libname, sources_path=None, compile_script=None,
                 force_compile=CompileLevel.compile_if_modified):

        self._ocp_ptr = None
        self._solver_ptr = None
        self.ocp_libname = ocp_libname
        self.sources_path = sources_path
        self.compile_script = compile_script
        self.force_compile = force_compile
        self._load_by_lib_name(ocp_libname, sources_path, compile_script)

    def __del__(self):        
        mvc.delete_maverick_solver(self._solver_ptr)
        mvc.delete_maverick_ocp(self._ocp_ptr)

    def solve(self, data):

        if type(data) is str:
            gc_in_id = mvc.lua_to_gc(data)
        elif type(data) is dict:
            gc_in_id = mvc.convert_dict_to_gc(data)
        else:
            raise RuntimeError("First argument must be a dictionary or a path to the lua data file")
        
        gc_in_ptr = mvc.get_gc_memptr(gc_in_id)
        out = mvc.solve(self._solver_ptr, gc_in_ptr)
        mvc.delete_gc(gc_in_id)
        return out

    def _load_by_lib_name(self, ocp_libname, sources_path, compile_script):

        # check if should compile the library
        should_compile = False
        if self.force_compile == CompileLevel.compile_always:
            should_compile = True
        elif self.force_compile == CompileLevel.compile_if_modified:
            if not os.path.isfile(ocp_libname):  # library does not exists
                should_compile = True

                if sources_path is None:
                    raise RuntimeError("The library '" + ocp_libname +
                                       "' does not exist, but 'sources_path' and 'compile_script' are None, "
                                       "thus cannot compile the library")
            else:
                # check modified date
                if sources_path is not None:
                    if os.path.getmtime(ocp_libname) < max(os.path.getmtime(sources_path + '/' + x)
                                                           for x in os.listdir(sources_path)):
                        # if the source files are more recent than library
                        should_compile = True

        elif self.force_compile == CompileLevel.compile_never:
            if not os.path.isfile(ocp_libname):  # if library does not exist
                raise RuntimeError(
                    "The library '" + ocp_libname + "' does not exist. Cannot continue.")

        if should_compile:
            call([compile_script])

        self._ocp_ptr = mvc.get_maverick_ocp_from_lib(ocp_libname)
        self._solver_ptr = mvc.get_maverick_solver(self._ocp_ptr)
