cdef extern from "MaverickCore/MaverickCInterface.h":
    # MAVERICK OCP OBJECTS
    cdef int    loadMaverickOcpFromLib(char lib_name[], int * const p_err_code, char ** err_mess );
    cdef void   unloadMaverickOcp(int maverick_ocp_id, int * const p_err_code);

    # MAVERICK SOLVER
    cdef void   loadMaverickSolver(int maverick_ocp_id, int * const p_err_code);
    cdef void   unloadMaverickSolver(int maverick_ocp_id, int * const p_err_code);

    cdef void   solveFromInternalGC(int maverick_id, int input_generic_container_id, int sol_type, void * p_output_generic_container,
                             int * const p_err_code,
                             char error[], size_t error_length);

    cdef void   solveFromExternalGC(int maverick_id, void * p_input_generic_container, int sol_type, void * p_output_generic_container,
                             int * const p_err_code,
                             char error[], size_t error_length);

    # GENERIC CONTAINER
    cdef int    loadGenericContainerFromLua(char lua_name[], char lua_global_var[],
                                           int print_content,
                                       int * const p_err_code,
                                       char error[], size_t error_length);
    cdef void   unloadGenericContainer(int generic_container_id, int * const p_err_code);
    cdef void   fillGenericContainerFromLua(void * p_generic_container,
                                       char lua_file[], char lua_global_var[],
                                       int * const p_err_code,
                                       char error[], size_t error_length);

cdef extern from "MaverickGC/GenericContainerCinterface.h":
    cdef int GC_new( char id[] ) ;
    cdef int GC_select( char id[] ) ;
    cdef int GC_delete( char id[] ) ;
    cdef int GC_fill_for_test( char id[] ) ;
    cdef int GC_pop_head() ;
    cdef int GC_reset_head() ;
    cdef int GC_print() ;
    cdef int GC_get_type() ;
    cdef char * GC_get_type_name() ;
    cdef void * GC_mem_ptr( char id[] ) ;
    # -----------------------------------------------------------------------------
    cdef int GC_set_bool( int a ) ;
    cdef int GC_set_int( int a ) ;
    cdef int GC_set_real( double a ) ;
    cdef int GC_set_string( char a[] ) ;
    # -----------------------------------------------------------------------------
    cdef int GC_get_bool() ;
    cdef int GC_get_int() ;
    cdef long GC_get_long() ;
    cdef double GC_get_real() ;
    cdef char * GC_get_string() ;
    # -----------------------------------------------------------------------------
    cdef int GC_push_bool( int a ) ;
    cdef int GC_push_int( int a ) ;
    cdef int GC_push_real( double a ) ;
    cdef int GC_push_string( char a[] ) ;
    ## -----------------------------------------------------------------------------
    cdef int GC_get_bool_at_pos( int pos ) ;
    cdef int GC_get_int_at_pos( int pos ) ;
    cdef int GC_get_long_at_pos( int pos ) ;
    cdef double GC_get_real_at_pos( int pos ) ;
    cdef char * GC_get_string_at_pos( int pos ) ;
    ## -----------------------------------------------------------------------------
    cdef double GC_get_real_at_coor( int i, int j ) ;
    # -----------------------------------------------------------------------------
    cdef int GC_set_empty_vector_of_bool() ;
    cdef int GC_set_empty_vector_of_int() ;
    cdef int GC_set_empty_vector_of_real() ;
    cdef int GC_set_empty_vector_of_string() ;
    # -----------------------------------------------------------------------------
    cdef int GC_set_vector_of_bool( int a[], int nelem ) ;
    cdef int GC_set_vector_of_int( int a[], int nelem ) ;
    cdef int GC_set_vector_of_real( double a[], int nelem ) ;
    cdef int GC_set_vector_of_string( char *a[], int nelem ) ;
    # -----------------------------------------------------------------------------
    cdef int GC_set_vector( int nelem ) ;
    cdef int GC_set_empty_vector() ;
    cdef int GC_get_vector_size() ;
    cdef int GC_get_matrix_num_rows() ;
    cdef int GC_get_matrix_num_cols() ;
    cdef int GC_push_vector_position( int pos ) ;
    # -----------------------------------------------------------------------------
    cdef int GC_set_map() ;
    cdef int GC_init_map_key() ;
    cdef char * GC_get_next_key() ;
    cdef int GC_push_map_position( char pos[] ) ;

cimport cython
from libc.stdlib cimport malloc, free
import os, platform
from subprocess import call
from enum import IntEnum

class InfoLevel(IntEnum):
    info_level_none = 0
    info_level_warning = 1
    info_level_few = 2
    info_level_normal = 3
    info_level_more = 4
    info_level_verbose = 5
    info_level_very_verbose = 6

class CompileLevel(IntEnum):
    compile_never = 0
    compile_if_modified = 1
    compile_always = 2

class Solver:

    #ID of the current solver
    __ocp_id = 0
    __is_ocp_from_lib = False
    __info_level = InfoLevel.info_level_normal
    __force_compile = CompileLevel.compile_if_modified
    __specific_solution_names = True

    def __init__(self):
        raise RuntimeError("Defualt constructor not allowed for this class")

    # initialize object: first argument
    def __init__(self, ocp_object, arg2=-1, arg3=-1):
        #self.__ocp_id = 0
        #self.__is_ocp_from_lib = False
        #self.__info_level = InfoLevel.info_level_normal
        #self.__force_compile = CompileLevel.compile_if_modified
        #self.__specific_solution_names = True

        path = "./"


        if (type(arg2) == str):
            path = arg2
        elif (type(arg2) == CompileLevel ):
            self.__force_compile = arg2
        elif (arg2 != -1):
            raise RuntimeError("Second argument must be a string containing the path of the shared library, or a CompileLevel enum")

        if (type(arg3) == str):
            path = arg3
        elif (type(arg3) == CompileLevel ):
            self.__force_compile = arg3
        elif (arg3 != -1):
            raise RuntimeError("Third argument must be a string containing the path of the shared library, or a CompileLevel enum")

        self.__ocp_id = 0

        #move to specified working directory
        original_dir = os.getcwd()
        os.chdir(path)

        if type(ocp_object) is str:
            self.__loadByLibName(ocp_object)

        #restore original working directory
        os.chdir(original_dir)

    def close(self):
        cdef int err_code = 0
        unloadMaverickSolver(self.__ocp_id, & err_code);
        if (err_code != 0):
            print('Error while deleting C++ Maverick solver object')

        err_code = 0
        if (self.__is_ocp_from_lib):
            unloadMaverickOcp(self.__ocp_id, & err_code);
        if (err_code != 0):
            print('Error while deleting C++ MaverickOcp object')

    def __del__(self):
        self.close()

    def solve(self, arg1, arg2=-1, arg3=-1):

        self.__specific_solution_names = True
        working_dir="./"

        if (type(arg2) == str):
            working_dir = arg2
        elif (type(arg2) == bool):
            self.__specific_solution_names = arg2
        elif (arg2 != -1):
            raise RuntimeError("Second argument must be a string containing the working directory path or a boolean indicating whether to use specific variable names in the solution")

        if (type(arg3) == str):
            working_dir = arg3
        elif (type(arg3) == bool):
            self.__specific_solution_names = arg3
        elif (arg3 != -1):
            raise RuntimeError("Third argument must be a string containing the working directory path or a boolean indicating whether to use specific variable names in the solution")

        #move to specified working directory
        original_dir = os.getcwd()
        os.chdir(working_dir)

        if type(arg1) is str:
            out_data = self.__solveFromLua(arg1)
        elif type(arg1) is dict:
            out_data = self.__solveFromDictionary(arg1)
        else:
            raise RuntimeError("First argument must be a dictionary or a path to the lua data file")

        #restore original working directory
        os.chdir(original_dir)

        return out_data

    def __loadByLibName(self, ocp_name):

        if (platform.system()=='Darwin'):
            sl_ext = '.dylib'
        elif (platform.system()=='Linux'):
            sl_ext = '.so'
        else:
            raise RuntimeError("Current platform, ", platform.system(), " is not supported")

        cdef int err_code = 0;

        # library and cc file names
        libname = os.getcwd() + "/lib" + ocp_name + sl_ext
        cc_file = os.getcwd() + "/" + ocp_name + ".cc"

        #check if should compile the library
        should_compile = False
        if (self.__force_compile == CompileLevel.compile_always):
            should_compile = True
        elif (self.__force_compile == CompileLevel.compile_if_modified):
            if not(os.path.isfile(libname)):  #library does not exists
                should_compile = True
            else:
                if not(os.path.isfile(cc_file)): # if cc file is missing
                    raise RuntimeError("The library '" + libname + "' does not exist, nor the source file '" + cc_file + "' can be found.")
                #check modified date
                if (os.path.getmtime(libname) < os.path.getmtime(cc_file)): #if osurce file is more recent than library
                    should_compile = True
        elif (self.__force_compile == CompileLevel.compile_never ):
            if not(os.path.isfile(libname)): #if library does not exists but compile is forbidden
                raise RuntimeError("The library '" + libname + "' does not exist, and the 'compile_never' options has been specified. Cannot continue.")

        #if should compile,
        if (should_compile):
            if not(os.path.isfile(cc_file)): #check that the source file script exists
                raise RuntimeError("The library '" + libname + "' does not exist, nor the source file '" + cc_file + "' can be found.")
            if not(os.path.isfile( os.getcwd() + "/compile.sh" )): #check that thecompile script exists
                raise RuntimeError("The library '" + libname + "' does not exist, nor the 'compile.sh' script can be found.")
            if (self.__info_level >= InfoLevel.info_level_normal):
                print("Compiling library...")
            call(["./compile.sh"])


        cdef bytes libname_bytes = libname.encode()
        cdef char* c_lib_name = libname_bytes
        cdef char * p_err_mess;

        self.__ocp_id = loadMaverickOcpFromLib(c_lib_name, &err_code, &p_err_mess);

        if err_code is 1:
            error_mess = 'Cannot find shared library ' + libname + ': ' + p_err_mess.decode("utf-8")
            raise RuntimeError(error_mess)
        if err_code is 2:
            raise RuntimeError('Shared library does not implement the method getMaverickOcpPointer')
        if err_code is -1:
            raise RuntimeError('Error creating new MaverickOcp object')
        if err_code != 0:
            raise RuntimeError('Unknown error when loading MaverickOcp')

        self.__is_ocp_from_lib = True

        loadMaverickSolver(self.__ocp_id, &err_code);
        if err_code is 1:
            raise RuntimeError('A solver already exists for this problem, please delete it')
        if err_code is 2:
            raise RuntimeError('MaverickOcp object not found')
        if err_code is -1:
            raise RuntimeError('Error creating new solver object')
        if err_code != 0:
            raise RuntimeError('Unknown error when loading Maverick Solver')

    def __solveFromLua(self, lua_filename):
        gc_id = str(self.__ocp_id)

        # delete GC if exists
        GC_delete(gc_id.encode())
        #create empty GC now
        if (GC_select( gc_id.encode() )!=0):
            raise RuntimeError('Error selecting generic container ', gc_id)

        cdef void * p_gc = NULL
        p_gc = GC_mem_ptr(gc_id.encode())
        if (p_gc == NULL):
            raise RuntimeError('Error getting generic container pointer')

        cdef bytes lua_filename_bytes = lua_filename.encode()
        cdef char* c_lua_filename = lua_filename_bytes

        cdef bytes lua_global_var_bytes = "Data".encode()
        cdef char* c_lua_global_var = lua_global_var_bytes

        cdef int err_code = 0;
        cdef char err_mess[500];

        if (self.__info_level >= InfoLevel.info_level_normal):
            print 'Reading lua file...'

        fillGenericContainerFromLua(p_gc,
                                    c_lua_filename, c_lua_global_var,
                                    &err_code,
                                    err_mess, 500);

        if err_code is 1:
            message = 'Error reading lua:\n' + err_mess.decode("utf-8")
            raise RuntimeError(message)
        elif err_code is 2:
            message = 'Error converting lua to gc:\n' + err_mess.decode("utf-8")
            raise RuntimeError(message)
        elif err_code is 3:
            message = 'Error: null generic container pointer detected'
            raise RuntimeError(message)
        elif err_code is -1:
            message = 'Error creating new generic container:\n' + err_mess.decode("utf-8")
            raise RuntimeError(message)
        elif err_code != 0:
            raise RuntimeError('Unknown error when loading lua file')
        elif err_code is 0:
            if (self.__info_level >= InfoLevel.info_level_normal):
                print 'Lua file successfully read'

        #convert gc into dictionary
        #input_data = dict()
        #GC_select(gc_id.encode())
        #input_data = getCurrentGCContent(False)

        if (self.__info_level >= InfoLevel.info_level_normal):
            print('Now call Maverick C++ solver')

        cdef int spec_names = 0
        if (self.__specific_solution_names):
            spec_names = 1

        solveFromExternalGC(self.__ocp_id, p_gc, spec_names, p_gc,
                            &err_code,
                            err_mess, 500);

        #do NOT delete the input GC: the input GC now containts output data

        #check error messages
        if err_code is 1:
            raise RuntimeError('Solver ID not found')
        if err_code is 2:
            raise RuntimeError('Input GenericContainer null pointer')
        if err_code is 3:
            message = 'Error computing the solution:\n' + err_mess.decode("utf-8")
            raise RuntimeError(message)
        if err_code is 4:
            raise RuntimeError('Output GenericContainer pointer is null')
        if err_code is 5:
            message = 'Error writing the solution:\n' + err_mess.decode("utf-8")
            raise RuntimeError(message)
        if err_code != 0:
            raise RuntimeError('Unknown error when solving')

        #extract the data from the solution
        GC_select(gc_id.encode())
        out_data = getCurrentGCContent(False)

        #clean the current generic container
        GC_delete(gc_id.encode())

        if (self.__info_level >= InfoLevel.info_level_normal):
            print('Done!')

        #return the solution
        return out_data

    def __solveFromDictionary(self, data):
        gc_id = str(self.__ocp_id)
        if (self.__info_level >= InfoLevel.info_level_normal):
            print 'Transferring input data to C++ library'

        # delete GC if exists
        GC_delete(gc_id.encode())
        #create empty GC now
        if (GC_select( gc_id.encode() )!=0):
            raise RuntimeError('Error selecting generic container ', gc_id)

        convertDict2CurrentGC(data, False)

        cdef void* p_gc = GC_mem_ptr(gc_id.encode())
        if (p_gc == NULL):
            raise RuntimeError("Null pointer to input data GenericContainer")

        cdef char err_mess[500]
        cdef int err_code = 0

        if (self.__info_level >= InfoLevel.info_level_normal):
            print('Now call Maverick C++ solver')

        cdef int spec_names = 0
        if (self.__specific_solution_names):
            spec_names = 1

        solveFromExternalGC(self.__ocp_id, p_gc, spec_names, p_gc,
                                 &err_code,
                                 err_mess, 500);

        if err_code is 1:
            raise RuntimeError('Solver ID not found')
        if err_code is 2:
            raise RuntimeError('Input GenericContainer pointer null')
        if err_code is 3:
            message = 'Error computing the solution:\n' + err_mess.decode("utf-8")
            raise RuntimeError(message)
        if err_code is 4:
            raise RuntimeError('Output GenericContainer pointer is null')
        if err_code is 5:
            message = 'Error writing the solution:\n' + err_mess.decode("utf-8")
            raise RuntimeError(message)
        if err_code != 0:
            raise RuntimeError('Unknown error when solving')

        #extract the data from the solution
        GC_select(gc_id.encode())
        out_data = getCurrentGCContent(False)

        #clean the current generic container
        GC_delete(gc_id.encode())

        if (self.__info_level >= InfoLevel.info_level_normal):
            print('Done!')

        #return the solution
        return out_data

#Convert a python dictionary into a C generic container
cdef convertDict2CurrentGC(data, print_content=False):
    if (type(data)!=dict):
        raise ValueError('First input argument must be a dictionary!')
    if (type(print_content)!=bool):
        raise ValueError('Second input argument must be a boolean!')

    #create variable to store the error code
    cdef err = 0
    err = GC_set_map()
    if (err != 0):
        raise RuntimeError("Error in creating new map in GenericContainer")

    #create variable to store the key in bytes
    cdef bytes key_bytes
    #create variable to store the key in char[]
    cdef char* c_key
    #create variable to store list of doubles
    cdef double * double_list = NULL

    #loop the values
    for key in data:
        #print 'current key: ', key

        # reset error (shouldn't be necessary)
        err = 0;

        # store the key
        key_bytes = key.encode()
        c_key = key_bytes
        # insert key
        err = abs(GC_push_map_position(c_key))

        val = data[key]
        if (type(val)==int):
            #print 'intero ', key
            if ( (val <= 2 ** 31) and (val >= -(2 ** 31)) ):
                err = max(abs(GC_set_int(val)),err)
            else:
                err = max(abs(GC_set_real(val)),err)
            if (err != 0):
                raise RuntimeError("Error creating new int in GenericContainer")
        elif (type(val)==float):
            #print 'float ', key
            err = max(abs(GC_set_real(val)),err)
            if (err != 0):
                raise RuntimeError("Error creating new real in GenericContainer")

        elif (type(val)==list):
            if (isListOfNumbers(val)):
                #print 'list ', key
                if (double_list != NULL):
                    raise RuntimeError('Double list pointer should be NULL')

                double_list = <double *>malloc(len(val)*cython.sizeof(double))

                #check it is not null
                if double_list is NULL:
                    raise MemoryError('Error when allocating memory to store the element ', key)

                for i in xrange(len(val)):
                    double_list[i] = val[i]

                err = max(err,abs(GC_set_vector_of_real(double_list, len(val)) ))
                free(double_list)
                double_list = NULL
                if (err != 0):
                    raise RuntimeError("Error creating new vector of real in GenericContainer")

            elif (isListOfDictionary(val)):
                #set GC as list of vectors
                GC_set_vector(len(val))
                for i in xrange(len(val)):
                    err = GC_push_vector_position(i)
                    if (err != 0):
                        raise RuntimeError('Error pushing vector position of GenericContainer')

                    convertDict2CurrentGC(val[i], False)
                    err = GC_pop_head();
                    if (err != 0):
                        raise RuntimeError('Error pop head from vector position of GenericContainer')

            else:
                raise ValueError('List in dictionary at key ', key, ' is neither a list of numbers nor a list of dictionaries')


        elif (type(val)==str):
            #print 'stringa ', key
            err = max(abs(GC_set_string(val.encode())),err)
            if (err != 0):
                raise RuntimeError("Error creating new string in GenericContainer")

        elif (type(val)==dict):
            err = max(abs(GC_set_map()),err)
            if (err != 0):
                raise RuntimeError("Error creating new map in GenericContainer")
            convertDict2CurrentGC(val, False)

        else:
            raise ValueError('Dictionary element at key ', key, ' cannot be translated into C code because it is of type ' + str(type(val)) + '. Currently supported data types are int, list(int), float, list(float-int), list(dictionary), string.')

        err = max(abs(GC_pop_head()), err)
        if (err != 0):
            raise RuntimeError('GenericContainer error adding the ',key, ' key')

    if (print_content):
        GC_print()

    #print 'exiting'

#Convert a C generic container into a python dictionary
cdef getCurrentGCContent(print_content = False):

    if (type(print_content)!=bool):
        print 'Second input argument must be a boolean!'
        return

    cdef int gc_type = GC_get_type()
    # print 'current type', gc_type
    # GC_POINTER 1,
    # GC_COMPLEX 6,
    # // vector type
    # GC_VEC_POINTER 8 ,
    # GC_VEC_BOOL 9 ,
    # GC_VEC_COMPLEX 13,
    # GC_VEC_STRING 14,
    # // matrix type
    # GC_MAT_INTEGER 15,
    # GC_MAT_LONG 16,
    # GC_MAT_REAL 17,
    # GC_MAT_COMPLEX 18

    cdef char * stringa

    if (gc_type == 0): #GC_NOTYPE
        out_data = None

    elif (gc_type == 2): #   GC_BOOL
        tmp = GC_get_bool()
        if (tmp == 1):
            out_data = True
        else:
            out_data = False

    elif (gc_type == 3): # GC_INTEGER
        out_data = GC_get_int()

    elif (gc_type == 4): # GC_LONG
        out_data = GC_get_long()

    elif (gc_type == 5): # GC_REAL
        out_data = GC_get_real()

    elif (gc_type == 7): # GC_STRING
        stringa = GC_get_string()
        out_data = stringa.decode("utf-8")

    elif (gc_type == 10): # GC_VEC_INTEGER
        vec_length = GC_get_vector_size()
        out_data = [0] * vec_length
        for i in xrange( vec_length ):
            out_data[i] = GC_get_int_at_pos(i)

    elif (gc_type == 11): # GC_VEC_LONG
        vec_length = GC_get_vector_size()
        out_data = [0] * vec_length
        for i in xrange( vec_length ):
            out_data[i] = GC_get_long_at_pos(i)

    elif (gc_type == 12): # GC_VEC_REAL
        vec_length = GC_get_vector_size()
        out_data = [0] * vec_length
        for i in xrange( vec_length ):
            out_data[i] = GC_get_real_at_pos(i)

    elif (gc_type == 19): # GC_VECTOR 19
        vec_length = GC_get_vector_size()
        out_data = [0] * vec_length
        for i in xrange( vec_length ):
            err = GC_push_vector_position(i)
            if (err != 0):
                raise RuntimeError('Error pushing vector position of GenericContainer')
            out_data[i] = getCurrentGCContent( False)
            err = GC_pop_head();
            if (err != 0):
                raise RuntimeError('Error pop head from vector position of GenericContainer')

    elif (gc_type == 20): # GC_MAP 20
        out_data = dict()
        map_keys = []
        GC_init_map_key()
        while ( True ):
            stringa = GC_get_next_key()
            if (stringa == NULL):
                break
            python_key = stringa.decode("utf-8")
            map_keys.append(python_key)

        for key in map_keys:
            err_code = GC_push_map_position(key.encode())
            if (err_code != 0):
                raise RuntimeError("Error in getting map content of key", key)

            out_data[key] = getCurrentGCContent(False)
            err_code = GC_pop_head()
            if (err_code != 0):
                raise RuntimeError("Error in returning to top level map from key", key)

    else:
            raise ValueError("Error converting GenericContainer to dictionary: unsupported data type of type", gc_type)

    if (print_content):
        print(out_data)

    return out_data

#check if a list contains only numbers
cdef isListOfNumbers(input_list):
    for i in xrange(len(input_list)):
        if ( ( type(input_list[i]) != int ) and ( type(input_list[i]) != float ) ):
            return False
    return True

#check if a list contains only dictionaries
cdef isListOfDictionary(input_list):
    for i in xrange(len(input_list)):
        if  ( type(input_list[i]) != dict ):
            return False
    return True

#check if a list contains only strings
cdef isListOfStrings(input_list):
    for i in xrange(len(input_list)):
        if  ( type(input_list[i]) != str ):
            return False
    return True


def readTextTable( filename, arg1 = 0, arg2 = 0 ):
    delimiter = ' '
    headers = []

    if type(filename) is not str:
        raise ValueError('First argument must be a string')

    gen_args = [arg1, arg2]
    for arg in gen_args:
        if type(arg) is str:
            delimiter = arg
        elif type(arg) is list:
            if not(isListOfStrings(arg)):
                raise ValueError('Headers list must be a list of strings')
            else:
                headers = arg
        else:
            if arg is not 0:
                raise ValueError('Optional argument can be the delimiter (string) or the headers list (list of strings)')
    #REAL FILE
    if (headers == []):
        is_first_line = True
    else:
        is_first_line = False
        n_cols = len(headers)
        data = dict()
        for header in headers:
            data[header] = []

    with open(filename, newline='') as f:
        for line in f:
            values = line.strip().split(delimiter)
            if (is_first_line):
                n_cols = len(values)
                headers = values
                data = dict()
                for header in values:
                    data[header] = []
                is_first_line = False
            else:
                for i in xrange(n_cols):
                    data[headers[i]].append(float(values[i]))

    return data
