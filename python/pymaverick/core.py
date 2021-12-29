import ctypes
import numbers
import string
import random
import numpy as np
from pymaverick import spline


class VoidPtr:

    def __init__(self, ptr):
        self.ptr = ptr

    def __str__(self):
        return str(self.ptr)

    def __repr__(self):
        return 'VoidPtr(' + self.__str__() + ')'


def set_library_name(fname) -> None:
    global _mav_lib_fname
    _mav_lib_fname = fname


def load_library(fname=None) -> None:
    global _mav_lib, _mav_lib_fname
    if fname is not None:
        set_library_name(fname)
    fname = _mav_lib_fname if _mav_lib_fname is not None else 'libmaverick.so'
    _mav_lib = ctypes.CDLL(fname)

    # Assert the library implements all the functions and specify the function singatures
    signatures = dict(_splines_signatures, **_gc_signatures, **_solver_signatures)
    
    for k, argtypes in signatures.items():
        if type(argtypes) is str:
            argtypes = signatures[argtypes]

        tmp = getattr(_mav_lib, k)
        tmp.argtypes = argtypes[1]
        tmp.restype = argtypes[0]


def check_init(function):
    def wrapper(*args, **kwargs):
        if _mav_lib is None:
            load_library()
        return function(*args, **kwargs)
    return wrapper


def numpy_to_maverick_array(x: np.ndarray, order='c', allow_recreate=True):
    assert order in ['c', 'f']
    if allow_recreate:
        if order == 'f':
            if not x.flags.f_contiguous:
                x = np.asfortranarray(x, dtype=np.float64)
        else:
            if not x.flags.c_contiguous:
                x = np.ascontiguousarray(x, dtype=np.float64)
        if x.dtype != np.float64:
            x = x.astype(np.float64)
    assert x.dtype == np.float64
    if order == 'f':
        assert x.flags.f_contiguous
    else:
        assert x.flags.c_contiguous
    return x.ctypes.data_as(_c_mav_double_p)


_mav_lib_fname = None
_mav_lib = None

_c_mav_int = ctypes.c_int64
_c_mav_max_int = np.iinfo(np.int64).max
_c_mav_int_p = ctypes.POINTER(ctypes.c_int64)
_c_mav_size = ctypes.c_size_t
_c_mav_double = ctypes.c_double
_c_mav_double_p = ctypes.POINTER(ctypes.c_double)
_c_mav_char_p = ctypes.c_char_p
_c_mav_char_p_p = ctypes.POINTER(ctypes.c_char_p)
_bf_len = 3000
_err_bf = ctypes.create_string_buffer(3000)

###########
# SPLINES #
###########

_splines_signatures = {
    'create1DSpline': (ctypes.c_void_p, [_c_mav_char_p, _c_mav_size, _c_mav_double_p, _c_mav_double_p, ctypes.c_bool,
                                         _c_mav_char_p, _c_mav_size]),
    'create2DSpline': (ctypes.c_void_p, [_c_mav_char_p, _c_mav_size, _c_mav_double_p, _c_mav_size, _c_mav_double_p,
                                         _c_mav_double_p, ctypes.c_bool, ctypes.c_bool, ctypes.c_bool, _c_mav_char_p,
                                         _c_mav_size]),
    'delete1DSpline': (None, [ctypes.c_void_p]),
    'delete2DSpline': (None, [ctypes.c_void_p]),

    'eval1DSpline': (_c_mav_int, [ctypes.c_void_p, _c_mav_double, _c_mav_double_p, _c_mav_char_p, _c_mav_size]),
    'eval1DSplineD': 'eval1DSpline',
    'eval1DSplineDD': 'eval1DSpline',
    'eval1DSplineVec': (_c_mav_int, [ctypes.c_void_p, _c_mav_size, _c_mav_double_p, _c_mav_double_p, _c_mav_char_p, _c_mav_size]),
    'eval1DSplineDVec': 'eval1DSplineVec',
    'eval1DSplineDDVec': 'eval1DSplineVec',

    'eval2DSpline': (_c_mav_int, [ctypes.c_void_p, _c_mav_double, _c_mav_double, _c_mav_double_p, _c_mav_char_p, _c_mav_size]),
    'eval2DSplineD1': 'eval2DSpline',
    'eval2DSplineD2': 'eval2DSpline',
    'eval2DSplineD11': 'eval2DSpline',
    'eval2DSplineD12': 'eval2DSpline',
    'eval2DSplineD22': 'eval2DSpline',
    'eval2DSplineVec': (_c_mav_int, [ctypes.c_void_p, _c_mav_size, _c_mav_double_p, _c_mav_double_p, _c_mav_double_p, _c_mav_char_p, _c_mav_size]),
    'eval2DSplineD1Vec': 'eval2DSplineVec',
    'eval2DSplineD2Vec': 'eval2DSplineVec',
    'eval2DSplineD11Vec': 'eval2DSplineVec',
    'eval2DSplineD22Vec': 'eval2DSplineVec',
    'eval2DSplineD12Vec': 'eval2DSplineVec',

    'getSpline1DSize': (_c_mav_size, [ctypes.c_void_p, _c_mav_char_p, _c_mav_size]),
    'getSpline1DPoints': (_c_mav_int, [ctypes.c_void_p, _c_mav_size, _c_mav_double_p,
                                       _c_mav_double_p, _c_mav_char_p, _c_mav_size]),

    'getSpline2DSizeX': (_c_mav_size, [ctypes.c_void_p, _c_mav_char_p, _c_mav_size]),
    'getSpline2DSizeY': (_c_mav_size, [ctypes.c_void_p, _c_mav_char_p, _c_mav_size]),
    'getSpline2DPoints': (_c_mav_int, [ctypes.c_void_p, _c_mav_size, _c_mav_double_p, _c_mav_size, _c_mav_double_p,
                                       _c_mav_double_p, _c_mav_char_p, _c_mav_size]),

}


@check_init
def create_c_spline_1d(spline_type, x, y):
    c_x = numpy_to_maverick_array(x)
    c_y = numpy_to_maverick_array(y)
    assert x.shape == y.shape
    check_range = True
    spline_ptr = _mav_lib.create1DSpline(
        spline_type.encode(), x.size, c_x, c_y, check_range, _err_bf, _bf_len)
    if spline_ptr is None:
        raise RuntimeError(_err_bf.value.decode('utf-8'))
    return VoidPtr(spline_ptr)


@check_init
def eval_c_spline_1d(spline_ptr, x, derivative=None):
    spline_ptr = spline_ptr.ptr
    function_name = 'eval1DSpline' + (derivative if derivative is not None else '')
    y_out = _c_mav_double(0)
    if getattr(_mav_lib, function_name)(spline_ptr, x, y_out, _err_bf, _bf_len) != 0:
        raise RuntimeError(_err_bf.value.decode('utf-8'))
    return y_out.value


@check_init
def eval_c_spline_1d_vec(spline_ptr, x, derivative=None):
    spline_ptr = spline_ptr.ptr
    function_name = 'eval1DSpline' + (derivative if derivative is not None else '') + 'Vec'
    y_out = np.empty(x.shape, dtype=np.float64)
    x = numpy_to_maverick_array(x)
    y = numpy_to_maverick_array(y_out, allow_recreate=False)
    if getattr(_mav_lib, function_name)(spline_ptr, y_out.size, x, y, _err_bf, _bf_len) != 0:
        raise RuntimeError(_err_bf.value.decode('utf-8'))
    return y_out


@check_init
def get_c_spline_1d_points(spline_ptr):
    spline_ptr = spline_ptr.ptr
    size = _mav_lib.getSpline1DSize(spline_ptr, _err_bf, _bf_len)
    if size == 0:
        raise RuntimeError(_err_bf.value.decode('utf-8'))
    x = np.empty(size)
    y = np.empty(size)
    c_x = numpy_to_maverick_array(x, allow_recreate=False)
    c_y = numpy_to_maverick_array(y, allow_recreate=False)
    err = _mav_lib.getSpline1DPoints(spline_ptr, size, c_x, c_y, _err_bf, _bf_len)
    if err != 0:
        raise RuntimeError(_err_bf.value.decode('utf-8'))
    return x, y


@check_init
def create_c_spline_2d(spline_type, x, y, z, order):
    c_x = numpy_to_maverick_array(x)
    c_y = numpy_to_maverick_array(y)
    c_z = numpy_to_maverick_array(z, order=order)
    assert z.size == x.size * y.size
    check_range = True
    fortran_order = order == 'f'
    transposed = False
    spline_ptr = _mav_lib.create2DSpline(
        spline_type.encode(), x.size, c_x, y.size, c_y, c_z, check_range, fortran_order, transposed, _err_bf, _bf_len)
    if spline_ptr is None:
        raise RuntimeError(_err_bf.value.decode('utf-8'))
    return VoidPtr(spline_ptr)


@check_init
def eval_c_spline_2d(spline_ptr, x, y, derivative=None):
    spline_ptr = spline_ptr.ptr
    function_name = 'eval2DSpline' + (derivative if derivative is not None else '')
    z_out = _c_mav_double(0)
    if getattr(_mav_lib, function_name)(spline_ptr, x, y, z_out, _err_bf, _bf_len) != 0:
        raise RuntimeError(_err_bf.value.decode('utf-8'))
    return z_out.value


@check_init
def eval_c_spline_2d_vec(spline_ptr, x, y, derivative=None):
    spline_ptr = spline_ptr.ptr
    function_name = 'eval2DSpline' + (derivative if derivative is not None else '') + 'Vec'
    assert x.shape == y.shape
    z_out = np.empty(x.shape, dtype=np.float64)
    x = numpy_to_maverick_array(x)
    y = numpy_to_maverick_array(y)
    z = numpy_to_maverick_array(z_out, allow_recreate=False)
    if getattr(_mav_lib, function_name)(spline_ptr, z_out.size, x, y, z, _err_bf, _bf_len) != 0:
        raise RuntimeError(_err_bf.value.decode('utf-8'))
    return z_out


@check_init
def get_c_spline_2d_points(spline_ptr):
    spline_ptr = spline_ptr.ptr
    size_x = _mav_lib.getSpline2DSizeX(spline_ptr, _err_bf, _bf_len)
    if size_x == 0:
        raise RuntimeError(_err_bf.value.decode('utf-8'))
    size_y = _mav_lib.getSpline2DSizeY(spline_ptr, _err_bf, _bf_len)
    if size_y == 0:
        raise RuntimeError(_err_bf.value.decode('utf-8'))
    x = np.empty(size_x)
    y = np.empty(size_y)
    z = np.empty((size_x, size_y))
    c_x = numpy_to_maverick_array(x, allow_recreate=False)
    c_y = numpy_to_maverick_array(y, allow_recreate=False)
    c_z = numpy_to_maverick_array(z, order='c', allow_recreate=False)
    err = _mav_lib.getSpline2DPoints(spline_ptr, size_x, c_x, size_y, c_y, c_z, _err_bf, _bf_len)
    if err != 0:
        raise RuntimeError(_err_bf.value.decode('utf-8'))
    return x, y, z


@check_init
def delete_spline_1d(spline_ptr):
    spline_ptr = spline_ptr.ptr
    _mav_lib.delete1DSpline(spline_ptr)


@check_init
def delete_spline_2d(spline_ptr):
    spline_ptr = spline_ptr.ptr
    _mav_lib.delete2DSpline(spline_ptr)


#####################
# GENERIC CONTAINER #
#####################

_gc_signatures = {
    'GC_new': (_c_mav_int, [_c_mav_char_p]),
    'GC_select': 'GC_new',
    'GC_delete': 'GC_new',
    'GC_fill_for_test': 'GC_new',
    'GC_pop_head': (_c_mav_int, []),
    'GC_reset_head': 'GC_pop_head',
    'GC_print': 'GC_pop_head',
    'GC_get_type': 'GC_pop_head',
    'GC_get_type_name': (_c_mav_char_p, []),
    'GC_mem_ptr': (ctypes.c_void_p, [_c_mav_char_p]),
    #####
    'GC_set_bool': (_c_mav_int, [_c_mav_int]),
    'GC_set_int': (_c_mav_int, [_c_mav_int]),
    'GC_set_real': (_c_mav_int, [_c_mav_double]),
    'GC_set_string': (_c_mav_int, [_c_mav_char_p]),
    'GC_set_void_pointer': (_c_mav_int, [ctypes.c_void_p]),
    #####
    'GC_get_bool': (_c_mav_int, []),
    'GC_get_int': (_c_mav_int, []),
    'GC_get_long': (ctypes.c_long, []),
    'GC_get_real': (_c_mav_double, []),
    'GC_get_string': (_c_mav_char_p, []),
    'GC_get_void_pointer': (ctypes.c_void_p, []),
    #####
    'GC_push_bool': (_c_mav_int, [_c_mav_int]),
    'GC_push_int': (_c_mav_int, [_c_mav_int]),
    'GC_push_real': (_c_mav_int, [_c_mav_double]),
    'GC_push_string': (_c_mav_int, [_c_mav_char_p]),
    #####
    'GC_get_bool_at_pos': (_c_mav_int, [_c_mav_int]),
    'GC_get_int_at_pos': (_c_mav_int, [_c_mav_int]),
    'GC_get_long_at_pos': (ctypes.c_long, [_c_mav_int]),
    'GC_get_real_at_pos': (_c_mav_double, [_c_mav_int]),
    'GC_get_string_at_pos': (_c_mav_char_p, [_c_mav_int]),
    #####
    'GC_get_real_at_coor': (_c_mav_double, [_c_mav_int, _c_mav_int]),
    #####
    'GC_set_empty_vector_of_bool': (_c_mav_int, []),
    'GC_set_empty_vector_of_int': (_c_mav_int, []),
    'GC_set_empty_vector_of_real': (_c_mav_int, []),
    'GC_set_empty_vector_of_string': (_c_mav_int, []),
    #####
    'GC_set_vector_of_bool': (_c_mav_int, [_c_mav_int_p, _c_mav_int]),
    'GC_set_vector_of_int': (_c_mav_int, [_c_mav_int_p, _c_mav_int]),
    'GC_set_vector_of_real': (_c_mav_int, [_c_mav_double_p, _c_mav_int]),
    'GC_set_vector_of_string': (_c_mav_int, [_c_mav_char_p, _c_mav_int]),
    #####
    'GC_set_vector': (_c_mav_int, [_c_mav_int]),
    'GC_set_empty_vector': (_c_mav_int, []),
    'GC_get_vector_size': (_c_mav_int, []),
    'GC_get_matrix_num_rows': (_c_mav_int, []),
    'GC_get_matrix_num_cols': (_c_mav_int, []),
    'GC_push_vector_position': (_c_mav_int, [_c_mav_int]),
    #####
    'GC_set_map': (_c_mav_int, []),
    'GC_init_map_key': (_c_mav_int, []),
    'GC_get_next_key': (_c_mav_char_p, []),
    'GC_push_map_position': (_c_mav_int, [_c_mav_char_p]),

    #####
    'fillGenericContainerFromLua': (_c_mav_int, [ctypes.c_void_p, _c_mav_char_p, _c_mav_char_p, _c_mav_char_p, _c_mav_size]),

}

_gc_ids = set()


def _get_random_string(length):
    # choose from all lowercase letter
    letters = string.ascii_lowercase
    return ''.join(random.choice(letters) for i in range(length))


@check_init
def delete_gc(gc_id: str):
    if _mav_lib.GC_delete(gc_id.encode()) != 0:
        raise RuntimeError('Error deleting generic container ', gc_id)


@check_init
def _select_or_new_gc(gc_id: str = None):
    if gc_id is None:
        gc_id = _get_random_string(20)
        while gc_id in _gc_ids:
            gc_id = _get_random_string(20)

    # delete GC if exists
    if _mav_lib.GC_select(gc_id.encode()) != 0:
        raise RuntimeError('Error selecting generic container ', gc_id)

    _gc_ids.add(gc_id)
    return gc_id


@check_init
def get_gc_memptr(gc_id: str):
    _select_or_new_gc(gc_id)
    ptr = _mav_lib.GC_mem_ptr(gc_id.encode())
    if ptr is None:
        raise RuntimeError("Error getting ptr for gc: " + gc_id)
    return ptr


@check_init
def convert_dict_to_gc(data, print_content=False):
    gc_id = _select_or_new_gc()
    _convert_data_to_current_gc(data, print_content)
    return gc_id


def _is_array_of_numbers(arr):
    if type(arr) is np.ndarray:
        return arr.dtype.kind in ['i', 'f']
    return all(isinstance(x, numbers.Number) for x in arr)


def _convert_data_to_current_gc(data, print_content=False):

    if type(data) is dict:
        err = _mav_lib.GC_set_map()
        if err != 0:
            raise RuntimeError("Error in creating new map in GenericContainer")
        # loop the values
        for key, val in data.items():
            # insert key
            err = _mav_lib.GC_push_map_position(key.encode())
            if err != 0:
                raise RuntimeError("Error in pushing map in GenericContainer")
            _convert_data_to_current_gc(val, print_content)

            err = _mav_lib.GC_pop_head()
            if err != 0:
                raise RuntimeError('GenericContainer error adding the ', key, ' key')

    elif isinstance(data, numbers.Number):

        is_int = (data <= _c_mav_max_int) and (data >= -_c_mav_max_int) and (data == int(data))
        if is_int:
            err = _mav_lib.GC_set_int(int(data))
        else:
            err = _mav_lib.GC_set_real(data)

        if err != 0:
            raise RuntimeError("Error creating new int/real in GenericContainer")

    elif type(data) in [list, np.ndarray]:

        if _is_array_of_numbers(data):
            if type(data) is not np.ndarray:
                data = np.array(data)
            if data.dtype != np.float64:
                data = data.astype(np.float64)

            err = _mav_lib.GC_set_vector_of_real(numpy_to_maverick_array(data), len(data))
            if err != 0:
                raise RuntimeError("Error creating new vector of real in GenericContainer")

        else:
            # set GC as list of vectors
            _mav_lib.GC_set_vector(len(data))
            for i, list_val in enumerate(data):
                err = _mav_lib.GC_push_vector_position(i)
                if err != 0:
                    raise RuntimeError('Error pushing vector position of GenericContainer')

                _convert_data_to_current_gc(list_val, print_content)
                err = _mav_lib.GC_pop_head()
                if err != 0:
                    raise RuntimeError('Error pop head from vector position of GenericContainer')

    elif type(data) is str:
        err = _mav_lib.GC_set_string(data.encode())
        if err != 0:
            raise RuntimeError("Error creating new string in GenericContainer")

    elif type(data) is bool:
        err = _mav_lib.GC_set_bool(data)
        if err != 0:
            raise RuntimeError("Error creating new bool in GenericContainer")

    elif type(data) is spline.Spline1D:
        x, y = data.get_points()
        new_data = {
            'type': "1arg_spline",
            'spline_type': data.get_type(),
            'x': x,
            'y': y,
        }
        _convert_data_to_current_gc(new_data, print_content)

    elif type(data) is spline.Spline2D:
        x, y, z = data.get_points()
        new_data = {
            'type': "2arg_spline",
            'spline_type': data.get_type(),
            'x': x,
            'y': y,
            'z': z,
        }
        _convert_data_to_current_gc(new_data, print_content)

    elif type(data) is VoidPtr:
        err = _mav_lib.GC_set_void_pointer(data.ptr)
        if err != 0:
            raise RuntimeError("Error creating new void* in GenericContainer")

    else:
        raise RuntimeError('Data of type ' + str(type(data))
                           + ' cannot be translated into C code because it is not supported yet')

    if print_content:
        _mav_lib.GC_print()


@check_init
def get_gc_content(gc_id):
    _select_or_new_gc(gc_id)
    return _get_current_gc_content()


def _get_current_gc_content(print_content=False):

    gc_type = _mav_lib.GC_get_type()
    if print_content:
        print('Current type:', gc_type)
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

    if gc_type == 0:  # GC_NOTYPE
        out_data = None

    elif gc_type == 1:  # GC_POINTER
        out_data = VoidPtr(_mav_lib.GC_get_void_pointer())

    elif gc_type == 2:  # GC_BOOL
        out_data = _mav_lib.GC_get_bool() == 1

    elif gc_type == 3:  # GC_INTEGER
        out_data = _mav_lib.GC_get_int()

    elif gc_type == 4:  # GC_LONG
        out_data = _mav_lib.GC_get_long()

    elif gc_type == 5:  # GC_REAL
        out_data = _mav_lib.GC_get_real()

    elif gc_type == 7:  # GC_STRING
        out_data = _mav_lib.GC_get_string().decode("utf-8")

    elif gc_type == 10:  # GC_VEC_INTEGER
        out_data = np.array([_mav_lib.GC_get_int_at_pos(i) for i in range(_mav_lib.GC_get_vector_size())])

    elif gc_type == 11:  # GC_VEC_LONG
        out_data = np.array([_mav_lib.GC_get_int_at_pos(i) for i in range(_mav_lib.GC_get_vector_size())])

    elif gc_type == 12:  # GC_VEC_REAL
        out_data = np.array([_mav_lib.GC_get_real_at_pos(i) for i in range(_mav_lib.GC_get_vector_size())])

    elif gc_type == 19:  # GC_VECTOR 19
        vec_length = _mav_lib.GC_get_vector_size()
        out_data = [0] * vec_length
        for i in range(vec_length):
            err = _mav_lib.GC_push_vector_position(i)
            if err != 0:
                raise RuntimeError('Error pushing vector position of GenericContainer')
            out_data[i] = _get_current_gc_content(print_content)
            err = _mav_lib.GC_pop_head()
            if err != 0:
                raise RuntimeError('Error pop head from vector position of GenericContainer')

    elif gc_type == 20:  # GC_MAP 20
        out_data = dict()
        map_keys = []
        err = _mav_lib.GC_init_map_key()
        if err != 0:
            raise RuntimeError('Error initializing map keys of GenericContainer')
        while True:
            the_string = _mav_lib.GC_get_next_key()
            if the_string is None:
                break
            map_keys.append(the_string.decode("utf-8"))

        for key in map_keys:
            err_code = _mav_lib.GC_push_map_position(_c_mav_char_p(key.encode()))
            if err_code != 0:
                raise RuntimeError("Error in getting map content of key", key)

            out_data[key] = _get_current_gc_content(print_content)
            err_code = _mav_lib.GC_pop_head()
            if err_code != 0:
                raise RuntimeError("Error in returning to top level map from key", key)

    else:
        raise ValueError("Error converting GenericContainer to dictionary: unsupported data type of type", gc_type)

    if print_content:
        print(out_data)

    return out_data


@check_init
def lua_to_gc(lua_filename):

    lua_global_var = "Data".encode()
    gc_id = _select_or_new_gc()
    gc_mem_ptr = get_gc_memptr(gc_id)

    err_code = _mav_lib.fillGenericContainerFromLua(
        gc_mem_ptr, lua_filename.encode(), lua_global_var,
        _err_bf, _bf_len)

    if err_code != 0:
        raise RuntimeError(_err_bf.value.decode('utf-8'))
    return gc_id


##########
# SOLVER #
##########

_solver_signatures = {
    'getMaverickOcpFromLib': (ctypes.c_void_p, [_c_mav_char_p, _c_mav_char_p, _c_mav_size]),
    'deleteMaverickOcp': (None, [ctypes.c_void_p]),
    'getMaverickOcpSolver': (ctypes.c_void_p, [ctypes.c_void_p, _c_mav_char_p, _c_mav_size]),
    'deleteMaverickOcpSolver': (None, [ctypes.c_void_p]),
    'solveMaverick': (_c_mav_int, [ctypes.c_void_p, ctypes.c_void_p, _c_mav_int, ctypes.c_void_p, _c_mav_char_p,
                                   _c_mav_size]),
}


@check_init
def solve(solver_ptr, gc_in_ptr):

    gc_out_id = _select_or_new_gc()
    gc_out_ptr = get_gc_memptr(gc_out_id)
    spec_names = 1

    err_code = _mav_lib.solveMaverick(solver_ptr, gc_in_ptr, spec_names, gc_out_ptr, _err_bf, _bf_len)
    if err_code != 0:
        delete_gc(gc_out_id)
        raise RuntimeError(_err_bf.value.decode('utf-8'))

    # Extract the data from the solution
    data = get_gc_content(gc_out_id)
    delete_gc(gc_out_id)
    
    return data


@check_init
def get_maverick_ocp_from_lib(libname):
    ptr = _mav_lib.getMaverickOcpFromLib(libname.encode(), _err_bf, _bf_len)
    if ptr is None:
        raise RuntimeError(_err_bf.value.decode('utf-8'))
    return ptr


@check_init
def delete_maverick_ocp(ptr):
    _mav_lib.deleteMaverickOcp(ptr)


@check_init
def get_maverick_solver(ocp_ptr):
    ptr = _mav_lib.getMaverickOcpSolver(ocp_ptr, _err_bf, _bf_len)
    if ptr is None:
        raise RuntimeError(_err_bf.value.decode('utf-8'))
    return ptr


@check_init
def delete_maverick_solver(ptr):
    _mav_lib.deleteMaverickOcpSolver(ptr)
