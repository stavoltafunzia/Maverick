import numbers
import pymaverick.core as mvc


class Spline1D:

    def __init__(self, spline_type: str, x, y) -> None:
        self.assert_array(x)
        self.assert_array(y)
        if x.size == 0:
            raise RuntimeError("Cannot create empty spline")
        assert x.shape[0] == y.shape[0]
        self._ptr = mvc.create_c_spline_1d(spline_type, x, y)
        self._spline_type = spline_type
        assert self._ptr != 0, "Creation of C spline failed"

    def __del__(self):
        mvc.delete_spline_1d(self._ptr)

    def get_points(self):
        return mvc.get_c_spline_1d_points(self._ptr)

    def eval(self, x):
        return self._eval(x, '')
    
    def eval_d(self, x):
        return self._eval(x, 'D')

    def eval_dd(self, x):
        return self._eval(x, 'DD')

    def get_type(self):
        return str(self._spline_type)

    def _eval(self, x, eval_type):
        if isinstance(x, numbers.Number):
            return mvc.eval_c_spline_1d(self._ptr, x, derivative=eval_type)
        self.assert_array(x)
        return mvc.eval_c_spline_1d_vec(self._ptr, x, derivative=eval_type)

    @staticmethod
    def assert_array(x):
        assert len(x.shape) == 1
        assert x.dtype.kind in {'i', 'f'}


class Spline2D:

    def __init__(self, spline_type: str, x, y, z, order='c', **super_kwargs) -> None:
        assert order in ['c', 'f']
        super().__init__(**super_kwargs)
        Spline1D.assert_array(x)
        Spline1D.assert_array(y)
        if x.size == 0 or y.size == 0:
            raise RuntimeError("Cannot create empty spline")
        self.assert_array(z)
        self._spline_type = spline_type
        assert z.shape == (x.size, y.size)
        self._ptr = mvc.create_c_spline_2d(spline_type, x, y, z, order=order)
        assert self._ptr != 0, "Creation of C spline failed"

    def __del__(self):
        mvc.delete_spline_2d(self._ptr)

    def get_points(self):
        return mvc.get_c_spline_2d_points(self._ptr)

    def eval(self, x, y):
        return self._eval(x, y, '')

    def eval_d1(self, x, y):
        return self._eval(x, y, 'D1')

    def eval_d2(self, x, y):
        return self._eval(x, y, 'D2')

    def eval_dd1(self, x, y):
        return self._eval(x, y, 'DD1')

    def eval_dd2(self, x, y):
        return self._eval(x, y, 'DD2')

    def eval_d1_d2(self, x, y):
        return self._eval(x, y, 'D1D2')

    def get_type(self):
        return str(self._spline_type)
    
    def _eval(self, x, y, eval_type):
        if isinstance(x, numbers.Number) and isinstance(y, numbers.Number):
            return mvc.eval_c_spline_2d(self._ptr, x, y, derivative=eval_type)
        Spline1D.assert_array(x)
        Spline1D.assert_array(y)
        return mvc.eval_c_spline_2d_vec(self._ptr, x, y, derivative=eval_type)

    @staticmethod
    def assert_array(x):
        assert len(x.shape) == 2
        assert x.dtype.kind in {'i', 'f'}
