from distutils.core import setup
from Cython.Build import cythonize
from distutils.extension import Extension

extensions = Extension("maverick", ["maverick.pyx"],
        include_dirs = ["/usr/local/maverick/include"],
        libraries = ["maverick"],
        library_dirs = ["/usr/local/maverick/lib","./"])
        #language="c++")

setup(
    ext_modules=cythonize(extensions)
)
