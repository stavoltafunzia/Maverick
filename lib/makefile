OS := $(shell uname)

# Define some variables for compilatin process
LIBNAME := libmaverick
LIBNAMEIP := libmavericktoip
LIBNAMETS := libmaverickts
PYTHONLIBNAME := maverick

ifeq ($(OS),Darwin) # OsX
	PLATFORM_SUFFIX := macosx
else ifeq ($(OS),Linux)
	PLATFORM_SUFFIX := linux
endif

#install directory
ifeq ($(INSTALL_PATH),)
	INSTALL_PATH := /usr/local/maverick
endif

#release or debug version
ifeq ($(MAVERICK_DEBUG),true)
	BUILD_TARGET:="debug"
else
	BUILD_TARGET:="release"
endif

#default ipopt
ifeq ($(IPOPT_INCLUDE),)
	IPOPT_INCLUDE := -I/usr/local/ipopt/include/coin
endif
ifeq ($(IPOPT_LIB),)
	IPOPT_LIB := -L/usr/local/ipopt/lib -lipopt
endif

#default lua
ifeq ($(LUA_LIB),)
	THISLUA=true
endif
ifeq ($(THISLUA),true)
	LUA_INCLUDE := -Isrc/Lua/src
	LUA_LIB := -Lsrc/Lua/src -llua_$(PLATFORM_SUFFIX)
endif

#default python command: python3
ifeq ($(PYTHON3),)
	PYTHON=python3
endif

#optimisation flags
ifeq ($(OPTIMISATION),)
	OPTIMISATION:="generic"
	#OPTIMISATION="native"
endif

ifeq ($(PLATFORM),)
	PLATFORM:="x86_64"
	#PLATFORM="armv7"
endif

# define the C compiler to use
CXX := g++
FF := gfortran
AR := ar
ARFLAGS := rvs

# END OF CUSTOM SETTINGS ----------------------------------------------------------

ifeq ($(PLATFORM),"x86_64")
    ifeq ($(OS),Darwin)
        CPPADDFLAG := -m64 -arch x86_64 -Wall
    else
        CPPADDFLAG := -m64 -fopenmp -Wall
	endif
else #armv7
    CPPADDFLAG := -fopenmp
endif

ifeq ($(OPTIMISATION),"native")
    OPTIM_FLAGS := -O3 -march=native -funroll-loops
else
    ifeq ($(PLATFORM),"x86_64")
        OPTIM_FLAGS := -O3 -msse2 -msse -mmmx -funroll-loops
	else #armv7
        OPTIM_FLAGS := -O3 -funroll-loops
	endif
endif

ifeq ($(BUILD_TARGET),"debug")
   # -g debug for gdb
   BUILD_FLAGS := -DMAVERICK_DEBUG -g -DDEBUG -D_GLIBXX_DEBUG_PEDANTIC
	 OPTIM_FLAGS :=
else
    #-DNDEBUG speed up eigen
   BUILD_FLAGS := -DNDEBUG
endif

MAVERICK_CORE_PATH = ./src/MaverickCore
MAVERICK_CORE_SRC_FILES = $(shell find $(MAVERICK_CORE_PATH) -name '*.cc')
MAVERICK_CORE_SRC = $(MAVERICK_CORE_SRC_FILES:$(MAVERICK_CORE_PATH)/%=%)
MAVERICK_CORE_OBJS = ${MAVERICK_CORE_SRC:.cc=.o}
MAVERICK_CORE_OBJS_FILES = ${MAVERICK_CORE_OBJS:%=$(MAVERICK_CORE_PATH)/%}

MAVERICK_TOIPOPT_PATH = ./src/Maverick2Ipopt
MAVERICK_TOIPOPT_SRC_FILES = $(shell find $(MAVERICK_TOIPOPT_PATH) -name '*.cc')
MAVERICK_TOIPOPT_SRC = $(MAVERICK_TOIPOPT_SRC_FILES:$(MAVERICK_TOIPOPT_PATH)/%=%)
MAVERICK_TOIPOPT_OBJS = ${MAVERICK_TOIPOPT_SRC:.cc=.o}
MAVERICK_TOIPOPT_OBJS_FILES = ${MAVERICK_TOIPOPT_OBJS:%=$(MAVERICK_TOIPOPT_PATH)/%}

MAVERICK_TENSOLVE_PATH = ./src/MaverickTensolve
MAVERICK_TENSOLVE_SRC_FILES = $(shell find $(MAVERICK_TENSOLVE_PATH) -name '*.cc')
MAVERICK_TENSOLVE_SRC = $(MAVERICK_TENSOLVE_SRC_FILES:$(MAVERICK_TENSOLVE_PATH)/%=%)
MAVERICK_TENSOLVE_OBJS = ${MAVERICK_TENSOLVE_SRC:.cc=.o}
MAVERICK_TENSOLVE_OBJS_FILES = ${MAVERICK_TENSOLVE_OBJS:%=$(MAVERICK_TENSOLVE_PATH)/%}

MAVERICK_GC_PATH = ./src/MaverickGC
MAVERICK_GC_SRC_FILES = $(shell find $(MAVERICK_GC_PATH) -name '*.cc')
MAVERICK_GC_SRC = $(MAVERICK_GC_SRC_FILES:$(MAVERICK_GC_PATH)/%=%)
MAVERICK_GC_OBJS = ${MAVERICK_GC_SRC:.cc=.o}
MAVERICK_GC_OBJS_FILES = ${MAVERICK_GC_OBJS:%=$(MAVERICK_GC_PATH)/%}

MAVERICK_SPLINES_PATH = ./src/MaverickSplines
MAVERICK_SPLINES_SRC_FILES = $(shell find $(MAVERICK_SPLINES_PATH) -name '*.cc')
MAVERICK_SPLINES_SRC = $(MAVERICK_SPLINES_SRC_FILES:$(MAVERICK_SPLINES_PATH)/%=%)
MAVERICK_SPLINES_OBJS = ${MAVERICK_SPLINES_SRC:.cc=.o}
MAVERICK_SPLINES_OBJS_FILES = ${MAVERICK_SPLINES_OBJS:%=$(MAVERICK_SPLINES_PATH)/%}

MAVERICK_UTILS_PATH = ./src/MaverickUtils
MAVERICK_UTILS_SRC_FILES = $(shell find $(MAVERICK_UTILS_PATH) -name '*.cc')
MAVERICK_UTILS_SRC = $(MAVERICK_UTILS_SRC_FILES:$(MAVERICK_UTILS_PATH)/%=%)
MAVERICK_UTILS_OBJS = ${MAVERICK_UTILS_SRC:.cc=.o}
MAVERICK_UTILS_OBJS_FILES = ${MAVERICK_UTILS_OBJS:%=$(MAVERICK_UTILS_PATH)/%}

MAVERICK_PYTHON_PATH = ./src/MaverickPython

ifeq ($(OS),Darwin) # OsX
	BLAS_LIB := -framework Accelerate
	FORTRAN_LIB := -L/usr/local/gfortran/lib -lgfortran
	SL_EXT := dylib

	LINKER_LOAD_ALL_LIB := -Wl,-all_load
	LINKER_NO_LOAD_ALL_LIB := -Wl,-noall_load
	MAKE_SHARED := -dynamiclib -undefined dynamic_lookup
	# Let's use LD_LIBRARY_PATH
	# INSTALL_PATH_LIB := -install_name $(INSTALL_PATH)/lib/$(LIBNAME).$(SL_EXT)
	# INSTALL_PATH_TOIPOPT_LIB := -install_name $(INSTALL_PATH)/lib/$(LIBNAMEIP).$(SL_EXT)
	# INSTALL_PATH_TENSOLVE_LIB := -install_name $(INSTALL_PATH)/lib/$(LIBNAMETS).$(SL_EXT)
	INSTALL_PATH_LIB :=
	INSTALL_PATH_TOIPOPT_LIB :=
	INSTALL_PATH_TENSOLVE_LIB :=

	ADD_SLLIB_COMMANDS :=
else ifeq ($(OS),Linux)
	#default blas libs is atlas
	ifeq ($(BLAS_LIB),)
		BLAS_LIB := -lf77blas
	endif

	FORTRAN_LIB := -lgfortran
	SL_EXT := so

	LINKER_LOAD_ALL_LIB := -Wl,--whole-archive
	LINKER_NO_LOAD_ALL_LIB := -Wl,--no-whole-archive
	MAKE_SHARED := -shared
	# Let's use LD_LIBRARY_PATH
	# INSTALL_PATH_LIB := -Wl,-soname,$(INSTALL_PATH)/lib/$(LIBNAME).$(SL_EXT)
	# INSTALL_PATH_TOIPOPT_LIB := -Wl,-soname,$(INSTALL_PATH)/lib/$(LIBNAMEIP).$(SL_EXT)
	# INSTALL_PATH_TENSOLVE_LIB := -Wl,-soname,$(INSTALL_PATH)/lib/$(LIBNAMETS).$(SL_EXT)
	INSTALL_PATH_LIB :=
	INSTALL_PATH_TOIPOPT_LIB :=
	INSTALL_PATH_TENSOLVE_LIB :=

	ADD_SLLIB_COMMANDS := -pthread -fopenmp
endif

CPP_FLAGS := -fPIC $(OPTIM_FLAGS) $(CPPADDFLAG)

CORE_INCLUDES = -Isrc -I/usr/include -I/usr/local/include
CORE_CFLAGS = $(CPP_FLAGS) $(BUILD_FLAGS) $(CORE_INCLUDES) $(IPOPT_INCLUDE)
TOIPOPT_CFLAGS = $(CPP_FLAGS) $(BUILD_FLAGS) $(CORE_INCLUDES) $(IPOPT_INCLUDE)
TENSOLVE_CFLAGS = $(CPP_FLAGS) $(BUILD_FLAGS) $(CORE_INCLUDES)
TENSOLVE_FFLAGS = $(OPTIM_FLAGS) $(BUILD_FLAGS)
GC_CFLAGS = $(CPP_FLAGS) $(BUILD_FLAGS) $(CORE_INCLUDES) $(LUA_INCLUDE)
SPLINES_CFLAGS = $(CPP_FLAGS) $(BUILD_FLAGS) $(CORE_INCLUDES)
UTILS_CFLAGS = $(CPP_FLAGS) $(BUILD_FLAGS) $(CORE_INCLUDES)

SL_FLAGS = $(MAKE_SHARED)
ifeq ($(THISLUA),true)
	MAV_LIB_SL_FLAGS = $(SL_FLAGS) -o $(LIBNAME).$(SL_EXT) $(INSTALL_PATH_LIB) $(LINKER_LOAD_ALL_LIB) $(LUA_LIB) -Lbuild/ -lmaverickcore -lmaverickgc -lmavericksplines -lmaverickutils $(LINKER_NO_LOAD_ALL_LIB) $(ADD_SLLIB_COMMANDS)
else
	MAV_LIB_SL_FLAGS = $(SL_FLAGS) -o $(LIBNAME).$(SL_EXT) $(INSTALL_PATH_LIB) $(LINKER_LOAD_ALL_LIB) -Lbuild/ -lmaverickcore -lmaverickgc -lmavericksplines -lmaverickutils $(LINKER_NO_LOAD_ALL_LIB) $(LUA_LIB) $(ADD_SLLIB_COMMANDS)
endif

MAV_TOIPOPT_LIB_SL_FLAGS = $(SL_FLAGS) -o $(LIBNAMEIP).$(SL_EXT) $(INSTALL_PATH_TOIPOPT_LIB) $(LINKER_LOAD_ALL_LIB) -Lbuild/ -lmavericktoip $(LINKER_NO_LOAD_ALL_LIB) -Lbuild/ -lmaverick $(IPOPT_LIB) $(BLAS_LIB)

MAV_TENSOLVE_LIB_SL_FLAGS = $(SL_FLAGS) -o $(LIBNAMETS).$(SL_EXT) $(INSTALL_PATH_TENSOLVE_LIB) $(LINKER_LOAD_ALL_LIB) -Lbuild/ -lmaverickts $(LINKER_NO_LOAD_ALL_LIB) -Lbuild/ -lmaverick $(FORTRAN_LIB)


.PHONY: depend clean

default: @(MAVERICK_LIB)
	@echo Maverick lib compiled

core: @(MAVERICK_CORE)
	@echo Maverick core compiled

ipopt: @(MAVERICK_TOIPOPT_LIB)
	@echo Maverick to ipopt lib compiled

tensolve: @(MAVERICK_TENSOLVE_LIB)
	@echo Maverick tensolve lib compiled

gc: @(MAVERICK_GC)
	@echo Maverick generic container compiled

splines: @(MAVERICK_SPLINES)
	@echo Maverick splines compiled

utils: @(MAVERICK_UTILS)
	@echo Maverick utils compiled

python: @(MAVERICK_PYTHON)
	@echo Maverick python compiled

install:
	rm -R -f "$(INSTALL_PATH)/lib"
	test -d "$(INSTALL_PATH)/lib" || mkdir -p "$(INSTALL_PATH)/lib"
	cp build/libmaverick.$(SL_EXT) $(INSTALL_PATH)/lib/
	cp build/$(LIBNAMEIP).$(SL_EXT) $(INSTALL_PATH)/lib/
	cp build/$(LIBNAMETS).$(SL_EXT) $(INSTALL_PATH)/lib/
#copy headers
	rm -R -f "$(INSTALL_PATH)/include"
	test -d "$(INSTALL_PATH)/include" || mkdir -p "$(INSTALL_PATH)/include"
	cp -R include/. $(INSTALL_PATH)/include/
#copy lua
	rm -R -f "$(INSTALL_PATH)/lua"
	test -d "$(INSTALL_PATH)/lua" || mkdir -p "$(INSTALL_PATH)/lua"
	cp -R ./src/Lua/*.lua $(INSTALL_PATH)/lua/
	@echo Maverick lib succesfully installed

install_python:
	rm -R -f "$(INSTALL_PATH)/python"
	test -d "$(INSTALL_PATH)/python" || mkdir -p "$(INSTALL_PATH)/python"
	cp ./build/$(PYTHONLIBNAME).so $(INSTALL_PATH)/python/
	@echo Maverick python lib succesfully installed

### CLEANS

clean: clean_core clean_ipopt clean_tensolve clean_gc clean_splines clean_utils clean_python clean_lua

clean_core:
	rm -f $(MAVERICK_CORE_OBJS_FILES)
	rm -f $(MAVERICK_CORE_PATH)/*.o
	rm -R -f build/libmaverick.$(SL_EXT)
	rm -R -f build/libmaverickcore.a
	rm -R -f include/MaverickCore/

clean_ipopt:
	rm -f $(MAVERICK_TOIPOPT_PATH)/*.o
	rm -f build/$(LIBNAMEIP).*

clean_tensolve:
	rm -f $(MAVERICK_TENSOLVE_PATH)/*.o
	rm -f $(MAVERICK_TENSOLVE_PATH)/*.mod
	rm -f build/$(LIBNAMETS).*

clean_gc:
	rm -f $(MAVERICK_GC_OBJS_FILES)
	rm -f $(MAVERICK_GC_PATH)/*.o
	rm -f build/libmaverickgc.a
	rm -f include/MaverickGC/*.hh

clean_splines:
	rm -f $(MAVERICK_SPLINES_OBJS_FILES)
	rm -f $(MAVERICK_SPLINES_PATH)/*.o
	rm -f build/libmavericksplines.a

clean_utils:
	rm -f $(MAVERICK_UTILS_OBJS_FILES)
	rm -f $(MAVERICK_UTILS_PATH)/*.o
	rm -f $(MAVERICK_UTILS_PATH)/ComponentBase/*.o
	rm -f $(MAVERICK_UTILS_PATH)/GenericFunction/*.o
	rm -f build/libmaverickutils.a
	rm -f include/MaverickUtils/ComponentBase/*.hh
	rm -f include/MaverickUtils/GenericFunction/*.hh

clean_python:
	rm -f $(MAVERICK_PYTHON_PATH)/*.so
	rm -f $(MAVERICK_PYTHON_PATH)/*.o
	rm -f build/$(PYTHONLIBNAME).so
	rm -f -R $(MAVERICK_PYTHON_PATH)/build

clean_lua:
	rm -f src/Lua/src/*.o
	rm -f src/Lua/src/liblua_$(PLATFORM_SUFFIX).a
	rm -f src/Lua/src/luac
	rm -f src/Lua/src/lua

### TARGETS
@(MAVERICK_LIB): @(MAVERICK_SPLINES) @(MAVERICK_GC) @(MAVERICK_UTILS) @(MAVERICK_CORE_LIB) @(MAVERICK_TOIPOPT_LIB) @(MAVERICK_TENSOLVE_LIB)


@(MAVERICK_CORE_LIB): @(MAVERICK_CORE) @(LUA)
	$(CXX) $(MAV_LIB_SL_FLAGS)
	mv $(LIBNAME).$(SL_EXT) build/

@(MAVERICK_CORE): $(MAVERICK_CORE_OBJS_FILES)
	test -d "build" || mkdir -p "build"
	rm -f build/$(LIBNAME).$(SL_EXT)
	rm -f build/$(LIBNAMEIP).$(SL_EXT)
	rm -f build/$(LIBNAMETS).$(SL_EXT)
	$(AR) $(ARFLAGS) build/libmaverickcore.a $(MAVERICK_CORE_OBJS_FILES)
	test -d include/MaverickCore || mkdir -p include/MaverickCore
	cp src/MaverickCore/Maverick.hh include/MaverickCore/
	cp src/MaverickCore/MaverickCInterface.h include/MaverickCore/
	cp src/MaverickCore/MaverickDefinitions.hh include/MaverickCore/
	cp src/MaverickCore/MaverickOcp.hh include/MaverickCore/
	cp src/MaverickCore/MaverickOcpSolver.hh include/MaverickCore/
	cp src/MaverickCore/MeshSinglePhase.hh include/MaverickCore/
	cp src/MaverickCore/Mesh.hh include/MaverickCore/
	cp src/MaverickCore/OcpSolution.hh include/MaverickCore/
	cp src/MaverickCore/OcpSolutionSinglePhase.hh include/MaverickCore/
	cp src/MaverickCore/OcpGuess.hh include/MaverickCore/
	cp src/MaverickCore/OcpScaling.hh include/MaverickCore/
	cp src/MaverickCore/OcpSolverOutput.hh include/MaverickCore/

@(MAVERICK_TOIPOPT_LIB): @(MAVERICK_TOIPOPT)
	rm -f build/$(LIBNAMEIP).$(SL_EXT)
	$(CXX) $(MAV_TOIPOPT_LIB_SL_FLAGS)
	test -d "build" || mkdir -p "build"
	mv $(LIBNAMEIP).$(SL_EXT) build/

@(MAVERICK_TOIPOPT):$(MAVERICK_TOIPOPT_OBJS_FILES)
	test -d build || mkdir -p build
	$(AR) $(ARFLAGS) build/$(LIBNAMEIP).a $(MAVERICK_TOIPOPT_OBJS_FILES)

@(MAVERICK_TENSOLVE_LIB): @(MAVERICK_TENSOLVE)
	rm -f build/$(LIBNAMETS).$(SL_EXT)
	$(CXX) $(MAV_TENSOLVE_LIB_SL_FLAGS)
	test -d "build" || mkdir -p "build"
	mv $(LIBNAMETS).$(SL_EXT) build/

@(MAVERICK_TENSOLVE): $(MAVERICK_TENSOLVE_OBJS_FILES)
	cd $(MAVERICK_TENSOLVE_PATH); make PLATFORM=$(PLATFORM) OPTIM_FLAGS='$(OPTIM_FLAGS)' FF=$(FF) BUILD_FLAGS='$(BUILD_FLAGS)'
	test -d build || mkdir -p build
	$(AR) $(ARFLAGS) build/$(LIBNAMETS).a $(MAVERICK_TENSOLVE_OBJS_FILES) $(MAVERICK_TENSOLVE_PATH)/BLAS_PRT.o $(MAVERICK_TENSOLVE_PATH)/UNCMIN.o $(MAVERICK_TENSOLVE_PATH)/TENSOLVE.o

@(MAVERICK_GC): $(MAVERICK_GC_OBJS_FILES)
	test -d build || mkdir -p build
	$(AR) $(ARFLAGS) build/libmaverickgc.a $(MAVERICK_GC_OBJS_FILES)
	test -d include/MaverickGC || mkdir -p include/MaverickGC
	cp src/MaverickGC/GenericContainer.hh             include/MaverickGC/
	cp src/MaverickGC/GenericContainerConfig.hh       include/MaverickGC/
	cp src/MaverickGC/GenericContainerLuaInterface.hh include/MaverickGC/
	cp src/MaverickGC/GenericContainerCinterface.h    include/MaverickGC/

@(MAVERICK_SPLINES): $(MAVERICK_SPLINES_OBJS_FILES)
	test -d build || mkdir -p build
	$(AR) $(ARFLAGS) build/libmavericksplines.a $(MAVERICK_SPLINES_OBJS_FILES)

@(MAVERICK_UTILS): $(MAVERICK_UTILS_OBJS_FILES)
	test -d build || mkdir -p build
	$(AR) $(ARFLAGS) build/libmaverickutils.a $(MAVERICK_UTILS_OBJS_FILES)
	test -d include/MaverickUtils/ComponentBase || mkdir -p include/MaverickUtils/ComponentBase
	test -d include/MaverickUtils/GenericFunction || mkdir -p include/MaverickUtils/GenericFunction
	cp src/MaverickUtils/ComponentBase/ComponentBase.hh include/MaverickUtils/ComponentBase/
	cp src/MaverickUtils/ComponentBase/ComponentDefinitions.hh include/MaverickUtils/ComponentBase/
	cp src/MaverickUtils/ComponentBase/ComponentInterface.hh include/MaverickUtils/ComponentBase/
	cp src/MaverickUtils/GenericFunction/GenericFunction1ABase.hh include/MaverickUtils/GenericFunction/
	cp src/MaverickUtils/GenericFunction/GenericFunction1AFormulasInterface.hh include/MaverickUtils/GenericFunction/
	cp src/MaverickUtils/GenericFunction/GenericFunction1AInterface.hh include/MaverickUtils/GenericFunction/
	cp src/MaverickUtils/GenericFunction/GenericFunction2ABase.hh include/MaverickUtils/GenericFunction/
	cp src/MaverickUtils/GenericFunction/GenericFunction2AFormulasInterface.hh include/MaverickUtils/GenericFunction/
	cp src/MaverickUtils/GenericFunction/GenericFunction2AInterface.hh include/MaverickUtils/GenericFunction/

@(MAVERICK_PYTHON):
	rm -f $(MAVERICK_PYTHON_PATH)/*.so
	rm -f $(MAVERICK_PYTHON_PATH)/*.o
	rm -f build/$(PYTHONLIBNAME).so
	rm -f -R $(MAVERICK_PYTHON_PATH)/build
	cd $(MAVERICK_PYTHON_PATH); $(PYTHON) setup_maverick.py build_ext --inplace
	cp $(MAVERICK_PYTHON_PATH)/*.so build/$(PYTHONLIBNAME).so

ifeq ($(THISLUA),true)
@(LUA):
	cd src/Lua/src; make $(PLATFORM_SUFFIX)
else
@(LUA):
endif


$(MAVERICK_TOIPOPT_PATH)/%.o: $(MAVERICK_TOIPOPT_PATH)/%.cc
	$(CXX) $(TOIPOPT_CFLAGS) -c $< -o $@

$(MAVERICK_TENSOLVE_PATH)/%.o: $(MAVERICK_TENSOLVE_PATH)/%.cc
	$(CXX) $(TENSOLVE_CFLAGS) -c $< -o $@

$(MAVERICK_GC_PATH)/%.o: $(MAVERICK_GC_PATH)/%.cc
	$(CXX) $(GC_CFLAGS) -c $< -o $@

$(MAVERICK_SPLINES_PATH)/%.o: $(MAVERICK_SPLINES_PATH)/%.cc
	$(CXX) $(SPLINES_CFLAGS) -c $< -o $@

$(MAVERICK_UTILS_PATH)/%.o: $(MAVERICK_UTILS_PATH)/%.cc
	$(CXX) $(UTILS_CFLAGS) -c $< -o $@

$(MAVERICK_CORE_PATH)/%.o: $(MAVERICK_CORE_PATH)/%.cc
	$(CXX) $(CORE_CFLAGS) -c $< -o $@
