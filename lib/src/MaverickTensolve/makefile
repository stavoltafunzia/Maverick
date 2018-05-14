OS := $(shell uname)

.PHONY: depend clean

default: @(TENSOLVE)
	rm -f blas_part.mod unconstrained_min.mod tensolve.mod

@(TENSOLVE): BLAS_PRT.o UNCMIN.o TENSOLVE.o

########################################################################

BLAS_PRT.o: BLAS_PRT.F90
	$(FF) $(OPTIM_FLAGS) $(BUILD_FLAGS) -fPIC -c BLAS_PRT.F90

UNCMIN.o: UNCMIN.F90
	$(FF) $(OPTIM_FLAGS) $(BUILD_FLAGS) -fPIC -c UNCMIN.F90

TENSOLVE.o: TENSOLVE.F90
	$(FF) $(OPTIM_FLAGS) $(BUILD_FLAGS) -fPIC -c TENSOLVE.F90
