MODULE tensolve
!USE blas_part
USE iso_c_binding
USE unconstrained_min

IMPLICIT NONE

!INTEGER, PARAMETER    :: dp = SELECTED_REAL_KIND(14, 60)
REAL (dp), PARAMETER  :: zero = 0.0_dp, one = 1.0_dp

!       FC   : FUCTION VALUES AT XC
!       ANLS : TENSOR TERM MATRIX AT CURRENT ITERATE
!       AJA  : JACOBIAN MATRIX AT CURRENT ITERATE
!       S    : MATRIX OF PAST LINEARLY INDEPENDENT DIRECTIONS
!       Q    : NUMERICAL RANK OF JACOBIAN :
!              Q > P : JACOBIAN IS SINGULAR
!              Q = P : OTHERWISE

! modified by nicola
!REAL (dp), ALLOCATABLE, SAVE  :: fc(:), anls(:,:), aja(:,:), s(:,:)
!INTEGER, SAVE                 :: qrank, meqns, nvars

CONTAINS


!      ALGORITHM 768, COLLECTED ALGORITHMS FROM ACM.
!      THIS WORK PUBLISHED IN TRANSACTIONS ON MATHEMATICAL SOFTWARE,
!      VOL. 232, NO. 21, June, 1997, pp. 174--195.
!
!
! TENSOLVE:  A Software Package for Solving Systems of Nonlinear Equations
!            and Nonlinear Least Squares Problems Using Tensor Methods.

! AUTHORS:   Ali Bouaricha
!            Argonne National Laboratory
!            MCS Division
!            e-mail: bouarich@mcs.anl.gov
! AND
!            Robert B. Schnabel
!            University of colorado at Boulder
!            Department of computer Science
!            e-mail: bobby@cs.colorado.edu

! DATE:      Version of January, 1997
! Conversion to Fortran 90 by Alan Miller
! amiller@bigpond.net.au

!  1 Nov 98: Increased dimension of several automatic and allocated arrays
!            to prevent array bound errors.   Several minor changes to reduce
!            the number of fatal errors reported by ELF90.
! 21 Jan 99: Fixed a bug in tsqfcn which was causing occasional underflows.
! 24 May 01: Changed the INTENTs of several variables in routine tschki.
!            Set termcd = 0 in routine tsnstp.
!            Initialized ierr in routine tsnesv.
! 9 Sept 03: Added interface to JAC (renamed from TSDUMJ) in routine TSNESI.
! 31 Jan 17: Reverted back from JAC to TSDUMJ in routine TSNESI. (nicola)

! Latest revision - 9 September 2003

! Purpose of Tensolve:

!     TENSOLVE finds roots of systems of n nonlinear equations in n unknowns,
!     or minimizers of the sum of squares of m > n nonlinear equations in
!     n unknowns.  It allows the user to choose between a tensor method based
!     on a quadratic model and a standard method based on a linear model.
!     Both models calculate an analytic or finite-difference Jacobian matrix
!     at each iteration.  The tensor method augments the linear model with a
!     low-rank, second-order term that is chosen so that the model is hardly
!     more expensive to form, store, or solve than the standard linear model.
!     Either a line search or trust-region step-selection strategy may be
!     selected.  The tensor method is significantly more efficient than the
!     standard method in iterations, function evaluations, and time.  It is
!     especially useful on problems where the Jacobian at the solution is
!     singular.
!     The software can be called with an interface where the user supplies
!     only the function, number of nonlinear equations, number of variables,
!     and starting point; default choices are made for all the other input
!     parameters.  An alternative interface allows the user to specify any
!     input parameters that are different from the defaults.

! List of subroutine and function names called by TENSOLVE:

! TS2DTR,TSBSLV,TSCHKI,TSCHKJ,TSCPMU,TSCPSS,TSD1SV,TSDFCN,TSDFLT,
! TSDUMJ,TSFAFA,TSFDFJ,TSFRMT,TSFSCL,TSFSLV,TSJMUV,TSJQTP,TSLMIN,TSLMSP,
! TSLSCH,TSMAFA,TSMDLS,TSMFDA,TSMFDV,TSMGSA,TSMSDA,TSMSDV,TSMSLV,TSNECI,
! TSNESI,TSNESV,TSNSTP,TSPRMV,TSRSLT,TSQ1P1,TSQFCN,TSQLFC,TSQMLV,TSQMTS,
! TSQMUV,TSQRFC,TSRSID,TSSCLF,TSSCLJ,TSSCLX,TSSLCT,TSSMIN,TSSMRD,TSSQP1,
! TSSSTP,TSSTMX,TSTRUD,TSUDQV,TSUNSF,TSUNSX,TSUPSF,TSUTMD.

! Packages called by TENSOLVE:

! UNCMIN (R. B. Schnabel, J. E. Koontz, and B. E. Weiss,
! "A Modular System of Algorithms of Unconstrained Minimization",
! ACM Trans. Math. Softw., 11 (1985), 419-440).

! BLAS called by TENSOLVE:

! LEVEL 1 BLAS: DNRM2,DSWAP,IDAMAX
! LEVEL 2 BLAS: DGEMV

! Parameters and Default Values for the interfaces TSNECI and TSNESI:
! ==================================================================

! Following each variable name in the list below appears a one- or
! two-headed arrow symbol of the form ->, <-, and <-->.
! These symbols signify that the variable is for input, output, and
! input-output, respectively.
! The symbol EPSM in some parts of this section designates the machine
! precision.

! X0->: An array of length N that contains an initial estimate
! of the solution x*.

! M->: A positive integer specifying the number of nonlinear equations.

! N->: A positive integer specifying the number of variables in the problem.

! TYPX->:  An array of length N in which the typical size of the
! components of X is specified. The typical component sizes should be
! positive real scalars. If a negative value is specified, its absolute
! value will be used. If 0.0 is specified, 1.0 will be used. This
! vector is used to determine the scaling matrix, Dx. Although the
! package may work reasonably well in a large number of instances without
! scaling, it may fail when the components of x* are of radically
! different magnitude and scaling is not invoked. If the sizes
! of the parameters are known to differ by many orders of magnitude, then
! the scale vector TYPX should definitely be used. For example, if
! it is anticipated that the range of values for the iterates xk would be

!                   x1 in [-10e+10,10e+10]
!                   x2 in [-10e+2,10e+4]
!                   x3 in [-6*10e-6,9*10e-6]

! then an appropriate choice would be TYPX = (1.0e+10,1.0e+3,7.0e-6).
! Module TSDFLT returns TYPX = (1.0,...,1.0).

! TYPF->: An array of length M in which the typical size of the components
! of F is specified. The typical component sizes should be positive real
! scalars.  If a negative value is specified, its absolute value will be
! used. If 0.0 is specified, 1.0 will be used. This vector is used to
! determine the scaling matrix DF. TYPF should be chosen so that all
! the components of DF(x) have similar typical magnitudes at points not
! too near a root, and should be chosen in conjunction with FTOL.  It is
! important to supply values of TYPF when the magnitudes of the components
! of F(x) are expected to be very different.  If the magnitudes of the
! components of F(x) are similar, the choice DF = I suffices.  Module
! TSDFLT returns TYPF = (1.0,...,1.0).

! ITNLIM->:  Positive integer specifying the maximum number of iterations to
! be performed before the program is terminated.   Module TSDFLT returns
! ITNLIM = 150. If the user specifies ITNLIM <= 0, the module TSCHKI will
! supply the value 150.

! JACFLG->: Integer designating whether or not an analytic Jacobian has
! been supplied by the user.
! JACFLG = 0 : No analytic Jacobian supplied.  The Jacobian is obtained
! by finite differences.
! JACFLG = 1 : Analytic Jacobian supplied.
! The module TSDFLT returns the value 0.  If the user specifies an illegal
! value, the module TSCHKI will supply the value 0.

! GRADTL->: Positive scalar giving the tolerance at which the scaled
! gradient of f(x) = 0.5*F(x)-trans*F(x) is considered close enough to
! zero to terminate the algorithm. The scaled gradient is a measure of
! the relative change in F in each direction xj divided by the relative
! change in xj. The module TSDFLT returns the value EPSM**(1/3).  If the
! user specifies a negative value, the module TSCHKI will supply
! the value EPSM**(1/3).

! STEPTL->: A positive scalar providing the minimum allowable relative
! step length. STEPTL should be at least as small as 10**(-d), where d
! is the number of accurate digits the user desires in the solution x*.
! The program may terminate prematurely if STEPTL is too large.  Module
! TSDFLT returns the value EPSM**(2/3).  If the user specifies a negative
! value, the module TSCHKI will supply the value EPSM**(2/3).

! FTOL->: A positive scalar giving the tolerance at which the scaled
! function DF*F(x) is considered close enough to zero to terminate the
! algorithm. The program is halted if ||DF*F(x)|| (in the infinity norm)
! is <= FTOL. This is the primary stopping condition for nonlinear
! equations; the values of TYPF and FTOL should be chosen so that this
! test reflects the user's idea of what constitutes a solution to the
! problem. The module TSDFLT returns the value EPSM**(2/3). If the
! user specifies a negative value, the module TSCHKI will supply the
! value EPSM**(2/3).

! METHOD->: An integer designating which method to use.
! METHOD = 0 : Newton or Gauss-Newton algorithm is used.
! METHOD = 1 : Tensor algorithm is used.
! Module TSDFLT returns value 1. If the user specifies an illegal value,
! module TSCHKI will reset METHOD to 1.

! GLOBAL->: An integer designating which global strategy to use.
! GLOBAL = 0 : Line search is used.
! GLOBAL = 1 : Two-dimensional trust region is used.
! Module TSDFLT returns value of 0. If the user specifies an illegal
! value, module TSCHKI will reset GLOBAL to 0.

! STEPMX->: A positive scalar providing the maximum allowable scaled step
! length ||Dx*(x+ - xc)||2, where Dx = diag(1/TYPX_j). STEPMX is used to
! prevent steps that would cause the nonlinear equations problem to
! overflow, and to prevent the algorithm from leaving the area of
! interest in parameter space.  STEPMX should be chosen small enough
! to prevent these occurrences but should be larger than any anticipated
! "reasonable" step. Module TSDFLT returns the value STEPMX = 10e+3.
! If the user specifies a nonpositive value, module TSCHKI sets STEPMX
! to 10e+3.

! DLT->: A positive scalar giving the initial trust region radius. When
! the line search strategy is used, this parameter is ignored. For the
! trust region algorithm, if DLT is supplied, its value should reflect
! what the user considers a maximum reasonable scaled step length at
! the first iteration. If DLT = -1.0, the routine uses the length of
! the Cauchy step at the initial iterate instead. The module TSDFLT
! returns the value -1.0. If the user specifies a nonpositive value,
! module TSCHKI sets DLT = -1.0.

! IPR->: The unit on which the package outputs information.  TSDFLT returns
! the value 6.

! FVEC->: The name of a user-supplied subroutine that evaluates the function F
! at an arbitrary vector X.  The subroutine must conform to the usage
!                      CALL FVEC(X, F, M, N),
! where X is a vector of length N and F is a vector of length M.  The
! subroutine must not alter the values of X.

! JAC->: The name of a user-supplied subroutine that evaluates the first
! derivative (Jacobian) of the function F(x).  The subroutine must conform
! to the usage
!                      CALL JAC(X, AJA, M, N)
! where X is a vector of length N and the 2-dimensional array AJA of row
! dimension MAXM and column dimension N is the analytic Jacobian of F at
! X.  When using the interface TSNECI, if no analytic Jacobian is supplied
! (JACFLG = 0), the user must use the dummy name TSDUMJ as the value of
! this parameter.

! MSG<-->: An integer variable that the user may set on input to inhibit
! certain automatic checks or to override certain default characteristics
! of the package. (For the short call it should be set to 0.) There are
! four "message" features that can be used individually or in combination
! as discussed below.
! MSG = 0 : Values of input parameters, final results, and termination code
! are printed.
! MSG = 2 : Do not check user's analytic Jacobian routine against its
! finite difference estimate.  This may be necessary if the user knows the
! Jacobian is properly coded, but the program aborts because the comparative
! tolerance is too tight.  Do not use MSG = 2 if the analytic Jacobian is
! not supplied.
! MSG = 8 : Suppress printing of the input state, the final results, and
! the stopping condition.
! MSG = 16 : Print the intermediate results; that is, the input state,
! each iteration including the current iterate xk, 0.5*||DF*F(xk)||2**2,
! and grad(f(x)) = J(x)-trans*DF**2 F(x), and the final results including
! the stopping conditions.
! The user may specify a combination of features by setting MSG to
! the sum of the individual components. The module TSDFLT returns a value
! of 0. On exit, if the program has terminated because of erroneous
! input, MSG contains an error code indicating the reason.
! MSG = 0   : No error.
! MSG = -1  : Illegal dimension, M <= 0.
! MSG = -2  : Illegal dimension, N <= 0.
! MSG = -3  : Illegal dimension, MAXM < M+N+2.
! MSG = -4  : Illegal dimension, MAXN < N+2.
! MSG = -5  : Illegal dimension, MAXP < NINT(sqrt(N)).
! MSG = -10 : Program asked to override check of analytic Jacobian
! against finite difference estimate, but routine JAC not
! supplied (incompatible input).
! MSG = -11  : Probable coding error in the user's analytic Jacobian
! routine JAC. Analytic and finite difference Jacobian do not agree
! within the assigned tolerance.

! XP<-: An array of length N containing the best approximation
! to the solution x*. (If the algorithm has not converged, the final
! iterate is returned).

! FP<-: An array of length M containing the function value F(XP).

! GP<-: An array of length N containing the gradient of the
! function 0.5*||F(x)||2**2 at XP.

! TERMCD<-:  An integer specifying the reason for termination.
! TERMCD = 0 : No termination criterion satisfied (occurs if package
! terminates because of illegal input).
! TERMCD = 1 : function tolerance reached.  The current iteration is
! probably a solution.
! TERMCD = 2 : gradient tolerance reached.  For nonlinear least
! squares, the current iteration is probably a solution; for nonlinear
! equations, it could be a solution or a local minimizer.
! TERMCD = 3 : Successive iterates within step tolerance.  The
! current iterate may be a solution, or the algorithm is making very slow
! progress and is not near a solution.
! TERMCD = 4 : Last global step failed to locate a point lower
! than XP. It is likely that either XP is an approximate solution
! of the problem or STEPTL is too large.
! TERMCD = 5 : Iteration limit exceeded.
! TERMCD = 6 : Five consecutive steps of length STEPMX have been taken.

! ===========================================================================
! Begin TENSOLVE
! ===========================================================================

SUBROUTINE ts2dtr(aja, shat, anls, dt, g, gbar, xc, method, nwtake, stepmx, &
                  steptl, epsm, mxtake, dlt, fq, maxm, m, n, p,    &
                  curpos, pivot, pbar, itn, ierr, flag, dxn, dfn, fvec,  &
                  fnorm, xpls, fp, fpls, retcd)

REAL (dp), INTENT(IN)      :: aja(:,:)
REAL (dp), INTENT(IN)      :: shat(:,:)
REAL (dp), INTENT(IN)      :: anls(:,:)
REAL (dp), INTENT(IN OUT)  :: dt(:)
REAL (dp), INTENT(IN)      :: g(:)
REAL (dp), INTENT(IN OUT)  :: gbar(:)
REAL (dp), INTENT(IN)      :: xc(:)
INTEGER, INTENT(IN)        :: method
LOGICAL, INTENT(IN)        :: nwtake
REAL (dp), INTENT(IN OUT)  :: stepmx
REAL (dp), INTENT(IN OUT)  :: steptl
REAL (dp), INTENT(IN)      :: epsm
LOGICAL, INTENT(IN OUT)    :: mxtake
REAL (dp), INTENT(IN OUT)  :: dlt
REAL (dp), INTENT(IN)      :: fq(:)
INTEGER, INTENT(IN)        :: maxm
INTEGER, INTENT(IN)        :: m
INTEGER, INTENT(IN)        :: n
INTEGER, INTENT(IN)        :: p
INTEGER, INTENT(IN)        :: curpos(:)
INTEGER, INTENT(IN)        :: pivot(:)
INTEGER, INTENT(IN)        :: pbar(:)
INTEGER, INTENT(IN)        :: itn
INTEGER, INTENT(IN)        :: ierr
INTEGER, INTENT(IN)        :: flag
REAL (dp), INTENT(IN)      :: dxn(:)
REAL (dp), INTENT(IN)      :: dfn(:)
REAL (dp), INTENT(IN)      :: fnorm
REAL (dp), INTENT(OUT)     :: xpls(:)
REAL (dp), INTENT(OUT)     :: fp(:)
REAL (dp), INTENT(OUT)     :: fpls
INTEGER, INTENT(OUT)       :: retcd

!**********************************************************************
! THIS ROUTINE FINDS A NEXT ITERATE BY A 2-DIMENSIONAL TRUST REGION.
!**********************************************************************

!       INPUT PARAMETERS :
!       -----------------

!       AJA    : JACOBIAN AT THE CURRENT ITERATE
!       SHAT   : MATRIX OF PAST LINEARLY INDEPENDENT DIRECTIONS
!                AFTER A QL FACTORIZATION
!       ANLS   : TENSOR TERM MATRIX
!       DT     : CURRENT STEP
!       G      : GRADIENT AT CURRENT ITERATE
!       GBAR   : STEEPEST DESCENT DIRECTION (= -G)
!       XC     : CURRENT ITERATE
!       METHOD : METHOD TO USE
!                  =  0  : STANDARD METHOD USED
!                  =  1  : TENSOR METHOD USED
!       NWTAKE : LOGICAL VARIABLE WITH THE FOLLOWING MEANINGS:
!                NWTAKE  =  .TRUE.  : STANDARD STEP TAKEN
!                NWTAKE  =  .FALSE. : TENSOR STEP TAKEN
!       STEPMX : MAXIMUM STEP ALLOWED
!       STEPTL : STEP TOLERANCE
!       EPSM   : MACHINE PRECISION
!       MXTAKE : BOOLEAN FLAG INDICATING STEP OF MAXIMUM LENGTH USED
!       FQ     : FUNCTION VALUE AT CURRENT ITERATE MULTIPLIED BY
!                ORTHOGONAL MATRICES
!       MAXM   : LEADING DIMENSION OF AJA AND ANLS
!       M,N    : DIMENSIONS OF PROBLEM
!       P      : COLUMN DIMENSION OF THE MATRICES SHAT AND ANLS
!       CURPOS : PIVOT VECTOR (USED DURING THE FACTORIZATION OF THE
!                JACOBIAN FROM COLUMN 1 TO N-P)
!       PIVOT  : PIVOT VECTOR ( USED DURING THE FACTORIZATION OF THE
!                JACOBIAN FROM COLUMN N-P+1 TO N)
!       PBAR   : PIVOT VECTOR (USED DURING THE FACTORIZATION OF THE
!                JACOBIAN IF IT IS SINGULAR
!       FNORM  :  0.5 * || FC ||**2
!       ITN    : ITERATION NUMBER
!       IERR   : RETURN CODE FROM THE QRP FACTORIZATION ROUTINE:
!                IERR = 0 : NO SINGULARITY OF JACOBIAN DETECTED
!                IERR = 1 : SINGULARITY OF JACOBIAN DETECTED
!       FLAG   : RETURN CODE WITH THE FOLLOWING MEANINGS :
!                FLAG  =  0 : NO SINGULARITY DETECTED DURING FACTORIZATION OF
!                             THE JACOBIAN FROM COLUMN 1 TO N
!                FLAG  =  1 : SINGULARITY DETECTED DURING FACTORIZATION
!                             OF THE JACOBIAN FROM COLUMN 1 TO N-P
!                FLAG  =  2 : SINGULARITY DETECTED DURING FACTORIZATION
!                             OF THE JACOBIAN FROM COLUMN N-P+1 TO N
!        DXN   : DIAGONAL SCALING MATRIX FOR X
!        DFN   : DIAGONAL SCALING MATRIX FOR F
!        FVEC  : SUBROUTINE TO EVALUATE THE USER'S FUNCTION

!       INPUT-OUTPUT PARAMETERS :
!       ------------------------

!       DLT    : INITIAL TRUST RADIUS (= -1.0D0) IF IT IS NOT SUPPLIED
!                BY THE USER ON ENTRY AND CURRENT TRUST RADIUS ON EXIT

!       OUTPUT PARAMETERS :
!       -------------------

!       XPLS   : NEXT ITERATE
!       FP     : FUNCTION VALUE AT NEXT ITERATE
!       FPLS   : 0.5 * || FP ||**2
!       RETCD  : RETURN CODE FROM SUBROUTINE (SEE SUBROUTINE TSTRUD FOR MEANING)

!       SUBPROGRAMS CALLED:

!       LEVEL 1 BLAS  ...  DNRM2
!       TENSOLVE      ...  TSPRMV,TSUTMD,TSJMUV,TSUDQV,TSSMIN,TSRSID,
!       TENSOLVE      ...  TSTRUD

INTERFACE
  SUBROUTINE fvec(x, f, m, n) bind(c)
    IMPLICIT NONE
    INTEGER, PARAMETER     :: dp = SELECTED_REAL_KIND(14, 60)
    REAL (dp), INTENT(IN)  :: x(n)
    REAL (dp), INTENT(OUT) :: f(m)
    INTEGER, INTENT(IN)    :: m, n
  END SUBROUTINE fvec
END INTERFACE

!***********************************************************************

! Workspace
REAL (dp) :: d(m), xplsp(n), adt(n), ag(n), temp(m), vn(m), vnp(m), vns(m), &
             const1(p), const2(p)

REAL (dp) :: res, alph, sum, resg, optim
REAL (dp) :: scres, fplsp, rresg
LOGICAL   :: dtaken

dtaken = .false.
retcd = 4
IF(dlt == -one) THEN

! set DLT to length of Cauchy step

  alph = dnrm2(n, g, 1)
  alph = alph**2
  CALL tsprmv(vn, g, pivot, n, 1)
  IF(ierr == 0) THEN
    CALL tsutmd(aja, vn, m, n, vnp)
  ELSE
    CALL tsprmv(vns, vn, pbar, n, 1)
    CALL tsutmd(aja, vns, m+n, n, vnp)
  END IF
  dlt = alph*SQRT(alph) / dnrm2(n, vnp, 1)**2
  IF(dlt > stepmx) THEN
    dlt = stepmx
  END IF
END IF

! form an orthonormal basis for the two-dimensional subspace

gbar(1:n) = -g(1:n)
res = dnrm2(n, dt, 1)
sum = -DOT_PRODUCT( gbar(1:n), dt(1:n) ) / res**2
gbar(1:n) = gbar(1:n) + sum * dt(1:n)
resg = dnrm2(n, gbar, 1)
IF(resg > zero) THEN
  rresg = one/resg
  gbar(1:n) = rresg * gbar(1:n)
END IF
dt(1:n) = dt(1:n) / res

! compute Jacobian times DT

CALL tsjmuv(itn, method, dt, curpos, pivot, pbar, aja, shat,  &
            flag, ierr, m, n, p, vn, adt)

! compute Jacobian times GBAR

CALL tsjmuv(itn, method, gbar, curpos, pivot, pbar, aja, shat,  &
            flag, ierr, m, n, p, vnp, ag)

IF(.NOT. nwtake) THEN

! compute SHAT times VN

  CALL tsudqv(shat, vn, n, p, const1)

! compute SHAT times VNP

  CALL tsudqv(shat, vnp, n, p, const2)
END IF

! normalize DT

70 IF(res <= dlt) THEN
  dtaken = .true.
  d(1:n) = dt(1:n)*res
  dlt = res

ELSE

! find the global minimizer of one-variable function in the
! interval (-dlt, dlt)

  CALL tssmin(anls, fq, adt, ag, const1, const2, dlt, m, n,  &
              p, nwtake, ierr, epsm, optim)

! compute the global step

  d(1:n) = optim*dt(1:n) + SQRT(dlt**2 - optim**2) * gbar(1:n)

END IF

! compute the tensor model residual

CALL tsrsid(itn, method, fq, d, curpos, pivot, pbar, aja, anls,  &
            shat, flag, nwtake, ierr, maxm, m, n, p, scres)

! check whether the global step is acceptable

CALL tstrud(m, n, xc, fnorm, g, d, dtaken, stepmx, steptl, dlt, mxtake, &
            dxn, dfn, fvec, scres, retcd, xplsp, fplsp, temp, xpls, fp, fpls)

IF(retcd >= 2) GO TO 70

RETURN
END SUBROUTINE ts2dtr



SUBROUTINE tsbslv(r, m, n, b, y)

REAL (dp), INTENT(IN)   :: r(:,:)
INTEGER, INTENT(IN)     :: m
INTEGER, INTENT(IN)     :: n
REAL (dp), INTENT(IN)   :: b(:)
REAL (dp), INTENT(OUT)  :: y(:)

!*********************************************************************
! THIS ROUTINE DOES A BACKWARD SOLVE.
!*********************************************************************

!    INPUT PARAMETERS :
!    -----------------

!    R  : UPPER TRIANGULAR MATRIX OBTAINED FROM A QR FACTORIZATION
!         OF AN M BY N MATRIX A. DIAG(R) IS STORED IN ROW M+2. THIS
!         IS THE STORAGE SCHEME USED IN STEWART, G. W., III(1973)
!         "INTRODUCTION TO MATRIX COMPUTATION", ACADEMIC PRESS, NEW YORK
!    M  : ROW DIMENSION OF MATRIX A
!    N  : COLUMN DIMENSION OF MATRIX A
!    B  : RIGHT HAND SIDE

!    OUTPUT PARAMETERS :
!    -----------------

!    Y :  VECTOR SOLUTION ON EXIT

!*********************************************************************

INTEGER :: j

! solve R Y = B

y(n) = b(n) / r(m+2,n)

IF(n > 2) THEN
  y(1:n-1) = zero
  DO j = n-1,2,-1
    y(1:j) = y(1:j) + y(j+1) * r(1:j,j+1)
    y(j) = (b(j) - y(j))/r(m+2,j)
  END DO
  y(1) = y(1) + r(1,2) * y(2)
  y(1) = (b(1) - y(1)) / r(m+2,1)
ELSE IF(n == 2) THEN
  y(1) = (b(1) - (r(1,2) * y(2))) / r(m+2,1)
END IF

RETURN
END SUBROUTINE tsbslv



SUBROUTINE tschki(maxm, maxn, maxp, m, n, gradtl, steptl, ftol, itnlim, &
                  jacflg, method, global, stepmx, dlt, epsm, msg, typx, &
                  typf, dxn, dfn, sqrn, termcd, ipr)

INTEGER, INTENT(IN)        :: maxm
INTEGER, INTENT(IN)        :: maxn
INTEGER, INTENT(IN)        :: maxp
INTEGER, INTENT(IN)        :: m
INTEGER, INTENT(IN)        :: n
REAL (dp), INTENT(IN OUT)  :: gradtl
REAL (dp), INTENT(IN OUT)  :: steptl
REAL (dp), INTENT(IN OUT)  :: ftol
INTEGER, INTENT(IN OUT)    :: itnlim
INTEGER, INTENT(IN OUT)    :: jacflg
INTEGER, INTENT(IN OUT)    :: method
INTEGER, INTENT(IN OUT)    :: global
REAL (dp), INTENT(IN OUT)  :: stepmx
REAL (dp), INTENT(IN OUT)  :: dlt
REAL (dp), INTENT(OUT)     :: epsm
INTEGER, INTENT(IN OUT)    :: msg
REAL (dp), INTENT(IN OUT)  :: typx(:)
REAL (dp), INTENT(IN OUT)  :: typf(:)
REAL (dp), INTENT(OUT)     :: dxn(:)
REAL (dp), INTENT(OUT)     :: dfn(:)
INTEGER, INTENT(OUT)       :: sqrn
INTEGER, INTENT(OUT)       :: termcd
INTEGER, INTENT(IN)        :: ipr

! N.B. INTENT of gradtl, steptl, ftol, stepmx, dlt, msg changed to IN/OUT
!      24 May 2001

!*********************************************************************
! THIS ROUTINE CHECKS INPUT FOR REASONABLENESS.
!*********************************************************************

!    INPUT PARAMETERS :
!    -----------------

!    MAXM : LEADING DIMENSION OF WORKSPACE WRKNEM
!           (SEE TOP OF THIS FILE )
!    MAXN : LEADING DIMENSION OF WORKSPACE WRKNEN
!           (SEE TOP OF THIS FILE )
!    MAXP : LEADING DIMENSION OF WORKSPACE WRKUNC
!           (SEE TOP OF THIS FILE )
!    M,N  : DIMENSIONS OF PROBLEM
!    IPR  : DEVICE TO WHICH TO SEND OUTPUT

!    INPUT-OUTPUT PARAMETERS :
!    ------------------------

!    GRADTL : TOLERANCE AT WHICH GRADIENT CONSIDERED CLOSE
!             ENOUGH TO ZERO TO TERMINATE ALGORITHM
!    STEPTL : TOLERANCE AT WHICH SUCCESSIVE ITERATES
!             CONSIDERED CLOSE ENOUGH TO TERMINATE ALGORITHM
!    FTOL   : TOLERANCE AT WHICH FUNCTION VALUE CONSIDERED
!             CLOSE ENOUGH TO ZERO
!    ITNLIM : MAXIMUM NUMBER OF ALLOWABLE ITERATIONS
!    STEPMX : MAXIMUM STEP ALLOWED IN TRUST REGION
!    DLT    : TRUST RADIUS
!    JACFLG : JACOBIAN FLAG WITH THE FOLLOWING MEANINGS :
!             JACFLG = 1 : ANALYTIC JACOBIAN SUPPLIED
!             JACFLG = 0 : ANALYTIC JACOBIAN NOT SUPPLIED
!    METHOD : METHOD TO USE
!             METHOD = 0 : STANDARD METHOD IS USED
!             METHOD = 1 : TENSOR METHOD IS USED
!    GLOBAL : GLOBAL STRATEGY USED
!             GLOBAL = 0 : LINE SEARCH USED
!             GLOBAL = 1 : 2-DIMENSIONAL TRUST REGION USED
!    TYPX   : TYPICAL SIZE FOR EACH COMPONENT OF X
!    TYPF   : TYPICAL SIZE FOR EACH COMPONENT OF F
!    MSG    : MESSAGE TO INHIBIT CERTAIN AUTOMATIC CHECKS + OUTPUT

!    OUTPUT PARAMETERS :
!    -------------------

!    TERMCD: TERMINATION CODE
!    DXN   : DIAGONAL SCALING MATRIX FOR X
!    DFN   : DIAGONAL SCALING MATRIX FOR F
!    SQRN  : MAXIMUM COLUMN DIMENSION OF S AND FV

!*********************************************************************

INTEGER              :: i, LEN
REAL (dp), PARAMETER :: two = 2._dp, three = 3._dp, thous = 1000._dp
REAL (dp)            :: temp

! check that parameters only take on acceptable values
! if not, set them to default values

! set TERMCD to zero in case we abort prematuraly

termcd = 0

! compute machine precision

epsm = EPSILON( 1.0D0 )

! check dimensions of the problem

IF(m <= 0) THEN
  WRITE(ipr,601) m
  msg = -1
  RETURN
END IF

IF(n <= 0) THEN
  WRITE(ipr,602) n
  msg = -2
  RETURN
END IF

! check leading dimensions of the problem

LEN = m+n+2
IF(maxm < LEN) THEN
  WRITE(ipr,603) maxm,LEN
  msg = -3
  RETURN
END IF

LEN = n+2
IF(maxn < LEN) THEN
  WRITE(ipr,604) maxn,LEN
  msg = -4
  RETURN
END IF

temp = SQRT(DBLE(n))
sqrn = nint(temp)

IF(maxp < sqrn) THEN
  WRITE(ipr,605) maxp,sqrn
  msg = -5
  RETURN
END IF

! check JACFLG, METHOD, and GLOBAL

IF(jacflg /= 1) jacflg = 0

IF(method /= 0 .AND. method /= 1) method = 1

IF(global /= 0 .AND. global /= 1) global = 0

IF(MOD(msg/2,2) == 1 .AND. jacflg == 0) THEN
  WRITE(ipr,610) msg,jacflg
  msg = -10
  RETURN
END IF

! check scale matrices

DO i = 1,n
  IF(typx(i) == zero) typx(i) = one
  IF(typx(i) < zero) typx(i) = -typx(i)
  dxn(i) = one/typx(i)
END DO

DO i = 1,m
  IF(typf(i) == zero) typf(i) = one
  IF(typf(i) < zero) typf(i) = -typf(i)
  dfn(i) = one/typf(i)
END DO

! check gradient, step, and function tolerances

temp = one/three
IF(gradtl < zero) THEN
  gradtl = epsm**temp
END IF

IF(steptl < zero) THEN
  steptl = epsm**(two*temp)
END IF

IF(ftol < zero) THEN
  ftol = epsm**(two*temp)
END IF

! check iteration limit

IF(itnlim <= 0) THEN
  itnlim = 150
END IF

! check STEPMX and DLT

IF(stepmx < zero) stepmx = thous

IF(dlt <= zero) THEN
  dlt = -one
  IF(dlt > stepmx) dlt = stepmx
END IF

RETURN

601 FORMAT('  TSCHKI     ILLEGAL DIMENSION M =',i5)
602 FORMAT('  TSCHKI     ILLEGAL DIMENSION N =',i5)
603 FORMAT('  TSCHKI     ILLEGAL DIMENSION MAXM =',i5,' < M+N+2 =',i5)
604 FORMAT('  TSCHKI     ILLEGAL DIMENSION MAXN =',i5,' < N+2 =',i5)
605 FORMAT('  TSCHKI     ILLEGAL DIMENSION MAXP =',i5,' <',  &
           '  NINT(SQRT (N)) =',i5)
610 FORMAT('  TSCHKI     USER REQUESTS THAT ANALYTIC JACOBIAN BE',  &
           ' ACCEPTED AS PROPERLY CODED (MSG =',i5,')'/  &
           '  TSCHKI     BUT ANALYTIC JACOBIAN NOT SUPPLIED',  &
           ' (JACFLG =',i5,')')
END SUBROUTINE tschki



SUBROUTINE tschkj(ajanal, xc, fc, m, n, epsm, dfn, dxn, typx, ipr, fvec, msg)

REAL (dp), INTENT(IN)      :: ajanal(:,:)
REAL (dp), INTENT(IN OUT)  :: xc(:)
REAL (dp), INTENT(IN OUT)  :: fc(:)
INTEGER, INTENT(IN)        :: m
INTEGER, INTENT(IN)        :: n
REAL (dp), INTENT(IN)      :: epsm
REAL (dp), INTENT(IN)      :: dfn(:)
REAL (dp), INTENT(IN)      :: dxn(:)
REAL (dp), INTENT(IN)      :: typx(:)
INTEGER, INTENT(IN)        :: ipr
INTEGER, INTENT(IN OUT)    :: msg

INTERFACE
  SUBROUTINE fvec(x, f, m, n) bind(c)
    IMPLICIT NONE
    INTEGER, PARAMETER     :: dp = SELECTED_REAL_KIND(14, 60)
    REAL (dp), INTENT(IN)  :: x(n)
    REAL (dp), INTENT(OUT) :: f(m)
    INTEGER, INTENT(IN)    :: m, n
  END SUBROUTINE fvec
END INTERFACE

!*********************************************************************
! THIS ROUTINE CHECKS THE ANALYTIC JACOBIAN AGAINST ITS FINITE
! DIFFERENCE APPROXIMATION.
!*********************************************************************

!     INPUT PARAMETERS :
!     -----------------

!   AJANAL : ANALYTIC JACOBIAN AT XC
!     XC   : CURRENT ITERATE
!     FC   : FUNCTION VALUE AT XC
!     M,N  : DIMENSIONS OF PROBLEM
!     EPSM : MACHINE PRECISION
!     DFN  : DIAGONAL SCALING MATRIX FOR F
!     DXN  : DIAGONAL SCALING MATRIX FOR X
!     TYPX : TYPICAL SIZE FOR EACH COMPONENT OF X
!     IPR  : DEVICE TO WHICH TO SEND OUTPUT
!     FVEC : SUBROUTINE TO EVALUATE THE USER'S FUNCTION

!     INPUT-OUTPUT PARAMETERS :
!     ------------------------

!     MSG : MESSAGE TO INHIBIT CERTAIN AUTOMATIC CHECKS + OUTPUT

!     SUBPROGRAMS CALLED:

!     LEVEL 1 BLAS  ...  IDAMAX
!     TENSOLVE      ...  TSUNSX,TSUNSF,TSSCLX,TSSCLF
!     USER          ...  FVEC

!*********************************************************************

! Workspace
REAL (dp) :: fhat(m), wrk1(m)
INTEGER   :: i, j
REAL (dp) :: ndigit, rnoise, sqrns, stepsz, xtmpj, dinf, rstpsz
REAL (dp) :: tol
REAL (dp), PARAMETER :: quart = 0.25_dp, ten = 10.0_dp

! unscale XC and FC

CALL tsunsx(xc, dxn, n)
CALL tsunsf(fc, dfn, m)

! compute the finite difference Jacobian and check it against the analytic one

ndigit = -LOG10(epsm)
rnoise = MAX(ten**(-ndigit),epsm)
sqrns  = SQRT(rnoise)
tol = epsm**quart

DO j = 1,n
  stepsz = sqrns*MAX(ABS(xc(j)),one)
  xtmpj = xc(j)
  xc(j) = xtmpj + stepsz
  CALL fvec(xc, fhat, m, n)
  xc(j) = xtmpj
  rstpsz = one/stepsz
  DO i = 1,m
    wrk1(i) = (fhat(i) - fc(i))*rstpsz
  END DO
  DO i = 1,m
    wrk1(i) = wrk1(i)*dfn(i)*typx(j)
  END DO
  dinf = ABS(wrk1(idamax(m, wrk1, 1)))
  DO i = 1,m
    IF(ABS(ajanal(i,j) - wrk1(i)) > tol*dinf) THEN
      WRITE(ipr,50)
      msg = -11
      RETURN
    END IF
  END DO
END DO

! scale back XC and FC

CALL tssclx(xc,dxn,n)
CALL tssclf(fc,dfn,m)

50 FORMAT(/'  TSCHKJ      PROBABLE ERROR IN CODING OF ANALYTIC JACOBIAN')

RETURN
END SUBROUTINE tschkj



SUBROUTINE tscpmu(r, n, epsm, mu)

REAL (dp), INTENT(IN)   :: r(:,:)
INTEGER, INTENT(IN)     :: n
REAL (dp), INTENT(IN)   :: epsm
REAL (dp), INTENT(OUT)  :: mu

!*********************************************************************
! THIS ROUTINE COMPUTES A SMALL PERTURBATION MU. MU IS USED IN THE
! COMPUTATION OF THE LEVENBERG-MARQUARDT STEP.
!*********************************************************************

!    INPUT PARAMETERS :
!    -----------------

!    R  : UPPER TRIANGULAR MATRIX
!    N  : COLUMN DIMENSION OF R
!    EPSM :  MACHINE PRECISION

!    OUTPUT PARAMETERS :
!    ------------------

!    MU = SQRT(L1 NORM OF R * INFINITY NORM OF R * N * EPSM * 100)

!*********************************************************************

INTEGER              :: i,j
REAL (dp)            :: aifnrm, total, al1nrm
REAL (dp), PARAMETER :: hund = 100.0_dp

! compute the infinity norm of R

aifnrm = zero
DO i = 1,n
  total = SUM( ABS(r(i,i:n)) )
  aifnrm = MAX(aifnrm,total)
END DO

! compute the l1 norm of R

al1nrm = zero
DO j = 1,n
  total = SUM( r(1:j,j) )
  al1nrm = MAX(al1nrm,total)
END DO

! compute MU

mu = SQRT(aifnrm*al1nrm*n*epsm*hund)

RETURN
END SUBROUTINE tscpmu



SUBROUTINE tscpss(s, m, n, p, method, global, epsm, fcq, qhat, anls,  &
                  dn, fqq, ptilda, curpos, pbar, zero1, ierr, resnew, flag)

REAL (dp), INTENT(IN)      :: s(:,:)
INTEGER, INTENT(IN)        :: m
INTEGER, INTENT(IN)        :: n
INTEGER, INTENT(IN)        :: p
INTEGER, INTENT(IN)        :: method
INTEGER, INTENT(IN)        :: global
REAL (dp), INTENT(IN)      :: epsm
REAL (dp), INTENT(IN)      :: fcq(:)
REAL (dp), INTENT(IN OUT)  :: qhat(:,:)
REAL (dp), INTENT(IN OUT)  :: anls(:,:)
REAL (dp), INTENT(OUT)     :: dn(:)
REAL (dp), INTENT(OUT)     :: fqq(:)
INTEGER, INTENT(OUT)       :: ptilda(:)
INTEGER, INTENT(IN)        :: curpos(:)
INTEGER, INTENT(OUT)       :: pbar(:)
INTEGER, INTENT(OUT)       :: zero1
INTEGER, INTENT(IN OUT)    :: ierr
REAL (dp), INTENT(OUT)     :: resnew
INTEGER, INTENT(OUT)       :: flag

!**********************************************************************
! THIS ROUTINE COMPUTES THE STANDARD STEP.  NOTE THAT AT THIS STAGE
! THE JACOBIAN MATRIX (QHAT) HAS ALREADY BEEN FACTORED FROM COLUMNS 1
! TO N-P DURING THE TENSOR STEP COMPUTATION.  THIS ROUTINE FACTORS
! THE MATRIX QHAT FROM COLUMN N-P+1 TO N, THEREBY OBTAINING A QR
! FACTORIZATION OF THE FULL MATRIX QHAT, THEN COMPUTES THE STANDARD
! STEP BY PREMULTIPLYING THE RIGH-HAND SIDE FCQ BY AN ORTHOGONAL
! MATRIX AND BY PERFORMING A BACKWARD SOLVE.
!**********************************************************************

!     INPUT PARAMETERS :
!     -----------------

!     S    : FACTORED MATRIX OF PAST LINEARLY INDEPENDENT DIRECTIONS
!            (OBTAINED FROM TSQLFC SUBROUTINE)
!     M,N  : DIMENSIONS OF PROBLEM
!     P    : COLUMN DIMENSION OF MATRIX S
!   METHOD : METHOD USED :
!            METHOD = 0 : STANDARD METHOD IS USED
!            METHOD = 1 : TENSOR METHOD IS USED
!   GLOBAL : GLOBAL STRATEGY USED
!            GLOBAL = 0 : LINE SEARCH IS USED
!            GLOBAL = 1 : 2-DIMENSIONAL TRUST REGION IS USED
!   EPSM   : MACHINE PRECISION
!   FCQ    : FUNCTION VALUE AT CURRENT ITERATE MULTIPLIED BY AN
!            ORTHOGONAL MATRIX
!   CURPOS : PIVOT VECTOR FOR THE FACTORIZATION OF QHAT FROM COLUMN 1 TO N-P

!     INPUT-OUTPUT PARAMETERS :
!     ------------------------

!     QHAT  : FACTORED MATRIX FROM COLUMN 1 TO N-P
!             ON ENTRY AND FACTORED MATRIX FROM 1 TO N ON EXIT
!     ANLS  : TENSOR TERM MATRIX ON ENTRY AND ANLS MULTIPLIED BY
!             ORTHOGONAL MATRICES ON EXIT (THIS IS PERFORMED IN THE
!             CASE WHERE THE GLOBAL STRATEGY USED IS THE 2-DIMENSIONAL
!             TRUST REGION)

!     OUTPUT PARAMETERS :
!     -------------------

!     DN    : STANDARD STEP
!     FQQ   : FUNCTION VALUE AT CURRENT ITERATE MULTIPLIED BY
!             ORTHOGONAL MATRICES (THIS IS USED IN THE CASE WHERE
!             THE GLOBAL STRATEGY USED IS THE 2-DIMENSIONAL
!             TRUST REGION)
!     PTILDA: PIVOT VECTOR FOR THE FACTORIZATION OF THE
!             MATRIX QHAT FROM N-P+1 TO N
!     PBAR  : PIVOT VECTOR FOR THE FACTORIZATION OF THE
!             TRANSFORMED MATRIX QHAT FROM 1 TO N
!             IN CASE OF SINGULARITY
!     ZERO1 : FIRST ZERO COLUMN OF MATRIX QHAT IN CASE OF SINGULARITY
!     IERR  : RETURNED CODE WITH THE FOLLOWING MEANING :
!             IERR = 1 : SINGULARITY OF JACOBIAN DETECTED
!             IERR = 0 : OTHERWISE
!     RESNEW: RESIDUAL OF THE STANDARD MODEL
!     FLAG  : RETURNED CODE WITH THE FOLLOWING MEANINGS :
!             FLAG = 0 : NO SINGULARITY DETECTED
!             FLAG = 1 : SINGULARITY DETECTED DURING QR FACTORIZATION
!                        OF QHAT FROM COLUMN 1 TO N-P
!             FLAG = 2 : SINGULARITY DETECTED DURING QR FACTORIZATION
!                        OF QHAT FROM COLUMN N-P+1 TO N

!     SUBPROGRAMS CALLED:

!     TENSOLVE      ...  TSQRFC,TSQMUV,TSQMTS,TSSMRD,TSBSLV,TSPRMV
!     TENSOLVE      ...  TSQMLV,TSCPMU

! **********************************************************************

! Workspace
REAL (dp) :: y(n), w(m+n), fqt(m+n)

INTEGER   :: zerotm, i, j
REAL (dp) :: mu

flag = 0

! initialization
w(m+1:m+n) = zero ! modified by nicola : missing initialisation
fqq(1:m+n) = zero

w(1:m) = -fcq(1:m)

! if the Jacobian is singular then compute the Levenberg-Marquardt
! step (label 20)

IF(ierr == 1) THEN
  flag = 1
  GO TO 20
END IF

! factor the matrix QHAT from column n-p+1 to n

CALL tsqrfc(qhat, n, m, n-p+1, n, ierr, epsm, ptilda, zero1)

IF(m == n .AND. ierr == 0) THEN
  zerotm = zero1 - 1
ELSE
  zerotm = zero1
END IF

! premultiply W by the orthogonal matrix resulting from the QR
! factorization of QHAT

CALL tsqmuv(qhat, w, fqq, m, n-p+1, zerotm, .false.)

IF(method == 1 .AND. global == 1) THEN

! premultiply ANLS by the orthogonal matrix resulting from the QR
! factorization of QHAT

  CALL tsqmts(anls, qhat, m, m, p, n-p+1, zerotm)
END IF

IF(ierr == 1) THEN
  flag = 2
  GO TO 20
END IF

! compute the residual of the standard model

CALL tssmrd(fqq, resnew, dn, mu, ierr, m, n)

! if QHAT is nonsingular perform a backward solve to obtain Y

CALL tsbslv(qhat, m, n, fqq, y)

! pivot Y

CALL tsprmv(dn, y, ptilda, n, 0)

IF(n /= 1) THEN

  CALL tsprmv(y, dn, curpos, n, 0)

! premultiply Y by the orthogonal matrix resulting from the QL
! factorization of S

  CALL tsqmlv(n, p, s, y, dn, .true.)

END IF

IF(global == 1) THEN
  ierr = 0
  fqq(1:m) = -fqq(1:m)
END IF

RETURN

!                    @   SINGULAR CASE   @

! solve ( QHAT-trans QHAT + MU I ) DN = -QHAT-trans W

! put the diagonal elements stored in row m+2 of QHAT into their
! propre positions and zero out the unwanted portions of QHAT

20 DO j = 1, zero1-1
  qhat(j,j) = qhat(m+2,j)
  qhat(j+1:m+n,j) = zero
END DO

DO j = zero1, n
  qhat(zero1:m+n,j) = zero
END DO

! compute a small perturbation MU

CALL tscpmu(qhat, n, epsm, mu)

! form the augmented matrix QHAT by adding an (n*n) diag(MU) in the bottom

DO i = m+1,m+n
  qhat(i,i-m) = mu
END DO

! factor the transformed matrix QHAT from 1 to n

CALL tsqrfc(qhat, n, m+n, 1, n, ierr, epsm, pbar, zero1)

IF(method == 1 .AND. global == 1) THEN

! premultiply ANLS by the orthogonal matrix resulting from the QR
! factorization of QHAT

  CALL tsqmts(anls, qhat, m+n, m, p, 1, zero1)
END IF

! compute the Levenberg-Marquardt step and the residual of the standard model

IF(flag == 1) THEN
  CALL tsqmuv(qhat, w, fqq, m+n, 1, n+1, .false.)
  CALL tsbslv(qhat, m+n, n, fqq, y)
  CALL tsprmv(dn, y, pbar, n, 0)
  CALL tsprmv(y, dn, curpos, n, 0)
  CALL tsqmlv(n, p, s, y, dn, .true.)
  CALL tssmrd(fqq, resnew, dn, mu, ierr, m, n)
  IF(global == 1) THEN
    ierr = 1
    fqq(1:m+n) = -fqq(1:m+n)
  END IF
  RETURN
ELSE
  CALL tsqmuv(qhat, fqq, fqt, m+n, 1, n+1, .false.)
  CALL tsbslv(qhat, m+n, n, fqt, dn)
  CALL tsprmv(y, dn, pbar, n, 0)
  CALL tsprmv(dn, y, ptilda, n, 0)
  CALL tsprmv(y, dn, curpos, n, 0)
  CALL tsqmlv(n, p, s, y, dn, .true.)
  CALL tssmrd(fqt, resnew, dn, mu, ierr, m, n)
  IF(global == 1) THEN
    ierr = 1
    fqq(1:m+n) = -fqt(1:m+n)
  END IF
END IF

RETURN
END SUBROUTINE tscpss



SUBROUTINE tsd1sv(aja, s, anls, fn, x, maxm, m, n, p, q, epsm, pivot, d1)

REAL (dp), INTENT(OUT)     :: aja(:,:)
REAL (dp), INTENT(IN OUT)  :: s(:,:)
REAL (dp), INTENT(IN OUT)  :: anls(:,:)
REAL (dp), INTENT(IN)      :: fn(:)
REAL (dp), INTENT(IN OUT)  :: x(:)
INTEGER, INTENT(IN)        :: maxm
INTEGER, INTENT(IN)        :: m
INTEGER, INTENT(IN)        :: n
INTEGER, INTENT(IN)        :: p
INTEGER, INTENT(IN)        :: q
REAL (dp), INTENT(IN)      :: epsm
INTEGER, INTENT(IN OUT)    :: pivot(:)
REAL (dp), INTENT(OUT)     :: d1(:)

!*********************************************************************
! THIS ROUTINE SOLVES THE FIRST N-Q LINEAR EQUATIONS IN N-P UNKNOWNS
! OF THE TENSOR MODEL.
!*********************************************************************

!    INPUT PARAMETERS :
!    ----------------

!    AJA : JACOBIAN MATRIX AT CURRENT ITERATE
!    S   : MATRIX OF PAST LINEARLY INDEPENDENT DIRECTIONS
!    ANLS: TENSOR TERM MATRIX AT CURRENT ITERATE
!    FN  : FUNCTION VALUE AT CURRENT ITERATE
!     X  : SOLUTION OF THE LOWER M-N+Q QUADRATIC EQUATIONS IN P
!          UNKNOWNS OF THE TENSOR MODEL
!    MAXM: LEADING DIMENSION OF AJA AND ANLS
!    M,N : DIMENSIONS OF PROBLEM
!    P   : COLUMN DIMENSION OF S AND ANLS
!    Q   : NUMERICAL RANK OF JACOBIAN :
!          Q > P : JACOBIAN IS SINGULAR
!          Q = P : OTHERWISE
!    EPSM: MACHINE PRECISION

!    OUTPUT PARAMETERS :
!    ------------------

!     PIVOT : PIVOT VECTOR
!     D1 : SOLUTION OF THE N-Q LINEAR EQUATIONS IN N-P UNKNOWNS OF
!          THE TENSOR MODEL

!    SUBPROGRAMS CALLED:

!    LEVEL 2 BLAS  ...  DGEMV
!    TENSOLVE      ...  TSSTMX,TSBSLV,TSQRFC,TSPRMV
!    TENSOLVE      ...  TSFSLV,TSQMUV

!*********************************************************************

INTEGER   :: zero1, i, j, ierr, icol
REAL (dp) :: wrk1(n), wrk2(n)
REAL (dp) :: epsm1
REAL (dp), PARAMETER :: alpha = 1.e-04_dp, half = 0.5_dp

! compute the top right (n-q) x p submatrix of AJA times X

CALL dgemv('N', n-q, p, one, aja(:,n-p+1:), maxm, x, 1, zero, d1, 1)

! compute S-trans times X

CALL tsstmx(s, x, n, p, wrk2)

! compute 0.5 * (S-trans times X)**2

wrk1(1:p) = half * wrk2(1:p)**2

! compute 0.5 * (top (n-q) x p submatrix of ANLS) * (S-trans times X)**2

CALL dgemv('N', n-q, p, one, anls(:,:), maxm, wrk1, 1, zero, wrk2, 1)

wrk1(1:n-q) = -fn(1:n-q) - d1(1:n-q) - wrk2(1:n-q)

! if the Jacobian is nonsingular then solve for the first
! n-p components of the tensor step and return

IF(p == q) THEN
  CALL tsbslv(aja, m, n-p, wrk1, d1)
  RETURN
END IF

wrk2(n-q+1:n-p) = zero

! copy top left (n-q) x (n-p) submatrix of AJA into bottom of AJA

aja(m+3:m+n-q+2,1:n-p) = aja(1:n-q,1:n-p)

! copy the transpose of the top left (n-q) x (n-p) submatrix of AJA
! into top of AJA

DO j = 1,n-q
  aja(j,j) = aja(m+2,j)
  DO i = j+1,n-p
    aja(i,j) = aja(j,i)
  END DO
END DO

! zero out the upper triangular (n-q) x (n-q) triangular part of
! the transpose of the top left (n-q) x (n-p) submatrix of AJA

DO j = 1,n-q
  aja(1:j-1,j) = zero
END DO

! factor the transpose of the top left (n-q) x (n-p) submatrix of AJA

epsm1 = epsm*alpha

CALL tsqrfc(aja, n-q, n-p, 1, n-q, ierr, epsm1, pivot, zero1)

IF(ierr == 0) THEN
  icol = n-q
ELSE
  icol = zero1-1
END IF

CALL tsprmv(d1, wrk1, pivot, n-q, 0)

! solve for the first n-p components of the tensor step

CALL tsfslv(aja, d1, n-p, icol, wrk2)

CALL tsqmuv(aja, wrk2, d1, n-p,1, zero1, .true.)

! copy the (n-q) x (n-p) submatrix of AJA from bottom back to top of AJA

aja(1:n-q,1:n-p) = aja(m+3:m+n-q+2,1:n-p)

RETURN
END SUBROUTINE tsd1sv



SUBROUTINE tsdfcn(p, x, g, qrank, meqns, nvars, fc, anls, aja, s )

INTEGER, INTENT(IN)       :: p
REAL (dp), INTENT(IN)     :: x(:)
REAL (dp), INTENT(OUT)    :: g(:)
INTEGER, INTENT(IN)       :: qrank, meqns, nvars
REAL (dp), INTENT(IN OUT) :: fc(:), anls(:,:), aja(:,:), s(:,:)

!*********************************************************************
! THIS ROUTINE COMPUTES THE ANALYTIC GRADIENT OF THE FUNCTION GIVEN
! BY SUBROUTINE TSQFCN.
!*********************************************************************

!    INPUT PARAMETERS :
!    -----------------

!    P    : COLUMN DIMENSION OF ANLS AND S
!    X    : POINT AT WHICH GRADIENT IS EVALUATED

!    OUTPUT PARAMETERS :
!    -----------------

!    G : GRADIENT AT X

!    SUBPROGRAMS CALLED:

!    LEVEL 2 BLAS  ...  DGEMV
!    TENSOLVE      ...  TSSTMX

!*********************************************************************

INTEGER              :: i, j, l
REAL (dp)            :: wrk1(meqns), wrk2(p), wrk3(p), wrk4(meqns), wrk5(meqns)
REAL (dp), PARAMETER :: half = 0.5_dp

! compute the lower right (m-n+q) x p submatrix of AJA times X

CALL dgemv('N', meqns-nvars+qrank, p, one, aja(nvars-qrank+1:,nvars-p+1:), &
           meqns, x, 1, zero, wrk1, 1)

! compute S-trans times X

CALL tsstmx(s, x, nvars, p, wrk3)

! compute 0.5 * (S-trans times X)**2

wrk2(1:p) = half * wrk3(1:p)**2

! compute 0.5 * (lower (m-n+q) x p submatrix of ANLS) *
! (S-trans times X)**2

CALL dgemv('N', meqns-nvars+qrank, p, one, anls(nvars-qrank+1:,:), meqns,  &
           wrk2, 1, zero, wrk4, 1)

DO i = 1,meqns-nvars+qrank
  wrk4(i) = wrk4(i) + fc(nvars-qrank+i) + wrk1(i)
END DO

! compute AJA-trans * WRK4

CALL dgemv('T', meqns-nvars+qrank, p, one, aja(nvars-qrank+1:,nvars-p+1:), &
           meqns, wrk4, 1, zero, wrk1, 1)

! compute ANLS-trans * WRK4

CALL dgemv('T', meqns-nvars+qrank, p, one, anls(nvars-qrank+1:,:), meqns,  &
           wrk4, 1, zero, wrk5, 1)

! compute S * diag(S-trans * WRK3) * WRK5

wrk2(1:p) = zero

l = p+1
DO j = 1,p
  l = l-1
  wrk2(l) = s(nvars+2,l)
  DO i = l+1,p
    wrk2(i) = s(nvars-p+j,i)
  END DO
  wrk2(1:p) = wrk2(1:p)*wrk3(1:p)
  g(j) = DOT_PRODUCT( wrk2(1:p), wrk5(1:p) )
END DO

g(1:p) = g(1:p) + wrk1(1:p)

RETURN
END SUBROUTINE tsdfcn



SUBROUTINE tensolve_tsdflt(m, n, itnlim, jacflg, gradtl, steptl, ftol, method,  &
                  global, stepmx, dlt, typx, typf, ipr) bind(c)

! N.B. Argument MSG has been removed.

INTEGER, INTENT(IN)        :: m
INTEGER, INTENT(IN)        :: n
INTEGER, INTENT(OUT)       :: itnlim
INTEGER, INTENT(OUT)       :: jacflg
REAL (dp), INTENT(OUT)     :: gradtl
REAL (dp), INTENT(OUT)     :: steptl
REAL (dp), INTENT(OUT)     :: ftol
INTEGER, INTENT(OUT)       :: method
INTEGER, INTENT(OUT)       :: global
REAL (dp), INTENT(OUT)     :: stepmx
REAL (dp), INTENT(OUT)     :: dlt
REAL (dp), INTENT(IN OUT)  :: typx(n)
REAL (dp), INTENT(IN OUT)  :: typf(m)
INTEGER, INTENT(OUT)       :: ipr

!*********************************************************************
! THIS ROUTINE SETS DEFAULT VALUES FOR EACH INPUT VARIABLE TO THE
! NONLINEAR EQUATION ALGORITHM.
!*********************************************************************

REAL (dp)            :: eps
REAL (dp), PARAMETER :: two = 2._dp, three = 3._dp, thous = 1000._dp

jacflg = 0
eps = EPSILON( 1.0_dp )
gradtl = eps**(one/three)
steptl = eps**(two/three)
ftol = eps**(two/three)
itnlim = 150
method = 1
global = 0
stepmx = thous
dlt = -one
! msg = 0
ipr = 6
typx(1:n) = one
typf(1:m) = one

RETURN
END SUBROUTINE tensolve_tsdflt



SUBROUTINE tensolve_tsdumj(x, aja, m, n) bind(c)

REAL (dp), INTENT(IN)   :: x(n)
REAL (dp), INTENT(OUT)  :: aja(m,n)
INTEGER, INTENT(IN)     :: m, n

!*********************************************************************
! THIS IS A DUMMY ROUTINE TO PREVENT UNSATISFIED EXTERNAL DIAGNOSTIC
! WHEN SPECIFIC ANALYTIC JACOBIAN IS NOT SUPPLIED.
!*********************************************************************

!   INPUT PARAMETERS:
!   -----------------

!   X   : POINT AT WHICH JACOBIAN IS EVALUATED
!   AJA : JACOBIAN MATRIX

!***********************************************************************

aja(m,n) = x(1)

RETURN
END SUBROUTINE tensolve_tsdumj



SUBROUTINE d2fcn(nr, n, x, h)

INTEGER, INTENT(IN)    :: nr
INTEGER, INTENT(IN)    :: n
REAL (dp), INTENT(IN)  :: x(:)
REAL (dp), INTENT(OUT) :: h(:,:)

!*********************************************************************
! THIS IS A DUMMY ROUTINE TO PREVENT UNSATISFIED EXTERNAL DIAGNOSTIC WHEN
! A REFERENCE TO THE MATRIX OF SECOND DERIVATIVES IS PASSED TO UNCMIN.
!*********************************************************************

h(nr,n) = x(1)

RETURN
END SUBROUTINE d2fcn



SUBROUTINE tsfafa(anls, fq, adt, ag, const1, const2, alpha, dlt, m, n, p,  &
                  nwtake, ierr, vn, fn_val)

REAL (dp), INTENT(IN)   :: anls(:,:)
REAL (dp), INTENT(IN)   :: fq(:)
REAL (dp), INTENT(IN)   :: adt(:)
REAL (dp), INTENT(IN)   :: ag(:)
REAL (dp), INTENT(IN)   :: const1(:)
REAL (dp), INTENT(IN)   :: const2(:)
REAL (dp), INTENT(IN)   :: alpha
REAL (dp), INTENT(IN)   :: dlt
INTEGER, INTENT(IN)     :: m
INTEGER, INTENT(IN)     :: n
INTEGER, INTENT(IN)     :: p
LOGICAL, INTENT(IN)     :: nwtake
INTEGER, INTENT(IN)     :: ierr
REAL (dp), INTENT(OUT)  :: vn(:)
REAL (dp), INTENT(OUT)  :: fn_val

!********************************************************************
! THIS SUBROUTINE COMPUTES || F + J*D + 0.5*A*D**2 ||**2 IN THE
! L2 NORM SENS, WHERE D = ALPHA*DT + SQRT(DLT**2-ALPHA**2).

! N.B. Changed to a subroutine by AJM
!********************************************************************

!    INPUT PARAMETERS
!    ----------------

!    ANLS   : TENSOR TERM MATRIX
!    FQ     : FUNCTION VALUE AT CURRENT ITERATE MULTIPLIED BY
!             ORTHOGONAL MATRICES
!    ADT    : JACOBIAN MATRIX TIMES DT
!     AG    : JACOBIAN MATRIX TIMES GBAR (SEE SUBROUTINE TS2DTR)
!    CONST1 : SHAT-TRANS TIMES DT
!    CONST2 : SHAT-TRANS TIMES GBAR
!    ALPHA  : POINT AT WHICH TO EVALUATE THE SUBROUTINE TSFAFA
!    DLT    : CURRENT TRUST RADIUS
!    M,N    : DIMENSIONS OF THE PROBLEM
!    P      : COLUMN DIMENSION OF THE MATRICES SHAT AND ANLS
!    NWTAKE : LOGICAL VARIABLE WITH THE FOLLOWING MEANINGS:
!             NWTAKE = .TRUE.  : STANDARD STEP TAKEN
!             NWTAKE = .FALSE. : TENSOR STEP TAKEN
!    IERR   : RETURN CODE FROM QRP FACTORIZATION ROUTINE:
!             IERR = 0 : NO SINGULARITY OF JACOBIAN DETECTED
!             IERR = 1 : SINGULARITY OF JACOBIAN DETECTED

!    OUTPUT PARAMETERS
!    -----------------

!    VN     : F + J*D + 0.5*A*D**2
!    TSFAFA :  || F + J*D + 0.5*A*D**2 ||**2
!             WHERE D = ALPHA*DT + SQRT(DLT**2-ALPHA**2)

!    SUBPROGRAMS CALLED:

!    TENSOLVE      ...  TSMAFA

!********************************************************************

INTEGER              :: LEN
REAL (dp), PARAMETER :: half = 0.5_dp

CALL tsmafa(anls, fq, adt, ag, const1, const2, alpha, dlt, m, n, p,  &
            nwtake, ierr, vn)

LEN = m
IF(ierr > 0) LEN = m + n

fn_val = half * SUM( vn(1:len)**2 )

RETURN
END SUBROUTINE tsfafa



SUBROUTINE tsfdfj(xc, fc, m, n, epsm, fvec, aja)

REAL (dp), INTENT(IN OUT)  :: xc(:)
REAL (dp), INTENT(IN)      :: fc(:)
INTEGER, INTENT(IN)        :: m
INTEGER, INTENT(IN)        :: n
REAL (dp), INTENT(IN)      :: epsm
REAL (dp), INTENT(OUT)     :: aja(:,:)

INTERFACE
  SUBROUTINE fvec(x, f, m, n) bind(c)
    IMPLICIT NONE
    INTEGER, PARAMETER     :: dp = SELECTED_REAL_KIND(14, 60)
    REAL (dp), INTENT(IN)  :: x(n)
    REAL (dp), INTENT(OUT) :: f(m)
    INTEGER, INTENT(IN)    :: m, n
  END SUBROUTINE fvec
END INTERFACE

!***********************************************************************
! THIS ROUTINE COMPUTES THE FINITE DIFFERENCE JACOBIAN AT THE CURRENT
! ITERATE XC.
!***********************************************************************

!    INPUT PARAMETERS :
!    ----------------

!    XC   : CURRENT ITERATE
!    FC   : FUNCTION VALUE AT XC
!    M,N  : DIMENSIONS OF PROBLEM
!    EPSM : MACHINE PRECISION
!    FVEC : SUBROUTINE TO EVALUATE THE USER'S FUNCTION

!    OUTPUT PARAMETERS :
!    --------------------

!    AJA : FINITE DIFFERENCE JACOBIAN AT XC

!    SUBPROGRAMS CALLED:

!    USER   ...  FVEC

!***********************************************************************

! Workspace
REAL (dp) :: fhat(m)

INTEGER              :: j
REAL (dp)            :: ndigit, rnoise, stepsz, xtmpj, sqrtr, rstpsz
REAL (dp), PARAMETER :: ten = 10.0_dp

ndigit = -LOG10(epsm)
rnoise = MAX(ten**(-ndigit),epsm)
sqrtr = SQRT(rnoise)

DO j = 1,n
  stepsz = sqrtr*MAX(ABS(xc(j)),one)
  xtmpj = xc(j)
  xc(j) = xtmpj+stepsz
  CALL fvec(xc, fhat, m, n)
  xc(j) = xtmpj
  rstpsz = one/stepsz
  aja(1:m,j) = (fhat(1:m) - fc(1:m))*rstpsz
END DO

RETURN
END SUBROUTINE tsfdfj



SUBROUTINE tsfrmt(shat, s, aja, fv, fn, m, n, p, idp, a)

REAL (dp), INTENT(IN OUT)  :: shat(:,:)
REAL (dp), INTENT(IN)      :: s(:,:)
REAL (dp), INTENT(IN)      :: aja(:,:)
REAL (dp), INTENT(IN)      :: fv(:,:)
REAL (dp), INTENT(IN)      :: fn(:)
INTEGER, INTENT(IN)        :: m
INTEGER, INTENT(IN)        :: n
INTEGER, INTENT(IN)        :: p
INTEGER, INTENT(IN)        :: idp(:)
REAL (dp), INTENT(OUT)     :: a(:,:)

!*********************************************************************
! THIS ROUTINE FORM THE TENSOR TERM MATRIX OF THE TENSOR MODEL.
!*********************************************************************

!    INPUT PARAMETERS :
!    ----------------

!    SHAT: MATRIX OF PAST LINEARLY INDEPENDENT DIRECTIONS
!    S   : MATRIX OF PREVIOUS DIRECTIONS
!    AJA : JACOBIAN MATRIX AT CURRENT ITERATE
!    FV  : MATRIX OF PAST FUNCTION VALUES
!    FN  : FUNCTION VALUE AT CURRENT ITERATE
!    M   : ROW DIMENSION OF MATRICES A,FV,AND AJA
!    N   : COLUMN DIMENSION OF JACOBIAN MATRIX
!    P   : COLUMN DIMENSION OF MATRIX SHAT
!    IDP : VECTOR WHICH KEEPS TRACK OF LINEARLY INDEPENDENT
!          DIRECTION POSITIONS WITHIN THE MATRIX S

!    OUTPUT PARAMETERS :
!    ------------------

!    A   : TENSOR TERM MATRIX

!    SUBPROGRAMS CALLED:

!    LEVEL 1 BLAS  ...  DNRM2
!    UNCMIN        ...  CHOLDC,LLTSLV

!*********************************************************************

! Workspace
REAL (dp) :: am(p,p), x(p), b(p), scale(p)

INTEGER   :: i, j, jj
REAL (dp) :: sum, sc, tol, diagmx
REAL (dp), PARAMETER :: two = 2.0_dp

! scale the matrix SHAT and save scaling in SCALE

DO j = 1,p
  sc = one/dnrm2(n, shat(:,j), 1)
  shat(1:n,j) = sc * shat(1:n,j)
  scale(j) = sc**2
END DO

! form the matrix AM = (Si Sj)**2

DO j = 1,p
  jj = idp(j)
  DO i = 1,p
    am(i,j) = DOT_PRODUCT( s(1:n,idp(i)), s(1:n,jj) )**2
  END DO
END DO

! scale the matrix AM

DO i = 1,p
  DO j = 1,p
    am(i,j) = scale(i)*scale(j)*am(i,j)
  END DO
END DO

! perform a Cholesky decomposition of AM

tol = zero
diagmx = zero
CALL choldc(p, am, diagmx, tol)

! form the tensor term matrix A

DO i = 1,m
  DO j = 1,p
    jj = idp(j)
    sum = DOT_PRODUCT( aja(i,1:n), s(1:n,jj) )
    b(j) = two*(fv(i,jj) - fn(i) - sum)
    b(j) = scale(j)*b(j)
  END DO

! solve AM*X = B

  CALL lltslv(p, am, x, b)

! copy X into row i of A

  a(i,1:p) = x(1:p)

END DO

RETURN
END SUBROUTINE tsfrmt



SUBROUTINE tsfscl(x, dx, df, m, n, fvec, f)

REAL (dp), INTENT(IN OUT)  :: x(:)
REAL (dp), INTENT(IN)      :: dx(:)
REAL (dp), INTENT(IN)      :: df(:)
INTEGER, INTENT(IN)        :: m
INTEGER, INTENT(IN)        :: n
REAL (dp), INTENT(OUT)     :: f(:)

INTERFACE
  SUBROUTINE fvec(x, f, m, n) bind(c)
    IMPLICIT NONE
    INTEGER, PARAMETER     :: dp = SELECTED_REAL_KIND(14, 60)
    REAL (dp), INTENT(IN)  :: x(n)
    REAL (dp), INTENT(OUT) :: f(m)
    INTEGER, INTENT(IN)    :: m, n
  END SUBROUTINE fvec
END INTERFACE

!********************************************************************
! THIS ROUTINE EVALUATES THE FUNCTION AT THE CURRENT ITERATE X THEN
! SCALES ITS VALUE.
!********************************************************************

!     INPUT PARAMETERS :
!     -----------------

!     X  : CURRENT ITERATE
!     DX : DIAGONAL SCALING MATRIX FOR X
!     DF : DIAGONAL SCALING MATRIX FOR F
!    M,N : DIMENSIONS OF PROBLEM
!   FVEC : SUBROUTINE TO EVALUATE FUNCTION

!     OUTPUT PARAMETERS :
!     -----------------

!     F  : SCALED FUNCTION VALUE AT CURRENT ITERATE X

!     SUBPROGRAMS CALLED:

!     TENSOLVE      ...  TSUNSX,TSSCLF,TSSCLX
!     USER          ...  FVEC

!********************************************************************

CALL tsunsx(x, dx, n)
CALL fvec(x,f, m, n)
CALL tssclf(f, df, m)
CALL tssclx(x, dx, n)

RETURN
END SUBROUTINE tsfscl



SUBROUTINE tsfslv(l, b, m, n, y)

REAL (dp), INTENT(IN)   :: l(:,:)
REAL (dp), INTENT(IN)   :: b(:)
INTEGER, INTENT(IN)     :: m
INTEGER, INTENT(IN)     :: n
REAL (dp), INTENT(OUT)  :: y(:)

!********************************************************************
! THIS ROUTINE DOES A FORWARD SOLVE.
!********************************************************************

!    INPUT PARAMETERS :
!    -----------------

!    L   : THE TRANSPOSE OF THE UPPER TRIANGULAR MATRIX OBTAINED
!          FROM A QR FACTORIZATION OF AN M BY N MATRIX A. DIAG(L)
!          IS STORED IN ROW M+2. THIS IS THE STORAGE SCHEME USED
!          IN STEWART, G. W., III(1973) "INTRODUCTION TO MATRIX
!          COMPUTATION", ACADEMIC PRESS,NEW YORK
!    B   : RIGHT HAND SIDE
!     M  : ROW DIMENSION OF MATRIX A
!     N  : COLUMN DIMENSION OF MATRIX A

!    OUTPUT PARAMETERS :
!    ------------------

!     Y  : VECTOR SOLUTION ON EXIT

!*********************************************************************

INTEGER   :: j
REAL (dp) :: s

! solve L Y = B

y(1) = b(1) / l(m+2,1)
IF(n > 1) THEN
  s = l(1,2) * y(1)
  y(2) = (b(2) - s) / l(m+2,2)
  DO j = 3,n
    s = DOT_PRODUCT( l(1:j-1,j), y(1:j-1) )
    y(j) = (b(j) - s) / l(m+2,j)
  END DO
END IF

RETURN
END SUBROUTINE tsfslv



SUBROUTINE tsjmuv(itn, method, v, curpos, pivot, pbar, aja, shat,  &
                  flag, ierr, m, n, p, vn, av)

INTEGER, INTENT(IN)     :: itn
INTEGER, INTENT(IN)     :: method
REAL (dp), INTENT(IN)   :: v(:)
INTEGER, INTENT(IN)     :: curpos(:)
INTEGER, INTENT(IN)     :: pivot(:)
INTEGER, INTENT(IN)     :: pbar(:)
REAL (dp), INTENT(IN)   :: aja(:,:)
REAL (dp), INTENT(IN)   :: shat(:,:)
INTEGER, INTENT(IN)     :: flag
INTEGER, INTENT(IN)     :: ierr
INTEGER, INTENT(IN)     :: m
INTEGER, INTENT(IN)     :: n
INTEGER, INTENT(IN)     :: p
REAL (dp), INTENT(OUT)  :: vn(:)
REAL (dp), INTENT(OUT)  :: av(:)

!****************************************************************
! THIS ROUTINE CALCULATES THE PRODUCT JACOBIAN TIMES A VECTOR.
!****************************************************************

!    INPUT PARAMETERS
!    ----------------

!     ITN    : CURRENT ITERATION NUMBER
!     METHOD : METHOD TO BE USED
!     V      : VECTOR TO BE MULTIPLIED BY AJA
!     CURPOS : PIVOT VECTOR (USED DURING THE FACTORIZATION OF AJA
!              FROM COLUMN 1 TO N-P)
!     PIVOT  : PIVOT VECTOR (USED DURING THE FACTORIZATION OF AJA
!              FROM COLUMN N-P+1 TO N)
!     PBAR   : PIVOT VECTOR (USED DURING THE FACTORIZATION OF AJA
!              IF IT IS SINGULAR
!     AJA    : JACOBIAN MATRIX AT CURRENT ITERATE
!     SHAT   : MATRIX OF LINEARLY INDEPENDENT DIRECTIONS AFTER
!              A QL FACTORIZATION
!     FLAG   : RETURN CODE WITH THE FOLLOWING MEANINGS:
!             FLAG = 0 : NO SINGULARITY DETECTED DURING FACTORIZATION
!                        OF THE JACOBIAN FROM COLUMN 1 TO N
!             FLAG = 1 : SINGULARITY DETECTED DURING FACTORIZATION
!                        OF THE JACOBIAN FROM COLUMN 1 TO N-P
!             FLAG = 2 : SINGULARITY DETECTED DURING FACTORIZATION
!                        OF THE JACOBIAN FROM COLUMN N-P+1 TO N
!     IERR   : RETURN CODE FROM QRP FACTORIZATION ROUTINE:
!             IERR = 0 : NO SINGULARITY OF JACOBIAN DETECTED
!             IERR = 1 : SINGULARITY OF JACOBIAN DETECTED
!     M,N    : DIMENSIONS OF THE PROBLEM
!     P      : COLUMN DIMENSION OF THE MATRICES SHAT AND ANLS

!    OUTPUT PARAMETERS
!    -----------------

!     VN     : ? previously erroneously described as workspace
!     AV     : JACOBIAN TIMES V

!    SUBPROGRAMS CALLED:

!    TENSOLVE      ...  TSPRMV,TSQMLV,TSUTMD

! **********************************************************************

! Workspace
REAL (dp) :: wrk1(n), wrk2(n)

INTEGER :: LEN

IF(itn == 1 .OR. method == 0) THEN
  CALL tsprmv(wrk1, v, pivot, n, 1)
  IF(ierr == 1) THEN
    CALL tsprmv(wrk2, wrk1, pbar, n, 1)
    wrk1 = wrk2
  END IF
ELSE IF(n == 1) THEN
  vn(1) = v(1)
ELSE
  CALL tsqmlv(n, p, shat, v, vn, .false.)
  CALL tsprmv(wrk2, vn, curpos, n, 1)
  IF(flag == 0) THEN
    CALL tsprmv(wrk1, wrk2, pivot, n, 1)
  ELSE IF(flag == 1) THEN
    CALL tsprmv(wrk1, wrk2, pbar, n, 1)
  ELSE IF(flag == 2 ) THEN
    CALL tsprmv(wrk1, wrk2, pivot, n, 1)
    CALL tsprmv(wrk2, wrk1, pbar, n, 1)
    wrk1 = wrk2
  END IF
END IF

LEN = m
IF(ierr > 0) LEN = m + n

CALL tsutmd(aja, wrk1, LEN, n, av)

RETURN
END SUBROUTINE tsjmuv



SUBROUTINE tsjqtp(q, n, m, p, aja)

REAL (dp), INTENT(IN)      :: q(:,:)
INTEGER, INTENT(IN)        :: n
INTEGER, INTENT(IN)        :: m
INTEGER, INTENT(IN)        :: p
REAL (dp), INTENT(IN OUT)  :: aja(:,:)

!**********************************************************************
! THIS ROUTINE GETS J*(Q-TRANS) BY COMPUTING EACH ROW OF THE
! RESULTING MATRIX AS FOLLOWS : (J*Q-TRANS)I-TH ROW<--Q*(J)I-TH ROW.
!**********************************************************************

!    INPUT PARAMETERS :
!    -----------------

!    Q    : RESULTING MATRIX FROM A QL FACTORIZATION
!    M,N  : DIMENSIONS OF PROBLEM
!    P    : COLUMN DIMENSION OF MATRIX Q

!    INPUT-OUTPUT PARAMETERS :
!    ------------------------

!    AJA : JACOBIAN MATRIX ON ENTRY AND JACOBIAN MULTIPLIED BY THE
!          ORTHOGONAL MATRIX Q ON EXIT

!    SUBPROGRAMS CALLED:

!    TENSOLVE      ...  TSQMLV

!**********************************************************************

! Workspace
REAL (dp) :: wrk1(n), wrk2(n)

INTEGER   :: i

DO i = 1,m

! copy the i-th row of AJA into WRK1

  wrk1 = aja(i,1:n)

  CALL tsqmlv(n, p, q, wrk1, wrk2, .false.)

! form the i-th row of AJA*(Q-trans)

  aja(i,1:n) = wrk2

END DO

RETURN
END SUBROUTINE tsjqtp



SUBROUTINE tslmin(xc, xp, p1, q, anls, fq, adt, ag, const1, const2,  &
                  dlt, m, n, p, nwtake, ierr, tol, xplus)

REAL (dp), INTENT(IN OUT)  :: xc
REAL (dp), INTENT(IN)      :: xp
REAL (dp), INTENT(IN OUT)  :: p1
REAL (dp), INTENT(OUT)     :: q
REAL (dp), INTENT(IN)      :: anls(:,:)
REAL (dp), INTENT(IN)      :: fq(:)
REAL (dp), INTENT(IN)      :: adt(:)
REAL (dp), INTENT(IN)      :: ag(:)
REAL (dp), INTENT(IN)      :: const1(:)
REAL (dp), INTENT(IN)      :: const2(:)
REAL (dp), INTENT(IN)      :: dlt
INTEGER, INTENT(IN)        :: m
INTEGER, INTENT(IN)        :: n
INTEGER, INTENT(IN)        :: p
LOGICAL, INTENT(IN)        :: nwtake
INTEGER, INTENT(IN)        :: ierr
REAL (dp), INTENT(IN)      :: tol
REAL (dp), INTENT(OUT)     :: xplus

!***********************************************************************
! THIS ROUTINE FINDS A LOCAL MINIMIZER OF A ONE-VARIABLE FUNCTION IN AN
! INTERVAL [XC XP].
!***********************************************************************

!    INPUT PARAMETERS :
!    -----------------

!    XC,XP  : LOWER AND UPPER BOUND OF INTERVAL IN WHICH THE SEARCH IS PERFORMED
!    P1,Q   : FIRST DERIVATIVES OF THE ONE-VARIABLE FUNCTION
!    ANLS   : TENSOR TERM MATRIX
!    FQ     : FUNCTION VALUE AT CURRENT ITERATE MULTIPLIED BY
!             ORTHOGONAL MATRICES
!    ADT    : JACOBIAN TIMES THE STEP DT (SEE SUBROUTINE TS2DTR)
!    AG     : JACOBIAN TIMES THE GRADIENT G (SEE SUBROUTINE TS2DTR)
!    CONST1 : SHAT-TRANS * DT  (SEE SUBROUTINE TS2DTR)
!    CONST2 : SHAT-TRANS * GBAR (SEE SUBROUTINE TS2DTR)
!    DLT    : TRUST RADIUS
!    M,N    : DIMENSIONS OF PROBLEM
!    P      : COLUMN DIMENSION OF MATRIX ANLS
!    NWTAKE : LOGICAL VARIABLE WITH THE FOLLOWING MEANINGS:
!             NWTAKE = .TRUE.  : STANDARD STEP TAKEN
!             NWTAKE = .FALSE. : TENSOR STEP TAKEN
!    IERR   : RETURN CODE FROM QRP FACTORIZATION ROUTINE:
!             IERR = 0 : NO SINGULARITY OF JACOBIAN DETECTED
!             IERR = 1 : OTHERWISE
!    TOL    : SMALL TOLERANCE

!    OUTPUT PARAMETERS :
!    -----------------

!    XPLUS  :  LOCAL MINIMIZER OF THE ONE-VARIABLE FUNCTION

!    SUBPROGRAMS CALLED :

!    TENSOLVE      ...  TSMSDA,TSFAFA,TSLMSP,TSMFDA

!***********************************************************************

! Workspace
REAL (dp) :: vn(n+m)

INTEGER              :: itercd, retcd, itncnt
REAL (dp)            :: aleft, aright, t, e, s, sinit, tmp
REAL (dp), PARAMETER :: ott = 1.0e-04_dp, two = 2.0_dp, small = 2.0e-20_dp
LOGICAL              :: skip

retcd = 0
aleft = MIN(xc,xp)
aright = MAX(xc,xp)
itncnt = 0
t = ABS(xc-xp)
skip = .false.

! compute the second derivative value at the current point

CALL tsmsda(anls, fq, adt, ag, const1, const2, xc, dlt, m, n, p,  &
            nwtake, ierr, skip, vn, e)

10 IF(e > zero) THEN
  s = -p1/e
  IF(ABS(s) > two*t) THEN
    IF (s < zero) THEN
      s = -two*t
    ELSE
      s = two*t
    END IF
  END IF
ELSE
  IF (p1 > zero) THEN
    s = -t
  ELSE
    s = t
  END IF
END IF

IF(xc+s > aright) s = aright - xc
IF(xc+s < aleft)  s = aleft - xc
sinit = ABS(s)

! compute a next iterate XPLUS

20 CALL tsfafa(anls, fq, adt, ag, const1, const2, xc+s, dlt, m, n, p,  &
               nwtake, ierr, vn, tmp)
IF (tmp > q + ott*s*p1) THEN
  s = s/2
  IF(ABS(s) < small*sinit .OR. s == zero) THEN
    retcd = 1
  ELSE
    GO TO 20
  END IF
END IF

xplus = xc + s
itncnt = itncnt + 1

! check stopping criteria

CALL tslmsp(xc, xplus, itncnt, retcd, itercd, anls, adt, ag,  &
            const1, const2, dlt, m, n, p, nwtake, ierr, tol, vn)

IF(itercd > 0) RETURN

! update XC

xc = xplus

! compute function and derivative values at the new point

CALL tsfafa(anls, fq, adt, ag, const1, const2, xc, dlt, m, n, p, nwtake, &
            ierr, vn, q)
p1 = tsmfda(anls, adt, ag, const1, const2, xc, dlt, m, n, p, nwtake, &
            ierr, vn)
skip = .true.
CALL tsmsda(anls, fq, adt, ag, const1, const2, xc, dlt, m, n, p,  &
            nwtake, ierr, skip, vn, e)
GO TO 10

END SUBROUTINE tslmin



SUBROUTINE tslmsp(xc, xp, itncnt, retcd, itercd, anls, adt, ag, const1,  &
                  const2, dlt, m, n, p, nwtake, ierr, tol, vn)

REAL (dp), INTENT(IN)  :: xc
REAL (dp), INTENT(IN)  :: xp
INTEGER, INTENT(IN)    :: itncnt
INTEGER, INTENT(IN)    :: retcd
INTEGER, INTENT(OUT)   :: itercd
REAL (dp), INTENT(IN)  :: anls(:,:)
REAL (dp), INTENT(IN)  :: adt(:)
REAL (dp), INTENT(IN)  :: ag(:)
REAL (dp), INTENT(IN)  :: const1(:)
REAL (dp), INTENT(IN)  :: const2(:)
REAL (dp), INTENT(IN)  :: dlt
INTEGER, INTENT(IN)    :: m
INTEGER, INTENT(IN)    :: n
INTEGER, INTENT(IN)    :: p
LOGICAL, INTENT(IN)    :: nwtake
INTEGER, INTENT(IN)    :: ierr
REAL (dp), INTENT(IN)  :: tol
REAL (dp), INTENT(IN)  :: vn(:)

!***********************************************************************
! THIS ROUTINE CHECKS THE STOPPING CRITERIA FOR A LOCAL MINIMIZER.
!***********************************************************************

!    INPUT PARAMETERS :
!    -----------------

!    XC       : CURRENT ITERATE (FROM SEARCH SUBROUTINE)
!    XP       : NEXT ITERATE (FROM SEARCH SUBROUTINE)
!    ITNCNT   : ITERATION LIMIT
!    RETCD    : RETURN CODE FROM LINE SEARCH
!    DLT      : TRUST RADIUS
!    AJA      : JACOBIAN AT THE CURRENT ITERATE
!    NR       : LEADING DIMENSION OF THE JACOBIAN MATRIX
!    M,N      : DIMENSIONS OF THE PROBLEM
!    P        : COLUMN DIMENSION OF THE MATRICES SHAT AND ANLS
!    NWTAKE   : LOGICAL VARIABLE WITH THE FOLLOWING MEANINGS :
!               NWTAKE = .TRUE.  : STANDARD STEP TAKEN
!               NWTAKE = .FALSE. : TENSOR STEP TAKEN
!    IERR     : RETURN CODE FROM THE QRP FACTORIZATION ROUTINE :
!               IERR = 0 : NO SINGULARITY OF JACOBIAN DETECTED
!               IERR = 1 : OTHERWISE
!    VN       : WORKING VECTOR
!    TOL      : SMALL TOLERANCE
!    METHOD   : METHOD TO USE
!             = 0   : STANDARD METHOD USED
!             = 1   : TENSOR METHOD USED

!    OUTPUT PARAMETERS :
!    ------------------

!    ITERCD  : RETURN CODE WITH FOLLOWING MEANINGS :
!              ITERCD = 1 : FIRST DERIVATIVE AT THE CURRENT POINT
!                           CLOSE TO 0
!              ITERCD = 2 : SUCCESSIVE ITERATES WITHIN TOLERANCE
!              ITERCD = 3 : LINE SEARCH FAILED TO LOCATE A POINT
!                           LOWER THAT THE CURRENT POINT
!              ITERCD = 4 : ITERATION LIMIT EXCEEDED

!***********************************************************************

REAL (dp) :: grdt

grdt = SQRT(tol)
itercd = 0

IF(retcd == 1) THEN
  itercd = 3
ELSE IF(ABS(tsmfda(anls, adt, ag, const1, const2, xp, dlt,  &
                   m, n, p, nwtake, ierr, vn)) < grdt) THEN
  itercd = 1
ELSE IF(xp /= zero .AND. ABS(xp-xc)/ABS(xp) <= tol) THEN
  itercd = 2
ELSE IF(itncnt >= 150) THEN
  itercd = 4
END IF

RETURN
END SUBROUTINE tslmsp



SUBROUTINE tslsch(m, n, xc, d, g, steptl, dx, df, fvec,  &
                  mxtake, stepmx, xp, fp, fcnorm, fpnorm, retcd)

INTEGER, INTENT(IN)        :: m
INTEGER, INTENT(IN)        :: n
REAL (dp), INTENT(IN)      :: xc(:)
REAL (dp), INTENT(IN OUT)  :: d(:)
REAL (dp), INTENT(IN)      :: g(:)
REAL (dp), INTENT(IN)      :: steptl
REAL (dp), INTENT(IN)      :: dx(:)
REAL (dp), INTENT(IN)      :: df(:)
LOGICAL, INTENT(OUT)       :: mxtake
REAL (dp), INTENT(IN)      :: stepmx
REAL (dp), INTENT(OUT)     :: xp(:)
REAL (dp), INTENT(OUT)     :: fp(:)
REAL (dp), INTENT(IN)      :: fcnorm
REAL (dp), INTENT(OUT)     :: fpnorm
INTEGER, INTENT(OUT)       :: retcd

INTERFACE
  SUBROUTINE fvec(x, f, m, n) bind(c)
    IMPLICIT NONE
    INTEGER, PARAMETER     :: dp = SELECTED_REAL_KIND(14, 60)
    REAL (dp), INTENT(IN)  :: x(n)
    REAL (dp), INTENT(OUT) :: f(m)
    INTEGER, INTENT(IN)    :: m, n
  END SUBROUTINE fvec
END INTERFACE

!**********************************************************************
! THIS ROUTINE FINDS A NEXT ITERATE USING A STANDARD LINE SEARCH METHOD.
!**********************************************************************

!       INPUT PARAMETERS :
!       -----------------

!       M,N : DIMENSIONS OF PROBLEM
!       XC  : CURRENT ITERATE
!       D   : SEARCH DIRECTION
!       G   : GRADIENT AT CURRENT ITERATE
!    STEPTL : RELATIVE STEP SIZE AT WHICH SUCCESSIVE ITERATES
!                ARE CONSIDERED CLOSE ENOUGH TO TERMINATE ALGORITHM
!       DX  : DIAGONAL SCALING MATRIX FOR X
!       DF  : DIAGONAL SCALING MATRIX FOR F
!       FVEC: SUBROUTINE TO EVALUATE THE FUNCTION
!     STEPMX: MAXIMUM ALLOWABLE STEP SIZE

!       OUTPUT PARAMETERS :
!       -----------------

!    MXTAKE: BOOLEAN FLAG INDICATING STEP OF MAXIMUM LENGTH USED
!       XP : NEXT ITARATE
!       FP : FUNCTION VALUE AT NEXT ITERATE
!   FCNORM : 0.5 * || F(XC) ||**2
!   FPNORM : 0.5 * || F(XP) ||**2
!    RETCD : RETURN CODE WITH THE FOLLOWING MEANING :
!                RETCD = 0 : SATISFACTORY LOCATION OF A NEW ITERATE
!                RETCD = 1 : NO SATISFACTORY POINT FOUND SUFFICIENTLY
!                            DISTINCT FROM X

!       SUBPROGRAMS CALLED:

!       LEVEL 1 BLAS  ...  DNRM2
!       TENSOLVE      ...  TSFSCL
!       USER          ...  FVEC

!**********************************************************************

INTEGER   :: i
REAL (dp) :: slope, releng, temp1, temp2, almda, temp, almdat, almdam, sln, scl
REAL (dp), PARAMETER :: alpha = 1.0D-4, tenth = 0.1_dp, half = 0.5_dp,  &
                        z99 = 0.99_dp, two = 2.0_dp, ten = 10.0_dp

mxtake = .false.
sln = dnrm2(n, d, 1)
IF(sln > stepmx) THEN

! step longer than maximum allowed

  scl = stepmx/sln
  d(1:n) = scl * d(1:n)
  sln = stepmx
END IF

! compute SLOPE  =  G-trans * D

slope = DOT_PRODUCT( g(1:n), d(1:n) )

! initialization of RETCD

retcd = 0

! compute the smallest value allowable for the damping
! parameter ALMDA, i.e, ALMDAM

releng = zero
DO i = 1,n
  temp1 = MAX(ABS(xc(i)), one)
  temp2 = ABS(d(i))/temp1
  releng = MAX(releng,temp2)
END DO
almdam = steptl/releng
almda = one

! compute the next iterate XP

40 xp(1:n) = xc(1:n) + almda*d(1:n)

! evaluate the objective function at XP and its residual

CALL tsfscl(xp, dx, df, m, n, fvec, fp)

fpnorm = half*dnrm2(m, fp, 1)**2

! test whether the full step produces enough decrease in the l2 norm of
! the objective function.  If not update ALMDA and compute a new step

IF (fpnorm > (fcnorm + (alpha* almda * slope))) THEN
  almdat = ((-almda**2)*slope) / (two*(fpnorm - fcnorm - almda*slope))
  temp = almda/ten
  almda = MAX(temp,almdat)
  IF(almda < almdam) THEN
    retcd = 1
    RETURN
  END IF
  GO TO 40
ELSE
  IF(almda == tenth .AND. sln > z99*stepmx) mxtake=.true.
END IF

RETURN
END SUBROUTINE tslsch



SUBROUTINE tsmafa(anls, f, adt, ag, const1, const2, alpha, dlt,  &
                  m, n, p, nwtake, ierr, vn)

REAL (dp), INTENT(IN)   :: anls(:,:)
REAL (dp), INTENT(IN)   :: f(:)
REAL (dp), INTENT(IN)   :: adt(:)
REAL (dp), INTENT(IN)   :: ag(:)
REAL (dp), INTENT(IN)   :: const1(:)
REAL (dp), INTENT(IN)   :: const2(:)
REAL (dp), INTENT(IN)   :: alpha
REAL (dp), INTENT(IN)   :: dlt
INTEGER, INTENT(IN)     :: m
INTEGER, INTENT(IN)     :: n
INTEGER, INTENT(IN)     :: p
LOGICAL, INTENT(IN)     :: nwtake
INTEGER, INTENT(IN)     :: ierr
REAL (dp), INTENT(OUT)  :: vn(:)

!***********************************************************************
! THIS ROUTINE COMPUTES THE VECTOR VN = F(XC) + J(XC)*D + 0.5*A*D**2,
! WHERE D = ALPHA*DT + SQRT(DLT**2-ALPHA**2).
!***********************************************************************

!    INPUT PARAMETERS :
!    -----------------

!    ANLS  : TENSOR TERM MATRIX
!     ADT  : JACOBIAN MATRIX TIMES DT (SEE SUBROUTINE TS2DTR)
!      AG  : JACOBIAN MATRIX TIMES GBAR (SEE SUBROUTINE TS2DTR)
!    CONST1: SHAT-TRANS * DT (SEE SUBROUTINE TS2DTR)
!    CONST2: SHAT-TRABS * GBAR (SEE SUBROUTINE TS2DTR)
!    ALPHA : POINT AT WHICH DERIVATIVE IS EVALUATED
!      DLT : CURRENT TRUST RADIUS
!      M,N : DIMENSIONS OF THE PROBLEM
!      P   : COLUMN DIMENSION OF THE MATRIX ANLS
!   NWTAKE : LOGICAL VARIABLE WITH THE FOLLOWING MEANINGS
!               NWTAKE = .TRUE.  : STANDARD STEP TAKEN
!               NWTAKE = .FALSE. : TENSOR STEP TAKEN
!     IERR : RETURN CODE FROM THE QRP FACTORIZATION ROUTINE :
!            IERR = 0 : NO SINGULARITY OF JACOBIAN DETECTED
!            IERR = 1 : SINGULARITY OF JACOBIAN DETECTED

!     OUTPUT PARAMETERS :
!     -------------------

!     VN  : F + J*D + 0.5*A*D**2, WHERE
!           D = ALPHA*DT + SQRT(DLT**2-ALPHA**2)

!*******************************************************************

INTEGER   :: i, j, LEN
REAL (dp) :: expr, const
REAL (dp), PARAMETER :: half = 0.5_dp

expr = SQRT(dlt**2 - alpha**2)
vn(1:n) = alpha*adt(1:n) + expr*ag(1:n)

vn(n+1:n+m) = zero

LEN = m
IF(ierr > 0) LEN = m + n

DO i = 1, LEN
  vn(i) = vn(i) + f(i)
END DO

IF(nwtake) RETURN
DO j = 1,p
  const = half*(alpha*const1(j) + expr*const2(j))**2
  vn(1:len) = vn(1:len) + const * anls(1:len,j)
END DO

RETURN
END SUBROUTINE tsmafa



SUBROUTINE tsmdls(aja, shat, anls, xc, m, n, p, dt, g, dx, df, &
                  fvec, method, steptl, global, stepmx, epsm, fq, dn, fqq, &
                  pivot, curpos, pbar, mxtake, xp, fp, fcnorm, fpnorm,   &
                  zero1, retcd, ierr)

REAL (dp), INTENT(IN OUT)  :: aja(:,:)
REAL (dp), INTENT(IN)      :: shat(:,:)
REAL (dp), INTENT(IN OUT)  :: anls(:,:)
REAL (dp), INTENT(IN)      :: xc(:)
INTEGER, INTENT(IN)        :: m
INTEGER, INTENT(IN)        :: n
INTEGER, INTENT(IN)        :: p
REAL (dp), INTENT(IN OUT)  :: dt(:)
REAL (dp), INTENT(IN)      :: g(:)
REAL (dp), INTENT(IN)      :: dx(:)
REAL (dp), INTENT(IN)      :: df(:)
INTEGER, INTENT(IN)        :: method
REAL (dp), INTENT(IN)      :: steptl
INTEGER, INTENT(IN)        :: global
REAL (dp), INTENT(IN)      :: stepmx
REAL (dp), INTENT(IN)      :: epsm
REAL (dp), INTENT(IN)      :: fq(:)
REAL (dp), INTENT(OUT)     :: dn(:)
REAL (dp), INTENT(OUT)     :: fqq(:)
INTEGER, INTENT(OUT)       :: pivot(:)
INTEGER, INTENT(IN)        :: curpos(:)
INTEGER, INTENT(OUT)       :: pbar(:)
LOGICAL, INTENT(OUT)       :: mxtake
REAL (dp), INTENT(OUT)     :: xp(:)
REAL (dp), INTENT(OUT)     :: fp(:)
REAL (dp), INTENT(IN)      :: fcnorm
REAL (dp), INTENT(OUT)     :: fpnorm
INTEGER, INTENT(OUT)       :: zero1
INTEGER, INTENT(OUT)       :: retcd
INTEGER, INTENT(IN OUT)    :: ierr

INTERFACE
  SUBROUTINE fvec(x, f, m, n) bind(c)
    IMPLICIT NONE
    INTEGER, PARAMETER     :: dp = SELECTED_REAL_KIND(14, 60)
    REAL (dp), INTENT(IN)  :: x(n)
    REAL (dp), INTENT(OUT) :: f(m)
    INTEGER, INTENT(IN)    :: m, n
  END SUBROUTINE fvec
END INTERFACE

!**********************************************************************
! THIS ROUTINE FINDS A NEXT ITERATE USING A LINE SEARCH METHOD.  IT
! TRIES THE FULL TENSOR STEP FIRST. IF THIS IS NOT SUCCESSFUL THEN
! IT COMPUTES THE STANDARD DIRECTION AND COMPUTES A STEP IN THAT
! DIRECTION. NEXT, IF THE TENSOR DIRECTION IS DESCENT, IT COMPUTES
! A STEP IN THE TENSOR DIRECTION.  THE ITERATE THAT PRODUCES
! THE LOWER RESIDUAL IS THE NEXT ITERATE FOR THE NONLINEAR ALGORITHM.
!**********************************************************************

!    INPUT PARAMETERS
!    ----------------

!    AJA    : JACOBIAN AT CURRENT ITERATE
!    SHAT   : MATRIX OF PAST LINEARLY INDEPENDENT DIRECTIONS
!             AFTER A QL FACORIZATION
!    ANLS   : TENSOR TERM MATRIX
!    XC     : CURRENT ITERATE
!    M,N    : DIMENSIONS OF THE PROBLEM
!    P      : COLUMN DIMENSION OF THE MATRICES SHAT AND ANLS
!    DT     : TENSOR STEP
!    G      : GRADIENT AT CURRENT ITERATE
!    DX     : DIAGONAL SCALING MATRIX FOR X
!    DF     : DIAGONAL SCALING MATRIX FOR F
!    GBAR   : STEEPEST DESCENT DIRECTION (= -G)
!    METHOD : METHOD TO USE
!             = 0  : STANDARD METHOD USED
!             = 1  : TENSOR METHOD USED
!    STEPTL : STEP TOLERANCE
!    GLOBAL : GLOBAL STRATEGY USED
!                =  0 : LINE SEARCH IS USED
!                =  1 : 2-DIMENSIONAL TRUST REGION IS USED
!    STEPMX : MAXIMUM ALLOWABLE STEP SIZE
!    EPSM   : MACHINE PRECISION
!    FQ     : FUNCTION VALUE AT CURRENT ITERATE MULTIPLIED BY AN
!             ORTHOGOL MATRIX

!    OUTPUT PARAMETERS
!    -----------------

!    DN     : NEWTON STEP
!    FQQ    : FQ MULTIPLIED BY AN ORTHOGONAL MATRIX
!    CURPOS : PIVOT VECTOR (USED DURING THE FACTORIZATION OF THE
!             JACOBIAN FROM COLUMN 1 TO N-P)
!    PIVOT  : PIVOT VECTOR (USED DURING THE FACTORIZATION OF THE
!             JACOBIAN FROM COLUMN N-P+1 TO N)
!    PBAR   : PIVOT VECTOR (USED DURING THE FACTORIZATION OF THE
!             JACOBIAN IF IT IS SINGULAR
!    MXTAKE : BOOLEAN FLAG INDICATING STEP OF MAXIMUM LENGTH USED
!    XP     : NEXT ITERATE
!    FP     : FUNCTION VALUE AT NEXT ITERATE
!    FCNORM :  0.5 * || F(XC) ||**2
!    FPNORM :  0.5 * || F(XP) ||**2
!    ZERO1  : FIRST ZERO COLUMN OF THE JACOBIAN IN CASE OF SINGULARITY
!    RETCD  : RETURN CODE WITH THE FOLLOWING MEANING :
!             RETCD  =  0 : SATISFACTORY LOCATION OF A NEW ITERATE
!             RETCD  =  1 : NO SATISFACTORY POINT FOUND SUFFICIENTLY
!                           DISTINCT FROM X
!    IERR   : RETURN CODE FROM THE QRP FACTORIZATION ROUTINE
!             IERR = 0 : NO SINGULARITY OF JACOBIAN DETECTED
!             IERR = 1 : SINGULARITY OF JACOBIAN DETECTED

!    SUBPROGRAMS CALLED:

!    LEVEL 1 BLAS  ...  DNRM2
!    TENSOLVE      ...  TSFSCL,TSCPSS,TSLSCH

!***********************************************************************

! Workspace
REAL (dp) :: wrk1(n), wrk2(m)

INTEGER   :: i, flag = 0, retcd1
REAL (dp) :: slope, releng, temp1, temp2, almda, resnew, f1n, dtnorm, gnorm, &
             sln, scl, beta, temp, almdat, almdam
REAL (dp), PARAMETER :: alpha = 1.0e-4_dp, tenth = 0.1_dp, half = 0.5_dp,  &
                        z99 = 0.99_dp, two = 2.0_dp, ten = 10.0_dp

mxtake = .false.
sln = dnrm2(n, dt, 1)
IF(sln > stepmx) THEN

! step longer than maximum allowed

  scl = stepmx/sln
  dt(1:n) = scl * dt(1:n)
  sln = stepmx
END IF

! compute SLOPE = G-Trans * DT

slope = DOT_PRODUCT( g(1:n), dt(1:n) )

! initialization of RETCD

retcd = 0

! compute the smallest value allowable for the damping
! parameter ALMDA, i.e, ALMDAM

releng = zero
DO i = 1,n
  temp1 = MAX(ABS(xc(i)), one)
  temp2 = ABS(dt(i))/temp1
  releng = MAX(releng, temp2)
END DO
almdam = steptl/releng
almda = one

! compute the next iterate XP

xp(1:n) = xc(1:n) + almda*dt(1:n)

! evaluate the objective function at XP and its residual

CALL tsfscl(xp, dx, df, m, n, fvec, fp)

fpnorm = half*dnrm2(m, fp, 1)**2

! test whether the full tensor step produces enough decrease in the
! l2 norm of of the objective function

IF (fpnorm < fcnorm + alpha* almda * slope) RETURN

! compute the standard direction

CALL tscpss(shat, m, n, p, method, global, epsm, fq, aja,  &
            anls, dn, fqq, pivot, curpos, pbar, zero1, ierr, resnew, flag)

! compute a step in the standard direction

CALL tslsch(m, n, xc, dn, g, steptl, dx, df, fvec,  &
            mxtake, stepmx, wrk1, wrk2, fcnorm, f1n, retcd1)

! test whether the tensor direction is descent

dtnorm = dnrm2(n, dt, 1)
gnorm = dnrm2(n, g, 1)
IF(m > n) THEN
  beta = tenth
ELSE
  beta = alpha
END IF
temp1 = -beta*dtnorm*gnorm

! compute a step in the tensor direction

IF(slope <= temp1) THEN
  50 almdat = ((-almda**2)*slope)/(two*(fpnorm - fcnorm - almda*slope))
  temp = almda/ten
  almda = MAX(temp, almdat)
  IF(almda < almdam) THEN
    IF(retcd1 == 1) THEN
      retcd = 1
      GO TO 70
    END IF
  END IF
  xp(1:n) = xc(1:n) + almda*dt(1:n)
  CALL tsfscl(xp, dx, df, m, n, fvec, fp)
  fpnorm = half*dnrm2(m, fp, 1)**2
  IF (fpnorm > (fcnorm + (alpha* almda * slope))) GO TO 50
  IF(almda == tenth .AND. sln > z99*stepmx) mxtake=.true.

! select the next iterate that produces the lower function value

  70 IF(f1n < fpnorm) THEN
    xp(1:n) = wrk1(1:n)
    fp(1:m) = wrk2(1:m)
    fpnorm  =  f1n
  END IF
ELSE
  xp(1:n) = wrk1(1:n)
  fp(1:m) = wrk2(1:m)
  fpnorm = f1n
END IF

RETURN
END SUBROUTINE tsmdls



FUNCTION tsmfda(anls, adt, ag, const1, const2, alpha, dlt,  &
                m, n, p, nwtake, ierr, vn) RESULT(fn_val)

REAL (dp), INTENT(IN)  :: anls(:,:)
REAL (dp), INTENT(IN)  :: adt(:)
REAL (dp), INTENT(IN)  :: ag(:)
REAL (dp), INTENT(IN)  :: const1(:)
REAL (dp), INTENT(IN)  :: const2(:)
REAL (dp), INTENT(IN)  :: alpha
REAL (dp), INTENT(IN)  :: dlt
INTEGER, INTENT(IN)    :: m
INTEGER, INTENT(IN)    :: n
INTEGER, INTENT(IN)    :: p
LOGICAL, INTENT(IN)    :: nwtake
INTEGER, INTENT(IN)    :: ierr
REAL (dp), INTENT(IN)  :: vn(:)

REAL (dp)              :: fn_val

!***********************************************************************
! THIS FUNCTION COMPUTES THE DERIVATIVE OF || F + J*D + 0.5*A*D**2 ||**2
! IN THE L2 NORM SENS, WHERE D = ALPHA*DT + SQRT(DLT**2-ALPHA**2).
!***********************************************************************

!    INPUT PARAMETERS
!    ----------------

!    ANLS   : TENSOR MATRIX
!    FQ     : FUNCTION VALUE AT CURRENT ITERATE MULTIPLIED BY
!             ORTHOGONAL MATRICES
!    ADT    : JACOBIAN MATRIX TIMES DT (SEE SUBROUTINE TS2DTR)
!    AG     : JACOBIAN MATRIX TIMES GBAR (SEE SUBROUTINE TS2DTR)
!    CONST1 : SHAT-TRANS TIMES DT (SEE SUBROUTINE TS2DTR)
!    CONST2 : SHAT-TRANS TIMES GBAR (SEE SUBROUTINE TS2DTR)
!    ALPHA  : POINT AT WHICH TO EVALUATE THE DERIVATIVE OF FUNCTION
!    DLT    : CURRENT TRUST RADIUS
!    M,N    : DIMENSIONS OF THE PROBLEM
!    P      : COLUMN DIMENSION OF THE MATRICES SHAT AND ANLS
!    NWTAKE : LOGICAL VARIABLE WITH THE FOLLOWING MEANINGS:
!             NWTAKE = .TRUE.  : STANDARD STEP TAKEN
!             NWTAKE = .FALSE. : TENSOR STEP TAKEN
!    IERR   : RETURN CODE FROM QRP FACTORIZATION ROUTINE:
!             IERR=0 : NO SINGULARITY OF JACOBIAN DETECTED
!             IERR=1 : SINGULARITY OF JACOBIAN DETECTED

!    OUTPUT PARAMETERS
!    -----------------

!    VN     : F + J*D + 0.5*A*D**2
!    TSMFDA : DERIVATIVE IN ALPHA OF || F + J*D + 0.5*A*D**2 ||**2
!             WHERE D = ALPHA*DT + SQRT(DLT**2 - ALPHA**2)

!    SUBPROGRAMS CALLED:

!    TENSOLVE      ...  TSMFDV

!***********************************************************************
! Workspace
!       VNP    : DERIVATIVE IN ALPHA OF F + J*D + 0.5*A*D**2
REAL (dp) :: vnp(n+m)

INTEGER   :: len

CALL tsmfdv(anls, adt, ag, const1, const2, alpha, dlt, m, n, p, nwtake, &
            ierr, vnp)

len = m
IF(ierr > 0) len = m + n

fn_val = DOT_PRODUCT( vnp(1:len), vn(1:len) )

RETURN
END FUNCTION tsmfda



SUBROUTINE tsmfdv(anls, adt, ag, const1, const2, alpha, dlt,  &
                  m, n, p, nwtake, ierr, vnp)

REAL (dp), INTENT(IN)   :: anls(:,:)
REAL (dp), INTENT(IN)   :: adt(:)
REAL (dp), INTENT(IN)   :: ag(:)
REAL (dp), INTENT(IN)   :: const1(:)
REAL (dp), INTENT(IN)   :: const2(:)
REAL (dp), INTENT(IN)   :: alpha
REAL (dp), INTENT(IN)   :: dlt
INTEGER, INTENT(IN)     :: m
INTEGER, INTENT(IN)     :: n
INTEGER, INTENT(IN)     :: p
LOGICAL, INTENT(IN)     :: nwtake
INTEGER, INTENT(IN)     :: ierr
REAL (dp), INTENT(OUT)  :: vnp(:)

!***********************************************************************
! THIS ROUTINE COMPUTES THE DERIVATIVE IN ALPHA OF THE VECTOR
! VN = F(XC) + J(XC)*D + 0.5*A*D**2, WHERE D = ALPHA*DT +
! SQRT(DLT**2-ALPHA**2).
!***********************************************************************

!    INPUT PARAMETERS :
!    -----------------

!    ANLS  : TENSOR TERM MATRIX
!     ADT  : JACOBIAN MATRIX TIMES DT (SEE SUBROUTINE TS2DTR)
!      AG  : JACOBIAN MATRIX TIMES GBAR (SEE SUBROUTINE TS2DTR)
!    CONST1: SHAT-TRANS TIMES DT (SEE SUBROUTINE TS2DTR)
!    CONST2: SHAT-TRANS TIMES GBAR (SEE SUBROUTINE TS2DTR)
!    ALPHA : POINT AT WHICH DERIVATIVE IS EVALUATED
!      DLT : CURRENT TRUST RADIUS
!      M,N : DIMENSIONS OF THE PROBLEM
!      P   : COLUMN DIMENSION OF THE MATRIX ANLS
!   NWTAKE : LOGICAL VARIABLE WITH THE FOLLOWING MEANINGS :
!              NWTAKE = .TRUE.  : STANDARD STEP TAKEN
!              NWTAKE = .FALSE. : TENSOR STEP TAKEN
!     IERR : RETURN CODE FROM THE QRP FACTORIZATION ROUTINE
!             IERR = 0 : NO SINGULARITY OF JACOBIAN DETECTED
!             IERR = 1 : SINGULARITY OF JACOBIAN DETECTED

!     OUTPUT PARAMETERS :
!     -------------------

!     VNP  : THE DERIVATIVE IN ALPHA OF VN = F(XC) + J(XC)*D +
!            0.5*A*D**2, WHERE D = ALPHA*DT +  SQRT(DLT**2-ALPHA**2)

!*******************************************************************

INTEGER   :: j, len
REAL (dp) :: quant1, quant2, expr, const
REAL (dp), PARAMETER :: half = 0.5_dp, two = 2.0_dp

quant1 = SQRT(dlt**2 - alpha**2)
expr = - alpha/quant1

vnp(1:n) = adt(1:n) + expr*ag(1:n)

vnp(n+1:n+m) = zero

IF(nwtake) RETURN

quant2 = quant1 - alpha**2/quant1

len = m
IF(ierr > 0) len = m + n

DO j = 1,p
  const = half*(two*alpha*(const1(j)**2 - const2(j)**2)  &
          + two*const1(j)*const2(j)*quant2)
  vnp(1:len) = vnp(1:len) + const * anls(1:len,j)
END DO

RETURN
END SUBROUTINE tsmfdv



SUBROUTINE tsmgsa(s, n, sqrn, itn, shat, p, idp)

REAL (dp), INTENT(IN)   :: s(:,:)
INTEGER, INTENT(IN)     :: n
INTEGER, INTENT(IN)     :: sqrn
INTEGER, INTENT(IN)     :: itn
REAL (dp), INTENT(OUT)  :: shat(:,:)
INTEGER, INTENT(OUT)    :: p
INTEGER, INTENT(OUT)    :: idp(:)

!*********************************************************************
! THIS ROUTINE FINDS A SET OF LINEARLY INDEPENDENT DIRECTIONS USING
! THE MODIFIED GRAM-SCHMIDT ALGORITHM.
!*********************************************************************

!    INPUT PARAMETERS :
!    ---------------

!    S   : MATRIX OF PAST DIRECTIONS
!    N   : ROW DIMENSION OF MATRIX S AND SHAT
!    SQRN: MAXIMUM COLUMN DIMENSION OF SHAT
!    ITN : CURRENT ITERATION NUMBER

!    OUTPUT PARAMETERS :
!    -------------------

!    SHAT: MATRIX OF LINEARLY INDEPENDENT DIRECTIONS
!    P   : COLUMN DIMENSION OF THE MATRIX SHAT
!    IDP : VECTOR THAT KEEPS TRACK OF THE INDICES CORRESPONDING TO
!          THE LINEARLY INDEPENDENT DIRECTIONS IN THE MATRIX S

!    SUBPROGRAMS CALLED:

!    LEVEL 1 BLAS  ...  DNRM2

!*********************************************************************

INTEGER   :: j, k, l
REAL (dp) :: tol, tj, sj, sum, rtjs
REAL (dp), PARAMETER :: two = 2.0_dp

IF(sqrn < itn) THEN
  k = sqrn
ELSE
  k = itn-1
END IF

tol = SQRT(two)/two

shat(1:n,1:k) = s(1:n,1:k)

p = 0
DO j = 1,k
  tj = dnrm2(n, shat(:,j), 1)
  sj = dnrm2(n, s(:,j), 1)
  IF(tj/sj > tol) THEN
    p = p + 1
    idp(p) = j
    rtjs = one/tj**2
    DO l = j+1,k
      sum = -rtjs * DOT_PRODUCT( shat(1:n,l), shat(1:n,j) )
      shat(1:n,l) = shat(1:n,l) + sum * shat(1:n,j)
    END DO
  END IF
END DO

DO j = 1,p
  shat(1:n,j) = s(1:n,idp(j))
END DO

RETURN
END SUBROUTINE tsmgsa



SUBROUTINE tsmsda(anls, fq, adt, ag, const1, const2, alpha, dlt, m, n, p, &
                  nwtake, ierr, skip, vn, fn_val)

REAL (dp), INTENT(IN)   :: anls(:,:)
REAL (dp), INTENT(IN)   :: fq(:)
REAL (dp), INTENT(IN)   :: adt(:)
REAL (dp), INTENT(IN)   :: ag(:)
REAL (dp), INTENT(IN)   :: const1(:)
REAL (dp), INTENT(IN)   :: const2(:)
REAL (dp), INTENT(IN)   :: alpha
REAL (dp), INTENT(IN)   :: dlt
INTEGER, INTENT(IN)     :: m
INTEGER, INTENT(IN)     :: n
INTEGER, INTENT(IN)     :: p
LOGICAL, INTENT(IN)     :: nwtake
INTEGER, INTENT(IN)     :: ierr
LOGICAL, INTENT(IN)     :: skip
REAL (dp), INTENT(OUT)  :: vn(:)
REAL (dp), INTENT(OUT)  :: fn_val

!***********************************************************************
! THIS FUNCTION COMPUTES THE SECOND DERIVATIVE OF || F + J*D +
! 0.5*A*D**2 ||**2 IN THE L2 NORM SENS, WHERE D = ALPHA*DT +
! SQRT(DLT**2-ALPHA**2).

! N.B. Changed from a function to a subroutine by AJM
!***********************************************************************

!    INPUT PARAMETERS
!    ----------------

!    ANLS   : TENSOR TERM MATRIX AT CURRENT ITERATE
!    FQ     : FUNCTION VALUE AT CURRENT ITERATE MULTIPLIED BY
!             ORTHOGONAL MATRICES
!    ADT    : JACOBIAN MATRIX TIMES DT (SEE SUBROUTINE TS2DTR)
!     AG    : JACOBIAN MATRIX TIMES GBAR (SEE SUBROUTINE TS2DTR)
!    CONST1 : SHAT-TRANS TIMES DT (SEE SUBROUTINE TS2DTR)
!    CONST2 : SHAT-TRANS TIMES GBAR (SEE SUBROUTINE TS2DTR)
!    ALPHA  : POINT AT WHICH TO EVALUATE THE SECOND DERIVATIVE OF FUNCTION
!    DLT    : CURRENT TRUST RADIUS
!    M,N    : DIMENSIONS OF THE PROBLEM
!    P      : COLUMN DIMENSION OF THE MATRICES SHAT AND ANLS
!    NWTAKE : LOGICAL VARIABLE WITH THE FOLLOWING MEANINGS:
!             NWTAKE = .TRUE.  : STANDARD STEP TAKEN
!             NWTAKE = .FALSE. : TENSOR STEP TAKEN
!    IERR   : RETURN CODE FROM QRP FACTORIZATION ROUTINE
!             IERR = 0 : NO SINGULARITY OF JACOBIAN DETECTED
!             IERR = 1 : SINGULARITY OF JACOBIAN DETECTED

!    OUTPUT PARAMETERS
!    -----------------

!    VN     : F + J*D + 0.5*A*D**2
!    TSMSDA : SECOND DERIVATIVE IN ALPHA OF || F + J*D + 0.5*A*D**2 ||**2
!             WHERE D=ALPHA*DT + SQRT(DLT**2-ALPHA**2)

!    SUBPROGRAMS CALLED:

!    TENSOLVE      ...  TSMAFA,TSMFDV,TSMSDV

!***********************************************************************

! Workspace
!       VNP    : DERIVATIVE IN ALPHA OF F + J*D + 0.5*A*D**2
!       VNS    : SECOND DERIVATIVE IN ALPHA OF F + J*D + 0.5*A*D**2
REAL (dp) :: vnp(n+m), vns(n+m)

INTEGER   :: len

IF(.NOT. skip) THEN
  CALL tsmafa(anls, fq, adt, ag, const1, const2, alpha, dlt, m, n, p,  &
              nwtake, ierr, vn)
  CALL tsmfdv(anls, adt, ag, const1, const2, alpha, dlt, m, n, p,   &
              nwtake, ierr, vnp)
END IF

CALL tsmsdv(anls, ag, const1, const2, alpha, dlt, m, n, p, nwtake, ierr, vns)

len = m
IF(ierr > 0) len = m + n

fn_val = SUM( vnp(1:len)**2 ) + DOT_PRODUCT( vns(1:m), vn(1:m) )

RETURN
END SUBROUTINE tsmsda



SUBROUTINE tsmsdv(anls, ag, const1, const2, alpha, dlt, m, n, p, nwtake,  &
                  ierr, vns)

REAL (dp), INTENT(IN)   :: anls(:,:)
REAL (dp), INTENT(IN)   :: ag(:)
REAL (dp), INTENT(IN)   :: const1(:)
REAL (dp), INTENT(IN)   :: const2(:)
REAL (dp), INTENT(IN)   :: alpha
REAL (dp), INTENT(IN)   :: dlt
INTEGER, INTENT(IN)     :: m
INTEGER, INTENT(IN)     :: n
INTEGER, INTENT(IN)     :: p
LOGICAL, INTENT(IN)     :: nwtake
INTEGER, INTENT(IN)     :: ierr
REAL (dp), INTENT(OUT)  :: vns(:)

!***********************************************************************
! THIS ROUTINE COMPUTES THE SECOND DERIVATIVE IN ALPHA OF THE VECTOR
! VN = F(XC) + J(XC)*D + 0.5*A*D**2, WHERE D = ALPHA*DT +
! SQRT(DLT**2-ALPHA**2).
!***********************************************************************

!    INPUT PARAMETERS :
!    -----------------

!    ANLS  : TENSOR TERM MATRIX
!     ADT  : JACOBIAN MATRIX TIMES DT (SEE SUBROUTINE TS2DTR)
!      AG  : JACOBIAN MATRIX TIMES GBAR (SEE SUBROUTINE TS2DTR)
!    CONST1: SHAT-TRANS * DT (SEE SUBROUTINE TS2DTR)
!    CONST2: SHAT-TRABS * GBAR (SEE SUBROUTINE TS2DTR)
!    ALPHA : POINT AT WHICH DERIVATIVE IS EVALUATED
!      DLT : CURRENT TRUST RADIUS
!      NR  : LEADING DIMENSION OF ANLS
!      M,N : DIMENSIONS OF THE PROBLEM
!      P   : COLUMN DIMENSION OF THE MATRIX ANLS
!   NWTAKE : LOGICAL VARIABLE WITH THE FOLLOWING MEANINGS :
!               NWTAKE = .TRUE.  : STANDARD STEP TAKEN
!               NWTAKE = .FALSE. : TENSOR STEP TAKEN
!     IERR : RETURN CODE FROM THE QRP FACTORIZATION ROUTINE :
!             IERR = 0 : NO SINGULARITY OF JACOBIAN DETECTED
!             IERR = 1 : SINGULARITY OF JACOBIAN DETECTED

!     OUTPUT PARAMETERS :
!     -------------------

!     VNS  : THE SECOND DERIVATIVE IN ALPHA OF VN = F(XC) + J(XC)*D
!            + 0.5*A*D**2, WHERE D = ALPHA*DT +  SQRT(DLT**2-ALPHA**2)

!*******************************************************************

INTEGER   :: j, len
REAL (dp) :: quant1, expr, const, quant2
REAL (dp), PARAMETER :: half = 0.5_dp, onepf = 1.5_dp, two = 2.0_dp,  &
                        three = 3.0_dp

quant1 = dlt**2 - alpha**2
expr = -dlt**2 * SQRT(quant1) / quant1**2
vns(1:n) =  expr*ag(1:n)

vns(n+1:n+m) = zero

IF(nwtake) RETURN

quant2 = -three*alpha/SQRT(quant1) - alpha**3/quant1**onepf

len = m
IF(ierr > 0) len = m + n

DO j = 1,p
  const = half*(two*(const1(j)**2 - const2(j)**2)  &
          + two*const1(j)*const2(j)*quant2)
  vns(1:len) = vns(1:len) + const * anls(1:len,j)
END DO

RETURN
END SUBROUTINE tsmsdv



SUBROUTINE tsmslv(p, shat, maxm, sqrn, m, n, epsm, method, &
                  global, x, typxu, xpls, gpls, curpos, pbar, pivot, fq, fqq, &
                  dn, dt, restns, resnew, itrmcd, flag, zero1, ierr, &
                  qrank, meqns, nvars, fc, anls, aja, s)

INTEGER, INTENT(IN)        :: p
REAL (dp), INTENT(IN OUT)  :: shat(:,:)
INTEGER, INTENT(IN)        :: maxm
INTEGER, INTENT(IN)        :: sqrn
INTEGER, INTENT(IN)        :: m
INTEGER, INTENT(IN)        :: n
REAL (dp), INTENT(IN)      :: epsm
INTEGER, INTENT(IN)        :: method
INTEGER, INTENT(IN)        :: global
REAL (dp), INTENT(OUT)     :: x(:)
REAL (dp), INTENT(IN OUT)  :: typxu(:)
REAL (dp), INTENT(OUT)     :: xpls(:)
REAL (dp), INTENT(IN OUT)  :: gpls(:)
INTEGER, INTENT(OUT)       :: curpos(:)
INTEGER, INTENT(OUT)       :: pbar(:)
INTEGER, INTENT(OUT)       :: pivot(:)
REAL (dp), INTENT(OUT)     :: fq(:)
REAL (dp), INTENT(OUT)     :: fqq(:)
REAL (dp), INTENT(OUT)     :: dn(:)
REAL (dp), INTENT(OUT)     :: dt(:)
REAL (dp), INTENT(OUT)     :: restns
REAL (dp), INTENT(OUT)     :: resnew
INTEGER, INTENT(OUT)       :: itrmcd
INTEGER, INTENT(OUT)       :: flag
INTEGER, INTENT(OUT)       :: zero1
INTEGER, INTENT(OUT)       :: ierr
INTEGER, INTENT(IN)        :: meqns, nvars
INTEGER, INTENT(IN OUT)    :: qrank
REAL (dp), INTENT(IN OUT)  :: fc(:), anls(:,:), aja(:,:), s(:,:)

!**********************************************************************
! THIS ROUTINE FINDS THE TENSOR AND STANDARD STEPS.
!**********************************************************************

!    INPUT PARAMETERS :
!    ---------------

!    P      : COLUMN DIMENSION OF MATRICES ANLS AND S
!    MAXM   : LEADING DIMENSION OF AJA AND ANLS
!    SQRN   : LEADING DIMENSION OF MATRICES A AND WRK
!    M,N    : DIMENSIONS OF PROBLEM
!    EPSM   : MACHINE PRECISION
!    X      : ESTIMATE TO A ROOT OF FCN (USED BY UNCMIN)
!    TYPXU  : TYPICAL SIZE FOR EACH COMPONENT OF X (USED BY UNCMIN)
!    METHOD : METHOD TO USE
!             METHOD = 0 : STANDARD METHOD IS USED
!             METHOD = 1 : TENSOR METHOD IS USED
!    GLOBAL : GLOBAL STRATEGY USED

!    INPUT/OUTPUT PARAMETERS :
!    ------------------------
!    SHAT   : MATRIX OF PAST LINEARLY INDEPENDENT DIRECTIONS

!    OUTPUT PARAMETERS :
!    ------------------

!    DN     : STANDARD STEP
!    DT     : TENSOR STEP
!    FLAG   : RETURNED CODE WITH THE FOLLOWING MEANING :
!             FLAG = 0 : NO SINGULARITY DETECTED WHEN FACTORIZING AJA
!             FLAG = 1 : SINGULARITY DETECTED WHEN FACTORIZING AJA
!                        FROM 1 TO N-P
!             FLAG = 2 : SINGULARITY DETECTED WHEN FACTORIZING AJA
!                        FROM N-P TO N
!    IERR   : RETURNED CODE WITH THE FOLLOWING MEANING :
!               IERR = 0 : NO SINGULARITY DETECTED WHEN FACTORIZING AJA
!               IERR = 1 : SINGULARITY DETECTED WHEN FACTORIZING AJA
!    XPLS   : LOCAL MINIMUM OF OPTIMIZATION FUNCTION FCN (USED BY UNCMIN)
!    FPLS   : FUNCTION VALUE AT SOLUTION OF OPTIMIZATION FUNCTION FCN
!             (USED IN UNCMIN)
!    GPLS   : GRADIENT AT SOLUTION XPLS (USED BY UNCMIN)
!    CURPOS,PIVOT,PBAR : PIVOT VECTORS
!    RESTNS : TENSOR RESIDUAL
!    RESNEW : STANDARD RESIDUAL
!    ITRMCD : TERMINATION CODE (FOR UNCMIN)

!    SUBPROGRAMS CALLED:

!    TENSOLVE      ...  TSQLFC,QTRNS,TSQRFC,TSQMTS,TSQMUV,TSSQP1
!    TENSOLVE      ...  TSQ1P1,TSD1SV,TSPRMV,TSQMLV,TSCPSS
!    UNCMIN        ...  DFAUT,OPTIF9

!**********************************************************************

! Workspace
REAL (dp) :: wrk3(m), wrk4(m)

INTEGER   :: msg, itnlim, ipr
INTEGER   :: q, meth, iexp, ndigit, iagflg, iahflg
REAL (dp) :: root, typfu, dlt, gradlt, stepmx, steptl, fpls
REAL (dp), PARAMETER :: two = 2.0_dp

itrmcd = 0
IF(n == 1) THEN
  shat(2,1) = one
  shat(3,1) = one
  curpos(1) = 1
  fq = fc(1:m)
ELSE

! perform a QL decomposition of S

  CALL tsqlfc(shat, n, p, ierr)

! compute AJA times Q-trans

  CALL tsjqtp(shat, n, m, p, aja)

! perform a QR factorization of AJA

  CALL tsqrfc(aja, n, m, 1, n-p, ierr, epsm, curpos, zero1)

  IF(ierr == 1) THEN
    q = n - zero1 + 1
  ELSE
    q = p
  END IF
  qrank = q

  CALL tsqmts(anls, aja, m, m, p, 1, zero1)

  CALL tsqmuv(aja, fc, fq, m, 1, zero1, .false.)
END IF

! Minimize the lower m-n+q quadratic equations in p unknowns of the tensor
! model.  The minimization is performed analytically if p=1,q>1, or
! p=1,q=1,m>n, or n=1,m>n.  Otherwise an unconstrained minimization package,
! UNCMIN, is used.

IF((p == 1 .AND. q > 1) .OR. (p == 1 .AND. q == 1 .AND. m > n)  &
      .OR. (n == 1 .AND. m > n)) THEN
  CALL tssqp1(aja, anls, shat, fq, m, n, q, root, restns)
  xpls(1) = root
ELSE IF(m == n .AND. p == 1 .AND. q == 1 .OR. (m == 1 .AND. n == 1)) THEN
  CALL tsq1p1(aja, anls, shat, fq, n, root, restns)
  xpls(1) = root
ELSE
  CALL dfaut(p, typxu, typfu, meth, iexp, msg, ndigit, itnlim,  &
             iagflg, iahflg, ipr, dlt, gradlt, stepmx, steptl)

  iagflg = 1
  iahflg = 0
  iexp = 0
  meth = 2
  msg = 9

  x(1:p) = zero

  CALL optif9(sqrn, p, x, tsqfcn, tsdfcn, d2fcn, typxu, typfu, meth, iexp,  &
              msg, ndigit, itnlim, iagflg, iahflg, dlt, gradlt,  &
              stepmx, steptl, xpls, fpls, gpls, itrmcd, &
              qrank, meqns, nvars, fc, anls, aja, s)

! compute the tensor residual

  restns = SQRT(two*fpls)
END IF

wrk4(n-p+1:n) = xpls(1:p)

IF(n == 1) THEN
  dt(1) = wrk4(1)
ELSE

! compute the first n-p components of the tensor step

  CALL tsd1sv(aja, shat, anls, fq, xpls, maxm, m, n, p, q, epsm, pivot, wrk3)
  CALL tsprmv(wrk4, wrk3, curpos, n-p, 0)

! premultiply the tensor step by the orthogonal matrix resulting
! from the QL factorization of S

  CALL tsqmlv(n, p, shat, wrk4, dt, .true.)
END IF

! compute the standard step if needed

IF(global == 1 .OR. (m > n .AND. global == 0)) THEN
  CALL tscpss(shat, m, n, p, method, global, epsm, fq, aja, anls, &
              dn, fqq, pivot, curpos, pbar, zero1, ierr, resnew, flag)
END IF

RETURN
END SUBROUTINE tsmslv



SUBROUTINE tensolve_tsneci(maxm, maxn, maxp, x0, m, n, typx, typf, itnlim, jacflg, &
                  gradtl, steptl, ftol, method, global, stepmx, dlt, ipr, &
                  fvec, jac, msg, xp, fp, gp, termcd) bind(c)

INTEGER, INTENT(IN)        :: maxm
INTEGER, INTENT(IN)        :: maxn
INTEGER, INTENT(IN)        :: maxp
REAL (dp), INTENT(IN OUT)  :: x0(n)
INTEGER, INTENT(IN)        :: m
INTEGER, INTENT(IN)        :: n
REAL (dp), INTENT(IN OUT)  :: typx(n)
REAL (dp), INTENT(IN OUT)  :: typf(m)
INTEGER, INTENT(IN OUT)    :: itnlim
INTEGER, INTENT(IN OUT)    :: jacflg
REAL (dp), INTENT(IN OUT)  :: gradtl
REAL (dp), INTENT(IN OUT)  :: steptl
REAL (dp), INTENT(IN OUT)  :: ftol
INTEGER, INTENT(IN OUT)    :: method
INTEGER, INTENT(IN OUT)    :: global
REAL (dp), INTENT(IN OUT)  :: stepmx
REAL (dp), INTENT(IN OUT)  :: dlt
INTEGER, INTENT(IN OUT)    :: ipr
INTEGER, INTENT(IN OUT)    :: msg
REAL (dp), INTENT(OUT)     :: xp(n)
REAL (dp), INTENT(OUT)     :: fp(m)
REAL (dp), INTENT(OUT)     :: gp(n)
INTEGER, INTENT(OUT)       :: termcd

INTERFACE
  SUBROUTINE fvec(x, f, m, n) bind(c)
    IMPLICIT NONE
    INTEGER, PARAMETER     :: dp = SELECTED_REAL_KIND(14, 60)
    REAL (dp), INTENT(IN)  :: x(n)
    REAL (dp), INTENT(OUT) :: f(m)
    INTEGER, INTENT(IN)    :: m, n
  END SUBROUTINE fvec

  SUBROUTINE jac(x, aja, m, n) bind(c)
    IMPLICIT NONE
    INTEGER, PARAMETER     :: dp = SELECTED_REAL_KIND(14, 60)
    REAL (dp), INTENT(IN)  :: x(n)
    REAL (dp), INTENT(OUT) :: aja(m,n)
    INTEGER, INTENT(IN)    :: m, n
  END SUBROUTINE jac
END INTERFACE

!**********************************************************************
! THIS ROUTINE PROVIDES A COMPLETE INTERFACE TO THE NONLINEAR EQUATION/
! NONLINEAR LEAST SQUARES PACKAGE.  THE USER HAS FULL CONTROL OVER
! THE OPTIONS.
!**********************************************************************

!    SUBPROGRAMS CALLED:

!    TENSOLVE      ...  TSCHKI,TSNESV

!**********************************************************************

INTEGER   :: sqrn
REAL (dp) :: epsm, dfn(m), dxn(n)

! check input parameters

CALL tschki(maxm, maxn, maxp, m, n, gradtl, steptl, ftol, itnlim, jacflg, &
            method, global, stepmx, dlt, epsm, msg, typx, typf, dxn, dfn, &
            sqrn, termcd, ipr)
IF(msg < 0) RETURN

! call nonlinear equations/nonlinear least squares solver

CALL tsnesv(maxm, x0, m, n, typx, typf, itnlim, jacflg,  &
            gradtl, steptl, ftol, method, global, stepmx, dlt, ipr,  &
            dfn, dxn, epsm, sqrn, fvec, jac, msg, xp, fp, gp, termcd)

RETURN
END SUBROUTINE tensolve_tsneci



SUBROUTINE tensolve_tsnesi(maxm, maxn, maxp, x0, m, n, fvec, msg, xp, fp, gp, termcd) bind(c)

INTEGER, INTENT(IN)        :: maxm
INTEGER, INTENT(IN)        :: maxn
INTEGER, INTENT(IN)        :: maxp
REAL (dp), INTENT(IN OUT)  :: x0(n)
INTEGER, INTENT(IN)        :: m
INTEGER, INTENT(IN)        :: n
INTEGER, INTENT(IN OUT)    :: msg
REAL (dp), INTENT(IN OUT)  :: xp(n)
REAL (dp), INTENT(IN OUT)  :: fp(m)
REAL (dp), INTENT(IN OUT)  :: gp(n)
INTEGER, INTENT(OUT)       :: termcd

INTERFACE
  SUBROUTINE fvec(x, f, m, n) bind(c)
    IMPLICIT NONE
    INTEGER, PARAMETER     :: dp = SELECTED_REAL_KIND(14, 60)
    REAL (dp), INTENT(IN)  :: x(n)
    REAL (dp), INTENT(OUT) :: f(m)
    INTEGER, INTENT(IN)    :: m, n
  END SUBROUTINE fvec

  ! modified by nicola: deleted and made tsnesi call tsdumj
  ! SUBROUTINE jac(x, aja, m, n)
  !   IMPLICIT NONE
  !   INTEGER, PARAMETER     :: dp = SELECTED_REAL_KIND(14, 60)
  !   REAL (dp), INTENT(IN)  :: x(n)
  !   REAL (dp), INTENT(OUT) :: aja(m,n)
  !   INTEGER, INTENT(IN)    :: m, n
  ! END SUBROUTINE jac
END INTERFACE

!**********************************************************************
! THIS ROUTINE PROVIDES A SIMPLE INTERFACE TO THE NONLINEAR EQUATION/
! NONLINEAR LEAST SQUARES PROBLEMS PACKAGE.  THE USER HAS NO CONTROL
! OVER THE PACKAGE OPTIONS.
!**********************************************************************

!    SUBPROGRAMS CALLED:

!    TENSOLVE      ...  TSDFLT,TSCHKI,TSNESV

!**********************************************************************

REAL (dp) :: typx(n), typf(m), dfn(m), dxn(n)
INTEGER   :: jacflg, itnlim, method
INTEGER   :: global, ipr
REAL (dp) :: steptl, gradtl, ftol, stepmx, dlt
INTEGER   :: sqrn
REAL (dp) :: epsm

! set default values for each variable to the nonlinear equations/
! nonlinear least squares solver

CALL tensolve_tsdflt(m, n, itnlim, jacflg, gradtl, steptl, ftol, method, global,  &
            stepmx, dlt, typx, typf, ipr)

! check input parameters

CALL tschki(maxm, maxn, maxp, m, n, gradtl, steptl, ftol, itnlim, jacflg,  &
            method, global, stepmx, dlt, epsm, msg, typx, typf, dxn, dfn,  &
            sqrn, termcd, ipr)
IF(msg < 0) RETURN

! call nonlinear equations/nonlinear least squares solver
! modified by nicola: made tsnesi call tensolve_tsdumj instead of jac
CALL tsnesv(maxm, x0, m, n, typx, typf, itnlim, jacflg,  &
            gradtl, steptl, ftol, method, global, stepmx, dlt, ipr,  &
            dfn, dxn, epsm, sqrn, fvec, tensolve_tsdumj, msg, xp, fp, gp, termcd)

RETURN
END SUBROUTINE tensolve_tsnesi



SUBROUTINE tsnesv(maxm, xc, m, n, typx, typf, itnlim, jacflg,  &
                  gradtl, steptl, ftol, method, global, stepmx, dlt, ipr,  &
                  dfn, dxn, epsm, sqrn, fvec, jac, msg, xp, fp, gp, termcd)

INTEGER, INTENT(IN)        :: maxm
REAL (dp), INTENT(IN OUT)  :: xc(n)
INTEGER, INTENT(IN)        :: m
INTEGER, INTENT(IN)        :: n
REAL (dp), INTENT(IN)      :: typx(n)
REAL (dp), INTENT(IN)      :: typf(m)
INTEGER, INTENT(IN)        :: itnlim
INTEGER, INTENT(IN)        :: jacflg
REAL (dp), INTENT(IN)      :: gradtl
REAL (dp), INTENT(IN OUT)  :: steptl
REAL (dp), INTENT(IN)      :: ftol
INTEGER, INTENT(IN)        :: method
INTEGER, INTENT(IN)        :: global
REAL (dp), INTENT(IN OUT)  :: stepmx
REAL (dp), INTENT(IN OUT)  :: dlt
INTEGER, INTENT(IN)        :: ipr
REAL (dp), INTENT(IN)      :: dfn(m)
REAL (dp), INTENT(IN)      :: dxn(n)
REAL (dp), INTENT(IN)      :: epsm
INTEGER, INTENT(IN)        :: sqrn
INTEGER, INTENT(IN OUT)    :: msg
REAL (dp), INTENT(OUT)     :: xp(n)
REAL (dp), INTENT(OUT)     :: fp(m)
REAL (dp), INTENT(OUT)     :: gp(n)
INTEGER, INTENT(OUT)       :: termcd


INTERFACE
  SUBROUTINE fvec(x, f, m, n) bind(c)
    IMPLICIT NONE
    INTEGER, PARAMETER     :: dp = SELECTED_REAL_KIND(14, 60)
    REAL (dp), INTENT(IN)  :: x(n)
    REAL (dp), INTENT(OUT) :: f(m)
    INTEGER, INTENT(IN)    :: m, n
  END SUBROUTINE fvec

  SUBROUTINE jac(x, aja, m, n) bind(c)
    IMPLICIT NONE
    INTEGER, PARAMETER     :: dp = SELECTED_REAL_KIND(14, 60)
    REAL (dp), INTENT(IN)  :: x(n)
    REAL (dp), INTENT(OUT) :: aja(m,n)
    INTEGER, INTENT(IN)    :: m, n
  END SUBROUTINE jac
END INTERFACE

!*****************************************************************************
! THIS IS THE DRIVER FOR NONLINEAR EQUATIONS/NONLINEAR LEAST SQUARES PROBLEMS.
!*****************************************************************************

!      INPUT PARAMETERS :
!      -----------------

!      MAXM   : LEADING DIMENSION OF AJA, ANLS, AND FV
!      XC     : INITIAL ESTIMATE OF SOLUTION
!      M,N    : DIMENSIONS OF PROBLEM
!      TYPX   : TYPICAL SIZE FOR EACH COMPONENT OF X
!      TYPF   : TYPICAL SIZE FOR EACH COMPONENT OF F
!      ITNLIM : MAXIMUM NUMBER OF ALLOWABLE ITERATIONS
!      JACFLG : JACOBIAN FLAG WITH THE FOLLOWING MEANINGS:
!               JACFLG = 1 IF ANALYTIC JACOBIAN SUPPLIED
!               JACFLG = 0 IF ANALYTIC JACOBIAN NOT SUPPLIED
!      GRADTL : TOLERANCE AT WHICH GRADIENT IS CONSIDERED CLOSE ENOUGH
!               TO ZERO TO TERMINATE ALGORITHM
!      STEPTL : TOLERANCE AT WHICH SUCCESSIVE ITERATES ARE CONSIDERED
!               CLOSE ENOUGH TO TERMINATE ALGORITHM
!      FTOL   : TOLERANCE AT WHICH FUNCTION VALUE IS CONSIDERED CLOSE
!               ENOUGH TO ZERO
!      METHOD : METHOD TO USE
!               METHOD = 0 : STANDARD METHOD IS USED
!               METHOD = 1 : TENSOR METHOD IS USED
!      GLOBAL : GLOBAL STRATEGY TO USE
!               GLOBAL = 0 : LINE SEARCH
!               GLOBAL = 1 : 2-DIMENSIONAL TRUST REGION
!      STEPMX : MAXIMUM ALLOWABLE STEP SIZE
!      DLT    : TRUST REGION RADIUS
!      IPR    : DEVICE TO WHICH TO SEND OUTPUT
!      DFN    : DIAGONAL SCALING MATRIX FOR F
!      DXN    : DIAGONAL SCALING MATRIX FOR X
!      EPSM   : MACHINE PRECISION
!      SQRN   : MAXIMUM COLUMN DIMENSION OF ANLS, S, AND SHAT
!      FVEC   : NAME OF SUBROUTINE TO EVALUATE FUNCTION
!      JAC    : (OPTIONAL) NAME OF SUBROUTINE TO EVALUATE JACOBIAN.
!               MUST BE DECLARED EXTERNAL IN CALLING ROUTINE

!      INPUT-OUTPUT PARAMETERS :
!      ------------------------

!      MSG : MESSAGE TO INHIBIT CERTAIN AUTOMATIC CHECKS + OUTPUT

!      OUTPUT PARAMETERS :
!      -----------------

!      XP : SOLUTION TO THE SYSTEM OF NONLINEAR EQUATIONS
!      FP : FUNCTION VALUE AT THE SOLUTION
!      GP : GRADIENT AT THE SOLUTION
!  TERMCD : TERMINATION CODE

!      SUBPROGRAMS CALLED:

!      LEVEL 1 BLAS  ...  DNRM2
!      LEVEL 2 BLAS  ...  DGEMV
!      TENSOLVE      ...  TSSCLX,TSFSCL,TSSCLJ,TSCHKJ,TSNSTP,TSSSTP,
!      TENSOLVE      ...  TSLSCH,TS2DTR,TSRSLT,TSMGSA,TSFRMT,TSMSLV,
!      TENSOLVE      ...  TSSLCT,TSMDLS,TSUPSF

!*********************************************************************

! Local variables (were dummy arguments in F77 version)

!      X      : ESTIMATE TO A ROOT OF FCN ( USED BY UNCMIN)
!      TYPXU  : TYPICAL SIZE FOR EACH COMPONENT OF X (USED BY UNCMIN)
!      XPLS   : LOCAL MINIMUM OF OPTIMIZATION FUNCTION FCN USED BY UNCMIN
!      GPLS   : GRADIENT AT SOLUTION XPLS (USED BY UNCMIN)
!      DN     : STANDARD STEP
!      DT     : TENSOR STEP
!      SHAT   : MATRIX OF PAST LINEARLY INDEPENDENT DIRECTIONS
!      CURPOS,PIVOT,PBAR : PIVOT VECTORS

! Workspace
REAL (dp) :: x(n), typxu(n), xpls(n), gpls(n), dn(n), dt(n), shat(m+2,sqrn)
INTEGER   :: curpos(n), pivot(n), pbar(n)
REAL (dp) :: df(n), gbar(n), fq(m), fqq(m+n), fhat(m), fv(m,n)

INTEGER              :: p, itn, flag, retcd, zero1, ierr, itrmcd, icscmx
REAL (dp)            :: fpls, fnorm, restns, resnew
REAL (dp), PARAMETER :: half = 0.5_dp
LOGICAL              :: nwtake, mxtake

! modified by nicola
REAL (dp), ALLOCATABLE  :: fc(:), anls(:,:), aja(:,:), s(:,:)
! REAL (dp)                  :: fc(m)
! REAL (dp)                  :: anls(m+2,sqrn)
! REAL (dp)                  :: aja(m+n+2,n)
! REAL (dp)                  :: s(m+2,sqrn)
INTEGER                    :: qrank
INTEGER                    :: meqns
INTEGER                    :: nvars

!-----------------
! initialization
!-----------------

itn = 0
ierr = 0
nwtake = .true.

ALLOCATE( fc(m), anls(m+2,sqrn), aja(m+n+2,n), s(m+2,sqrn) ) ! modified by nicola
fc(:)     = 0
anls(:,:) = 0
aja(:,:)  = 0
s(:,:)    = 0

meqns = m
nvars = n

CALL tssclx(xc, dxn, n)

IF(MOD(msg/8,2) /= 1) THEN
  WRITE(ipr,896)
  WRITE(ipr,900) typx(1:n)
  WRITE(ipr,897)
  WRITE(ipr,900) dxn(1:n)
  WRITE(ipr,898)
  WRITE(ipr,900) typf(1:m)
  WRITE(ipr,899)
  WRITE(ipr,900) dfn(1:m)
  WRITE(ipr,901) jacflg
  WRITE(ipr,902) method
  WRITE(ipr,903) global
  WRITE(ipr,904) itnlim
  WRITE(ipr,905) epsm
  WRITE(ipr,906) steptl
  WRITE(ipr,907) gradtl
  WRITE(ipr,908) ftol
  WRITE(ipr,909) stepmx
  WRITE(ipr,910) dlt
END IF

! Evaluate analytic or finite difference Jacobian and check analytic
! Jacobian, if requested

CALL tsfscl(xc, dxn, dfn, m, n, fvec, fc)
CALL tssclj(xc, dxn, typx, fc, dfn, m, n, epsm, jacflg, fvec, jac, aja)
IF(jacflg == 1) THEN
  IF(MOD(msg/2,2) == 0) THEN
    CALL tschkj(aja, xc, fc, m, n, epsm, dfn, dxn, typx, ipr, fvec, msg)
    IF(msg < 0) RETURN
  END IF
END IF

! compute the gradient at the current iterate XC

CALL dgemv('T', m, n, one, aja, maxm, fc, 1, zero, gp, 1)

! compute the residual of FC

fnorm = half*dnrm2(m, fc, 1)**2

! check stopping criteria for input XC

CALL tsnstp(gp, xc, fc, xc, steptl, gradtl, retcd, ftol, itn,  &
            itnlim, icscmx, mxtake, m, n, msg, ipr, fnorm, termcd)

IF(termcd > 0) THEN
  xp(1:n) = xc(1:n) ! modified by nicola: copy the guess to the solution vector
  call fvec(xp, fp, m, n) ! modified by nicola: copy the function value to the solution vector
  call jac(xp, aja, m, n) ! modified by nicola: copy the function gradient value to the solution vector
  fpls = fnorm
  GO TO 120
END IF

!---------------
! iteration 1
!---------------

itn = 1

! compute the standard step

fhat(1:m) = fc(1:m)

CALL tssstp(aja, fhat, m, n, epsm, global, dn, fqq, pivot, pbar, ierr)

! choose next iterate XP by a global strategy

IF(global == 0) THEN
  CALL tslsch(m, n, xc, dn, gp, steptl, dxn, dfn, fvec,  &
              mxtake, stepmx, xp, fp, fnorm, fpls, retcd)
ELSE
  shat(1:n,1:sqrn) = zero
  CALL ts2dtr(aja, shat, anls, dn, gp, gbar, xc, method, nwtake, stepmx,   &
              steptl, epsm, mxtake, dlt, fqq, maxm, m, n, sqrn,      &
              curpos, pivot, pbar, itn, ierr, flag, dxn, dfn, fvec, fnorm, &
              xp, fp, fpls, retcd)
END IF

IF(MOD(msg/8,2) == 0) CALL tsrslt(n, xc, fnorm, gp, 0, ipr)

! evaluate the Jacobian at the new iterate XP

CALL tssclj(xp, dxn, typx, fp, dfn, m, n, epsm, jacflg, fvec, jac, aja)

! compute the gradient at the new iterate XP

CALL dgemv('T', m, n, one, aja, maxm, fp, 1, zero, gp, 1)

! check stopping criteria for the new iterate XP

CALL tsnstp(gp, xp, fp, xc, steptl, gradtl, retcd, ftol, itn,  &
            itnlim, icscmx, mxtake, m, n, msg, ipr, fpls, termcd)

IF(termcd > 0) GO TO 120
IF(MOD(msg/16,2) == 1) CALL tsrslt(n, xp, fpls, gp, itn, ipr)

! update S and FV

s(1:n,1) = xc(1:n) - xp(1:n)
fv(1:m,1) = fc(1:m)

! update XC and FC

xc(1:n) = xp(1:n)
fc(1:m) = fp(1:m)
fnorm = fpls

!---------------
! iteration > 1
!---------------

80 itn = itn + 1

! if the standard method is selected then compute the standard step

IF(method == 0) THEN
  fhat(1:m) = fc(1:m)
  CALL tssstp(aja, fhat, m, n, epsm, global, df, fqq, pivot, pbar, ierr)
END IF

! if the tensor method is selected then form the tensor model

IF(method == 1) THEN

! select the past linearly independent directions

  CALL tsmgsa(s, n, sqrn, itn, shat, p, curpos)

! form the tensor term

  CALL tsfrmt(shat, s, aja, fv, fc, m, n, p, curpos, anls)

! solve the tensor model for the tensor step DT and compute DN
! as a by-product if the global strategy selected is the
! two-dimensional trust region or M > N

  CALL tsmslv(p, shat, maxm, sqrn, m, n, epsm, method, &
              global, x, typxu, xpls, gpls, curpos, pbar, pivot,  &
              fq, fqq, dn, dt, restns, resnew, itrmcd, flag, zero1, ierr, &
              qrank, meqns, nvars, fc, anls, aja, s) ! modified by nicola

! decide which step to use (DN or DT)

  IF(global == 1 .OR. (m > n .AND. global == 0)) THEN
    CALL tsslct(restns, resnew, itrmcd, fc, m, n, dn, dt, gp, df, nwtake)
  END IF

END IF

! choose the next iterate XP by a global strategy

IF(global == 0) THEN
  IF(method == 0) THEN
    CALL tslsch(m, n, xc, df, gp, steptl, dxn, dfn, fvec,  &
                mxtake, stepmx, xp, fp, fnorm, fpls, retcd)
  ELSE IF(m == n) THEN
    CALL tsmdls(aja, shat, anls, xc, m, n, p, dt, gp, dxn, dfn, &
                fvec, method, steptl, global, stepmx, epsm, fq, dn, fqq,   &
                pivot, curpos, pbar, mxtake, xp, fp, fnorm, fpls, zero1,   &
                retcd, ierr)
  ELSE
    CALL tslsch(m, n, xc, df, gp, steptl, dxn, dfn, fvec,  &
                mxtake, stepmx, xp, fp, fnorm, fpls, retcd)
  END IF
ELSE
  CALL ts2dtr(aja, shat, anls, df, gp, gbar, xc, method, nwtake, stepmx,   &
              steptl, epsm, mxtake, dlt, fqq, maxm, m, n, p, curpos, &
              pivot, pbar, itn, ierr, flag, dxn, dfn, fvec, fnorm, xp, fp, &
              fpls, retcd)
END IF

! evaluate the Jacobian at the new iterate XP

CALL tssclj(xp, dxn, typx, fp, dfn, m, n, epsm, jacflg, fvec, jac, aja)

! evaluate the gradient at the new iterate XP

CALL dgemv('T', m, n, one, aja, maxm, fp, 1, zero, gp, 1)

! check stopping criteria for the new iterate XP

CALL tsnstp(gp, xp, fp, xc, steptl, gradtl, retcd, ftol, itn,  &
            itnlim, icscmx, mxtake, m, n, msg, ipr, fpls, termcd)

IF(termcd > 0) GO TO 120
IF(MOD(msg/16,2) == 1) CALL tsrslt(n, xp, fpls, gp, itn, ipr)

! if tensor method is selected then update the matrices S and FV

IF(method == 1) THEN
  CALL tsupsf(fc, xc, xp, sqrn, itn, m, n, s, fv)
END IF

! update XC, FC, and FNORM

xc(1:n) = xp(1:n)
fc(1:m) = fp(1:m)
fnorm = fpls
GO TO 80

! termination

120 IF(MOD(msg/8,2) == 0) THEN
  IF(itn /= 0) THEN
    CALL tsrslt(n, xp, fpls, gp, itn, ipr)
  ELSE
    fpls = half*dnrm2(m, fc, 1)**2
    CALL tsrslt(n, xc, fpls, gp, itn, ipr)
  END IF
END IF

DEALLOCATE( fc, anls, aja, s )
RETURN

896 FORMAT('  TSNESV      TYPICAL X')
897 FORMAT('  TSNESV      DIAGONAL SCALING MATRIX FOR X')
898 FORMAT('  TSNESV      TYPICAL F')
899 FORMAT('  TSNESV      DIAGONAL SCALING MATRIX FOR F')
900 FORMAT(100('  TSNESV     ', 3(g20.13, "   ")/))
901 FORMAT('  TSNESV      JACOBIAN FLAG      = ', i1)
902 FORMAT('  TSNESV      METHOD USED        = ', i1)
903 FORMAT('  TSNESV      GLOBAL STRATEGY    = ', i1)
904 FORMAT('  TSNESV      ITERATION LIMIT    = ', i5)
905 FORMAT('  TSNESV      MACHINE EPSILON    = ', g20.13)
906 FORMAT('  TSNESV      STEP TOLERANCE     = ', g20.13)
907 FORMAT('  TSNESV      GRADIENT TOLERANCE = ', g20.13)
908 FORMAT('  TSNESV      FUNCTION TOLERANCE = ', g20.13)
909 FORMAT('  TSNESV      MAXIMUM STEP SIZE  = ', g20.13)
910 FORMAT('  TSNESV      TRUST REG RADIUS   = ', g20.13)
END SUBROUTINE tsnesv



SUBROUTINE tsnstp(g, xplus, fplus, xc, steptl, gradtl, retcd, ftol, itn,  &
                  itnlim, icscmx, mxtake, m, n, msg, ipr, fnorm, termcd)

! N.B. Added TERMCD = 0  24 May 2001

REAL (dp), INTENT(IN)    :: g(:)
REAL (dp), INTENT(IN)    :: xplus(:)
REAL (dp), INTENT(IN)    :: fplus(:)
REAL (dp), INTENT(IN)    :: xc(:)
REAL (dp), INTENT(IN)    :: steptl
REAL (dp), INTENT(IN)    :: gradtl
INTEGER, INTENT(IN)      :: retcd
REAL (dp), INTENT(IN)    :: ftol
INTEGER, INTENT(IN)      :: itn
INTEGER, INTENT(IN)      :: itnlim
INTEGER, INTENT(IN OUT)  :: icscmx
LOGICAL, INTENT(IN)      :: mxtake
INTEGER, INTENT(IN)      :: m
INTEGER, INTENT(IN)      :: n
INTEGER, INTENT(IN)      :: msg
INTEGER, INTENT(IN)      :: ipr
REAL (dp), INTENT(IN)    :: fnorm
INTEGER, INTENT(OUT)     :: termcd

!**********************************************************************
! THIS ROUTINE DECIDES WHETHER TO TERMINATE THE NONLINEAR ALGORITHM.
!**********************************************************************

!    INPUT PARAMETERS :
!    ------------------

!    G     : GRADIENT AT XC
!    XPLUS : NEW ITERATE
!    FPLUS : FUNCTION VALUE AT XPLUS
!    XC    : CURRENT ITERATE
!    STEPTL: STEP TOLERANCE
!    GRADTL: GRADIENT TOLERANCE
!    RETCD : RETURN CODE WITH THE FOLLOWING MEANINGS :
!            RETCD = 0 : SUCCESSFUL GLOBAL STRATEGY
!            RETCD = 1 : UNSUCCESSFUL GLOBAL STRATEGY
!    FTOL  : FUNCTION TOLERANCE
!    ITN   : ITERATION NUMBER
!    ITNLIM: ITERATION NUMBER LIMIT
!    ICSCMX: NUMBER CONSECUTIVE STEPS >= STEPMX
!    MXTAKE: BOOLEAN FLAG INDICATING STEP OF MAXIMUM LENGTH
!    M     : DIMENSION OF FPLUS
!    N     : DIMENSION OF G, XC, AND XPLUS
!    MSG   : MESSAGE TO INHIBIT CERTAIN AUTOMATIC CHECKS + OUTPUT
!    IPR   : DEVICE TO WHICH TO SEND OUTPUT

!    OUTPUT PARAMETERS :
!    ------------------

!    TERMCD: RETURN CODE WITH THE FOLLOWING MEANINGS :
!           TERMCD = 0 NO TERMINATION CRITERION SATISFIED

!           TERMCD > 0 : SOME TERMINATION CRITERION SATISFIED
!           TERMCD = 1 : NORM OF SCALED FUNCTION VALUE IS LESS THAN FTOL
!           TERMCD = 2 : GRADIENT TOLERANCE REACHED
!           TERMCD = 3 : SCALED DISTANCE BETWEEN LAST TWO STEPS < STEPTL
!           TERMCD = 4 : UNSUCCESSFUL GLOBAL STRATEGY
!           TERMCD = 5 : ITERATION LIMIT EXCEEDED
!           TERMCD = 6 : 5 CONSECUTIVE STEPS OF LENGTH STEPMX HAVE BEEN TAKEN

!    SUBPROGRAMS CALLED:

!    LEVEL 1 BLAS  ...  IDAMAX

!**********************************************************************

INTEGER   :: i
REAL (dp) :: res, d, rgx, relgrd, rsx, relstp

termcd = 0

! check whether scaled function is within tolerance

res = ABS(fplus(idamax(m, fplus, 1)))
IF(res <= ftol) THEN
  termcd = 1
  IF(MOD(msg/8,2) == 0) THEN
    WRITE(ipr,701)
  END IF
  RETURN
END IF

! check whether scaled gradient is within tolerance

d = one/MAX(fnorm, DBLE(n/2))
rgx = zero
DO i = 1,n
  relgrd = ABS(g(i)) * MAX(ABS(xplus(i)), one)*d
  rgx = MAX(rgx,relgrd)
END DO
IF(rgx <= gradtl) THEN
  termcd = 2
  IF(MOD(msg/8,2) == 0) THEN
    WRITE(ipr,702)
  END IF
  RETURN
END IF

IF(itn == 0) RETURN

IF(retcd == 1) THEN
  termcd = 4
  IF(MOD(msg/8,2) == 0)  THEN
    WRITE(ipr,703)
  END IF
  RETURN
END IF

! check whether relative step length is within tolerance

rsx = zero
DO i = 1,n
  relstp = ABS(xplus(i) - xc(i))/MAX(xplus(i), one)
  rsx = MAX(rsx, relstp)
END DO
IF(rsx <= steptl) THEN
  termcd = 3
  IF(MOD(msg/8,2) == 0) THEN
    WRITE(ipr,704)
  END IF
  RETURN
END IF

! check iteration limit

IF(itn >= itnlim) THEN
  termcd = 5
  IF(MOD(msg/8,2) == 0) THEN
    WRITE(ipr,705)
  END IF
END IF

! check number of consecutive steps .ge. stepmx

IF(mxtake) THEN
  icscmx = icscmx + 1
  IF(icscmx >= 5) THEN
    termcd = 6
    IF(MOD(msg/8,2) == 0) THEN
      WRITE(ipr,706)
    END IF
  END IF
ELSE
  icscmx = 0
END IF

RETURN

701 FORMAT(/,'  TSNSTP      FUNCTION VALUE CLOSE TO ZERO')
702 FORMAT(/,'  TSNSTP      RELATIVE GRADIENT CLOSE TO ZERO')
703 FORMAT(/,'  TSNSTP      LAST GLOBAL STEP FAILED TO LOCATE A',/  &
    '  TSNSTP      POINT LOWER THAN THE CURRENT ITERATE')
704 FORMAT(/,'  TSNSTP      SUCCESSIVE ITERATES WITHIN TOLERANCE',/  &
    '  TSNSTP      CURRENT ITERATE IS PROBABLY SOLUTION')
705 FORMAT(/,'  TSNSTP      ITERATION LIMIT EXCEEDED',/  &
    '  TSNSTP      ALGORITHM FAILED')
706 FORMAT(/,'  TSNSTP      MAXIMUM STEP SIZE EXCEEDED 5',  &
    ' CONSECUTIVE TIMES',/  &
    '  TSNSTP      EITHER THE FUNCTION IS UNBOUNDED BELOW',/  &
    '  TSNSTP      BECOMES ASYMPTOTIC TO A FINITE VALUE',/  &
    '  TSNSTP      FROM ABOVE IN SOME DIRECTION',/  &
    '  TSNSTP      OR STEPMX IS TOO SMALL')

END SUBROUTINE tsnstp



SUBROUTINE tsprmv(x, y, pivot, n, job)

REAL (dp), INTENT(OUT)  :: x(:)
REAL (dp), INTENT(IN)   :: y(:)
INTEGER, INTENT(IN)     :: pivot(:)
INTEGER, INTENT(IN)     :: n
INTEGER, INTENT(IN)     :: job

!**********************************************************************
! THIS SUBROUTINE PERFORMS A VECTOR PERMUTATION.
!**********************************************************************

!      INPUT PARAMETERS :
!      -----------------

!      Y :  VECTOR TO TSPRMV
!  PIVOT :  PIVOT VECTOR
!      N :  DIMENSION OF THE VECTORS Y AND PIVOT

!      OUTPUT PARAMETERS :
!      -------------------

!      X : PIVOTED VECTOR

!**********************************************************************

INTEGER :: i

IF(job == 0) THEN

! permute Y

  DO i = 1,n
    x(pivot(i)) = y(i)
  END DO
ELSE

! reverse permute of Y

  DO i = 1,n
    x(i) = y(pivot(i))
  END DO

END IF

RETURN
END SUBROUTINE tsprmv



SUBROUTINE tsrslt(n, xp, fval, gp, itn, ipr)

INTEGER, INTENT(IN)    :: n
REAL (dp), INTENT(IN)  :: xp(:)
REAL (dp), INTENT(IN)  :: fval
REAL (dp), INTENT(IN)  :: gp(:)
INTEGER, INTENT(IN)    :: itn
INTEGER, INTENT(IN)    :: ipr

!**********************************************************************
! THIS ROUTINE PRINTS INFORMATION.
!**********************************************************************

!    INPUT PARAMETERS :
!    -----------------

!    M,N  : DIMENSIONS OF PROBLEM
!    XP   : NEXT ITERATE
!    FVAL : SUM OF SQUARES OF F(XP)
!    GP   : GRADIENT AT XP
!    ITN  : ITERATION NUMBER
!    IPR  : DEVICE TO WHICH TO SEND OUTPUT

!**********************************************************************

WRITE(ipr,801) itn
WRITE(ipr,802)

WRITE(ipr,803) xp(1:n)
WRITE(ipr,804)
WRITE(ipr,805) fval
WRITE(ipr,806)
WRITE(ipr,807) gp(1:n)

801 FORMAT(/,'  TSRSLT    ITERATION K   = ',i5)
802 FORMAT('  TSRSLT    X(K)')
803 FORMAT(100('  TSRSLT    ', 3(g20.13, "   "),/))
804 FORMAT('  TSRSLT    FUNCTION AT X(K)')
805 FORMAT('  TSRSLT       ', g20.13)
806 FORMAT('  TSRSLT    GRADIENT AT X(K)')
807 FORMAT(100('  TSRSLT    ', 3(g20.13, "   "),/))

RETURN
END SUBROUTINE tsrslt



SUBROUTINE tsq1p1(aja, anls, s, f, n, root, restns)

REAL (dp), INTENT(IN)   :: aja(:,:)
REAL (dp), INTENT(IN)   :: anls(:,:)
REAL (dp), INTENT(IN)   :: s(:,:)
REAL (dp), INTENT(IN)   :: f(:)
INTEGER, INTENT(IN)     :: n
REAL (dp), INTENT(OUT)  :: root
REAL (dp), INTENT(OUT)  :: restns

!**********************************************************************
! THIS ROUTINE SOLVES THE LOWER M-N+Q QUADRATIC EQUATIONS IN P UNKNOWNS
! OF THE TENSOR MODEL WHEN Q = 1 AND P = 1.
!**********************************************************************

!    INPUT PARAMETERS :
!    -----------------

!    AJA  : JACOBIAN MATRIX AT CURRENT ITERATE
!    ANLS : TENSOR TERM MATRIX AT CURRENT ITERATE
!    S    : MATRIX OF PAST LINEARLY INDEPENDENT DIRECTIONS
!    F    : FUNCTION VALUE AT CURRENT ITERATE MULTIPIED BY AN
!           ORTHOGONAL MATRIX
!    N    : COLUMN DIMENSION OF AJA

!    OUTPUT PARAMETERS :
!    -----------------

!    ROOT   : SOLUTION TO THE SYSTEM
!    RESTNS : TENSOR RESIDUAL

!**********************************************************************

REAL (dp)            :: delta, t1, t2
REAL (dp), PARAMETER :: half = 0.5_dp, two = 2.0_dp

! find the roots of the equation:
! F(N) + AJA(N,N)*D + 0.5*ANLS(N,1)*(S(N+2,1)*D)**2

t1 = aja(n,n)
t2 = anls(n,1) * s(n+2,1)**2
IF(anls(n,1) == zero) THEN
  root = -f(n)/t1
ELSE
  delta = t1**2 - two*f(n)*t2
  IF(delta >= zero) THEN
    root = (-t1 + SIGN(one,t1) * SQRT(delta))/t2
  ELSE
    root = -t1/t2
  END IF
END IF

! compute tensor residual

restns = ABS(f(n) + aja(n,n)*root + half*anls(n,1)*(s(n+2,1)**2)* (root**2))
RETURN
END SUBROUTINE tsq1p1



SUBROUTINE tsqfcn(p, x, sum, qrank, meqns, nvars, fc, anls, aja, s)

INTEGER, INTENT(IN)       :: p
REAL (dp), INTENT(IN)     :: x(:)
REAL (dp), INTENT(OUT)    :: sum
INTEGER, INTENT(IN)       :: qrank, meqns, nvars
REAL (dp), INTENT(IN OUT) :: fc(:), anls(:,:), aja(:,:), s(:,:)

!*********************************************************************
! THIS ROUTINE IS USED TO EVALUATE THE RESIDUAL OF THE LAST M-N+P
! QUADRATIC EQUATIONS IN P UNKNOWNS OF THE TENSOR MODEL. NOTE THAT
! THIS ROUTINE IS CALLED BY UNCMIN TO SOLVE THE NONLINEAR LEAST SQUARES
! PART OF THE TENSOR MODEL.
!*********************************************************************

!    INPUT PARAMETERS :
!    -----------------

!    P : DIMENSION OF THE PROBLEM SOLVED BY UNCMIN

!    INPUT-OUTPUT PARAMETERS :
!    -----------------------

!    X : nullptr VECTOR ON ENTRY AND APPROXIMATION OF THE SOLUTION
!        TO THE SYSTEM OF M-N+Q QUADRATIC EQUATIONS IN P UNKNOWNS
!        OF THE TENSOR MODEL ON EXIT

!    OUTPUT PARAMETERS :
!    -----------------

!    SUM : RESIDUAL OF THE LAST M-N+P QUADRATIC EQUATIONS IN P
!          UNKNOWNS OF THE TENSOR MODEL

!    SUBPROGRAMS CALLED:

!    LEVEL 1 BLAS  ...  DNRM2
!    LEVEL 2 BLAS  ...  DGEMV
!    TENSOLVE      ...  TSSTMX

!*********************************************************************

! Workspace
REAL (dp) :: wrk1(meqns), wrk2(p), wrk3(p), wrk4(meqns), wrk5(meqns)

INTEGER              :: i
REAL (dp), PARAMETER :: half = 0.5_dp
REAL (dp)            :: small

small = 4.0_dp * SQRT( TINY(1.0_dp) )

! compute the lower right (m-n+q) x p submatrix of AJA times X

CALL dgemv('N', meqns-nvars+qrank, p, one, aja(nvars-qrank+1:,nvars-p+1:), &
           meqns, x, 1, zero, wrk1, 1)

! compute S-trans times X

CALL tsstmx(s, x, nvars, p, wrk3)

! compute 0.5 * (S-trans times X)**2


DO i = 1, p
  IF (ABS(wrk3(i)) > small) THEN
    wrk2(i) = half * wrk3(i)**2
  ELSE
    wrk2(i) = zero
  END IF
END DO

! compute 0.5 * (down (m-n+q) x p submatrix of ANLS) * (S-trans times X)**2

CALL dgemv('N', meqns-nvars+qrank, p, one, anls(nvars-qrank+1:,:), meqns,  &
           wrk2, 1, zero, wrk4, 1)

DO i = 1,meqns-nvars+qrank
  wrk5(i) = wrk4(i) + fc(nvars-qrank+i) + wrk1(i)
END DO

sum = half*dnrm2(meqns-nvars+qrank, wrk5, 1)**2

RETURN
END SUBROUTINE tsqfcn



SUBROUTINE tsqlfc(ql, m, n, ierr)

REAL (dp), INTENT(IN OUT)  :: ql(:,:)
INTEGER, INTENT(IN)        :: m
INTEGER, INTENT(IN)        :: n
INTEGER, INTENT(OUT)       :: ierr

!**********************************************************************
! THIS ROUTINE PERFORMS A QL DECOMPOSITION.
!**********************************************************************

!    INPUT PARAMETERS :
!    ----------------

!     M   : ROW DIMENSION OF QL
!     N   : COLUMN DIMENSION OF QL

!    INPUT-OUTPUT PARAMETERS :
!    -----------------------

!     QL : INPUT MATRIX ON ENTRY AND FACTORED MATRIX ON EXIT

!    OUTPUT PARAMETERS :
!    ------------------

!     IERR : RETURN CODE WITH THE FOLLOWING MEANINGS :
!            IERR = 1 : SINGULARITY DETECTED
!            IERR = 0 : OTHERWISE

!**********************************************************************

INTEGER   :: i, j, k
REAL (dp) :: nu, sigma, tau, rnu

! zero out rows m+1 and m+2 of matrix QL

DO j = 1,n
  ql(m+1,j) = zero
  ql(m+2,j) = zero
END DO

ierr = 0

k = 1

20 IF(k < m .AND. k <= n) THEN

! find NU = max element of col K on or above l-diagonal

  nu = zero
  DO i = 1, m+1-k
    nu = MAX(nu, ABS(ql(i,k)))
  END DO

  IF(nu /= zero) THEN

! normalize col K on or above l-diagonal

    rnu = one/nu
    ql(1:m-k+1,k) = rnu * ql(1:m-k+1,k)

! code to find SIGMA = SGN(QL(M+1-K,K))*l2-norm of col K on or
! above l-diagonal

    sigma = dnrm2(m-k+1, ql(:,k), 1)
    sigma = SIGN(sigma, ql(m+1-k,k))

! store last element(1st in normal QR) of U-vector in QL(M+1-K,K)

    ql(m+1-k,k) = ql(m+1-k,k)+sigma

! store value of <U,U>/2 in QL(M+1,K)

    ql(m+1,k) = sigma*ql(m+1-k,k)
    IF(ql(m+1,k) == zero) THEN
      ierr = 1
      RETURN
    END IF

! store L(M+1-K,K) in QL(M+2,K)

    ql(m+2,k) = -nu*sigma

! code to get (I-2U*UT/<U,U>)*QL for kth iteration

    IF(k < n) THEN
      DO j = k+1,n

! loop to get TAU = <U,J-TH COL OF QL>

        tau = (DOT_PRODUCT( ql(1:m-k+1,k), ql(1:m-k+1,j) ) )/ql(m+1,k)  ! modified by nicola

! loop to get (I-2U*UT/<U,U>)*j-th col of QL

        ql(1:m-k+1,j) = ql(1:m-k+1,j) + tau * ql(1:m-k+1,k)
      END DO
    END IF
    k = k + 1
  ELSE
    ierr = 1
    RETURN
  END IF

  GO TO 20

END IF

IF(m == n) ql(m+2,n) = ql(1,n)

IF(ql(m+2,n) == zero) THEN
  ierr = 1
END IF

RETURN
END SUBROUTINE tsqlfc



SUBROUTINE tsqmlv(n, p, q, v, w, trans)

INTEGER, INTENT(IN)     :: n
INTEGER, INTENT(IN)     :: p
REAL (dp), INTENT(IN)   :: q(:,:)
REAL (dp), INTENT(IN)   :: v(:)
REAL (dp), INTENT(OUT)  :: w(:)
LOGICAL, INTENT(IN)     :: trans

!**********************************************************************
! THIS ROUTINE MULTIPLIES AN ORTHOGONAL MATRTIX Q OR ITS TRANSPOSE BY
! A VECTOR.
!**********************************************************************

!    INPUT PARAMETERS :
!    ----------------

!    N  : DIMENSION OF VECTORS V AND W
!    P  : COLUMN DIMENSION OF MATRIX Q
!    Q  : ORTHOGONAL MATRIX (OBTAINED FROM TSQLFC SUBROUTINE)
!    V  : VECTOR TO BE MULTIPLIED BY Q
!    TRANS : BOOLEAN PARAMETER:
!            = TRUE  : PERFORM Q-TRANS*V
!            = FALSE : PERFORM Q*V

!    OUTPUT PARAMETERS :
!    -----------------

!    W  : VECTOR Q*V OR Q-TRANS*V ON EXIT

!**********************************************************************

INTEGER   :: j, k
REAL (dp) :: tau, const

w(1:n) = v(1:n)

DO j = 1,p
  IF(trans) THEN
    k = p + 1 - j
  ELSE
    k = j
  END IF
  tau = DOT_PRODUCT( q(1:n-k+1,k), w(1:n-k+1) )
  const = -tau/q(n+1,k)
  w(1:n-k+1) = w(1:n-k+1) + const * q(1:n-k+1,k)
END DO

RETURN
END SUBROUTINE tsqmlv



SUBROUTINE tsqmts(anls, qhat, mj, m, p, lb, zero1)

REAL (dp), INTENT(IN OUT)  :: anls(:,:)
REAL (dp), INTENT(IN)      :: qhat(:,:)
INTEGER, INTENT(IN)        :: mj
INTEGER, INTENT(IN)        :: m
INTEGER, INTENT(IN)        :: p
INTEGER, INTENT(IN)        :: lb
INTEGER, INTENT(IN)        :: zero1

!**********************************************************************
! THIS ROUTINE MULTIPLIES AN ORTHOGONAL MATRIX QHAT BY THE TENSOR
! MATRIX ANLS.
!**********************************************************************

!     INPUT PARAMETERS :
!     ----------------

!   QHAT : ORTHOGONAL MATRIX (OBTAINED FROM TSQRFC SUBROUTINE)
!     MJ : ROW DIMENSION OF QHAT
!     M  : ROW DIMENSION OF MATRIX ANLS
!     P  : COLUMN DIMENSION OF MATRIX ANLS
!     LB : STARTING COLUMN FROM WHICH QR DECOMPOSITION WAS PERFORMED
!   ZERO1: FIRST ZERO COLUMN OF MATRIX QHAT IN CASE OF SINGULARITY

!      INPUT-OUTPUT PARAMETERS :
!     ------------------------

!     ANLS : MATRIX TO BE MULTIPLIED BY AN ORTHOGONAL MATRIX
!     ON ENTRY AND THE MATRIX QHAT*ANLS ON EXIT

!     SUBPROGRAMS CALLED:

!     TENSOLVE      ...  TSQMUV

!**********************************************************************

! Workspace
REAL (dp) :: wrk1(m)

INTEGER   :: j

DO j = 1,p
  CALL tsqmuv(qhat, anls(:,j), wrk1, mj, lb, zero1, .false.)
  anls(1:m,j) = wrk1(1:m)
END DO

RETURN
END SUBROUTINE tsqmts



SUBROUTINE tsqmuv(q, v, w, m, lb, zero1, transp)

REAL (dp), INTENT(IN)   :: q(:,:)
REAL (dp), INTENT(IN)   :: v(:)
REAL (dp), INTENT(OUT)  :: w(:)
INTEGER, INTENT(IN)     :: m
INTEGER, INTENT(IN)     :: lb
INTEGER, INTENT(IN)     :: zero1
LOGICAL, INTENT(IN)     :: transp

!**********************************************************************
! THIS SUBROUTINE MULTIPLIES AN ORTHOGONAL MATRIX BY A VECTOR.
!**********************************************************************

!      INPUT PARAMETERS :
!      -----------------

!      Q  : FACTORED MATRIX (OBTAINED FROM SUBROUTINE TSQRFC)
!      V  : VECTOR TO BE MULTIPLIED BY THE ORTHOGONAL MATRIX Q
!      M  : ROW DIMENSION OF MATRIX Q
!      LB : STARTING COLUMN FROM WHICH QR DECOMPOSITION WAS PERFORMED
!    ZERO1: FIRST ZERO COLUMN OF THE MATRIX Q
!  TRANSP : BOOLEAN PARAMETER :
!               = TRUE  : PERFORM Q-TRANS*V
!               = FALSE : PERFORM Q*V

!      OUTPUT PARAMETERS :
!      -----------------

!      W : Q*V OR Q-TRANS*V ON EXIT

!********************************************************************

INTEGER   :: ub, a, b, c, k
REAL (dp) :: tau, const

! copy the vector V to W

w(1:m) = v(1:m)

ub = zero1-1
IF(transp) THEN
  a = ub
  b = lb
  c = -1
ELSE
  a = lb
  b = ub
  c = 1
END IF

DO k = a,b,c
  tau = DOT_PRODUCT( q(k:m,k), w(k:m) )
  const = -tau/q(m+1,k)
  w(k:m) = w(k:m) + const * q(k:m,k)
END DO

RETURN
END SUBROUTINE tsqmuv



SUBROUTINE tsqrfc(qr, n, m, lb, ub, ierr, epsm, curpos, zero1)

REAL (dp), INTENT(IN OUT)  :: qr(:,:)
INTEGER, INTENT(IN)        :: n
INTEGER, INTENT(IN)        :: m
INTEGER, INTENT(IN)        :: lb
INTEGER, INTENT(IN)        :: ub
INTEGER, INTENT(OUT)       :: ierr
REAL (dp), INTENT(IN)      :: epsm
INTEGER, INTENT(OUT)       :: curpos(:)
INTEGER, INTENT(OUT)       :: zero1

!**********************************************************************
! THIS ROUTINE PERFORMS COLUMN-PIVOTED QR DECOMPOSITION ON AN M*N
! MATRIX. THE DECOMPOSITION IS DONE BETWEEN COLS LB AND UB.
!**********************************************************************

!      INPUT PARAMETERS :
!      -----------------

!      N  : COLUMN DIMENSION OF MATRIX QR
!      M  : ROW DIMENSION OF MATRIX QR
!   LB,UB : SUBSPACE OF QR DECOMPOSITION
!   EPSM  : MACHINE PRECISION

!      INPUT-OUTPUT PARAMETERS :
!      ------------------------
!      QR  : INPUT MATRIX ON ENTRY AND FACTORED MATRIX ON EXIT

!      OUTPUT PARAMETERS :
!      ------------------

!      IERR : RETURN CODE WITH TH FOLLOWING MEANINGS:
!             IERR  =  1 : SINGULARITY DETECTED
!             IERR  =  0 : OTHERWISE
!      CURPOS :  PIVOT VECTOR
!      ZERO1  :  FIRST ZERO COLUMN OF MATRIX QR IN CASE OF SINGULARITY

!      SUBPROGRAMS CALLED:

!      LEVEL 1 BLAS  ...  DNRM2,DSWAP,IDAMAX

! **********************************************************************

! Workspace
REAL (dp) :: al2nrm(m)

INTEGER   :: colpiv, i, j, k, l
REAL (dp) :: colmax, sigma, tau, amax
REAL (dp) :: nu, rnu

! zero rows m+1 and m+2 of QR matrix

DO j = 1,n
  curpos(j) = j
END DO

DO j = lb,ub
  qr(m+1,j) = zero
  qr(m+2,j) = zero
END DO

ierr = 0
zero1 = ub+1
k = lb

!  get L2NORM**2 of columns (LB to UB)

DO j = k,ub
  al2nrm(j) = dnrm2(m-k+1, qr(k:,j), 1)**2
END DO

40 IF(k < m .AND. k <= ub) THEN

  amax = zero
  DO j = k,ub
    IF(al2nrm(j) >= amax) THEN
      amax = al2nrm(j)
      colpiv = j
    END IF
  END DO

  IF(amax == zero) THEN
    ierr = 1
    zero1 = k
    RETURN
  END IF

  IF(k == lb) THEN
    colmax = amax
  END IF

  IF(al2nrm(colpiv) <= epsm*colmax) THEN
    ierr = 1
    zero1 = k
    RETURN
  END IF

  IF(colpiv /= k) THEN
    CALL dswap(m+2, qr(:,colpiv), 1, qr(:,k), 1)
    l = curpos(k)
    curpos(k) = curpos(colpiv)
    curpos(colpiv) = l
    CALL dswap(1, al2nrm(colpiv:), 1, al2nrm(k:), 1)
  END IF

! find NU = max element of col K on or below diagonal

  l = idamax(m-k+1, qr(k:,k), 1) + k - 1
  nu = ABS(qr(l,k))

  IF(nu == zero) THEN
    ierr = 1
    zero1 = k
    RETURN
  END IF

! normalize col K on or below diagonal

  rnu = one/nu
  qr(k:m,k) = rnu * qr(k:m,k)

! code to find SIGMA = SGN(QR(K,K))*l2-norm of col K on or
! below diagonal

  sigma = dnrm2(m-k+1, qr(k:,k), 1)
  sigma = SIGN(sigma, qr(k,k))

! store 1st element of U-vector in QR(K,K)

  qr(k,k) = qr(k,k)+sigma

! store value of <U,U>/2 in QR(M+1,K)

  qr(m+1,k) = sigma*qr(k,k)
  IF(qr(m+1,k) == zero) THEN
    ierr = 1
    zero1 = k
    RETURN
  END IF

! store R(K,K) in QR(M+2,K)

  qr(m+2,k) = -nu*sigma
  IF(qr(m+2,k) == zero) THEN
    ierr = 1
    zero1 = k
    RETURN
  END IF

! code to get (I-2U*UT/<U,U>)*QR for kth iteration

  IF(k < n) THEN
    DO j = k+1,n

! loop to get UT*J-TH col of QR

      tau = -DOT_PRODUCT( qr(k:m,k), qr(k:m,j) ) / qr(m+1,k)

! loop to get (I-2U*UT/<U,U>)*j-th col of QR

      qr(k:m,j) = qr(k:m,j) + tau * qr(k:m,k)
    END DO
  END IF

! update l2norm**2 (K+1 to UB)

  DO i = k+1,ub
    al2nrm(i) = al2nrm(i) - qr(k,i)**2
  END DO

  k = k+1
  GO TO 40

ELSE

  IF(lb == ub) colmax = al2nrm(lb)

END IF

IF(m == ub) qr(m+2,ub) = qr(m,m)
IF(ABS(qr(m+2,ub)) <= epsm*colmax) THEN
  ierr = 1
  zero1 = ub
END IF

RETURN
END SUBROUTINE tsqrfc



SUBROUTINE tsrsid(itn, method, fq, d, curpos, pivot, pbar, aja, anls,  &
                  shat, flag, nwtake, ierr, maxm, m, n, p, scres)

INTEGER, INTENT(IN)     :: itn
INTEGER, INTENT(IN)     :: method
REAL (dp), INTENT(IN)   :: fq(:)
REAL (dp), INTENT(IN)   :: d(:)
INTEGER, INTENT(IN)     :: curpos(:)
INTEGER, INTENT(IN)     :: pivot(:)
INTEGER, INTENT(IN)     :: pbar(:)
REAL (dp), INTENT(IN)   :: aja(:,:)
REAL (dp), INTENT(IN)   :: anls(:,:)
REAL (dp), INTENT(IN)   :: shat(:,:)
INTEGER, INTENT(IN)     :: flag
LOGICAL, INTENT(IN)     :: nwtake
INTEGER, INTENT(IN)     :: ierr
INTEGER, INTENT(IN)     :: maxm
INTEGER, INTENT(IN)     :: m
INTEGER, INTENT(IN)     :: n
INTEGER, INTENT(IN)     :: p
REAL (dp), INTENT(OUT)  :: scres

!**********************************************************************
! THIS ROUTINE COMPUTES || F + J*D + 0.5*A*D**2 ||**2 IN THE L2
! NORM SENS AT A GIVEN STEP D.
!**********************************************************************

!    INPUT PARAMETERS
!    ----------------

!     ITN   : CURRENT ITERATION NUMBER
!     METHOD: METHOD TO BE USED
!     FQ    : FUNCTION VALUE AT CURRENT ITERATE MULTIPLIED BY
!             ORTHOGONAL MATRICES
!     D     : STEP AT WHICH TO EVALUATE THE TENSOR MODEL
!     CURPOS: PIVOT VECTOR (USED DURING THE FACTORIZATION OF AJA
!             FROM COLUMN 1 TO N-P)
!     PIVOT : PIVOT VECTOR ( USED DURING THE FACTORIZATION OF AJA
!             FROM COLUMN N-P+1 TO N)
!     PBAR  : PIVOT VECTOR (USED DURING THE FACTORIZATION OF AJA
!             IF IT IS SINGULAR
!     AJA   : JACOBIAN MATRIX AT CURRENT ITERATE
!     ANLS  : TENSOR TERM MATRIX AT CURRENT ITERATE
!     SHAT  : MATRIX OF PAST LINEARLY INDEPENDENT DIRECTIONS AFTER
!             A QL FACTORIZATION
!     FLAG  : RETURN CODE WITH THE FOLLOWING MEANINGS:
!             FLAG = 0 : NO SINGULARITY DETECTED DURING FACTORIZATION
!                        OF THE JACOBIAN FROM COLUMN 1 TO N
!             FLAG = 1 : SINGULARITY DETECTED DURING FACTORIZATION
!                        OF THE JACOBIAN FROM COLUMN 1 TO N-P
!             FLAG = 2 : SINGULARITY DETECTED DURING FACTORIZATION
!                        OF THE JACOBIAN FROM COLUMN N-P+1 TO N
!     NWTAKE: LOGICAL VARIABLE WITH THE FOLLOWING MEANINGS:
!             NWTAKE = .TRUE.  : NEWTON STEP TAKEN
!             NWTAKE = .FALSE. : TENSOR STEP TAKEN
!     IERR  : RETURN CODE FROM QRP FACTORIZATION ROUTINE:
!             IERR = 0 : NO SINGULARITY DETECTED
!             IERR = 1 : SINGULARITY DETECTED
!    MAXM   : LEADING DIMENSION OF AJA AND ANLS
!    M,N    : DIMENSIONS OF THE PROBLEM
!    P      : COLUMN DIMENSION OF THE MATRICES SHAT AND ANLS

!    OUTPUT PARAMETERS
!    -----------------

!     SCRES :  || F + J*D + 0.5*A*D**2 ||**2

!    SUBPROGRAMS CALLED:

!    LEVEL 1 BLAS  ...  DNRM2
!    LEVEL 2 BLAS  ...  DGEMV
!    TENSOLVE      ...  TSJMUV,TSUDQV

! **********************************************************************

! Workspace
REAL (dp) :: wrk1(n+m), vn(n+m), vnp(m), vns(m)

INTEGER   :: len
REAL (dp), PARAMETER :: half = 0.5_dp

CALL tsjmuv(itn, method, d, curpos, pivot, pbar, aja, shat, flag,  &
            ierr, m, n, p, vns, wrk1)

wrk1(n+1:n+m) = zero

len = m
IF(ierr > 0) len = m + n

vn(1:len) = wrk1(1:len) + fq(1:len)

IF( .NOT. nwtake) THEN
  CALL tsudqv(shat, vns, n, p, vnp)
  vnp(1:p) = vnp(1:p) ** 2
  CALL dgemv('N', len, p, half, anls, maxm, vnp, 1, one, vn, 1)
END IF

scres = dnrm2(len, vn, 1)

RETURN
END SUBROUTINE tsrsid



SUBROUTINE tssclf(f,df,m)

REAL (dp), INTENT(IN OUT)  :: f(:)
REAL (dp), INTENT(IN)      :: df(:)
INTEGER, INTENT(IN)        :: m

!*******************************************************************
! THIS ROUTINE SCALES A FUNCTION VALUE F.
!*******************************************************************

!    INPUT PARAMETERS :
!    ------------------

!    DF : DIAGONAL SCALING MATRIX FOR F
!    M  : DIMENSION OF F

!    INPUT-OUTPUT PARAMETERS :
!    ------------------

!    F  : UNSCALED FUNCTION VALUE ON ENTRY AND SCALED FUNCTION
!         VALUE ON EXIT

!*********************************************************************

f(1:m) = df(1:m)*f(1:m)

RETURN
END SUBROUTINE tssclf



SUBROUTINE tssclj(x, dx, typx, f, df, m, n, epsm, jacflg, fvec, jac, aja)

REAL (dp), INTENT(IN OUT)  :: x(:)
REAL (dp), INTENT(IN)      :: dx(:)
REAL (dp), INTENT(IN)      :: typx(:)
REAL (dp), INTENT(IN OUT)  :: f(:)
REAL (dp), INTENT(IN)      :: df(:)
INTEGER, INTENT(IN)        :: m
INTEGER, INTENT(IN)        :: n
REAL (dp), INTENT(IN)      :: epsm
INTEGER, INTENT(IN)        :: jacflg
REAL (dp), INTENT(IN OUT)  :: aja(:,:)

INTERFACE
  SUBROUTINE fvec(x, f, m, n) bind(c)
    IMPLICIT NONE
    INTEGER, PARAMETER     :: dp = SELECTED_REAL_KIND(14, 60)
    REAL (dp), INTENT(IN)  :: x(n)
    REAL (dp), INTENT(OUT) :: f(m)
    INTEGER, INTENT(IN)    :: m, n
  END SUBROUTINE fvec

  SUBROUTINE jac(x, aja, m, n) bind(c)
    IMPLICIT NONE
    INTEGER, PARAMETER     :: dp = SELECTED_REAL_KIND(14, 60)
    REAL (dp), INTENT(IN)  :: x(n)
    REAL (dp), INTENT(OUT) :: aja(m,n)
    INTEGER, INTENT(IN)    :: m, n
  END SUBROUTINE jac
END INTERFACE

!**********************************************************************
! THIS ROUTINE COMPUTES THE JACOBIAN MATRIX AT THE CURRENT ITERATE
! X AND SCALES ITS VALUE.
!**********************************************************************

!     INPUT PARAMETERS :
!     -----------------

!     X    : SCALED CURRENT ITERATE
!     DX   : DIAGONAL SCALING MATRIX FOR X
!     F    : SCALED FUNCTION VALUE AT X
!     DF   : DIAGONAL SCALING MATRIX FOR F
!     M,N  : DIMENSIONS OF PROBLEM
!     EPSM : MACHINE PRECISION
!   JACFLG : JACOBIAN FLAG
!     FVEC : SUBROUTINE TO EVALUATE FUNCTION
!     JAC  : SUBROUTINE TO EVALUATE ANALYTIC JACOBIAN


!     INPUT-OUTPUT PARAMETERS :
!     ------------------------

!     AJA  : SCALED JACOBIAN AT CURRENT ITERATE

!     SUBPROGRAMS CALLED:

!     TENSOLVE      ...  TSUNSX,TSUNSF,TSFDFJ,TSSCLF,TSSCLX
!     USER          ...  FVEC,JAC

!********************************************************************

INTEGER :: i,j

! unscale X AND F

CALL tsunsx(x, dx, n)
CALL tsunsf(f, df, m)

! compute the finite difference or analytic Jacobian at X

IF(jacflg == 0) THEN
  CALL tsfdfj(x, f, m, n, epsm, fvec, aja)
ELSE
  CALL jac(x, aja, m, n)
END IF

! scale the Jacobian matrix

DO j = 1,n
  DO i = 1,m
    aja(i,j) = aja(i,j)*df(i)*typx(j)
  END DO
END DO

! scale back X AND F

CALL tssclf(f, df, m)
CALL tssclx(x, dx, n)

RETURN
END SUBROUTINE tssclj



SUBROUTINE tssclx(x, dx, n)

REAL (dp), INTENT(IN OUT)  :: x(:)
REAL (dp), INTENT(IN)      :: dx(:)
INTEGER, INTENT(IN)        :: n

!**********************************************************************
! THIS ROUTINE SCALES A VECTOR X.
!**********************************************************************

!    INPUT PARAMETERS :
!    ------------------

!    DX : DIAGONAL SCALING MATRIX FOR X
!    N  : DIMENSION OF X

!    OUTPUT PARAMETERS :
!    ------------------

!    X  : SCALED VECTOR X

!**********************************************************************

x(1:n) = dx(1:n)*x(1:n)

RETURN
END SUBROUTINE tssclx



SUBROUTINE tsslct(residt, residn, itrmcd, fc, m, n, dn, dt, g, df, nwtake)

REAL (dp), INTENT(IN)   :: residt
REAL (dp), INTENT(IN)   :: residn
INTEGER, INTENT(IN)     :: itrmcd
REAL (dp), INTENT(IN)   :: fc(:)
INTEGER, INTENT(IN)     :: m
INTEGER, INTENT(IN)     :: n
REAL (dp), INTENT(IN)   :: dn(:)
REAL (dp), INTENT(IN)   :: dt(:)
REAL (dp), INTENT(IN)   :: g(:)
REAL (dp), INTENT(OUT)  :: df(:)
LOGICAL, INTENT(OUT)    :: nwtake

!*********************************************************************
! THIS ROUTINE DECIDES WHICH DIRECTION TO CHOOSE: THE TENSOR OR THE
! STANDARD DIRECTION. THE STANDARD DIRECTION IS CHOSEN WHENEVER
! THE TENSOR DIRECTION IS NOT DESCENT OR THE TENSOR DIRECTION IS TO A
! MINIMIZER OF THE TENSOR MODEL AND DOESN'T PROVIDE ENOUGH DECREASE
! IN THE TENSOR MODEL, OR THE QUADRATIC SYSTEM OF Q EQUATIONS IN P
! UNKNOWNS CANNOT BE SOLVED BY UNCMIN WITHIN 150 ITERATIONS.
!*********************************************************************

!    INPUT PARAMETERS :
!    ------------------

!    RESIDT : TENSOR RESIDUAL
!    RESIDN : NEWTON RESIDUAL
!    ITRMCD : UNCMIN TERMINATION CODE
!    FC : FUNCTION VALUE AT XC
!    M,N: DIMENSIONS OF PROBLEM
!    DN : STANDARD STEP
!    DT : TENSOR STEP
!    G  : GRADIENT VALUE AT XC

!    OUTPUT PARAMETERS :
!    -----------------

!    DF     : EITHER THE STANDARD OR TENSOR STEP ON EXIT
!    NWTAKE : BOOLEAN VALUE WITH THE FOLLOWING MEANINGS:
!            =.TRUE.  : STANDARD STEP IS TAKEN
!            =.FALSE. : TENSOR STEP IS TAKEN

!    SUBPROGRAMS CALLED:

!    LEVEL 1 BLAS  ....  DNRM2

!*********************************************************************

REAL (dp) :: anrmfc, dtnorm, gnorm
REAL (dp) :: temp, temp1, beta, gama
REAL (dp), PARAMETER :: tenth = 0.1_dp, onett = 1.0e-04_dp, half = 0.5_dp

nwtake = .false.
anrmfc = dnrm2(m, fc, 1)
dtnorm = dnrm2(n, dt, 1)
gnorm = dnrm2(n, g, 1)
temp = DOT_PRODUCT( dt(1:n), g(1:n) )

gama = half
IF(m > n) THEN
  beta = tenth
ELSE
  beta = onett
END IF

temp1 = -beta*dtnorm*gnorm

IF(residt >= gama*(anrmfc + residn) .OR. temp > temp1 .OR. itrmcd == 4) THEN
  df(1:n) = dn(1:n)
  nwtake = .true.
ELSE
  df(1:n) = dt(1:n)
END IF

RETURN
END SUBROUTINE tsslct



SUBROUTINE tssmin(anls, fq, adt, ag, const1, const2, dlt, m, n,  &
                  p, nwtake, ierr, epsm, sol)

REAL (dp), INTENT(IN)   :: anls(:,:)
REAL (dp), INTENT(IN)   :: fq(:)
REAL (dp), INTENT(IN)   :: adt(:)
REAL (dp), INTENT(IN)   :: ag(:)
REAL (dp), INTENT(IN)   :: const1(:)
REAL (dp), INTENT(IN)   :: const2(:)
REAL (dp), INTENT(IN)   :: dlt
INTEGER, INTENT(IN)     :: m
INTEGER, INTENT(IN)     :: n
INTEGER, INTENT(IN)     :: p
LOGICAL, INTENT(IN)     :: nwtake
INTEGER, INTENT(IN)     :: ierr
REAL (dp), INTENT(IN)   :: epsm
REAL (dp), INTENT(OUT)  :: sol

!***********************************************************************
! THIS ROUTINE MINIMIZES THE TENSOR MODEL OVER THE SUBSPACE SPANNED BY
! THE TENSOR STEP AND THE STEEPEST DECENT DIRECTION. IF THE NEWTON STEP
! WERE CHOSEN, IT WILL MINIMIZE THE NEWTON MODEL OVER THE SUBSPACE
! SPANNED BY THE NEWTON STEP AND THE STEEPEST DESCENT DIRECTION.
!***********************************************************************

!     INPUT PARAMETERS :
!     -----------------

!     ANLS : TENSOR TERM MATRIX AT CURRENT ITERATE
!     FQ   : FUNCTION VALUE AT CURRENT ITERATE MULTIPLIED BY
!            ORTHOGONAL MATRICES
!     ADT  : JACOBIAN MATRIX TIMES DT (SEE SUBROUTINE TS2DTR)
!      AG  : JACOBIAN MATRIX TIMES GBAR (SEE SUBROUTINE TS2DTR)
!    CONST1: SHAT-TRANS * DT  (SEE SUBROUTINE TS2DTR)
!    CONST2: SHAT-TRANS * GBAR (SEE SUBROUTINE TS2DTR)
!    ALPHA : POINT AT WHICH DERIVATIVE IS EVALUATED
!      DLT : CURRENT TRUST RADIUS
!       M,N: DIMENSIONS OF THE PROBLEM
!         P: COLUMN DIMENSION OF MATRIX ANLS
!   NWTAKE : LOGICAL VARIABLE WITH THE FOLLOWING MEANINGS :
!            NWTAKE = .TRUE.  : STANDARD STEP TAKEN
!            NWTAKE = .FALSE. : TENSOR STEP TAKEN
!   IERR   : RETURN CODE FROM QRP FACTORIZATION ROUTINE
!            IERR = 0 : NO SINGULARITY OF JACOBIAN DETECTED
!            IERR = 1 : SINGULARITY OF JACOBIAN DETECTED
!   EPSM   : MACHINE PRECISION

!     OUTPUT PARAMETERS :
!     -----------------

!     SOL   : GLOBAL MINIMIZER OF THE ONE VARIABLE FUNCTION IN ALPHA
!             ||F + J*(ALPHA*DT + BETA*GBAR) + 0.5*A*(ALPHA*DT +
!             BETA*GBAR)**2||**2 WHERE BETA = SQRT(DLT**2 - ALPHA**2)

!     SUBPROGRAMS CALLED:

!     TENSOLVE      ...  TSFAFA,TSMFDA,TSLMIN

!**********************************************************************

! Workspace
REAL (dp) :: vn(n+m)

INTEGER   :: INT
REAL (dp) :: tol, dl, s, sp, c, a
REAL (dp) :: d, s1, b, q, bc, optim, ac, glopt, bloop, eloop, incr
REAL (dp), PARAMETER :: ohund = 0.01_dp, tenth = 0.1_dp, two = 2.0_dp,  &
                        three = 3.0_dp, ten = 10.0_dp
LOGICAL   :: first

first = .true.
tol = epsm**(two/three)
INT = 40
dl = tol
IF(dlt < tol) THEN
  dl = tol*tenth
ELSE IF(dlt > tol*ten) THEN
  dl = tol*ten
END IF
IF(dlt <= ohund) THEN
  INT = 10
END IF

! find global minimizer of FALPHA

bloop = -dlt+dl
eloop = dlt*(INT-2)/INT
incr = two*(dlt-dl)/INT
s = bloop

10 sp = s
s1 = s+incr

! evaluate FALPHA(SP) and the derivative of FALPHA at SP

IF(first) THEN
  CALL tsfafa(anls, fq, adt, ag, const1, const2, sp, dlt, m, n, p,  &
              nwtake, ierr, vn, c)
  a = tsmfda(anls, adt, ag, const1, const2, sp, dlt, m, n, p, nwtake,  &
             ierr, vn)
ELSE
  c = d
  a = b
END IF

! evaluate FALPHA(S1) and the derivative of FALPHA at S1

CALL tsfafa(anls, fq, adt, ag, const1, const2, s1, dlt, m, n, p, nwtake, &
           ierr, vn, d)
b = tsmfda(anls, adt, ag, const1, const2, s1, dlt, m, n, p, nwtake, ierr, vn)

! minimize FALPHA in the subinterval [SP,S1]

IF(a <= zero .AND. b >= zero) THEN
  IF(c > d) THEN
    q = d
    bc = b
    CALL tslmin(s1, sp, bc, q, anls, fq, adt, ag, const1, const2,  &
                dlt, m, n, p, nwtake, ierr, tol, optim)
  ELSE
    q = c
    ac = a
    CALL tslmin(sp, s1, ac, q, anls, fq, adt, ag, const1, const2,  &
                dlt, m, n, p, nwtake, ierr, tol, optim)
  END IF
ELSE IF(a <= zero .AND. b <= zero) THEN
  IF(c <= d) THEN
    q = c
    ac = a
    CALL tslmin(sp, s1, ac, q, anls, fq, adt, ag, const1, const2,  &
                dlt, m, n, p, nwtake, ierr, tol, optim)
  ELSE
    optim = s1
    q = d
  END IF
ELSE IF(a >= zero .AND. b >= zero) THEN
  IF(c >= d) THEN
    q = d
    bc = b
    CALL tslmin(s1, sp, bc, q, anls, fq, adt, ag, const1, const2,  &
                dlt, m, n, p, nwtake, ierr, tol, optim)
  ELSE
    optim = sp
    q = c
  END IF
END IF

! update the global minimizer of FALPHA so far

IF(first) THEN
  IF(a > zero .AND. b < zero) THEN
    glopt = c
    sol = sp
    IF(c > d) THEN
      glopt = d
      sol = s1
    END IF
  ELSE
    glopt = q
    sol = optim
  END IF
  first = .false.
ELSE IF(glopt >= q) THEN
  glopt = q
  sol = optim
END IF

s = s + incr
IF(s <= eloop) GO TO 10

RETURN
END SUBROUTINE tssmin



SUBROUTINE tssmrd(vect, resnew, x, mu, ierr, m, n)

REAL (dp), INTENT(IN)   :: vect(:)
REAL (dp), INTENT(OUT)  :: resnew
REAL (dp), INTENT(IN)   :: x(:)
REAL (dp), INTENT(IN)   :: mu
INTEGER, INTENT(IN)     :: ierr
INTEGER, INTENT(IN)     :: m
INTEGER, INTENT(IN)     :: n

!**********************************************************************
! THIS ROUTINE COMPUTES THE RESIDUAL OF THE STANDARD MODEL.
!**********************************************************************

!    INPUT PARAMETERS :
!    -----------------

!    VECT : RIGHT HAND SIDE VECTOR OF THE NEWTON/GAUSS-NEWTON
!           EQUATIONS AFTER BEING MULTIPLIED BY ORTHOGONAL MATRICES
!           (SEE SUBROUTINE TSCPSS)
!    X    : STANDARD STEP COMPUTED BY THE SUBROUTINE TSCPSS
!    MU   : A SMALL PERTURBATION USED IN COMPUTING THE STANDARD
!           STEP WHEN THE JACOBIAN IS SINGULAR
!    IERR : RETURN CODE WITH THE FOLLOWING MEANINGS :
!           IERR = 0 : NO SINGULARITY OF JACOBIAN DETECTED
!           IERR = 1 : OTHERWISE
!    M,N  : DIMENSION OF PROBLEM

!    OUTPUT PARAMETERS :
!    ------------------

!    RESNEW : RESIDUAL OF THE STANDARD MODEL

!    SUBPROGRAMS CALLED:

!    LEVEL 1 BLAS  ...  DNRM2

!**********************************************************************

REAL (dp) :: temp, prod

IF(ierr == 0) THEN
  IF(m == n) THEN
    resnew = zero
  ELSE
    resnew = dnrm2(m-n, vect(n+1:), 1)
  END IF
ELSE
  temp = dnrm2(m, vect(n+1:), 1)**2
  prod = mu * dnrm2(n, x, 1)**2
  resnew = SQRT(temp-prod)
END IF

RETURN
END SUBROUTINE tssmrd



SUBROUTINE tssqp1(aja, anls, s, f, m, n, q, root, restns)

REAL (dp), INTENT(IN)   :: aja(:,:)
REAL (dp), INTENT(IN)   :: anls(:,:)
REAL (dp), INTENT(IN)   :: s(:,:)
REAL (dp), INTENT(IN)   :: f(:)
INTEGER, INTENT(IN)     :: m
INTEGER, INTENT(IN)     :: n
INTEGER, INTENT(IN)     :: q
REAL (dp), INTENT(OUT)  :: root
REAL (dp), INTENT(OUT)  :: restns

!**********************************************************************
! THIS ROUTINE SOLVES THE LOWER M-N+Q QUADRATIC EQUATIONS IN P UNKNOWNS
! OF THE TENSOR MODEL WHEN Q > 1 AND P = 1.
!**********************************************************************

!    INPUT PARAMETERS :
!    -----------------

!    AJA  : JACOBIAN MATRIX AT CURRENT ITERATE
!    ANLS : TENSOR TERM MATRIX AT CURRENT ITERATE
!    S    : MATRIX OF PAST LINEARLY INDEPENDENT DIRECTIONS
!    F    : FUNCTION VALUE AT CURRENT ITERATE MULTIPLIED BY AN
!           ORTHOGONAL MATRIX
!    M,N  : ROW AND COLUMN DIMENSIONS OF AJA
!    Q    : NUMERICAL RANK OF JACOBIAN :
!           Q > P : JACOBIAN IS SINGULAR
!           Q = P : OTHERWISE

!    OUTPUT PARAMETERS :
!    -----------------

!    ROOT   : SOLUTION TO THE SYSTEM
!    RESTNS : TENSOR RESIDUAL

!    SUBPROGRAMS CALLED:

!    LEVEL 1 BLAS  ...  DNRM2

! **********************************************************************

INTEGER   :: i
REAL (dp) :: temp, a, b, c, d, res1, res2, res3, res, s1, s2, s3
REAL (dp) :: t, t0, t1, t2, t3, a1, a2, a3, theta, onetrd
REAL (dp), PARAMETER :: quart = 0.25_dp, half = 0.5_dp, two = 2.0_dp,  &
                        three = 3.0_dp, four = 4.0_dp, nine = 9.0_dp
REAL (dp), PARAMETER :: tseven = 27.0_dp, ffour = 54.0_dp
REAL (dp), PARAMETER :: pi = 3.141592653589793_dp

! compute the coefficients of a third degree polynomial

onetrd = one/three
a = zero
b = zero
c = zero

temp = dnrm2(m-n+q, f(n-q+1:), 1)**2
d = two * DOT_PRODUCT( aja(n-q+1:m,n), f(n-q+1:m) )
t0 = s(n+2,1)**2
t1 = t0**2
DO i = n-q+1,m
  t2 = aja(i,n)
  t3 = anls(i,1) * t0
  c = c + two * (t2**2 + f(i) * t3)
  b = b + three * t2 * t3
  a = a + anls(i,1)**2 * t1
END DO

! compute the roots of the third degree polynomial

IF(a == zero) THEN
  IF(b /= zero) THEN
    t0 = SQRT(c**2 - four*b*d)
    t1 = two*b
    s1 = (-c + t0)/t1
    s2 = (-c - t0)/t1
    res1 = ABS(temp + d*s1 + half*c*(s1**2) + onetrd*b*(s1**3))
    res2 = ABS(temp + d*s2 + half*c*(s2**2) + onetrd*b*(s2**3))
    IF(res1 > res2) THEN
      root =  s2
      res  =  res2
    ELSE
      root =  s1
      res  =  res1
    END IF
    restns  =  SQRT(res)
    RETURN
  ELSE IF(c /= zero) THEN
    root = -d/c
    res = ABS(temp + d*root + half*c*(root**2))
    restns = SQRT(res)
    RETURN
  ELSE
    root = zero
    restns = SQRT(temp)
    RETURN
  END IF
ELSE IF(d == zero) THEN
  root = zero
  restns = SQRT(temp)
  RETURN
END IF
t3 = d

a1 = b/a
a2 = c/a
a3 = d/a
t0 = (three*a2 - a1**2)/nine
t1 = (nine*a1*a2 - tseven*a3 - two*a1**3)/ffour
d = t0**3 + t1**2

IF(d > zero) THEN
  t2 = t1 + SQRT(d)
  t = t1 - SQRT(d)
  IF(t < zero) THEN
    t = -(-t)**onetrd
  ELSE
    t = t**onetrd
  END IF
  IF(t2 < zero)THEN
    t2 = -(-t2)**onetrd
  ELSE
    t2 = t2**onetrd
  END IF
  s1 = t2 + t - a1/three
  s3 = s1
  s2 = s1
ELSE
  t = t1/SQRT(-t0**3)
  theta = ACOS(t)
  theta = theta/three
  t = two*SQRT(-t0)
  s1 = t*COS(theta) - a1/three
  s2 = t*COS(theta + pi*two/three) - a1/three
  s3 = t*COS(theta + pi*four/three) - a1/three
END IF

! compute the tensor residual for each root

res1 = ABS(temp + t3*s1 + half*c*(s1**2) + onetrd*b*(s1**3)+ quart*a*(s1**4))
res2 = ABS(temp + t3*s2 + half*c*(s2**2) + onetrd*b*(s2**3)+ quart*a*(s2**4))
res3 = ABS(temp + t3*s3 + half*c*(s3**2) + onetrd*b*(s3**3)+ quart*a*(s3**4))

! select the root that produces the smallest tensor residual

res = res1
root = s1
IF(res > res2) THEN
  res = res2
  root = s2
END IF
IF(res > res3) THEN
  res = res3
  root = s3
END IF
restns = SQRT(res)

RETURN
END SUBROUTINE tssqp1



SUBROUTINE tssstp(aja, fn, m, n, epsm, iglobl, dn, fq, pivot, pbar, ierr)

REAL (dp), INTENT(IN OUT)  :: aja(:,:)
REAL (dp), INTENT(IN OUT)  :: fn(:)
INTEGER, INTENT(IN)        :: m
INTEGER, INTENT(IN)        :: n
REAL (dp), INTENT(IN)      :: epsm
INTEGER, INTENT(IN)        :: iglobl
REAL (dp), INTENT(OUT)     :: dn(:)
REAL (dp), INTENT(OUT)     :: fq(:)
INTEGER, INTENT(OUT)       :: pivot(:)
INTEGER, INTENT(OUT)       :: pbar(:)
INTEGER, INTENT(OUT)       :: ierr

!**********************************************************************
! THIS ROUTINE FINDS THE STANDARD STEP WHEN THE ITERATION NUMBER IS
! EQUAL TO 1 OR THE INPUT PARAMETER "METHOD" IS SET TO 0.
!**********************************************************************

!    INPUT PARAMETERS :
!    -----------------

!    AJA   : JACOBIAN MATRIX AT CURRENT ITERATE
!    FN    : FUNCTION VALUE AT CURRENT ITERATE
!    M,N   : DIMENSIONS OF PROBLEM
!    EPSM  : MACHINE EPSILON
!    IGLOBL: GLOBAL STRATEGY USED :
!             = 0 : LINE SEARCH USED
!             = 1 : 2-DIMENSIONAL TRUST REGION USED

!    OUTPUT PARAMETERS :
!    ------------------

!    DN : STANDARD STEP
!    FQ : FUNCTION VALUE AT CURRENT ITERATE MULTIPLIED BY
!           ORTHOGONAL MATRICES (THIS IS USED IN THE CASE WHERE
!           THE GLOBAL STRATEGY IS THE 2-DIMENSIONAL TRUST REGION)
!    PIVOT,PBAR : PIVOT VECTORS
!    IERR : RETURNED CODE WITH THE FOLLOWING MEANING :
!           IERR  =  1 : SINGULARITY OF JACOBIAN DETECTED (ZERO1 IS USED TO
!                        KEEP TRACK OF THE FIRST COLUMN WHERE SINGULARITY IS
!                        DETECTED)
!           IERR  =  0 : OTHERWISE

!    SUBPROGRAMS CALLED:

!    TENSOLVE      ...  TSQRFC,TSQMUV,TSBSLV,TSPRMV,TSCPMU

!**********************************************************************

! Workspace
REAL (dp) :: y(n), w(m+n)

INTEGER   :: zero1, zerotm, i, j
REAL (dp) :: mu

w(m+1:m+n)=0 ! modified by nicola : missing initialisation

! perform a QR factorization of AJA
CALL tsqrfc(aja, n, m, 1, n, ierr, epsm, pivot, zero1)

fn(1:m) = -fn(1:m)

IF(ierr == 0) THEN
  IF(m == n) THEN
    zerotm = zero1-1
  ELSE
    zerotm = zero1
  END IF

! multiply (-FN) by the orthogonal matrix resulting from the QR
! decomposition of AJA

  CALL tsqmuv(aja, fn, w, m, 1, zerotm, .false.)

! solve AJA*DN  =  W

  CALL tsbslv(aja, m, n, w, y)
  CALL tsprmv(dn, y, pivot, n, 0)

  IF(iglobl == 1) THEN
    ierr = 0
    fq(1:m) = -w(1:m)
  END IF
  RETURN
ELSE

! AJA is singular
  CALL tsqmuv(aja, fn, w, m, 1, zero1, .false.)

! solve ( AJA-trans AJA + MU I ) DN = -AJA-trans FN

! put the diagonal elements stored in row m+2 of AJA into their
! proper positions and zero out the unwanted portions of AJA

  DO j = 1, zero1-1
    aja(j,j) = aja(m+2,j)
    aja(j+1:m+n,j) = zero
  END DO

  DO j = zero1, n
    aja(zero1:m+n,j) = zero
  END DO

! compute a perturbation MU

  CALL tscpmu(aja, n, epsm, mu)

! form the augmented Jacobian matrix by adding an nxn diag(mu) in
! the bottom of AJA

  DO i = m+1, m+n
    aja(i,i-m) = mu
  END DO

! factorize the transformed matrix AJA from 1 to n and compute
! the standard step DN
  CALL tsqrfc(aja, n, m+n, 1, n, ierr, epsm, pbar, zero1)
  CALL tsqmuv(aja, w, fq, m+n, 1, n+1, .false.)
  CALL tsbslv(aja, m+n, n, fq, dn)
  CALL tsprmv(y, dn, pbar, n, 0)
  CALL tsprmv(dn, y, pivot, n, 0)
END IF

IF(iglobl == 1) THEN
  ierr = 1
  fq(1:m+n) = - fq(1:m+n)
END IF

RETURN
END SUBROUTINE tssstp



SUBROUTINE tsstmx(s, x, n, p, wrk2)

REAL (dp), INTENT(IN)   :: s(:,:)
REAL (dp), INTENT(IN)   :: x(:)
INTEGER, INTENT(IN)     :: n
INTEGER, INTENT(IN)     :: p
REAL (dp), INTENT(OUT)  :: wrk2(:)

!*********************************************************************
! THIS ROUTINE COMPUTES SHAT-TRANS * X, WHERE SHAT IS AN UPSIDE DOWN
! TRIANGULAR MATRIX RESULTING FROM A QL FACTORIZATION OF A MATRIX
! A AND X IS A VECTOR.
!*********************************************************************

!   INPUT PARAMETERS :
!   -----------------

!   SHAT  : UPSIDE DOWN TRIANGULAR MATRIX RESULTING FROM A QL FACTORIZATION
!   X     : VECTOR TO BE MULTIPLIED BY SHAT
!   N     : ROW DIMENSION OF THE MATRIX A
!   P     : COLUMN DIMENSION OF SHAT

!   OUTPUT PARAMETERS :
!   -----------------

!   WRK2  :  SHAT-TRANS * X

!*********************************************************************

! Workspace
REAL (dp) :: wrk1(p)

INTEGER   :: col

wrk1(1:p) = zero

wrk2(1) = s(n+2,1) * x(p)
IF(p > 1) THEN
  wrk1(p) = s(n,2)
  wrk1(p-1) = s(n+2,2)
  wrk2(2) = DOT_PRODUCT( wrk1, x(1:p) )
  DO col = 3, p
    wrk1(p-col+2:p) = s(n-col+2:n,col)
    wrk1(p-col+1) = s(n+2,col)
    wrk2(col) = DOT_PRODUCT( wrk1, x(1:p) )
  END DO
END IF

RETURN
END SUBROUTINE tsstmx



SUBROUTINE tstrud(m, n, x, f, g, sc, nwtake, stepmx, steptl, dlt, mxtake, dxn, &
                  dfn, fvec, scres, iretcd, xplsp, fplsp, fprev, xpls, fp, fpls)

INTEGER, INTENT(IN)        :: m
INTEGER, INTENT(IN)        :: n
REAL (dp), INTENT(IN)      :: x(:)
REAL (dp), INTENT(IN)      :: f
REAL (dp), INTENT(IN)      :: g(:)
REAL (dp), INTENT(IN)      :: sc(:)
LOGICAL, INTENT(IN)        :: nwtake
REAL (dp), INTENT(IN)      :: stepmx
REAL (dp), INTENT(IN)      :: steptl
REAL (dp), INTENT(IN OUT)  :: dlt
LOGICAL, INTENT(OUT)       :: mxtake
REAL (dp), INTENT(IN)      :: dxn(:)
REAL (dp), INTENT(IN)      :: dfn(:)
REAL (dp), INTENT(IN)      :: scres
INTEGER, INTENT(IN OUT)    :: iretcd
REAL (dp), INTENT(IN OUT)  :: xplsp(:)
REAL (dp), INTENT(IN OUT)  :: fplsp
REAL (dp), INTENT(IN OUT)  :: fprev(:)
REAL (dp), INTENT(OUT)     :: xpls(:)
REAL (dp), INTENT(OUT)     :: fp(:)
REAL (dp), INTENT(OUT)     :: fpls

INTERFACE
  SUBROUTINE fvec(x, f, m, n) bind(c)
    IMPLICIT NONE
    INTEGER, PARAMETER     :: dp = SELECTED_REAL_KIND(14, 60)
    REAL (dp), INTENT(IN)  :: x(n)
    REAL (dp), INTENT(OUT) :: f(m)
    INTEGER, INTENT(IN)    :: m, n
  END SUBROUTINE fvec
END INTERFACE

!***********************************************************************
! THIS ROUTINE DECIDES WHETHER TO ACCEPT XPLS=X+SC AS THE NEXT ITERATE
! AND UPDATES THE TRUST REGION DLT.
!***********************************************************************

! PARAMETERS
! ----------
! M,N          --> DIMENSIONS OF PROBLEM
! X(N)         --> OLD ITERATE X[K-1]
! F            --> 0.50D0 * || FC ||**2
! G(N)         --> GRADIENT AT OLD ITERATE, G(X), OR APPROXIMATE
! SC(N)        --> CURRENT STEP
! NWTAKE       --> BOOLEAN, =.TRUE. IF INPUT STEP TAKEN
! STEPMX       --> MAXIMUM ALLOWABLE STEP SIZE
! STEPTL       --> RELATIVE STEP SIZE AT WHICH SUCCESSIVE ITERATES
!                  CONSIDERED CLOSE ENOUGH TO TERMINATE ALGORITHM
! DLT         <--> TRUST REGION RADIUS
! MXTAKE      <--  BOOLEAN FLAG INDICATING STEP OF MAXIMUM LENGTH USED
! DXN         --->DIAGONAL SCALING MATRIX FOR X
! DFN         --->DIAGONAL SCALING MATRIX FOR F
! FVEC        --->SUBROUTINE TO EVALUATE FUNCTION

! IRETCD      <--> RETURN CODE
!                    =0 XPLS ACCEPTED AS NEXT ITERATE;
!                       DLT TRUST REGION FOR NEXT ITERATION.
!                    =1 XPLS UNSATISFACTORY BUT ACCEPTED AS NEXT ITERATE
!                       BECAUSE XPLS-X < SMALLEST ALLOWABLE STEP LENGTH.
!                    =2 F(XPLS) TOO LARGE.  CONTINUE CURRENT ITERATION
!                       WITH NEW REDUCED DLT.
!                    =3 F(XPLS) SUFFICIENTLY SMALL, BUT QUADRATIC MODEL
!                       PREDICTS F(XPLS) SUFFICIENTLY WELL TO CONTINUE
!                       CURRENT ITERATION WITH NEW DOUBLED DLT.
! XPLSP(N)    <--> WORKSPACE [VALUE NEEDS TO BE RETAINED BETWEEN
!                  SUCCESSIVE CALLS OF K-TH GLOBAL STEP]
! FPLSP       <--> [RETAIN VALUE BETWEEN SUCCESSIVE CALLS]
! FPREV       ---> WORKING VECTOR
! XPLS(N)     <--  NEW ITERATE X[K]
! FP          <--  FUNCTION VALUE AT NEXT ITERATE
! FPLS        <--  FUNCTION VALUE AT NEW ITERATE, F(XPLS)

!    SUBPROGRAMS CALLED:

!    LEVEL 1 BLAS  ...  DNRM2
!    TENSOLVE      ...  TSFSCL

!**********************************************************************

INTEGER              :: i
REAL (dp)            :: stepln, dltfp, slope, dltf, slp, pq, rln, dltmp
REAL (dp), PARAMETER :: tenth = 0.1_dp, half = 0.5_dp, znn = 0.99_dp,  &
                        two = 2.0_dp

mxtake = .false.
xpls(1:n) = x(1:n) + sc(1:n)
stepln = dnrm2(n, sc, 1)
CALL tsfscl(xpls, dxn, dfn, m, n, fvec, fp)
fpls = half*dnrm2(m, fp, 1)**2
dltf = fpls - f
slope = DOT_PRODUCT( g(1:n), sc(1:n) )
slp = half*scres**2 - f

! next statement added for case of compilers which do not optimize
! evaluation of next "IF" statement (in which case fplsp could be undefined)

IF(iretcd == 4) fplsp = zero
IF(iretcd /= 3 .OR. (fpls < fplsp .AND. dltf <= 1.e-4_dp*slp)) GO TO 130

!       reset XPLS to XPLSP and terminate global step

iretcd = 0
xpls(1:n) = xplsp(1:n)
fpls = fplsp
dlt = half*dlt
fp(1:m) = fprev(1:m)
GO TO 230

!       FPLS too large

130 IF(dltf <= 1.e-4_dp*slp) GO TO 170
pq = one
rln = zero
DO i = 1,n
  rln = MAX(rln,ABS(sc(i)) / MAX(ABS(xpls(i)),one/pq))
END DO
IF(rln >= steptl) GO TO 150

!           cannot find satisfactory XPLS sufficiently distinct from X

iretcd = 1
GO TO 230

!           reduce trust region and continue global step

150 iretcd = 2
dltmp = -slope*stepln/(two*(dltf-slope))
IF(dltmp >= tenth*dlt) GO TO 155

dlt = tenth*dlt
GO TO 230

155 IF(dltmp > half*dlt) THEN
  dlt = half*dlt
ELSE
  dlt = dltmp
END IF

!         FPLS sufficiently small

170 dltfp = half*scres**2-f
IF(iretcd == 2 .OR. (ABS(dltfp-dltf) > tenth*ABS(dltf)  &
    .AND. dltfp > slope) .OR. nwtake .OR. dlt > znn*stepmx) GO TO 210

!           double trust region and continue global step

iretcd = 3
xplsp(1:n) = xpls(1:n)
fplsp = fpls
dlt = MIN(two*dlt,stepmx)
fprev(1:m) = fp(1:m)
GO TO 230

!           accept XPLS as next iterate.  Choose new trust region.

210 iretcd = 0
IF(dlt > znn*stepmx) mxtake = .true.
IF(dltf < tenth*dltfp) GO TO 220

!             Decrease trust region for next iteration

dlt = half*dlt
GO TO 230
!             Check whether to increase trust region for next iteration

220 IF(dltf <= .75_dp*dltfp) dlt = MIN(two*dlt, stepmx)
230 RETURN

END SUBROUTINE tstrud



SUBROUTINE tsudqv(shat, wrk1, n, p, const1)

REAL (dp), INTENT(IN)   :: shat(:,:)
REAL (dp), INTENT(IN)   :: wrk1(:)
INTEGER, INTENT(IN)     :: n
INTEGER, INTENT(IN)     :: p
REAL (dp), INTENT(OUT)  :: const1(:)

!**********************************************************************
! THIS ROUTINE COMPUTES SHAT-TRANS * WRK1, WHERE SHAT IS AN UPSIDE
! DOWN TRIANGULAR MATRIX RESULTING FROM A QL FACTORIZATION OF A
! MATRIX A AND WRK1 IS A VECTOR OF LENGTH N.
!**********************************************************************

!  INPUT PARAMETERS
!  ----------------

!      SHAT : UPSIDE DOWN TRIANGULAR MATRIX RESULTING FROM A QL FACTORIZATION
!      WRK1 : VECTOR TO BE MULTIPLIED BY SHAT
!      N    : DIMENSION OF MATRIX A
!      P    : COLUMN DIMENSION OF SHAT

!  OUTPUT PARAMETERS
!  -----------------

!      CONST1 : SHAT * WRK1

! **********************************************************************

INTEGER   :: j

const1(1) = shat(n+2,1) * wrk1(n)
IF(p > 1) THEN
  const1(2) = shat(n,2) * wrk1(n) + shat(n+2,2) * wrk1(n-1)
END IF
DO j = 3,p
  const1(j) = DOT_PRODUCT( shat(n-j+2:n,j), wrk1(n-j+2:n) ) +  &
              shat(n+2,j)*wrk1(n-j+1)
END DO

RETURN
END SUBROUTINE tsudqv



SUBROUTINE tsunsf(f,df,m)

REAL (dp), INTENT(IN OUT)  :: f(:)
REAL (dp), INTENT(IN)      :: df(:)
INTEGER, INTENT(IN)        :: m

!*********************************************************************
! THIS ROUTINE UNSCALES A FUNCTION VALUE F.
!*********************************************************************

!    INPUT PARAMETERS :
!    ------------------

!    DF : DIAGONAL SCALING MATRIX FOR F
!    M  : DIMENSION OF F

!    INPUT-OUTPUT PARAMETERS :
!    ------------------

!    F  : SCALED FUNCTION VALUE ON ENTRY AND UNSCALED FUNCTION
!         VALUE ON EXIT

!**********************************************************************

f(1:m) = f(1:m) / df(1:m)

RETURN
END SUBROUTINE tsunsf



SUBROUTINE tsunsx(x, dx, n)

REAL (dp), INTENT(IN OUT)  :: x(:)
REAL (dp), INTENT(IN)      :: dx(:)
INTEGER, INTENT(IN)        :: n

!**********************************************************************
! THIS ROUTINE UNSCALES A VECTOR X.
!**********************************************************************

!    INPUT PARAMETERS :
!    ------------------

!    DX : DIAGONAL SCALING MATRIX FOR X
!    N  : DIMENSION OF X

!    OUTPUT PARAMETERS :
!    ------------------

!    X  : UNSCALED VECTOR X

!**********************************************************************

x(1:n) = x(1:n) / dx(1:n)

RETURN
END SUBROUTINE tsunsx



SUBROUTINE tsupsf(fc, xc, xp, sqrn, itn, m, n, s, fv)

REAL (dp), INTENT(IN OUT)  :: fc(:)
REAL (dp), INTENT(IN)      :: xc(:)
REAL (dp), INTENT(IN)      :: xp(:)
INTEGER, INTENT(IN)        :: sqrn
INTEGER, INTENT(IN)        :: itn
INTEGER, INTENT(IN)        :: m
INTEGER, INTENT(IN)        :: n
REAL (dp), INTENT(OUT)     :: s(:,:)
REAL (dp), INTENT(IN OUT)  :: fv(:,:)

!**********************************************************************
! THIS ROUTINE UPDATE THE MATRIX S OF PAST DIRECTIONS AND THE MATRIX
! FV OF FUNCTION VALUES.
!**********************************************************************

!    INPUT PARAMETERS :
!    ----------------

!    FC  : FUNCTION VALUE AT CURRENT ITERATE
!    XC  : CURRENT ITERATE X[K-1]
!    XP  : NEW ITERATE X[K]
!    SQRN: MAXIMUM COLUMN DIMENSION OF S AND FV
!    ITN : ITERATION NUMBER
!    M   : ROW DIMENSION OF MATRIX FV
!    N   : ROW DIMENSION OF MATRIX S
!    STEP: WORKING VECTOR


!    INPUT-OUTPUT PARAMETERS :
!     -----------------------

!    S   :  MATRIX OF PAST DIRECTIONS (XK - XC)
!    FV  :  MATRIX OF PAST FUNCTIONS VALUES

!**********************************************************************

INTEGER   :: j, l
REAL (dp) :: step(n)

! update FV

IF(sqrn < itn) THEN
  l = sqrn
ELSE
  l = itn
END IF
DO j = l-1, 1, -1
  fv(1:m,j+1) = fv(1:m,j)
END DO

fv(1:m,1) = fc(1:m)

! update S

step(1:n) = xc(1:n) - xp(1:n)

DO j = l-1,1,-1
  s(1:n,j+1) = s(1:n,j) + step(1:n)
END DO
s(1:n,1) = step(1:n)

RETURN
END SUBROUTINE tsupsf



SUBROUTINE tsutmd(aja, d, m, n, v)

REAL (dp), INTENT(IN)   :: aja(:,:)
REAL (dp), INTENT(IN)   :: d(:)
INTEGER, INTENT(IN)     :: m
INTEGER, INTENT(IN)     :: n
REAL (dp), INTENT(OUT)  :: v(:)

!**********************************************************************
! THIS ROUTINE MULTIPLIES AN UPPER TRIANGULAR MATRIX (AS STORED IN
! STEWART) BY A VECTOR D.
!**********************************************************************

!    INPUT PARAMETERS :
!    -----------------

!    AJA  : JACOBIAN AT CURRENT ITERATE
!    D    : VECTOR TO BE MULTIPLIED BY AJA
!    M,N  : DIMENSIONS OF PROBLEM

!    OUTPUT PARAMETERS :
!    -----------------

!    V  : VECTOR AJA * D ON EXIT

!**********************************************************************

INTEGER :: j

v(1) = aja(m+2,1) * d(1) + aja(1,2) * d(2)
v(2) = aja(m+2,2) * d(2)
DO j = 3, n
  v(j) = aja(m+2,j) * d(j)
  v(1:j-1) = v(1:j-1) + d(j) * aja(1:j-1,j)
END DO

RETURN
END SUBROUTINE tsutmd

END MODULE tensolve
