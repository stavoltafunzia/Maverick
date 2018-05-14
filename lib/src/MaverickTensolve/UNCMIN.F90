MODULE unconstrained_min

! UNCMIN (R. B. Schnabel, J. E. Koontz, and B. E. Weiss,
! "A Modular System of Algorithms of Unconstrained Minimization",
! ACM Trans. Math. Softw., 11 (1985), 419-440).

! Translated to free-format Fortran 90 style by Alan Miller
! Alan.Miller @ vic.cmis.csiro.au
! Latest revision - 10 November 2003

! 24 May 2001: Automatic array wk(n) added to routine lltslv.
! 10 Nov. 2003: Removed the PRIVATE declaration from dp

USE blas_part
IMPLICIT NONE

INTEGER, PARAMETER            :: dp = SELECTED_REAL_KIND(14, 60)
REAL (dp), PARAMETER, PRIVATE :: zero = 0.0_dp, one = 1.0_dp

CONTAINS


!****************************** uncmin.f *********************

SUBROUTINE bakslv(n, a, x, b)

! PURPOSE
! -------
! SOLVE  AX=B  WHERE A IS UPPER TRIANGULAR MATRIX.
! NOTE THAT A IS INPUT AS A LOWER TRIANGULAR MATRIX AND
! THAT THIS ROUTINE TAKES ITS TRANSPOSE IMPLICITLY.

! PARAMETERS
! ----------
! N            --> DIMENSION OF PROBLEM
! A(N,N)       --> LOWER TRIANGULAR MATRIX (PRESERVED)
! X(N)        <--  SOLUTION VECTOR
! B(N)         --> RIGHT-HAND SIDE VECTOR

! NOTE
! ----
! IF B IS NO LONGER REQUIRED BY CALLING ROUTINE,
! THEN VECTORS B AND X MAY SHARE THE SAME STORAGE.

INTEGER, INTENT(IN)     :: n
REAL (dp), INTENT(IN)   :: a(:,:)
REAL (dp), INTENT(OUT)  :: x(:)
REAL (dp), INTENT(IN)   :: b(:)

INTEGER   :: i,ip1
REAL (dp) :: sum

! SOLVE (L-TRANSPOSE)X=B. (BACK SOLVE)

i = n
x(i) = b(i) / a(i,i)
IF(n == 1) RETURN

DO
  ip1=i
  i=i-1
  sum = DOT_PRODUCT( a(ip1:n,i), x(ip1:n) )
  x(i) = (b(i) - sum)/a(i,i)
  IF(i <= 1) EXIT
END DO

RETURN
END SUBROUTINE bakslv



SUBROUTINE choldc(n, a, diagmx, tol, addmax)

! PURPOSE
! -------
! FIND THE PERTURBED L(L-TRANSPOSE) [WRITTEN LL+] DECOMPOSITION
! OF A+D, WHERE D IS A NON-NEGATIVE DIAGONAL MATRIX ADDED TO A IF
! NECESSARY TO ALLOW THE CHOLESKY DECOMPOSITION TO CONTINUE.

! PARAMETERS
! ----------
! N            --> DIMENSION OF PROBLEM
! A(N,N)      <--> ON ENTRY: MATRIX FOR WHICH TO FIND PERTURBED
!                       CHOLESKY DECOMPOSITION
!                  ON EXIT:  CONTAINS L OF LL+ DECOMPOSITION
!                  IN LOWER TRIANGULAR PART AND DIAGONAL OF "A"
! DIAGMX       --> MAXIMUM DIAGONAL ELEMENT OF "A"
! TOL          --> TOLERANCE
! ADDMAX      <--  MAXIMUM AMOUNT IMPLICITLY ADDED TO DIAGONAL OF "A"
!                  IN FORMING THE CHOLESKY DECOMPOSITION OF A+D
! INTERNAL VARIABLES
! ------------------
! AMINL    SMALLEST ELEMENT ALLOWED ON DIAGONAL OF L
! AMNLSQ   = AMINL**2
! OFFMAX   MAXIMUM OFF-DIAGONAL ELEMENT IN COLUMN OF A

! DESCRIPTION
! -----------
! THE NORMAL CHOLESKY DECOMPOSITION IS PERFORMED.  HOWEVER, IF AT ANY
! POINT THE ALGORITHM WOULD ATTEMPT TO SET L(I,I)=SQRT(TEMP)
! WITH TEMP < TOL*DIAGMX, THEN L(I,I) IS SET TO SQRT(TOL*DIAGMX)
! INSTEAD.  THIS IS EQUIVALENT TO ADDING TOL*DIAGMX-TEMP TO A(I,I)

INTEGER, INTENT(IN)              :: n
REAL (dp), INTENT(IN OUT)        :: a(:,:)
REAL (dp), INTENT(IN)            :: diagmx
REAL (dp), INTENT(IN)            :: tol
REAL (dp), INTENT(OUT), OPTIONAL :: addmax

INTEGER   :: j, i
REAL (dp) :: aminl, total, temp, amnlsq, offmax

IF (PRESENT(addmax)) addmax = zero
aminl = SQRT(diagmx*tol)
amnlsq = aminl*aminl

! FORM COLUMN J OF L

DO j = 1,n
! FIND DIAGONAL ELEMENTS OF L
  temp = a(j,j) - SUM( a(j,1:j-1)**2 )
  IF(temp > amnlsq) THEN
    a(j,j) = SQRT(temp)
  ELSE

! FIND MAXIMUM OFF-DIAGONAL ELEMENT IN COLUMN
    offmax = zero
    DO i = j+1,n
      IF(ABS(a(i,j)) > offmax) offmax = ABS(a(i,j))
    END DO
    IF(offmax <= amnlsq) offmax = amnlsq

! ADD TO DIAGONAL ELEMENT  TO ALLOW CHOLESKY DECOMPOSITION TO CONTINUE
    a(j,j) = SQRT(offmax)
    IF (PRESENT(addmax)) addmax = MAX(addmax,offmax-temp)
  END IF

! FIND I,J ELEMENT OF LOWER TRIANGULAR MATRIX
  DO i = j+1,n
    total = DOT_PRODUCT( a(i,1:j-1), a(j,1:j-1) )
    a(i,j) = (a(i,j) - total) / a(j,j)
  END DO
END DO

RETURN
END SUBROUTINE choldc



SUBROUTINE chlhsn(n, a, epsm, sx, udiag)

! PURPOSE
! -------
! FIND THE L(L-TRANSPOSE) [WRITTEN LL+] DECOMPOSITION OF THE PERTURBED
! MODEL HESSIAN MATRIX A + MU*I (WHERE MU\=0 AND I IS THE IDENTITY MATRIX)
! WHICH IS SAFELY POSITIVE DEFINITE.  IF A IS SAFELY POSITIVE DEFINITE
! UPON ENTRY, THEN MU = 0.

! PARAMETERS
! ----------
! N            --> DIMENSION OF PROBLEM
! A(N,N)      <--> ON ENTRY; "A" IS MODEL HESSIAN (ONLY LOWER
!                  TRIANGULAR PART AND DIAGONAL STORED)
!                  ON EXIT:  A CONTAINS L OF LL+ DECOMPOSITION OF PERTURBED
!                  MODEL HESSIAN IN LOWER TRIANGULAR PART AND DIAGONAL
!                  AND CONTAINS HESSIAN IN UPPER TRIANGULAR PART AND UDIAG
! EPSM         --> MACHINE EPSILON
! SX(N)        --> DIAGONAL SCALING MATRIX FOR X
! UDIAG(N)    <--  ON EXIT: CONTAINS DIAGONAL OF HESSIAN

! INTERNAL VARIABLES
! ------------------
! TOL              TOLERANCE
! DIAGMN           MINIMUM ELEMENT ON DIAGONAL OF A
! DIAGMX           MAXIMUM ELEMENT ON DIAGONAL OF A
! OFFMAX           MAXIMUM OFF-DIAGONAL ELEMENT OF A
! OFFROW           SUM OF OFF-DIAGONAL ELEMENTS IN A ROW OF A
! EVMIN            MINIMUM EIGENVALUE OF A
! EVMAX            MAXIMUM EIGENVALUE OF A

! DESCRIPTION
! -----------
! 1. IF "A" HAS ANY NEGATIVE DIAGONAL ELEMENTS, THEN CHOOSE MU>0
! SUCH THAT THE DIAGONAL OF A:=A+MU*I IS ALL POSITIVE
! WITH THE RATIO OF ITS SMALLEST TO LARGEST ELEMENT ON THE ORDER OF SQRT(EPSM).

! 2. "A" UNDERGOES A PERTURBED CHOLESKY DECOMPOSITION WHICH RESULTS IN AN LL+
! DECOMPOSITION OF A+D, WHERE D IS A NON-NEGATIVE DIAGONAL MATRIX WHICH IS
! IMPLICITLY ADDED TO "A" DURING THE DECOMPOSITION IF "A" IS NOT POSITIVE
! DEFINITE.
! "A" IS RETAINED AND NOT CHANGED DURING THIS PROCESS BY COPYING L INTO THE
! UPPER TRIANGULAR PART OF "A" AND THE DIAGONAL INTO UDIAG.  THEN THE CHOLESKY
! DECOMPOSITION ROUTINE IS CALLED.  ON RETURN, ADDMAX CONTAINS MAXIMUM ELEMENT
! OF D.

! 3. IF ADDMAX = 0, "A" WAS POSITIVE DEFINITE GOING INTO STEP 2 AND RETURN
! IS MADE TO CALLING PROGRAM.  OTHERWISE, THE MINIMUM NUMBER SDD WHICH MUST
! BE ADDED TO THE DIAGONAL OF A TO MAKE IT SAFELY STRICTLY DIAGONALLY DOMINANT
! IS CALCULATED.  SINCE A+ADDMAX*I AND A+SDD*I ARE SAFELY POSITIVE DEFINITE,
! CHOOSE MU = MIN(ADDMAX,SDD) AND DECOMPOSE A+MU*I TO OBTAIN L.

INTEGER, INTENT(IN)        :: n
REAL (dp), INTENT(IN OUT)  :: a(:,:)
REAL (dp), INTENT(IN)      :: epsm
REAL (dp), INTENT(IN)      :: sx(:)
REAL (dp), INTENT(OUT)     :: udiag(:)

INTEGER   :: i, j
REAL (dp) :: tol, diagmx, diagmn, posmax, amu, offmax
REAL (dp) :: addmax, evmin, evmax, offrow, sdd

! SCALE HESSIAN
! PRE- AND POST- MULTIPLY "A" BY INV(SX)

DO j = 1,n
  a(j:n,j) = a(j:n,j) / (sx(j:n)*sx(j))
END DO

! STEP1
! -----
! NOTE:  IF A DIFFERENT TOLERANCE IS DESIRED THROUGHOUT THIS
! ALGORITHM, CHANGE TOLERANCE HERE:
tol = SQRT(epsm)

diagmx = a(1,1)
diagmn = a(1,1)
DO i = 2,n
  IF(a(i,i) < diagmn) diagmn = a(i,i)
  IF(a(i,i) > diagmx) diagmx = a(i,i)
END DO
posmax = MAX(diagmx, zero)

! DIAGMN <= 0

IF(diagmn <= posmax*tol) THEN
  amu = tol*(posmax - diagmn) - diagmn
  IF(amu == zero) THEN

! FIND LARGEST OFF-DIAGONAL ELEMENT OF A
    offmax = zero
    DO i = 2,n
      DO j = 1,i-1
        IF(ABS(a(i,j)) > offmax) offmax = ABS(a(i,j))
      END DO
    END DO
    amu = offmax
    IF(amu == zero) THEN
      amu = one
    ELSE
      amu = amu*(one + tol)
    END IF
  END IF

! A = A + MU*I

  DO i = 1,n
    a(i,i) = a(i,i) + amu
  END DO
  diagmx = diagmx + amu
END IF

! STEP2
! -----
! COPY LOWER TRIANGULAR PART OF "A" TO UPPER TRIANGULAR PART
! AND DIAGONAL OF "A" TO UDIAG

DO j = 1,n
  udiag(j) = a(j,j)
  IF(j == n) CYCLE
  a(j,j+1:n) = a(j+1:n,j)
END DO

CALL choldc(n, a, diagmx, tol, addmax)

! STEP3
! -----
! IF ADDMAX = 0, "A" WAS POSITIVE DEFINITE GOING INTO STEP 2,
! THE LL+ DECOMPOSITION HAS BEEN DONE, AND WE RETURN.
! OTHERWISE, ADDMAX >0.  PERTURB "A" SO THAT IT IS SAFELY
! DIAGONALLY DOMINANT AND FIND LL+ DECOMPOSITION

IF(addmax > zero) THEN

! RESTORE ORIGINAL "A" (LOWER TRIANGULAR PART AND DIAGONAL)

  DO j = 1,n
    a(j,j) = udiag(j)
    DO i = j+1,n
      a(i,j) = a(j,i)
    END DO
  END DO

! FIND SDD SUCH THAT A+SDD*I IS SAFELY POSITIVE DEFINITE
! NOTE:  EVMIN<0 SINCE A IS NOT POSITIVE DEFINITE;

  evmin = zero
  evmax = a(1,1)
  DO i = 1,n
    offrow = zero
    DO j = 1,i-1
      offrow = offrow + ABS(a(i,j))
    END DO
    DO j = i+1,n
      offrow = offrow + ABS(a(j,i))
    END DO
    evmin = MIN(evmin, a(i,i) - offrow)
    evmax = MAX(evmax, a(i,i) + offrow)
  END DO
  sdd = tol*(evmax-evmin) - evmin

! PERTURB "A" AND DECOMPOSE AGAIN

  amu = MIN(sdd,addmax)
  DO i = 1,n
    a(i,i) = a(i,i) + amu
    udiag(i) = a(i,i)
  END DO

! "A" NOW GUARANTEED SAFELY POSITIVE DEFINITE

  CALL choldc(n, a, zero, tol, addmax)
END IF

! UNSCALE HESSIAN AND CHOLESKY DECOMPOSITION MATRIX

DO j = 1,n
  a(j:n,j) = sx(j:n)*a(j:n,j)
  a(1:j-1,j) = sx(1:j-1)*sx(j)*a(1:j-1,j)
  udiag(j) = udiag(j)*sx(j)*sx(j)
END DO

RETURN
END SUBROUTINE chlhsn



SUBROUTINE dfaut(n, typsiz, fscale, method, iexp, msg, ndigit,  &
                 itnlim, iagflg, iahflg, ipr, dlt, gradtl, stepmx, steptl)

! PURPOSE
! -------
! SET DEFAULT VALUES FOR EACH INPUT VARIABLE TO MINIMIZATION ALGORITHM.

! PARAMETERS
! ----------
! N            --> DIMENSION OF PROBLEM
! TYPSIZ(N)   <--  TYPICAL SIZE FOR EACH COMPONENT OF X
! FSCALE      <--  ESTIMATE OF SCALE OF MINIMIZATION FUNCTION
! METHOD      <--  ALGORITHM TO USE TO SOLVE MINIMIZATION PROBLEM
! IEXP        <--  = 0 IF MINIMIZATION FUNCTION NOT EXPENSIVE TO EVALUATE
! MSG         <--  MESSAGE TO INHIBIT CERTAIN AUTOMATIC CHECKS + OUTPUT
! NDIGIT      <--  NUMBER OF GOOD DIGITS IN MINIMIZATION FUNCTION
! ITNLIM      <--  MAXIMUM NUMBER OF ALLOWABLE ITERATIONS
! IAGFLG      <--  = 0 IF ANALYTIC GRADIENT NOT SUPPLIED
! IAHFLG      <--  = 0 IF ANALYTIC HESSIAN NOT SUPPLIED
! IPR         <--  DEVICE TO WHICH TO SEND OUTPUT
! DLT         <--  TRUST REGION RADIUS
! GRADTL      <--  TOLERANCE AT WHICH GRADIENT CONSIDERED CLOSE ENOUGH
!                  TO ZERO TO TERMINATE ALGORITHM
! STEPMX      <--  VALUE OF ZERO TO TRIP DEFAULT MAXIMUM IN OPTCHK
! STEPTL      <--  TOLERANCE AT WHICH SUCCESSIVE ITERATES CONSIDERED
!                  CLOSE ENOUGH TO TERMINATE ALGORITHM

INTEGER, INTENT(IN)     :: n
REAL (dp), INTENT(OUT)  :: typsiz(:)
REAL (dp), INTENT(OUT)  :: fscale
INTEGER, INTENT(OUT)    :: method
INTEGER, INTENT(OUT)    :: iexp
INTEGER, INTENT(OUT)    :: msg
INTEGER, INTENT(OUT)    :: ndigit
INTEGER, INTENT(OUT)    :: itnlim
INTEGER, INTENT(OUT)    :: iagflg
INTEGER, INTENT(OUT)    :: iahflg
INTEGER, INTENT(OUT)    :: ipr
REAL (dp), INTENT(OUT)  :: dlt
REAL (dp), INTENT(OUT)  :: gradtl
REAL (dp), INTENT(OUT)  :: stepmx
REAL (dp), INTENT(OUT)  :: steptl

REAL (dp), PARAMETER :: epsm = EPSILON( 1.0_dp ), three = 3.0_dp

! SET TYPICAL SIZE OF X AND MINIMIZATION FUNCTION
typsiz(1:n) = one
fscale = one

! SET TOLERANCES
dlt = -one
gradtl = epsm**(one/three)
stepmx = zero
steptl = SQRT(epsm)

! SET FLAGS
method = 1
iexp = 1
msg = 0
ndigit = -1
itnlim = 150
iagflg = 0
iahflg = 0
ipr = 6

RETURN
END SUBROUTINE dfaut



SUBROUTINE dogdrv(n, x, f, g, a, p, xpls, fpls, fcn, sx, stepmx,  &
                  steptl, dlt, iretcd, mxtake, &
                  qrank, meqns, nvars, fc, anls, aja, s)

! PURPOSE
! -------
! FIND A NEXT NEWTON ITERATE (XPLS) BY THE DOUBLE DOGLEG METHOD

! PARAMETERS
! ----------
! N            --> DIMENSION OF PROBLEM
! X(N)         --> OLD ITERATE X[K-1]
! F            --> FUNCTION VALUE AT OLD ITERATE, F(X)
! G(N)         --> GRADIENT  AT OLD ITERATE, G(X), OR APPROXIMATE
! A(N,N)       --> CHOLESKY DECOMPOSITION OF HESSIAN
!                  IN LOWER TRIANGULAR PART AND DIAGONAL
! P(N)         --> NEWTON STEP
! XPLS(N)     <--  NEW ITERATE X[K]
! FPLS        <--  FUNCTION VALUE AT NEW ITERATE, F(XPLS)
! FCN          --> NAME OF SUBROUTINE TO EVALUATE FUNCTION
! SX(N)        --> DIAGONAL SCALING MATRIX FOR X
! STEPMX       --> MAXIMUM ALLOWABLE STEP SIZE
! STEPTL       --> RELATIVE STEP SIZE AT WHICH SUCCESSIVE ITERATES
!                  CONSIDERED CLOSE ENOUGH TO TERMINATE ALGORITHM
! DLT         <--> TRUST REGION RADIUS
!                  [RETAIN VALUE BETWEEN SUCCESSIVE CALLS]
! IRETCD      <--  RETURN CODE
!                    = 0 SATISFACTORY XPLS FOUND
!                    = 1 FAILED TO FIND SATISFACTORY XPLS SUFFICIENTLY
!                        DISTINCT FROM X
! MXTAKE      <--  BOOLEAN FLAG INDICATING STEP OF MAXIMUM LENGTH USED

INTEGER, INTENT(IN)        :: n
REAL (dp), INTENT(IN)      :: x(:)
REAL (dp), INTENT(IN)      :: f
REAL (dp), INTENT(IN)      :: g(:)
REAL (dp), INTENT(IN)      :: a(:,:)
REAL (dp), INTENT(IN)      :: p(:)
REAL (dp), INTENT(OUT)     :: xpls(:)
REAL (dp), INTENT(OUT)     :: fpls
REAL (dp), INTENT(IN)      :: sx(:)
REAL (dp), INTENT(IN)      :: stepmx
REAL (dp), INTENT(IN)      :: steptl
REAL (dp), INTENT(IN OUT)  :: dlt
INTEGER, INTENT(OUT)       :: iretcd
LOGICAL, INTENT(OUT)       :: mxtake
INTEGER, INTENT(IN)        :: qrank, meqns, nvars
REAL (dp), INTENT(IN OUT)  :: fc(:), anls(:,:), aja(:,:), s(:,:)

INTERFACE
  SUBROUTINE fcn(n, x, f, qrank, meqns, nvars, fc, anls, aja, s)
    IMPLICIT NONE
    INTEGER, PARAMETER     :: dp = SELECTED_REAL_KIND(14, 60)
    INTEGER, INTENT(IN)    :: n
    REAL (dp), INTENT(IN)  :: x(:)
    REAL (dp), INTENT(OUT) :: f
    INTEGER, INTENT(IN)       :: qrank, meqns, nvars
    REAL (dp), INTENT(IN OUT) :: fc(:), anls(:,:), aja(:,:), s(:,:)
  END SUBROUTINE fcn
END INTERFACE

! Workspace
REAL (dp) :: wrk1(n), wrk2(n), wrk3(n), sc(n)

REAL (dp) :: rnwtln, cln
REAL (dp) :: eta, fplsp
LOGICAL   :: fstdog, nwtake

iretcd = 4
fstdog = .true.
rnwtln = dnrm2(n, sx(1:n)*p(1:n), 1)

! FIND NEW STEP BY DOUBLE DOGLEG ALGORITHM
100 CALL dogstp(n, g, a, p, sx, rnwtln, dlt, nwtake, fstdog,  &
                wrk1, wrk2, cln, eta, sc, stepmx)

! CHECK NEW POINT AND UPDATE TRUST REGION
CALL tregup(n, x, f, g, a, fcn, sc, sx, nwtake, stepmx, steptl, dlt,  &
            iretcd, wrk3, fplsp, xpls, fpls, mxtake, 2, wrk1, &
            qrank, meqns, nvars, fc, anls, aja, s)
IF(iretcd <= 1) RETURN
GO TO 100
END SUBROUTINE dogdrv



SUBROUTINE dogstp(n, g, a, p, sx, rnwtln, dlt, nwtake, fstdog,  &
                  ssd, v, cln, eta, sc, stepmx)

! PURPOSE
! -------
! FIND NEW STEP BY DOUBLE DOGLEG ALGORITHM

! PARAMETERS
! ----------
! N            --> DIMENSION OF PROBLEM
! G(N)         --> GRADIENT AT CURRENT ITERATE, G(X)
! A(N,N)       --> CHOLESKY DECOMPOSITION OF HESSIAN IN
!                  LOWER PART AND DIAGONAL
! P(N)         --> NEWTON STEP
! SX(N)        --> DIAGONAL SCALING MATRIX FOR X
! RNWTLN       --> NEWTON STEP LENGTH
! DLT         <--> TRUST REGION RADIUS
! NWTAKE      <--> BOOLEAN, =.TRUE. IF NEWTON STEP TAKEN
! FSTDOG      <--> BOOLEAN, =.TRUE. IF ON FIRST LEG OF DOGLEG
! SSD(N)      <--> WORKSPACE [CAUCHY STEP TO THE MINIMUM OF THE
!                  QUADRATIC MODEL IN THE SCALED STEEPEST DESCENT
!                  DIRECTION] [RETAIN VALUE BETWEEN SUCCESSIVE CALLS]
! V(N)        <--> WORKSPACE  [RETAIN VALUE BETWEEN SUCCESSIVE CALLS]
! CLN         <--> CAUCHY LENGTH
!                  [RETAIN VALUE BETWEEN SUCCESSIVE CALLS]
! ETA              [RETAIN VALUE BETWEEN SUCCESSIVE CALLS]
! SC(N)       <--  CURRENT STEP
! STEPMX       --> MAXIMUM ALLOWABLE STEP SIZE

! INTERNAL VARIABLES
! ------------------
! CLN              LENGTH OF CAUCHY STEP

INTEGER, INTENT(IN)        :: n
REAL (dp), INTENT(IN)      :: g(:)
REAL (dp), INTENT(IN)      :: a(:,:)
REAL (dp), INTENT(IN)      :: p(:)
REAL (dp), INTENT(IN)      :: sx(:)
REAL (dp), INTENT(IN)      :: rnwtln
REAL (dp), INTENT(IN OUT)  :: dlt
LOGICAL, INTENT(IN OUT)    :: nwtake
LOGICAL, INTENT(IN OUT)    :: fstdog
REAL (dp), INTENT(IN OUT)  :: ssd(:)
REAL (dp), INTENT(IN OUT)  :: v(:)
REAL (dp), INTENT(IN OUT)  :: cln
REAL (dp), INTENT(IN OUT)  :: eta
REAL (dp), INTENT(OUT)     :: sc(:)
REAL (dp), INTENT(IN)      :: stepmx

INTEGER   :: i, j
REAL (dp) :: alpha, beta, tmp
REAL (dp) :: dot1, dot2, alam

! CAN WE TAKE NEWTON STEP

IF(rnwtln > dlt) GO TO 100
nwtake = .true.
sc(1:n) = p(1:n)
dlt = rnwtln
GO TO 700

! NEWTON STEP TOO LONG
! CAUCHY STEP IS ON DOUBLE DOGLEG CURVE

100 nwtake = .false.
IF(.NOT.fstdog) GO TO 200

!         CALCULATE DOUBLE DOGLEG CURVE (SSD)
fstdog = .false.
ssd(1:n) = g(1:n)/sx(1:n)
alpha = SUM( ssd(1:n)**2 )
beta = zero
DO i = 1,n
  tmp = zero
  DO j = i,n
    tmp = tmp + (a(j,i)/sx(j))*ssd(j)
  END DO
  beta = beta + tmp*tmp
END DO
ssd(1:n) = -(alpha/beta)*ssd(1:n)
cln = alpha*SQRT(alpha)/beta
eta = .2 + (.8*alpha*alpha) / (-beta * DOT_PRODUCT( g(1:n), p(1:n) ))
v(1:n) = eta*sx(1:n)*p(1:n) - ssd(1:n)
IF (dlt == -one) dlt = MIN(cln, stepmx)

200 IF(eta*rnwtln > dlt) GO TO 220

!         TAKE PARTIAL STEP IN NEWTON DIRECTION

sc(1:n) = (dlt/rnwtln)*p(1:n)
GO TO 700

220 IF(cln < dlt) GO TO 240

!         IF(CLN >= DLT) THEN TAKE STEP IN STEEPEST DESCENT DIRECTION

sc(1:n) = (dlt/cln)*ssd(1:n)/sx(1:n)
GO TO 700

!           CALCULATE CONVEX COMBINATION OF SSD AND ETA*P
!           WHICH HAS SCALED LENGTH DLT

240 dot1 = DOT_PRODUCT( v(1:n), ssd(1:n) )
dot2 = SUM( v(1:n)**2 )
alam = (-dot1 + SQRT((dot1*dot1) - dot2*(cln*cln - dlt*dlt)))/dot2
sc(1:n) = (ssd(1:n) + alam*v(1:n))/sx(1:n)

700 RETURN
END SUBROUTINE dogstp



SUBROUTINE forslv (n, a, x, b)

! PURPOSE
! --------
! SOLVE  AX = B  WHERE A  IS LOWER TRIANGULAR  MATRIX

! PARAMETERS
! ---------

! N         -----> DIMENSION OF PROBLEM
! A(N,N)    -----> LOWER TRIANGULAR MATRIX (PRESERVED)
! X(N)      <----  SOLUTION VECTOR
! B(N)       ----> RIGHT-HAND SIDE VECTOR

! NOTE
!-----
! THEN VECTORS B AND X MAY SHARE THE SAME STORAGE

INTEGER, INTENT(IN)     :: n
REAL (dp), INTENT(IN)   :: a(:,:)
REAL (dp), INTENT(OUT)  :: x(:)
REAL (dp), INTENT(IN)   :: b(:)

INTEGER   :: i
REAL (dp) :: sum

! SOLVE LX = B.  (FORWARD  SOLVE)

x(1) = b(1)/a(1,1)
DO i = 2,n
  sum = DOT_PRODUCT( a(i,1:i-1), x(1:i-1) )
  x(i) = (b(i) - sum)/a(i,i)
END DO

RETURN
END SUBROUTINE forslv



SUBROUTINE fstocd (n, x, fcn, sx, rnoise, g, &
                   qrank, meqns, nvars, fc, anls, aja, s)
! PURPOSE
! -------
! FIND CENTRAL DIFFERENCE APPROXIMATION G TO THE FIRST DERIVATIVE
! (GRADIENT) OF THE FUNCTION DEFINED BY FCN AT THE POINT X.

! PARAMETERS
! ----------
! N            --> DIMENSION OF PROBLEM
! X            --> POINT AT WHICH GRADIENT IS TO BE APPROXIMATED.
! FCN          --> NAME OF SUBROUTINE TO EVALUATE FUNCTION.
! SX           --> DIAGONAL SCALING MATRIX FOR X.
! RNOISE       --> RELATIVE NOISE IN FCN [F(X)].
! G           <--  CENTRAL DIFFERENCE APPROXIMATION TO GRADIENT.


INTEGER, INTENT(IN)        :: n
REAL (dp), INTENT(IN OUT)  :: x(:)
REAL (dp), INTENT(IN)      :: sx(:)
REAL (dp), INTENT(IN)      :: rnoise
REAL (dp), INTENT(OUT)     :: g(:)
INTEGER, INTENT(IN)        :: qrank, meqns, nvars
REAL (dp), INTENT(IN OUT)  :: fc(:), anls(:,:), aja(:,:), s(:,:)

INTERFACE
  SUBROUTINE fcn(n, x, f, qrank, meqns, nvars, fc, anls, aja, s)
    IMPLICIT NONE
    INTEGER, PARAMETER     :: dp = SELECTED_REAL_KIND(14, 60)
    INTEGER, INTENT(IN)    :: n
    REAL (dp), INTENT(IN)  :: x(:)
    REAL (dp), INTENT(OUT) :: f
    INTEGER, INTENT(IN)       :: qrank, meqns, nvars
    REAL (dp), INTENT(IN OUT) :: fc(:), anls(:,:), aja(:,:), s(:,:)
  END SUBROUTINE fcn
END INTERFACE

REAL (dp) :: third, xtempi, fplus, fminus, stepi
INTEGER   :: i

! FIND I TH  STEPSIZE, EVALUATE TWO NEIGHBORS IN DIRECTION OF I TH
! UNIT VECTOR, AND EVALUATE I TH  COMPONENT OF GRADIENT.

third = one/3.0
DO i = 1, n
  stepi = rnoise**third * MAX(ABS(x(i)), one/sx(i))
  xtempi = x(i)
  x(i) = xtempi + stepi
  CALL fcn (n, x, fplus,qrank, meqns, nvars, fc, anls, aja, s)
  x(i) = xtempi - stepi
  CALL fcn (n, x, fminus,qrank, meqns, nvars, fc, anls, aja, s)
  x(i) = xtempi
  g(i) = (fplus - fminus)/(2.0*stepi)
END DO

RETURN
END SUBROUTINE fstocd



SUBROUTINE fstofd(m, n, xpls, fcn, fpls, a, sx, rnoise, icase, &
                  qrank, meqns, nvars, fc, anls, aja, s)
! PURPOSE
! -------
! FIND FIRST ORDER FORWARD FINITE DIFFERENCE APPROXIMATION "A" TO THE
! FIRST DERIVATIVE OF THE FUNCTION DEFINED BY THE SUBPROGRAM "FNAME"
! EVALUATED AT THE NEW ITERATE "XPLS".


! FOR OPTIMIZATION USE THIS ROUTINE TO ESTIMATE:
! 1) THE FIRST DERIVATIVE (GRADIENT) OF THE OPTIMIZATION FUNCTION "FCN
!    ANALYTIC USER ROUTINE HAS BEEN SUPPLIED;
! 2) THE SECOND DERIVATIVE (HESSIAN) OF THE OPTIMIZATION FUNCTION
!    IF NO ANALYTIC USER ROUTINE HAS BEEN SUPPLIED FOR THE HESSIAN BUT
!    ONE HAS BEEN SUPPLIED FOR THE GRADIENT ("FCN") AND IF THE
!    OPTIMIZATION FUNCTION IS INEXPENSIVE TO EVALUATE

! NOTE
! ----
! _M=1 (OPTIMIZATION) ALGORITHM ESTIMATES THE GRADIENT OF THE FUNCTION
!      (FCN).   FCN(X) # F: R(N)-->R(1)
! _M=N (SYSTEMS) ALGORITHM ESTIMATES THE JACOBIAN OF THE FUNCTION
!      FCN(X) # F: R(N)-->R(N).
! _M=N (OPTIMIZATION) ALGORITHM ESTIMATES THE HESSIAN OF THE OPTIMIZATIO
!      FUNCTION, WHERE THE HESSIAN IS THE FIRST DERIVATIVE OF "FCN"

! PARAMETERS
! ----------
! M            --> NUMBER OF ROWS IN A
! N            --> NUMBER OF COLUMNS IN A; DIMENSION OF PROBLEM
! XPLS(N)      --> NEW ITERATE:  X[K]
! FCN          --> NAME OF SUBROUTINE TO EVALUATE FUNCTION
! FPLS(M)      --> _M=1 (OPTIMIZATION) FUNCTION VALUE AT NEW ITERATE:
!                       FCN(XPLS)
!                  _M=N (OPTIMIZATION) VALUE OF FIRST DERIVATIVE
!                       (GRADIENT) GIVEN BY USER FUNCTION FCN
!                  _M=N (SYSTEMS)  FUNCTION VALUE OF ASSOCIATED
!                       MINIMIZATION FUNCTION
! A(NR,N)     <--  FINITE DIFFERENCE APPROXIMATION (SEE NOTE).  ONLY
!                  LOWER TRIANGULAR MATRIX AND DIAGONAL ARE RETURNED
! SX(N)        --> DIAGONAL SCALING MATRIX FOR X
! RNOISE       --> RELATIVE NOISE IN FCN [F(X)]
! ICASE        --> =1 OPTIMIZATION (GRADIENT)
!                  =2 SYSTEMS
!                  =3 OPTIMIZATION (HESSIAN)

! INTERNAL VARIABLES
! ------------------
! STEPSZ - STEPSIZE IN THE J-TH VARIABLE DIRECTION

INTEGER, INTENT(IN)        :: m
INTEGER, INTENT(IN)        :: n
REAL (dp), INTENT(IN OUT)  :: xpls(:)
REAL (dp), INTENT(IN)      :: fpls(:)
REAL (dp), INTENT(OUT)     :: a(:,:)
REAL (dp), INTENT(IN)      :: sx(:)
REAL (dp), INTENT(IN)      :: rnoise
INTEGER, INTENT(IN)        :: icase
INTEGER, INTENT(IN)        :: qrank, meqns, nvars
REAL (dp), INTENT(IN OUT)  :: fc(:), anls(:,:), aja(:,:), s(:,:)

INTERFACE
  SUBROUTINE fcn(n, x, f, qrank, meqns, nvars, fc, anls, aja, s)
    IMPLICIT NONE
    INTEGER, PARAMETER     :: dp = SELECTED_REAL_KIND(14, 60)
    INTEGER, INTENT(IN)    :: n
    REAL (dp), INTENT(IN)  :: x(:)
    REAL (dp), INTENT(OUT) :: f
    INTEGER, INTENT(IN)       :: qrank, meqns, nvars
    REAL (dp), INTENT(IN OUT) :: fc(:), anls(:,:), aja(:,:), s(:,:)
  END SUBROUTINE fcn
END INTERFACE

! Workspace
REAL (dp) :: fhat(m)

INTEGER              :: i, j, nm1, jp1
REAL (dp)            :: stepsz, xtmpj, sqrtr, rstepsz
REAL (dp), PARAMETER :: half = 0.5_dp

! FIND J-TH COLUMN OF A
! J-TH COLUMN IS DERIVATIVE OF F(FCN) WITH RESPECT TO XPLS(J)

sqrtr = SQRT(rnoise)
DO j = 1,n
  stepsz = sqrtr * MAX(ABS(xpls(j)), one/sx(j))
  xtmpj = xpls(j)
  xpls(j) = xtmpj + stepsz
  CALL fcn(n, xpls, fhat(j),qrank, meqns, nvars, fc, anls, aja, s)
  xpls(j) = xtmpj
  rstepsz = one/stepsz
  DO i = 1,m
    a(i,j) = (fhat(i) - fpls(i))*rstepsz
  END DO
END DO
IF(icase /= 3) RETURN

! IF COMPUTING HESSIAN, A MUST BE SYMMETRIC

IF(n == 1) RETURN
nm1 = n-1
DO j = 1,nm1
  jp1 = j+1
  DO i = jp1,m
    a(i,j) = (a(i,j) + a(j,i))*half
  END DO
END DO

RETURN
END SUBROUTINE fstofd



SUBROUTINE estimate_Hessian(m, n, xpls, d1fcn, fpls, a, sx, rnoise, icase, &
                            qrank, meqns, nvars, fc, anls, aja, s)
! PURPOSE
! -------
! FIND FIRST ORDER FORWARD FINITE DIFFERENCE APPROXIMATION "A" TO THE
! HESSIAN OF THE FUNCTION DEFINED BY THE SUBPROGRAM "FNAME"
! EVALUATED AT THE NEW ITERATE "XPLS".


! FOR OPTIMIZATION USE THIS ROUTINE TO ESTIMATE:
!   THE SECOND DERIVATIVE (HESSIAN) OF THE OPTIMIZATION FUNCTION
!   IF NO ANALYTIC USER ROUTINE HAS BEEN SUPPLIED FOR THE HESSIAN BUT
!   ONE HAS BEEN SUPPLIED FOR THE GRADIENT ("FCN") AND IF THE
!   OPTIMIZATION FUNCTION IS INEXPENSIVE TO EVALUATE

! NOTE
! ----
! _M=1 (OPTIMIZATION) ALGORITHM ESTIMATES THE GRADIENT OF THE FUNCTION
!      (FCN).   FCN(X) # F: R(N)-->R(1)
! _M=N (SYSTEMS) ALGORITHM ESTIMATES THE JACOBIAN OF THE FUNCTION
!      FCN(X) # F: R(N)-->R(N).
! _M=N (OPTIMIZATION) ALGORITHM ESTIMATES THE HESSIAN OF THE OPTIMIZATIO
!      FUNCTION, WHERE THE HESSIAN IS THE FIRST DERIVATIVE OF "FCN"

! PARAMETERS
! ----------
! M            --> NUMBER OF ROWS IN A
! N            --> NUMBER OF COLUMNS IN A; DIMENSION OF PROBLEM
! XPLS(N)      --> NEW ITERATE:  X[K]
! D1FCN        --> NAME OF SUBROUTINE TO EVALUATE FIRST DERIVATIVES
! FPLS(M)      --> _M=1 (OPTIMIZATION) FUNCTION VALUE AT NEW ITERATE:
!                       FCN(XPLS)
!                  _M=N (OPTIMIZATION) VALUE OF FIRST DERIVATIVE
!                       (GRADIENT) GIVEN BY USER FUNCTION FCN
!                  _M=N (SYSTEMS)  FUNCTION VALUE OF ASSOCIATED
!                       MINIMIZATION FUNCTION
! A(NR,N)     <--  FINITE DIFFERENCE APPROXIMATION (SEE NOTE).  ONLY
!                  LOWER TRIANGULAR MATRIX AND DIAGONAL ARE RETURNED
! SX(N)        --> DIAGONAL SCALING MATRIX FOR X
! RNOISE       --> RELATIVE NOISE IN FCN [F(X)]
! ICASE        --> =1 OPTIMIZATION (GRADIENT)
!                  =2 SYSTEMS
!                  =3 OPTIMIZATION (HESSIAN)

! INTERNAL VARIABLES
! ------------------
! STEPSZ - STEPSIZE IN THE J-TH VARIABLE DIRECTION


INTEGER, INTENT(IN)        :: m
INTEGER, INTENT(IN)        :: n
REAL (dp), INTENT(IN OUT)  :: xpls(:)
REAL (dp), INTENT(IN)      :: fpls(:)
REAL (dp), INTENT(OUT)     :: a(:,:)
REAL (dp), INTENT(IN)      :: sx(:)
REAL (dp), INTENT(IN)      :: rnoise
INTEGER, INTENT(IN)        :: icase
INTEGER, INTENT(IN)        :: qrank, meqns, nvars
REAL (dp), INTENT(IN OUT)  :: fc(:), anls(:,:), aja(:,:), s(:,:)

INTERFACE
  SUBROUTINE d1fcn(n, x, g, qrank, meqns, nvars, fc, anls, aja, s)
    IMPLICIT NONE
    INTEGER, PARAMETER     :: dp = SELECTED_REAL_KIND(14, 60)
    INTEGER, INTENT(IN)    :: n
    REAL (dp), INTENT(IN)  :: x(:)
    REAL (dp), INTENT(OUT) :: g(:)
    INTEGER, INTENT(IN)       :: qrank, meqns, nvars
    REAL (dp), INTENT(IN OUT) :: fc(:), anls(:,:), aja(:,:), s(:,:)
  END SUBROUTINE d1fcn
END INTERFACE

! Workspace
REAL (dp) :: fhat(m)

INTEGER              :: i, j, nm1, jp1
REAL (dp)            :: stepsz, xtmpj, sqrtr, rstepsz
REAL (dp), PARAMETER :: half = 0.5_dp

! FIND J-TH COLUMN OF A
! EACH COLUMN IS DERIVATIVE OF F(FCN) WITH RESPECT TO XPLS(J)

sqrtr = SQRT(rnoise)
DO j = 1,n
  stepsz = sqrtr * MAX(ABS(xpls(j)), one/sx(j))
  xtmpj = xpls(j)
  xpls(j) = xtmpj + stepsz
  CALL d1fcn(n, xpls, fhat, qrank, meqns, nvars, fc, anls, aja, s)
  xpls(j) = xtmpj
  rstepsz = one/stepsz
  DO i = 1,m
    a(i,j) = (fhat(i) - fpls(i))*rstepsz
  END DO
END DO
IF(icase /= 3) RETURN

! IF COMPUTING HESSIAN, A MUST BE SYMMETRIC

IF(n == 1) RETURN
nm1 = n-1
DO j = 1,nm1
  jp1 = j+1
  DO i = jp1,m
    a(i,j) = (a(i,j) + a(j,i))*half
  END DO
END DO

RETURN
END SUBROUTINE estimate_Hessian



SUBROUTINE hookdr(n, g, a, udiag, p, sx, stepmx, dlt, &
                  iretcd, amu, dltp, phi, phip0, epsm, itncnt)

! PURPOSE
! -------
! FIND A NEXT NEWTON ITERATE (XPLS) BY THE MORE-HEBDON METHOD

! PARAMETERS
! ----------
! N            --> DIMENSION OF PROBLEM
! G(N)         --> GRADIENT AT OLD ITERATE, G(X), OR APPROXIMATE
! A(N,N)       --> CHOLESKY DECOMPOSITION OF HESSIAN IN LOWER
!                  TRIANGULAR PART AND DIAGONAL.
!                  HESSIAN IN UPPER TRIANGULAR PART AND UDIAG.
! UDIAG(N)     --> DIAGONAL OF HESSIAN IN A(.,.)
! P(N)         --> NEWTON STEP
! SX(N)        --> DIAGONAL SCALING MATRIX FOR X
! STEPMX       --> MAXIMUM ALLOWABLE STEP SIZE
! DLT         <--> TRUST REGION RADIUS
! IRETCD      <--  RETURN CODE
!                     = 0 SATISFACTORY XPLS FOUND
!                     = 1 FAILED TO FIND SATISFACTORY XPLS SUFFICIENTLY
!                       DISTINCT FROM X
! AMU         <--> [RETAIN VALUE BETWEEN SUCCESSIVE CALLS]
! DLTP        <--> [RETAIN VALUE BETWEEN SUCCESSIVE CALLS]
! PHI         <--> [RETAIN VALUE BETWEEN SUCCESSIVE CALLS]
! PHIP0       <--> [RETAIN VALUE BETWEEN SUCCESSIVE CALLS]
! EPSM         --> MACHINE EPSILON
! ITNCNT       --> ITERATION COUNT

INTEGER, INTENT(IN)        :: n
REAL (dp), INTENT(IN)      :: g(:)
REAL (dp), INTENT(IN OUT)  :: a(:,:)
REAL (dp), INTENT(IN)      :: udiag(:)
REAL (dp), INTENT(IN)      :: p(:)
REAL (dp), INTENT(IN)      :: sx(:)
REAL (dp), INTENT(IN)      :: stepmx
REAL (dp), INTENT(IN OUT)  :: dlt
INTEGER, INTENT(OUT)       :: iretcd
REAL (dp), INTENT(IN OUT)  :: amu
REAL (dp), INTENT(IN OUT)  :: dltp
REAL (dp), INTENT(IN OUT)  :: phi
REAL (dp), INTENT(IN OUT)  :: phip0
REAL (dp), INTENT(IN)      :: epsm
INTEGER, INTENT(IN)        :: itncnt

INTEGER   :: i

REAL (dp) :: tmp, rnwtln, alpha, beta
LOGICAL   :: fstime

iretcd = 4
fstime = .true.
tmp = SUM( (sx(1:n)*p(1:n))**2 )
rnwtln = SQRT(tmp)
IF(itncnt == 1) THEN
  amu = zero

!       IF FIRST ITERATION AND TRUST REGION NOT PROVIDED BY USER,
!       COMPUTE INITIAL TRUST REGION.

  IF(dlt == -one) THEN
    alpha = SUM( (g(1:n)/sx(1:n))**2 )
    beta = zero
    DO i = 1,n
      tmp = SUM( a(i:n,i)*g(i:n) / (sx(i:n)*sx(i:n)) )
      beta = beta + tmp*tmp
    END DO
    dlt = alpha*SQRT(alpha)/beta
    dlt = MIN(dlt, stepmx)
  END IF
END IF

! FIND NEW STEP BY MORE-HEBDON ALGORITHM
CALL hookst(n, g, a, udiag, p, sx, rnwtln, dlt, amu,  &
            dltp, phi, phip0, fstime, epsm)
dltp = dlt

! CHECK NEW POINT AND UPDATE TRUST REGION
!     CALL TREGUP(NR,N,X,F,G,A,FCN,SC,SX,NWTAKE,STEPMX,STEPTL,
!    +         DLT,IRETCD,XPLSP,FPLSP,XPLS,FPLS,MXTAKE,IPR,3,UDIAG)
! IF(iretcd <= 1) RETURN
! GO TO 100

RETURN
END SUBROUTINE hookdr



SUBROUTINE hookst(n, g, a, udiag, p, sx, rnwtln, dlt, amu,  &
                  dltp, phi, phip0, fstime, epsm)

! PURPOSE
! -------
! FIND NEW STEP BY MORE-HEBDON ALGORITHM

! PARAMETERS
! ----------
! N            --> DIMENSION OF PROBLEM
! G(N)         --> GRADIENT AT CURRENT ITERATE, G(X)
! A(N,N)       --> CHOLESKY DECOMPOSITION OF HESSIAN IN
!                  LOWER TRIANGULAR PART AND DIAGONAL.
!                  HESSIAN OR APPROX IN UPPER TRIANGULAR PART
! UDIAG(N)     --> DIAGONAL OF HESSIAN IN A(.,.)
! P(N)         --> NEWTON STEP
! SX(N)        --> DIAGONAL SCALING MATRIX FOR N
! RNWTLN       --> NEWTON STEP LENGTH
! DLT         <--> TRUST REGION RADIUS
! AMU         <--> [RETAIN VALUE BETWEEN SUCCESSIVE CALLS]
! DLTP         --> TRUST REGION RADIUS AT LAST EXIT FROM THIS ROUTINE
! PHI         <--> [RETAIN VALUE BETWEEN SUCCESSIVE CALLS]
! PHIP0       <--> [RETAIN VALUE BETWEEN SUCCESSIVE CALLS]
! FSTIME      <--> BOOLEAN. =.TRUE. IF FIRST ENTRY TO THIS ROUTINE
!                  DURING K-TH ITERATION
! SC(N)       <--  CURRENT STEP
! NWTAKE      <--  BOOLEAN, =.TRUE. IF NEWTON STEP TAKEN
! EPSM         --> MACHINE EPSILON


INTEGER, INTENT(IN)        :: n
REAL (dp), INTENT(IN)      :: g(:)
REAL (dp), INTENT(IN OUT)  :: a(:,:)
REAL (dp), INTENT(IN)      :: udiag(:)
REAL (dp), INTENT(IN)      :: p(:)
REAL (dp), INTENT(IN)      :: sx(:)
REAL (dp), INTENT(IN)      :: rnwtln
REAL (dp), INTENT(IN OUT)  :: dlt
REAL (dp), INTENT(IN OUT)  :: amu
REAL (dp), INTENT(IN)      :: dltp
REAL (dp), INTENT(IN OUT)  :: phi
REAL (dp), INTENT(IN OUT)  :: phip0
LOGICAL, INTENT(IN OUT)    :: fstime
REAL (dp), INTENT(IN)      :: epsm

! Workspace
REAL (dp) :: wrk0(n), sc(n)

INTEGER   :: i, jp1, j
REAL (dp) :: hi, alo, phip
REAL (dp) :: amuup, stepln, amulo

LOGICAL   :: done

! HI AND ALO ARE CONSTANTS USED IN THIS ROUTINE.
! CHANGE HERE IF OTHER VALUES ARE TO BE SUBSTITUTED.
hi = 1.5
alo = .75
! -----
IF(rnwtln <= hi*dlt) THEN

!       TAKE NEWTON STEP

  sc(1:n) = p(1:n)
  dlt = MIN(dlt,rnwtln)
  amu = zero
  RETURN
END IF

!     NEWTON STEP NOT TAKEN

! SET PHIP TO 1.0 FOR COMPILATION.  THIS SUBROUTINE IS NOT CURRENTLY
! USED BY TENSOLVE.

phip = one
IF(amu > zero) THEN
  amu = amu - (phi+dltp) *((dltp-dlt) + phi) / (dlt*phip)
END IF
phi = rnwtln - dlt
IF(fstime) THEN
  wrk0(1:n) = sx(1:n)*sx(1:n)*p(1:n)

!         SOLVE L*Y = (SX**2)*P

  CALL forslv(n, a, wrk0, wrk0)
  phip0 = - dnrm2(n, wrk0, 1)**2 / rnwtln
  fstime = .false.
END IF
phip = phip0
amulo = -phi/phip
amuup = SUM( (g(1:n)/sx(1:n))**2 )
amuup = SQRT(amuup)/dlt
done = .false.

!       TEST VALUE OF AMU; GENERATE NEXT AMU IF NECESSARY

100 IF(done) RETURN
IF(amu < amulo .OR. amu > amuup) THEN
  amu = MAX(SQRT(amulo*amuup), amuup*1.0E-3)
END IF

!     COPY (H,UDIAG) TO L
!     WHERE H <-- H+AMU*(SX**2) [DO NOT ACTUALLY CHANGE (H,UDIAG)]
DO j = 1,n
  a(j,j) = udiag(j) + amu*sx(j)*sx(j)
  IF(j == n) CYCLE
  jp1 = j+1
  DO i = jp1,n
    a(i,j) = a(j,i)
  END DO
END DO

!     FACTOR H = L(L+)

CALL choldc(n, a, zero, SQRT(epsm))

!     SOLVE H*P = L(L+)*SC = -G

wrk0(1:n) = -g(1:n)
CALL lltslv(n, a, sc, wrk0)

!     RESET H.  NOTE SINCE UDIAG HAS NOT BEEN DESTROYED WE NEED DO
!     NOTHING HERE.  H IS IN THE UPPER PART AND IN UDIAG, STILL INTACT

stepln = SUM( sx(1:n)**2 * sc(1:n)**2 )
stepln = SQRT(stepln)
phi = stepln - dlt
wrk0(1:n) = sx(1:n)*sx(1:n)*sc(1:n)
CALL forslv(n, a, wrk0, wrk0)
phip = -dnrm2(n,wrk0,1)**2/stepln
IF((alo*dlt > stepln .OR. stepln > hi*dlt) .AND.  &
    (amuup-amulo > zero)) GO TO 170

!       SC IS ACCEPTABLE HOOKSTEP

done = .true.
GO TO 100

!       SC NOT ACCEPTABLE HOOKSTEP.  SELECT NEW AMU

170 amulo = MAX(amulo, amu-(phi/phip))
IF(phi < zero) amuup = MIN(amuup,amu)
amu = amu - (stepln*phi)/(dlt*phip)
GO TO 100
END SUBROUTINE hookst



SUBROUTINE hsnint(n, a, sx, method)

! PURPOSE
! -------
! PROVIDE INITIAL HESSIAN WHEN USING SECANT UPDATES

! PARAMETERS
! ----------
! N            --> DIMENSION OF PROBLEM
! A(N,N)      <--  INITIAL HESSIAN (LOWER TRIANGULAR MATRIX)
! SX(N)        --> DIAGONAL SCALING MATRIX FOR X
! METHOD       --> ALGORITHM TO USE TO SOLVE MINIMIZATION PROBLEM
!                     = 1,2 FACTORED SECANT METHOD USED
!                     = 3   UNFACTORED SECANT METHOD USED

INTEGER, INTENT(IN)     :: n
REAL (dp), INTENT(OUT)  :: a(:,:)
REAL (dp), INTENT(IN)   :: sx(:)
INTEGER, INTENT(IN)     :: method

INTEGER :: j

DO j = 1,n
  IF(method == 3) a(j,j) = sx(j)*sx(j)
  IF(method /= 3) a(j,j) = sx(j)
  IF(j == n) CYCLE
  a(j+1:n,j) = zero
END DO

RETURN
END SUBROUTINE hsnint



SUBROUTINE lltslv(n, a, x, b)

! PURPOSE
! -------
! SOLVE AX = B WHERE A HAS THE FORM L(L-TRANSPOSE)
! BUT ONLY THE LOWER TRIANGULAR PART, L, IS STORED.

! PARAMETERS
! ----------
! N            --> DIMENSION OF PROBLEM
! A(N,N)       --> MATRIX OF FORM L(L-TRANSPOSE).
!                  ON RETURN A IS UNCHANGED.
! X(N)        <--  SOLUTION VECTOR
! B(N)         --> RIGHT-HAND SIDE VECTOR

! NOTE
! ----
! IF B IS NOT REQUIRED BY CALLING PROGRAM, THEN
! B AND X MAY SHARE THE SAME STORAGE.

INTEGER, INTENT(IN)     :: n
REAL (dp), INTENT(IN)   :: a(:,:)
REAL (dp), INTENT(OUT)  :: x(:)
REAL (dp), INTENT(IN)   :: b(:)

REAL (dp)  :: wk(n)

! FORWARD SOLVE, RESULT IN WK

CALL forslv(n, a, wk, b)

! BACK SOLVE, RESULT IN X

CALL bakslv(n, a, x, wk)
RETURN
END SUBROUTINE lltslv



SUBROUTINE optchk(n, x, typsiz, sx, fscale, itnlim, ndigit, epsm,  &
                  dlt, method, iexp, iagflg, iahflg, stepmx, msg)

! PURPOSE
! -------
! CHECK INPUT FOR REASONABLENESS

! PARAMETERS
! ----------
! N            --> DIMENSION OF PROBLEM
! X(N)         --> ON ENTRY, ESTIMATE TO ROOT OF FCN
! TYPSIZ(N)   <--> TYPICAL SIZE OF EACH COMPONENT OF X
! SX(N)       <--  DIAGONAL SCALING MATRIX FOR X
! FSCALE      <--> ESTIMATE OF SCALE OF OBJECTIVE FUNCTION FCN
! ITNLIM      <--> MAXIMUM NUMBER OF ALLOWABLE ITERATIONS
! NDIGIT      <--> NUMBER OF GOOD DIGITS IN OPTIMIZATION FUNCTION FCN
! EPSM         --> MACHINE EPSILON
! DLT         <--> TRUST REGION RADIUS
! METHOD      <--> ALGORITHM INDICATOR
! IEXP        <--> EXPENSE FLAG
! IAGFLG      <-->  = 1 IF ANALYTIC GRADIENT SUPPLIED
! IAHFLG      <-->  = 1 IF ANALYTIC HESSIAN SUPPLIED
! STEPMX      <--> MAXIMUM STEP SIZE
! MSG         <--> MESSAGE AND ERROR CODE

INTEGER, INTENT(IN)        :: n
REAL (dp), INTENT(IN)      :: x(:)
REAL (dp), INTENT(IN OUT)  :: typsiz(:)
REAL (dp), INTENT(OUT)     :: sx(:)
REAL (dp), INTENT(IN OUT)  :: fscale
INTEGER, INTENT(IN OUT)    :: itnlim
INTEGER, INTENT(IN OUT)    :: ndigit
REAL (dp), INTENT(IN)      :: epsm
REAL (dp), INTENT(IN OUT)  :: dlt
INTEGER, INTENT(IN OUT)    :: method
INTEGER, INTENT(IN OUT)    :: iexp
INTEGER, INTENT(IN OUT)    :: iagflg
INTEGER, INTENT(IN OUT)    :: iahflg
REAL (dp), INTENT(IN OUT)  :: stepmx
INTEGER, INTENT(IN OUT)    :: msg

REAL (dp) :: stpsiz
INTEGER   :: i

! COMPUTE SCALE MATRIX

DO i = 1,n
  IF(typsiz(i) == zero) typsiz(i) = one
  IF(typsiz(i) < zero) typsiz(i) = -typsiz(i)
  sx(i) = one/typsiz(i)
END DO

! CHECK MAXIMUM STEP SIZE

stpsiz = zero
DO i = 1, n
  stpsiz = stpsiz + x(i)*x(i)*sx(i)*sx(i)
END DO
stpsiz =SQRT(stpsiz)
stepmx = MAX(1.0D3*stpsiz, 1.0D3)

! CHECK NUMBER OF DIGITS OF ACCURACY IN FUNCTION FCN
ndigit = -LOG10(epsm)
RETURN
END SUBROUTINE optchk



SUBROUTINE optdrv(nr, n, x, fcn, d1fcn, d2fcn, typsiz, fscale,  &
                  method, iexp, msg, ndigit, itnlim, iagflg, iahflg, &
                  dlt, gradtl, stepmx, steptl, xpls, fpls, gpls, itrmcd, &
                  qrank, meqns, nvars, fc, anls, aja, s)

! PURPOSE
! -------
! DRIVER FOR NON-LINEAR OPTIMIZATION PROBLEM

! PARAMETERS
! ----------
! NR           --> ROW DIMENSION OF MATRIX
! N            --> DIMENSION OF PROBLEM
! X(N)         --> ON ENTRY: ESTIMATE TO A ROOT OF FCN
! FCN          --> NAME OF SUBROUTINE TO EVALUATE OPTIMIZATION FUNCTION
!                            FCN: R(N) --> R(1)
! D1FCN        --> NAME OF SUBROUTINE TO EVALUATE GRADIENT OF FCN.
! D2FCN        --> NAME OF SUBROUTINE TO EVALUATE HESSIAN OF OF FCN.
! TYPSIZ(N)    --> TYPICAL SIZE FOR EACH COMPONENT OF X
! FSCALE       --> ESTIMATE OF SCALE OF OBJECTIVE FUNCTION
! METHOD       --> ALGORITHM TO USE TO SOLVE MINIMIZATION PROBLEM
!                    =1 LINE SEARCH
!                    =2 DOUBLE DOGLEG
!                    =3 MORE-HEBDON
! IEXP         --> =1 IF OPTIMIZATION FUNCTION FCN IS EXPENSIVE TO
!                  EVALUATE, =0 OTHERWISE.  IF SET THEN HESSIAN WILL
!                  BE EVALUATED BY SECANT UPDATE INSTEAD OF
!                  ANALYTICALLY OR BY FINITE DIFFERENCES
! MSG         <--> ON INPUT:  ( > 0) MESSAGE TO INHIBIT CERTAIN
!                    AUTOMATIC CHECKS
!                  ON OUTPUT: (.LT.0) ERROR CODE; =0 NO ERROR
! NDIGIT       --> NUMBER OF GOOD DIGITS IN OPTIMIZATION FUNCTION FCN
! ITNLIM       --> MAXIMUM NUMBER OF ALLOWABLE ITERATIONS
! IAGFLG       --> =1 IF ANALYTIC GRADIENT SUPPLIED
! IAHFLG       --> =1 IF ANALYTIC HESSIAN SUPPLIED
! DLT          --> TRUST REGION RADIUS
! GRADTL       --> TOLERANCE AT WHICH GRADIENT CONSIDERED CLOSE
!                  ENOUGH TO ZERO TO TERMINATE ALGORITHM
! STEPMX       --> MAXIMUM ALLOWABLE STEP SIZE
! STEPTL       --> RELATIVE STEP SIZE AT WHICH SUCCESSIVE ITERATES
!                  CONSIDERED CLOSE ENOUGH TO TERMINATE ALGORITHM
! XPLS(N)     <--> ON EXIT:  XPLS IS LOCAL MINIMUM
! FPLS        <--> ON EXIT:  FUNCTION VALUE AT SOLUTION, XPLS
! GPLS(N)     <--> ON EXIT:  GRADIENT AT SOLUTION XPLS
! ITRMCD      <--  TERMINATION CODE

! INTERNAL VARIABLES
! ------------------
! ANALTL           TOLERANCE FOR COMPARISON OF ESTIMATED AND
!                  ANALYTICAL GRADIENTS AND HESSIANS
! EPSM             MACHINE EPSILON
! F                FUNCTION VALUE: FCN(X)
! ITNCNT           CURRENT ITERATION, K
! RNF              RELATIVE NOISE IN OPTIMIZATION FUNCTION FCN.
!                       NOISE = 10.**(-NDIGIT)

INTEGER, INTENT(IN)        :: nr
INTEGER, INTENT(IN)        :: n
REAL (dp), INTENT(IN OUT)  :: x(:)
REAL (dp), INTENT(IN OUT)  :: typsiz(:)
REAL (dp), INTENT(IN OUT)  :: fscale
INTEGER, INTENT(IN OUT)    :: method
INTEGER, INTENT(IN OUT)    :: iexp
INTEGER, INTENT(IN OUT)    :: msg
INTEGER, INTENT(IN OUT)    :: ndigit
INTEGER, INTENT(IN OUT)    :: itnlim
INTEGER, INTENT(IN OUT)    :: iagflg
INTEGER, INTENT(IN OUT)    :: iahflg
REAL (dp), INTENT(IN OUT)  :: dlt
REAL (dp), INTENT(IN)      :: gradtl
REAL (dp), INTENT(IN OUT)  :: stepmx
REAL (dp), INTENT(IN)      :: steptl
REAL (dp), INTENT(IN OUT)  :: xpls(:)
REAL (dp), INTENT(IN OUT)  :: fpls
REAL (dp), INTENT(IN OUT)  :: gpls(:)
INTEGER, INTENT(OUT)       :: itrmcd
INTEGER, INTENT(IN)        :: qrank, meqns, nvars
REAL (dp), INTENT(IN OUT)  :: fc(:), anls(:,:), aja(:,:), s(:,:)

INTERFACE
  SUBROUTINE fcn(n, x, f, qrank, meqns, nvars, fc, anls, aja, s)
    IMPLICIT NONE
    INTEGER, PARAMETER     :: dp = SELECTED_REAL_KIND(14, 60)
    INTEGER, INTENT(IN)    :: n
    REAL (dp), INTENT(IN)  :: x(:)
    REAL (dp), INTENT(OUT) :: f
    INTEGER, INTENT(IN)       :: qrank, meqns, nvars
    REAL (dp), INTENT(IN OUT) :: fc(:), anls(:,:), aja(:,:), s(:,:)
  END SUBROUTINE fcn

  SUBROUTINE d1fcn(n, x, g, qrank, meqns, nvars, fc, anls, aja, s)
    IMPLICIT NONE
    INTEGER, PARAMETER     :: dp = SELECTED_REAL_KIND(14, 60)
    INTEGER, INTENT(IN)    :: n
    REAL (dp), INTENT(IN)  :: x(:)
    REAL (dp), INTENT(OUT) :: g(:)
    INTEGER, INTENT(IN)       :: qrank, meqns, nvars
    REAL (dp), INTENT(IN OUT) :: fc(:), anls(:,:), aja(:,:), s(:,:)
  END SUBROUTINE d1fcn

  SUBROUTINE d2fcn(nr, n, x, h)
    IMPLICIT NONE
    INTEGER, PARAMETER     :: dp = SELECTED_REAL_KIND(14, 60)
    INTEGER, INTENT(IN)    :: nr
    INTEGER, INTENT(IN)    :: n
    REAL (dp), INTENT(IN)  :: x(:)
    REAL (dp), INTENT(OUT) :: h(:,:)
  END SUBROUTINE d2fcn
END INTERFACE

! Workspace
REAL (dp) :: a(n,n), udiag(n), g(n), p(n), sx(n), wrk1(n)

INTEGER   :: i, itncnt, iretcd, icscmx

REAL (dp) :: epsm, f, rnf, dltsav
REAL (dp) :: amusav, amu, dlpsav, dltp, phisav, phi, phpsav, phip0

REAL (dp) :: work2(n,1)

LOGICAL   :: mxtake

! INITIALIZATION
! --------------
p(1:n) = zero
itncnt = 0
iretcd = -1
epsm = EPSILON( 1.D0 )
CALL optchk(n, x, typsiz, sx, fscale, itnlim, ndigit, epsm,  &
            dlt, method, iexp, iagflg, iahflg, stepmx, msg)
IF(msg < 0) RETURN
rnf = MAX(10.0D0**(-ndigit), epsm)

! EVALUATE FCN(X)

CALL fcn(n,x,f,qrank, meqns, nvars, fc, anls, aja, s)

! EVALUATE ANALYTIC OR FINITE DIFFERENCE GRADIENT AND CHECK ANALYTIC
! GRADIENT, IF REQUESTED.

IF (iagflg == 1) THEN
  CALL d1fcn (n, x, g, qrank, meqns, nvars, fc, anls, aja, s)
ELSE
  CALL fstofd (1, n, x, fcn, g, a, sx, rnf, 1, qrank, meqns, nvars, fc, anls, aja, s)
END IF

CALL optstp(n, x, f, g, wrk1, itncnt, icscmx, itrmcd, gradtl, steptl, sx, &
            fscale, itnlim, iretcd, mxtake)
IF(itrmcd /= 0) GO TO 700

IF(iexp /= 1) GO TO 80

! IF OPTIMIZATION FUNCTION EXPENSIVE TO EVALUATE (IEXP=1), THEN
! HESSIAN WILL BE OBTAINED BY SECANT UPDATES.  GET INITIAL HESSIAN.

CALL hsnint(n, a, sx, method)
GO TO 100

! EVALUATE ANALYTIC OR FINITE DIFFERENCE HESSIAN AND CHECK ANALYTIC
! HESSIAN IF REQUESTED (ONLY IF USER-SUPPLIED ANALYTIC HESSIAN
! ROUTINE D2FCN FILLS ONLY LOWER TRIANGULAR PART AND DIAGONAL OF A).

80 IF (iahflg == 0) THEN
  IF (iagflg == 1) CALL estimate_Hessian (n, n, x, d1fcn, g, a, sx, rnf, 3, qrank, meqns, nvars, fc, anls, aja, s)
  IF (iagflg /= 1) CALL sndofd (n, x, fcn, f, a, sx, rnf, qrank, meqns, nvars, fc, anls, aja, s)
ELSE
  CALL d2fcn(nr, n, x, a)              ! Inserted by AJM
END IF


! ITERATION
! ---------
100 itncnt = itncnt + 1

! FIND PERTURBED LOCAL MODEL HESSIAN AND ITS LL+ DECOMPOSITION
! (SKIP THIS STEP IF LINE SEARCH OR DOGSTEP TECHNIQUES BEING USED WITH
! SECANT UPDATES.  CHOLESKY DECOMPOSITION L ALREADY OBTAINED FROM SECFAC.)

IF(iexp == 1 .AND. method /= 3) GO TO 105
103 CALL chlhsn(n, a, epsm, sx, udiag)

! SOLVE FOR NEWTON STEP:  AP = -G

105 wrk1(1:n) = -g(1:n)
CALL lltslv(n, a, p, wrk1)

! DECIDE WHETHER TO ACCEPT NEWTON STEP  XPLS = X + P
! OR TO CHOOSE XPLS BY A GLOBAL STRATEGY.

IF (iagflg /= 0 .OR. method == 1) GO TO 111
dltsav = dlt
IF (method == 2) GO TO 111
amusav = amu
dlpsav = dltp
phisav = phi
phpsav = phip0

111 IF(method == 2) CALL dogdrv(n, x, f, g, a, p, xpls, fpls, fcn, sx, &
                                stepmx, steptl, dlt, iretcd, mxtake, &
                                qrank, meqns, nvars, fc, anls, aja, s)
IF(method == 3) CALL hookdr(n, g, a, udiag, p, sx, stepmx, dlt, iretcd, &
                            amu, dltp, phi, phip0, epsm, itncnt)

IF (method /= 2) xpls(1:n) = x(1:n) + p(1:n)            ! Added by AJM

! IF COULD NOT FIND SATISFACTORY STEP AND FORWARD DIFFERENCE
! GRADIENT WAS USED, RETRY USING CENTRAL DIFFERENCE GRADIENT.

IF (iretcd == 1 .AND. iagflg == 0) THEN

!        SET IAGFLG FOR CENTRAL DIFFERENCES

  iagflg = -1
  CALL fstocd (n, x, fcn, sx, rnf, g, qrank, meqns, nvars, fc, anls, aja, s)
  IF (method == 1) GO TO 105
  dlt = dltsav
  IF (method == 2) GO TO 105
  amu = amusav
  dltp = dlpsav
  phi = phisav
  phip0 = phpsav
  GO TO 103
END IF

! CALCULATE STEP FOR OUTPUT

p(1:n) = xpls(1:n) - x(1:n)

! CALCULATE GRADIENT AT XPLS

IF (iagflg == -1) GO TO 116
IF (iagflg == 0) GO TO 118

! ANALYTIC GRADIENT

CALL d1fcn (n, xpls, gpls, qrank, meqns, nvars, fc, anls, aja, s)
GO TO 120

! CENTRAL DIFFERENCE GRADIENT

116 CALL fstocd (n, xpls, fcn, sx, rnf, gpls, qrank, meqns, nvars, fc, anls, aja, s)
GO TO 120

! FORWARD DIFFERENCE GRADIENT

118 CALL fstofd (1, n, xpls, fcn, wrk1, work2, sx, rnf, 1, qrank, meqns, nvars, fc, anls, aja, s)
gpls(1:n) = work2(1:n,1)

! CHECK WHETHER STOPPING CRITERIA SATISFIED

120 CALL optstp(n, xpls, fpls, gpls, x, itncnt, icscmx, itrmcd, gradtl, &
                steptl, sx, fscale, itnlim, iretcd, mxtake)
IF(itrmcd /= 0) GO TO 690

! EVALUATE HESSIAN AT XPLS

IF(iexp /= 0) GO TO 150
IF(iahflg == 1) GO TO 140
IF(iagflg == 1) CALL estimate_Hessian(n, n, xpls, d1fcn, gpls, a, sx, rnf, 3, qrank, meqns, nvars, fc, anls, aja, s)
IF(iagflg /= 1) CALL sndofd(n, xpls, fcn, fpls, a, sx, rnf, qrank, meqns, nvars, fc, anls, aja, s)
GO TO 150
140 CALL d2fcn(nr, n, xpls, a)

! X <-- XPLS  AND  G <-- GPLS  AND  F <-- FPLS

150 f = fpls
DO i = 1,n
  x(i) = xpls(i)
  g(i) = gpls(i)
END DO
GO TO 100

! TERMINATION
! -----------
! RESET XPLS,FPLS,GPLS,  IF PREVIOUS ITERATE SOLUTION

690 IF(itrmcd /= 3) GO TO 710
700 fpls = f
DO i = 1,n
  xpls(i) = x(i)
  gpls(i) = g(i)
END DO

710 msg = 0
RETURN
END SUBROUTINE optdrv



SUBROUTINE optif9(nr, n, x, fcn, d1fcn, d2fcn, typsiz, fscale, method,  &
                  iexp, msg, ndigit, itnlim, iagflg, iahflg, dlt,  &
                  gradtl, stepmx, steptl, xpls, fpls, gpls, itrmcd, &
                  qrank, meqns, nvars, fc, anls, aja, s)

! PURPOSE
! -------
! PROVIDE COMPLETE INTERFACE TO MINIMIZATION PACKAGE.
! USER HAS FULL CONTROL OVER OPTIONS.

! PARAMETERS
! ----------
! NR           --> ROW DIMENSION OF MATRIX
! N            --> DIMENSION OF PROBLEM
! X(N)         --> ON ENTRY: ESTIMATE TO A ROOT OF FCN
! FCN          --> NAME OF SUBROUTINE TO EVALUATE OPTIMIZATION FUNCTION
!                            FCN: R(N) --> R(1)
! D1FCN        --> NAME OF SUBROUTINE TO EVALUATE GRADIENT OF FCN.
! D2FCN        --> NAME OF SUBROUTINE TO EVALUATE HESSIAN OF OF FCN.
! TYPSIZ(N)    --> TYPICAL SIZE FOR EACH COMPONENT OF X
! FSCALE       --> ESTIMATE OF SCALE OF OBJECTIVE FUNCTION
! METHOD       --> ALGORITHM TO USE TO SOLVE MINIMIZATION PROBLEM
!                     = 1 LINE SEARCH
!                     = 2 DOUBLE DOGLEG
!                     = 3 MORE-HEBDON
! IEXP         -->  = 1 IF OPTIMIZATION FUNCTION FCN IS EXPENSIVE TO
!                  EVALUATE,  = 0 OTHERWISE.  IF SET THEN HESSIAN WILL
!                  BE EVALUATED BY SECANT UPDATE INSTEAD OF
!                  ANALYTICALLY OR BY FINITE DIFFERENCES
! MSG         <--> ON INPUT:  (.GT.0) MESSAGE TO INHIBIT CERTAIN
!                    AUTOMATIC CHECKS
!                  ON OUTPUT: (.LT.0) ERROR CODE;  = 0 NO ERROR
! NDIGIT       --> NUMBER OF GOOD DIGITS IN OPTIMIZATION FUNCTION FCN
! ITNLIM       --> MAXIMUM NUMBER OF ALLOWABLE ITERATIONS
! IAGFLG       -->  = 1 IF ANALYTIC GRADIENT SUPPLIED
! IAHFLG       -->  = 1 IF ANALYTIC HESSIAN SUPPLIED
! DLT          --> TRUST REGION RADIUS
! GRADTL       --> TOLERANCE AT WHICH GRADIENT CONSIDERED CLOSE
!                  ENOUGH TO ZERO TO TERMINATE ALGORITHM
! STEPMX       --> MAXIMUM ALLOWABLE STEP SIZE
! STEPTL       --> RELATIVE STEP SIZE AT WHICH SUCCESSIVE ITERATES
!                  CONSIDERED CLOSE ENOUGH TO TERMINATE ALGORITHM
! XPLS(N)     <--> ON EXIT:  XPLS IS LOCAL MINIMUM
! FPLS        <--> ON EXIT:  FUNCTION VALUE AT SOLUTION, XPLS
! GPLS(N)     <--> ON EXIT:  GRADIENT AT SOLUTION XPLS
! ITRMCD      <--  TERMINATION CODE

INTEGER, INTENT(IN)        :: nr
INTEGER, INTENT(IN)        :: n
REAL (dp), INTENT(IN OUT)  :: x(:)
REAL (dp), INTENT(IN OUT)  :: typsiz(:)
REAL (dp), INTENT(IN OUT)  :: fscale
INTEGER, INTENT(IN OUT)    :: method
INTEGER, INTENT(IN OUT)    :: iexp
INTEGER, INTENT(IN OUT)    :: msg
INTEGER, INTENT(IN OUT)    :: ndigit
INTEGER, INTENT(IN OUT)    :: itnlim
INTEGER, INTENT(IN OUT)    :: iagflg
INTEGER, INTENT(IN OUT)    :: iahflg
REAL (dp), INTENT(IN OUT)  :: dlt
REAL (dp), INTENT(IN)      :: gradtl
REAL (dp), INTENT(IN OUT)  :: stepmx
REAL (dp), INTENT(IN)      :: steptl
REAL (dp), INTENT(IN OUT)  :: xpls(:)
REAL (dp), INTENT(IN OUT)  :: fpls
REAL (dp), INTENT(IN OUT)  :: gpls(:)
INTEGER, INTENT(OUT)       :: itrmcd
INTEGER, INTENT(IN)        :: qrank, meqns, nvars
REAL (dp), INTENT(IN OUT)  :: fc(:), anls(:,:), aja(:,:), s(:,:)

INTERFACE
  SUBROUTINE fcn(n, x, f, qrank, meqns, nvars, fc, anls, aja, s)
    IMPLICIT NONE
    INTEGER, PARAMETER     :: dp = SELECTED_REAL_KIND(14, 60)
    INTEGER, INTENT(IN)    :: n
    REAL (dp), INTENT(IN)  :: x(:)
    REAL (dp), INTENT(OUT) :: f
    INTEGER, INTENT(IN)       :: qrank, meqns, nvars
    REAL (dp), INTENT(IN OUT) :: fc(:), anls(:,:), aja(:,:), s(:,:)
  END SUBROUTINE fcn

  SUBROUTINE d1fcn(n, x, g, qrank, meqns, nvars, fc, anls, aja, s)
    IMPLICIT NONE
    INTEGER, PARAMETER     :: dp = SELECTED_REAL_KIND(14, 60)
    INTEGER, INTENT(IN)    :: n
    REAL (dp), INTENT(IN)  :: x(:)
    REAL (dp), INTENT(OUT) :: g(:)
    INTEGER, INTENT(IN)       :: qrank, meqns, nvars
    REAL (dp), INTENT(IN OUT) :: fc(:), anls(:,:), aja(:,:), s(:,:)
  END SUBROUTINE d1fcn

  SUBROUTINE d2fcn(nr, n, x, h)
    IMPLICIT NONE
    INTEGER, PARAMETER     :: dp = SELECTED_REAL_KIND(14, 60)
    INTEGER, INTENT(IN)    :: nr
    INTEGER, INTENT(IN)    :: n
    REAL (dp), INTENT(IN)  :: x(:)
    REAL (dp), INTENT(OUT) :: h(:,:)
  END SUBROUTINE d2fcn
END INTERFACE

CALL optdrv(nr, n, x, fcn, d1fcn, d2fcn, typsiz, fscale,  &
            method, iexp, msg, ndigit, itnlim, iagflg, iahflg, &
            dlt, gradtl, stepmx, steptl, xpls, fpls, gpls, itrmcd, &
            qrank, meqns, nvars, fc, anls, aja, s)
RETURN
END SUBROUTINE optif9



SUBROUTINE optstp(n, xpls, fpls, gpls, x, itncnt, icscmx, itrmcd, gradtl, &
                  steptl, sx, fscale, itnlim, iretcd, mxtake)

! UNCONSTRAINED MINIMIZATION STOPPING CRITERIA
! --------------------------------------------
! FIND WHETHER THE ALGORITHM SHOULD TERMINATE, DUE TO ANY OF THE FOLLOWING:
! 1) PROBLEM SOLVED WITHIN USER TOLERANCE
! 2) CONVERGENCE WITHIN USER TOLERANCE
! 3) ITERATION LIMIT REACHED
! 4) DIVERGENCE OR TOO RESTRICTIVE MAXIMUM STEP (STEPMX) SUSPECTED

! PARAMETERS
! ----------
! N            --> DIMENSION OF PROBLEM
! XPLS(N)      --> NEW ITERATE X[K]
! FPLS         --> FUNCTION VALUE AT NEW ITERATE F(XPLS)
! GPLS(N)      --> GRADIENT AT NEW ITERATE, G(XPLS), OR APPROXIMATE
! X(N)         --> OLD ITERATE X[K-1]
! ITNCNT       --> CURRENT ITERATION K
! ICSCMX      <--> NUMBER CONSECUTIVE STEPS .GE. STEPMX
!                  [RETAIN VALUE BETWEEN SUCCESSIVE CALLS]
! ITRMCD      <--  TERMINATION CODE
! GRADTL       --> TOLERANCE AT WHICH RELATIVE GRADIENT CONSIDERED CLOSE
!                  ENOUGH TO ZERO TO TERMINATE ALGORITHM
! STEPTL       --> RELATIVE STEP SIZE AT WHICH SUCCESSIVE ITERATES
!                  CONSIDERED CLOSE ENOUGH TO TERMINATE ALGORITHM
! SX(N)        --> DIAGONAL SCALING MATRIX FOR X
! FSCALE       --> ESTIMATE OF SCALE OF OBJECTIVE FUNCTION
! ITNLIM       --> MAXIMUM NUMBER OF ALLOWABLE ITERATIONS
! IRETCD       --> RETURN CODE
! MXTAKE       --> BOOLEAN FLAG INDICATING STEP OF MAXIMUM LENGTH USED

INTEGER, INTENT(IN)      :: n
REAL (dp), INTENT(IN)    :: xpls(:)
REAL (dp), INTENT(IN)    :: fpls
REAL (dp), INTENT(IN)    :: gpls(:)
REAL (dp), INTENT(IN)    :: x(:)
INTEGER, INTENT(IN)      :: itncnt
INTEGER, INTENT(IN OUT)  :: icscmx
INTEGER, INTENT(OUT)     :: itrmcd
REAL (dp), INTENT(IN)    :: gradtl
REAL (dp), INTENT(IN)    :: steptl
REAL (dp), INTENT(IN)    :: sx(:)
REAL (dp), INTENT(IN)    :: fscale
INTEGER, INTENT(IN)      :: itnlim
INTEGER, INTENT(IN)      :: iretcd
LOGICAL, INTENT(IN)      :: mxtake

INTEGER   :: i, jtrmcd
REAL (dp) :: d, rgx
REAL (dp) :: relgrd, relstp, rsx

itrmcd = 0

! LAST GLOBAL STEP FAILED TO LOCATE A POINT LOWER THAN X
IF(iretcd == 1) THEN
  jtrmcd = 3
  GO TO 600
END IF

! FIND DIRECTION IN WHICH RELATIVE GRADIENT MAXIMUM.
! CHECK WHETHER WITHIN TOLERANCE

d = MAX(ABS(fpls),fscale)
rgx = zero
DO i = 1,n
  relgrd = ABS(gpls(i)) * MAX(ABS(xpls(i)), one/sx(i))/d
  rgx = MAX(rgx,relgrd)
END DO
jtrmcd = 1
IF(rgx <= gradtl) GO TO 600

IF(itncnt == 0) RETURN

! FIND DIRECTION IN WHICH RELATIVE STEPSIZE MAXIMUM
! CHECK WHETHER WITHIN TOLERANCE.

rsx = zero
DO i = 1,n
  relstp = ABS(xpls(i) - x(i)) / MAX(ABS(xpls(i)), one/sx(i))
  rsx = MAX(rsx,relstp)
END DO
jtrmcd = 2
IF(rsx <= steptl) GO TO 600

! CHECK ITERATION LIMIT

jtrmcd = 4
IF(itncnt >= itnlim) GO TO 600

! CHECK NUMBER OF CONSECUTIVE STEPS \ STEPMX

IF(.NOT. mxtake) THEN
  icscmx = 0
  RETURN
END IF

icscmx = icscmx + 1
IF(icscmx < 5) RETURN
jtrmcd = 5

600 itrmcd = jtrmcd

RETURN
END SUBROUTINE optstp



SUBROUTINE sndofd(n, xpls, fcn, fpls, a, sx, rnoise, &
                  qrank, meqns, nvars, fc, anls, aja, s)
! PURPOSE
! -------
! FIND SECOND ORDER FORWARD FINITE DIFFERENCE APPROXIMATION "A"
! TO THE SECOND DERIVATIVE (HESSIAN) OF THE FUNCTION DEFINED BY THE SUBP
! "FCN" EVALUATED AT THE NEW ITERATE "XPLS"

! FOR OPTIMIZATION USE THIS ROUTINE TO ESTIMATE
! 1) THE SECOND DERIVATIVE (HESSIAN) OF THE OPTIMIZATION FUNCTION
!    IF NO ANALYTICAL USER FUNCTION HAS BEEN SUPPLIED FOR EITHER
!    THE GRADIENT OR THE HESSIAN AND IF THE OPTIMIZATION FUNCTION
!    "FCN" IS INEXPENSIVE TO EVALUATE.

! PARAMETERS
! ----------
! N            --> DIMENSION OF PROBLEM
! XPLS(N)      --> NEW ITERATE:   X[K]
! FCN          --> NAME OF SUBROUTINE TO EVALUATE FUNCTION
! FPLS         --> FUNCTION VALUE AT NEW ITERATE, F(XPLS)
! A(N,N)      <--  FINITE DIFFERENCE APPROXIMATION TO HESSIAN
!                  ONLY LOWER TRIANGULAR MATRIX AND DIAGONAL ARE RETURNED
! SX(N)        --> DIAGONAL SCALING MATRIX FOR X
! RNOISE       --> RELATIVE NOISE IN FNAME [F(X)]

INTEGER, INTENT(IN)        :: n
REAL (dp), INTENT(IN OUT)  :: xpls(:)
REAL (dp), INTENT(IN)      :: fpls
REAL (dp), INTENT(OUT)     :: a(:,:)
REAL (dp), INTENT(IN)      :: sx(:)
REAL (dp), INTENT(IN)      :: rnoise
INTEGER, INTENT(IN)        :: qrank, meqns, nvars
REAL (dp), INTENT(IN OUT)  :: fc(:), anls(:,:), aja(:,:), s(:,:)

INTERFACE
  SUBROUTINE fcn(n, x, f, qrank, meqns, nvars, fc, anls, aja, s)
    IMPLICIT NONE
    INTEGER, PARAMETER     :: dp = SELECTED_REAL_KIND(14, 60)
    INTEGER, INTENT(IN)    :: n
    REAL (dp), INTENT(IN)  :: x(:)
    REAL (dp), INTENT(OUT) :: f
    INTEGER, INTENT(IN)       :: qrank, meqns, nvars
    REAL (dp), INTENT(IN OUT) :: fc(:), anls(:,:), aja(:,:), s(:,:)
  END SUBROUTINE fcn
END INTERFACE

! Workspace
! STEPSZ(N)    --> WORKSPACE (STEPSIZE IN I-TH COMPONENT DIRECTION)
! ANBR(N)      --> WORKSPACE (NEIGHBOR IN I-TH DIRECTION)
REAL (dp) :: stepsz(n), anbr(n)

INTEGER   :: i,j,ip1
REAL (dp) :: ov3,xtmpi,xtmpj,fhat

! FIND I-TH STEPSIZE AND EVALUATE NEIGHBOR IN DIRECTION OF I-TH UNIT VECTOR.

ov3 = one/3.0_dp
DO i = 1,n
  stepsz(i) = rnoise**ov3 * MAX(ABS(xpls(i)), one/sx(i))
  xtmpi = xpls(i)
  xpls(i) = xtmpi + stepsz(i)
  CALL fcn(n,xpls,anbr(i),qrank, meqns, nvars, fc, anls, aja, s)
  xpls(i) = xtmpi
END DO

! CALCULATE COLUMN I OF A

DO i = 1,n
  xtmpi = xpls(i)
  xpls(i) = xtmpi + 2.0*stepsz(i)
  CALL fcn(n,xpls,fhat,qrank, meqns, nvars, fc, anls, aja, s)
  a(i,i) = ((fpls - anbr(i)) + (fhat-anbr(i))) / (stepsz(i)*stepsz(i))

! CALCULATE SUB-DIAGONAL ELEMENTS OF COLUMN
  IF(i == n) GO TO 25
  xpls(i) = xtmpi + stepsz(i)
  ip1 = i + 1
  DO j = ip1,n
    xtmpj = xpls(j)
    xpls(j) = xtmpj + stepsz(j)
    CALL fcn(n,xpls,fhat,qrank, meqns, nvars, fc, anls, aja, s)
    a(j,i) = ((fpls - anbr(i)) + (fhat-anbr(j))) / (stepsz(i)*stepsz(j))
    xpls(j) = xtmpj
  END DO
  25 xpls(i) = xtmpi
END DO
RETURN
END SUBROUTINE sndofd



SUBROUTINE tregup(n, x, f, g, a, fcn, sc, sx, nwtake, stepmx, steptl, dlt,  &
                  iretcd, xplsp, fplsp, xpls, fpls, mxtake, method, udiag, &
                  qrank, meqns, nvars, fc, anls, aja, s)

! PURPOSE
! -------
! DECIDE WHETHER TO ACCEPT XPLS = X+SC AS THE NEXT ITERATE AND UPDATE THE
! TRUST REGION DLT.

! PARAMETERS
! ----------
! N            --> DIMENSION OF PROBLEM
! X(N)         --> OLD ITERATE X[K-1]
! F            --> FUNCTION VALUE AT OLD ITERATE, F(X)
! G(N)         --> GRADIENT AT OLD ITERATE, G(X), OR APPROXIMATE
! A(N,N)       --> CHOLESKY DECOMPOSITION OF HESSIAN IN
!                  LOWER TRIANGULAR PART AND DIAGONAL.
!                  HESSIAN OR APPROX IN UPPER TRIANGULAR PART
! FCN          --> NAME OF SUBROUTINE TO EVALUATE FUNCTION
! SC(N)        --> CURRENT STEP
! SX(N)        --> DIAGONAL SCALING MATRIX FOR X
! NWTAKE       --> BOOLEAN, =.TRUE. IF NEWTON STEP TAKEN
! STEPMX       --> MAXIMUM ALLOWABLE STEP SIZE
! STEPTL       --> RELATIVE STEP SIZE AT WHICH SUCCESSIVE ITERATES
!                  CONSIDERED CLOSE ENOUGH TO TERMINATE ALGORITHM
! DLT         <--> TRUST REGION RADIUS
! IRETCD      <--> RETURN CODE
!                     = 0 XPLS ACCEPTED AS NEXT ITERATE;
!                       DLT TRUST REGION FOR NEXT ITERATION.
!                     = 1 XPLS UNSATISFACTORY BUT ACCEPTED AS NEXT ITERATE
!                       BECAUSE XPLS-X .LT. SMALLEST ALLOWABLE STEP LENGTH.
!                     = 2 F(XPLS) TOO LARGE.  CONTINUE CURRENT ITERATION
!                       WITH NEW REDUCED DLT.
!                     = 3 F(XPLS) SUFFICIENTLY SMALL, BUT QUADRATIC MODEL
!                       PREDICTS F(XPLS) SUFFICIENTLY WELL TO CONTINUE
!                       CURRENT ITERATION WITH NEW DOUBLED DLT.
! XPLSP(N)    <--> WORKSPACE [VALUE NEEDS TO BE RETAINED BETWEEN
!                  SUCCESSIVE CALLS OF K-TH GLOBAL STEP]
! FPLSP       <--> [RETAIN VALUE BETWEEN SUCCESSIVE CALLS]
! XPLS(N)     <--  NEW ITERATE X[K]
! FPLS        <--  FUNCTION VALUE AT NEW ITERATE, F(XPLS)
! MXTAKE      <--  BOOLEAN FLAG INDICATING STEP OF MAXIMUM LENGTH USED
! METHOD       --> ALGORITHM TO USE TO SOLVE MINIMIZATION PROBLEM
!                     = 1 LINE SEARCH
!                     = 2 DOUBLE DOGLEG
!                     = 3 MORE-HEBDON
! UDIAG(N)     --> DIAGONAL OF HESSIAN IN A(.,.)

INTEGER, INTENT(IN)        :: n
REAL (dp), INTENT(IN)      :: x(:)
REAL (dp), INTENT(IN)      :: f
REAL (dp), INTENT(IN)      :: g(:)
REAL (dp), INTENT(IN)      :: a(:,:)
REAL (dp), INTENT(IN)      :: sc(:)
REAL (dp), INTENT(IN)      :: sx(:)
LOGICAL, INTENT(IN)        :: nwtake
REAL (dp), INTENT(IN)      :: stepmx
REAL (dp), INTENT(IN)      :: steptl
REAL (dp), INTENT(IN OUT)  :: dlt
INTEGER, INTENT(IN OUT)    :: iretcd
REAL (dp), INTENT(IN OUT)  :: xplsp(:)
REAL (dp), INTENT(IN OUT)  :: fplsp
REAL (dp), INTENT(OUT)     :: xpls(:)
REAL (dp), INTENT(OUT)     :: fpls
LOGICAL, INTENT(OUT)       :: mxtake
INTEGER, INTENT(IN)        :: method
REAL (dp), INTENT(IN)      :: udiag(:)
INTEGER, INTENT(IN)        :: qrank, meqns, nvars
REAL (dp), INTENT(IN OUT)  :: fc(:), anls(:,:), aja(:,:), s(:,:)

INTERFACE
  SUBROUTINE fcn(n, x, f, qrank, meqns, nvars, fc, anls, aja, s)
    IMPLICIT NONE
    INTEGER, PARAMETER     :: dp = SELECTED_REAL_KIND(14, 60)
    INTEGER, INTENT(IN)    :: n
    REAL (dp), INTENT(IN)  :: x(:)
    REAL (dp), INTENT(OUT) :: f
    INTEGER, INTENT(IN)       :: qrank, meqns, nvars
    REAL (dp), INTENT(IN OUT) :: fc(:), anls(:,:), aja(:,:), s(:,:)
  END SUBROUTINE fcn
END INTERFACE

INTEGER   :: i, j, ip1
REAL (dp) :: slp, rln
REAL (dp) :: dltmp, dltfp, temp, dltf

mxtake = .false.
xpls(1:n) = x(1:n) + sc(1:n)
CALL fcn(n, xpls, fpls,qrank, meqns, nvars, fc, anls, aja, s)
dltf = fpls - f
slp = DOT_PRODUCT( g(1:n), sc(1:n) )

! NEXT STATEMENT ADDED FOR CASE OF COMPILERS WHICH DO NOT OPTIMIZE
! EVALUATION OF NEXT "IF" STATEMENT (IN WHICH CASE FPLSP COULD BE UNDEFINED).
IF(iretcd == 4) fplsp = zero
IF(iretcd /= 3 .OR. (fpls < fplsp .AND. dltf <= 1.e-4*slp)) GO TO 130

!     IF(IRETCD.EQ.3 .AND. (FPLS.GE.FPLSP .OR. DLTF.GT. 1.E-4*SLP))
!     THEN

!       RESET XPLS TO XPLSP AND TERMINATE GLOBAL STEP

iretcd = 0
xpls(1:n) = xplsp(1:n)
fpls = fplsp
dlt = .5*dlt
GO TO 230
!     ELSE

!       FPLS TOO LARGE

130 IF(dltf <= 1.e-4*slp) GO TO 170
!       IF(DLTF.GT. 1.E-4*SLP)
!       THEN
rln = zero
DO i = 1,n
  rln = MAX(rln,ABS(sc(i)) / MAX(ABS(xpls(i)),1./sx(i)))
END DO
IF(rln >= steptl) GO TO 150
!         IF(RLN.LT.STEPTL)
!         THEN

!           CANNOT FIND SATISFACTORY XPLS SUFFICIENTLY DISTINCT FROM X

iretcd = 1
GO TO 230
!         ELSE

!           REDUCE TRUST REGION AND CONTINUE GLOBAL STEP

150 iretcd = 2
dltmp = -slp*dlt/(2.*(dltf-slp))
IF(dltmp >= .1*dlt) GO TO 155
!           IF(DLTMP.LT. .1*DLT)
!           THEN
dlt = .1*dlt
GO TO 160
!           ELSE
155 dlt = dltmp
!           ENDIF

160 GO TO 230
!         ENDIF
!       ELSE

!         FPLS SUFFICIENTLY SMALL

170 dltfp = zero
IF (method == 3) GO TO 180

!         IF (METHOD .EQ. 2)
!         THEN

DO i = 1, n
  temp = DOT_PRODUCT( a(i:n,i), sc(i:n) )
  dltfp = dltfp + temp*temp
END DO
GO TO 190

!         ELSE

180 DO i = 1, n
  dltfp = dltfp + udiag(i)*sc(i)*sc(i)
  IF (i == n) CYCLE
  temp = 0
  ip1 = i + 1
  DO j = ip1, n
    temp = temp + a(i, j)*sc(i)*sc(j)
  END DO
  dltfp = dltfp + 2.0*temp
END DO

!         END IF

190 dltfp = slp + dltfp/2.0
IF(iretcd == 2 .OR. (ABS(dltfp-dltf) > .1*ABS(dltf))  &
    .OR. nwtake .OR. (dlt > .99*stepmx)) GO TO 210
!         IF(IRETCD.NE.2 .AND. (ABS(DLTFP-DLTF) .LE. .1*ABS(DLTF))
!    +         .AND. (.NOT.NWTAKE) .AND. (DLT.LE. .99*STEPMX))
!         THEN

!           DOUBLE TRUST REGION AND CONTINUE GLOBAL STEP

iretcd = 3
xplsp(1:n) = xpls(1:n)
fplsp = fpls
dlt = MIN(2.0D0*dlt,stepmx)
GO TO 230
!         ELSE

!           ACCEPT XPLS AS NEXT ITERATE.  CHOOSE NEW TRUST REGION.

210 iretcd = 0
IF(dlt > .99*stepmx) mxtake = .true.
IF(dltf < .1*dltfp) GO TO 220
!           IF(DLTF.GE. .1*DLTFP)
!           THEN

!             DECREASE TRUST REGION FOR NEXT ITERATION

dlt = .5*dlt
GO TO 230
!           ELSE

!             CHECK WHETHER TO INCREASE TRUST REGION FOR NEXT ITERATION

220 IF(dltf <= .75*dltfp) dlt = MIN(2.*dlt,stepmx)
!           ENDIF
!         ENDIF
!       ENDIF
!     ENDIF

230 RETURN
END SUBROUTINE tregup

END MODULE unconstrained_min
