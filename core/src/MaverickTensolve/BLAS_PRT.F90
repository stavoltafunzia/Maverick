MODULE blas_part
! Contains only DNRM2, DSWAP, IDAMAX & DGEMV

! This very much simplified BLAS module for use by TOMS 768 is by Alan Miller
! alan @ vic.cmis.csiro.au    URL: www.ozemail.com.au/~milleraj

! Latest revision - 19 January 1999

IMPLICIT NONE

INTEGER, PARAMETER, PRIVATE   :: dp = SELECTED_REAL_KIND(14, 60)
REAL (dp), PARAMETER, PRIVATE :: zero = 0.0_dp, one = 1.0_dp


CONTAINS


FUNCTION dnrm2 ( n, x, incx) RESULT(fn_val)

!  Euclidean norm of the n-vector stored in x() with storage increment incx .
!  if n <= 0 return with result = 0.
!  if n >= 1 then incx must be >= 1

!  c.l.lawson, 1978 jan 08
!  modified to correct failure to update ix, 1/25/92.
!  modified 3/93 to return if incx <= 0.
!  This version by Alan.Miller @ vic.cmis.csiro.au
!  Latest revision - 22 January 1999

!  four phase method using two built-in constants that are
!  hopefully applicable to all machines.
!      cutlo = maximum of  SQRT(u/eps)  over all known machines.
!      cuthi = minimum of  SQRT(v)      over all known machines.
!  where
!      eps = smallest no. such that eps + 1. > 1.
!      u   = smallest positive no.   (underflow limit)
!      v   = largest  no.            (overflow  limit)

!  brief outline of algorithm..

!  phase 1    scans zero components.
!  move to phase 2 when a component is nonzero and <= cutlo
!  move to phase 3 when a component is > cutlo
!  move to phase 4 when a component is >= cuthi/m
!  where m = n for x() real and m = 2*n for complex.

INTEGER, INTENT(IN)   :: n, incx
REAL (dp), INTENT(IN) :: x(:)
REAL (dp)             :: fn_val

! Local variables
INTEGER              :: i, ix, j, next
REAL (dp)            :: cuthi, cutlo, hitest, sum, xmax

IF(n <= 0 .OR. incx <= 0) THEN
  fn_val = zero
  RETURN
END IF

! Set machine-dependent constants

cutlo = SQRT( TINY(one) / EPSILON(one) )
cuthi = SQRT( HUGE(one) )

next = 1
sum = zero
i = 1
ix = 1
!                                                 begin main loop
20 SELECT CASE (next)
  CASE (1)
     IF( ABS(x(i)) > cutlo) GO TO 85
     next = 2
     xmax = zero
     GO TO 20

  CASE (2)
!                   phase 1.  sum is zero

     IF( x(i) == zero) GO TO 200
     IF( ABS(x(i)) > cutlo) GO TO 85

!                                prepare for phase 2.   x(i) is very small.
     next = 3
     GO TO 105

  CASE (3)
!                   phase 2.  sum is small.
!                             scale to avoid destructive underflow.

     IF( ABS(x(i)) > cutlo ) THEN
!                  prepare for phase 3.

       sum = (sum * xmax) * xmax
       GO TO 85
     END IF

  CASE (4)
     GO TO 110
END SELECT

! ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
!                     common code for phases 2 and 4.
!                     in phase 4 sum is large.  scale to avoid overflow.

110 IF( ABS(x(i)) <= xmax ) GO TO 115
sum = one + sum * (xmax / x(i))**2
xmax = ABS(x(i))
GO TO 200

! ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

!                   phase 3.  sum is mid-range.  no scaling.

!     for real or d.p. set hitest = cuthi/n
!     for complex      set hitest = cuthi/(2*n)

85 hitest = cuthi / REAL( n, dp )

DO j = ix, n
  IF(ABS(x(i)) >= hitest) GO TO 100
  sum = sum + x(i)**2
  i = i + incx
END DO
fn_val = SQRT( sum )
RETURN

! ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

!                                prepare for phase 4.
!                                ABS(x(i)) is very large
100 ix = j
next = 4
sum = (sum / x(i)) / x(i)
!                                Set xmax; large if next = 4, small if next = 3
105 xmax = ABS(x(i))

115 sum = sum + (x(i)/xmax)**2

200 ix = ix + 1
i = i + incx
IF( ix <= n ) GO TO 20

!              end of main loop.

!              compute square root and adjust for scaling.

fn_val = xmax * SQRT(sum)

RETURN
END FUNCTION dnrm2




SUBROUTINE dswap (n, x, incx, y, incy)

!     interchanges two vectors.

INTEGER, INTENT(IN)       :: n, incx, incy
REAL (dp), INTENT(IN OUT) :: x(:), y(:)

! Local variables
REAL (dp) :: temp(n)

IF(n <= 0) RETURN
IF(incx == 1 .AND. incy == 1) THEN
  temp = x(:n)
  x(:n) = y(:n)
  y(:n) = temp
  RETURN
END IF

temp = x(:n*incx:incx)
x(:n*incx:incx) = y(:n*incy:incy)
y(:n*incy:incy) = temp

RETURN
END SUBROUTINE dswap


FUNCTION idamax(n, x, incx) RESULT(fn_val)

!     finds the index of element having max. absolute value.
!     jack dongarra, linpack, 3/11/78.
!     modified 3/93 to return if incx .le. 0.
!     modified 12/3/93, array(1) declarations changed to array(*)

INTEGER, INTENT(IN)   :: n, incx
REAL (dp), INTENT(IN) :: x(:)
INTEGER               :: fn_val

INTEGER :: imax(1)

fn_val = 0
IF( n < 1 .OR. incx <= 0 ) RETURN
fn_val = 1
IF(n == 1) RETURN
IF(incx == 1) THEN
  imax = MAXLOC( ABS(x(:n)) )
ELSE
  imax = MAXLOC( ABS(x(:n*incx:incx)) )
END IF
fn_val = imax(1)

RETURN
END FUNCTION idamax


SUBROUTINE dgemv ( trans, m, n, alpha, a, lda, x, incx, beta, y, incy )
! ELF90 translation by Alan Miller   31-Aug-1997

!     .. Scalar Arguments ..
REAL (dp), INTENT(IN)         :: alpha, beta
INTEGER, INTENT(IN)           :: incx, incy, lda, m, n
CHARACTER (LEN=1), INTENT(IN) :: trans
!     .. Array Arguments ..
REAL (dp), INTENT(IN)         :: a(:,:), x(:)
REAL (dp), INTENT(IN OUT)     :: y(:)
!     ..

!  Purpose
!  =======

!  DGEMV  performs one of the matrix-vector operations

!     y := alpha*A*x + beta*y,   or   y := alpha*A'*x + beta*y,

!  where alpha and beta are scalars, x and y are vectors and A is an
!  m by n matrix.

!  Parameters
!  ==========

!  TRANS  - CHARACTER*1.
!           On entry, TRANS specifies the operation to be performed as
!           follows:

!              TRANS = 'N' or 'n'   y := alpha*A*x + beta*y.

!              TRANS = 'T' or 't'   y := alpha*A'*x + beta*y.

!              TRANS = 'C' or 'c'   y := alpha*A'*x + beta*y.

!           Unchanged on exit.

!  M      - INTEGER.
!           On entry, M specifies the number of rows of the matrix A.
!           M must be at least zero.
!           Unchanged on exit.

!  N      - INTEGER.
!           On entry, N specifies the number of columns of the matrix A.
!           N must be at least zero.
!           Unchanged on exit.

!  ALPHA  - DOUBLE PRECISION.
!           On entry, ALPHA specifies the scalar alpha.
!           Unchanged on exit.

!  A      - DOUBLE PRECISION array of DIMENSION ( LDA, n ).
!           Before entry, the leading m by n part of the array A must
!           contain the matrix of coefficients.
!           Unchanged on exit.

!  LDA    - INTEGER.
!           On entry, LDA specifies the first dimension of A as declared
!           in the calling (sub) program. LDA must be at least max( 1, m ).
!           Unchanged on exit.

!  X      - DOUBLE PRECISION array of DIMENSION at least
!           ( 1 + ( n - 1 )*abs( INCX ) ) when TRANS = 'N' or 'n'
!           and at least
!           ( 1 + ( m - 1 )*abs( INCX ) ) otherwise.
!           Before entry, the incremented array X must contain the
!           vector x.
!           Unchanged on exit.

!  INCX   - INTEGER.
!           On entry, INCX specifies the increment for the elements of
!           X. INCX must not be zero.
!           Unchanged on exit.

!  BETA   - DOUBLE PRECISION.
!           On entry, BETA specifies the scalar beta. When BETA is
!           supplied as zero then Y need not be set on input.
!           Unchanged on exit.

!  Y      - DOUBLE PRECISION array of DIMENSION at least
!           ( 1 + ( m - 1 )*abs( INCY ) ) when TRANS = 'N' or 'n'
!           and at least
!           ( 1 + ( n - 1 )*abs( INCY ) ) otherwise.
!           Before entry with BETA non-zero, the incremented array Y
!           must contain the vector y. On exit, Y is overwritten by the
!           updated vector y.

!  INCY   - INTEGER.
!           On entry, INCY specifies the increment for the elements of
!           Y. INCY must not be zero.
!           Unchanged on exit.


!  Level 2 Blas routine.

!  -- Written on 22-October-1986.
!     Jack Dongarra, Argonne National Lab.
!     Jeremy Du Croz, Nag Central Office.
!     Sven Hammarling, Nag Central Office.
!     Richard Hanson, Sandia National Labs.


!     .. Local Scalars ..
REAL (dp) :: temp
INTEGER   ::  i, info, ix, iy, j, jx, jy, kx, ky, lenx, leny
!     .. External Functions ..
! LOGICAL ::  lsame
! EXTERNAL           lsame
!     .. External Subroutines ..
! EXTERNAL           erinfo
!     .. Intrinsic Functions ..
! INTRINSIC          MAX
!     ..
!     .. Executable Statements ..

!     Test the input parameters.

info = 0
IF ( trans /= 'N' .AND. trans /= 'n' .AND. trans /= 'T' .AND. trans /= 't' &
     .AND. trans /= 'C' .AND. trans /= 'c') THEN
  info = 1
ELSE IF( m < 0 ) THEN
  info = 2
ELSE IF( n < 0 ) THEN
  info = 3
ELSE IF( lda < MAX( 1, m ) ) THEN
  info = 6
ELSE IF( incx == 0 ) THEN
  info = 8
ELSE IF( incy == 0 ) THEN
  info = 11
END IF
IF( info /= 0 ) THEN
  WRITE(*, '(a, i4, a)') ' Error number: ', info, ' in BLAS2 routine DGEMV'
  RETURN
END IF

!     Quick return if possible.

IF( ( m == 0 ) .OR. ( n == 0 ).OR.  &
    ( ( alpha == zero ) .AND. ( beta == one ) ) )RETURN

!     Set  LENX  and  LENY, the lengths of the vectors x and y, and set
!     up the start points in  X  and  Y.

IF ( trans == 'N' .OR. trans == 'n' ) THEN
  lenx = n
  leny = m
ELSE
  lenx = m
  leny = n
END IF
IF( incx > 0 ) THEN
  kx = 1
ELSE
  kx = 1 - ( lenx - 1 )*incx
END IF
IF( incy > 0 ) THEN
  ky = 1
ELSE
  ky = 1 - ( leny - 1 )*incy
END IF

!     Start the operations. In this version the elements of A are
!     accessed sequentially with one pass through A.

!     First form  y := beta*y.

IF( beta /= one ) THEN
  IF( incy == 1 ) THEN
    IF( beta == zero ) THEN
      y( :leny ) = zero
    ELSE
      y( :leny ) = beta*y( :leny )
    END IF
  ELSE
    iy = ky
    IF( beta == zero ) THEN
      DO i = 1, leny
        y( iy ) = zero
        iy      = iy   + incy
      END DO
    ELSE
      DO i = 1, leny
        y( iy ) = beta*y( iy )
        iy      = iy           + incy
      END DO
    END IF
  END IF
END IF
IF( alpha == zero ) RETURN
IF ( trans == 'N' .OR. trans == 'n' ) THEN

!        Form  y := alpha*A*x + y.

  jx = kx
  IF( incy == 1 ) THEN
    DO j = 1, n
      IF( x( jx ) /= zero ) THEN
        temp = alpha*x( jx )
        y( 1:m ) = y( 1:m ) + temp*a( 1:m, j )
      END IF
      jx = jx + incx
    END DO
  ELSE
    DO j = 1, n
      IF( x( jx ) /= zero ) THEN
        temp = alpha*x( jx )
        iy   = ky
        DO i = 1, m
          y( iy ) = y( iy ) + temp*a( i, j )
          iy      = iy      + incy
        END DO
      END IF
      jx = jx + incx
    END DO
  END IF
ELSE

!        Form  y := alpha*A'*x + y.

  jy = ky
  IF( incx == 1 ) THEN
    DO j = 1, n
      temp = DOT_PRODUCT( a(1:m,j), x(1:m) )
      y( jy ) = y( jy ) + alpha*temp
      jy      = jy      + incy
    END DO
  ELSE
    DO j = 1, n
      temp = zero
      ix   = kx
      DO i = 1, m
        temp = temp + a( i, j )*x( ix )
        ix   = ix   + incx
      END DO
      y( jy ) = y( jy ) + alpha*temp
      jy      = jy      + incy
    END DO
  END IF
END IF

RETURN

!     End of DGEMV.

END SUBROUTINE dgemv

END MODULE blas_part
