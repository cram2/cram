;; Random number generation
;; Liam Healy, Tue Jul 11 2006 - 23:39
;; Time-stamp: <2010-06-27 18:13:40EDT rng-types.lisp>
;;
;; Copyright 2006, 2007, 2008, 2009 Liam M. Healy
;; Distributed under the terms of the GNU General Public License
;;
;; This program is free software: you can redistribute it and/or modify
;; it under the terms of the GNU General Public License as published by
;; the Free Software Foundation, either version 3 of the License, or
;; (at your option) any later version.
;;
;; This program is distributed in the hope that it will be useful,
;; but WITHOUT ANY WARRANTY; without even the implied warranty of
;; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;; GNU General Public License for more details.
;;
;; You should have received a copy of the GNU General Public License
;; along with this program.  If not, see <http://www.gnu.org/licenses/>.

;;; Random number generator types and information functions.

(in-package :gsl)

;;;;****************************************************************************
;;;; Auxiliary functions
;;;;****************************************************************************

(export '(all-random-number-generators))

(defmfun rng-types-setup ()
  "gsl_rng_types_setup" ()
  :c-return :pointer
  :export nil
  :documentation
  "A pointer to an array of all the available generator types,
   terminated by a null pointer. The function should be
   called once at the start of the program, if needed.
   Users should call all-random-number-generators.")

(defun all-random-number-generators ()
  "A list of all random number generator types."
  (let ((start (rng-types-setup)))
    (loop for i from 0
       for ptr
       = (cffi:mem-ref 
	  (cffi:inc-pointer start (* (cffi:foreign-type-size :pointer) i))
	  :pointer)
       until (cffi:null-pointer-p ptr)
       collect ptr)))

;;;;****************************************************************************
;;;; Defining RNGs and default
;;;;****************************************************************************

(defmacro def-rng-type (lisp-name &optional documentation gsl-name gsl-version)
  "Define the random number generator type."
  (let ((cname
	 (or gsl-name
	     (remove #\+
		     (substitute
		      #\_ #\- 
		      (format nil "gsl_rng_~(~a~)" lisp-name))))))
    `(defmpar ,lisp-name ,cname ,documentation :gsl-version ,gsl-version)))

(def-rng-type +default-type+
    "The default random number generator type,
     set by environment variables GSL_RNG_TYPE and GSL_RNG_SEED"
  "gsl_rng_default")

(defmfun rng-environment-setup ()
  "gsl_rng_env_setup" ()
  :c-return :pointer
  :documentation			; FDL
  "Read the environment variables GSL_RNG_TYPE and
  GSL_RNG_SEED and use their values to set the corresponding
  library variables *default-type* and *default-seed*")

;;;;****************************************************************************
;;;; Modern random number generators
;;;;****************************************************************************

(def-rng-type +cmrg+			; FDL
    "Combined multiple recursive random number generator
     This is a combined multiple recursive generator by L'Ecuyer. 
     Its sequence is z_n = (x_n - y_n) mod m_1
     where the two underlying generators x_n and y_n are,
     x_n = (a_1 x_{n-1} + a_2 x_{n-2} + a_3 x_{n-3}) mod m_1
     y_n = (b_1 y_{n-1} + b_2 y_{n-2} + b_3 y_{n-3}) mod m_2
     with coefficients 
     a_1 = 0, a_2 = 63308, a_3 = -183326,
     b_1 = 86098, b_2 = 0, b_3 = -539608,
     and moduli 
     m_1 = 2^31 - 1 = 2147483647 and m_2 = 2145483479.
     The period of this generator is 2^205 (about 10^61).  It uses
     6 words of state per generator.  For more information see,
     P. L'Ecuyer, ``Combined Multiple Recursive Random Number
     Generators'', Operations Research, 44, 5 (1996), 816--822.")

(def-rng-type +gfsr4+			; FDL
    "Four-tap Generalized Feedback Shift Register
    The *gfsr4* generator is like a lagged-Fibonacci generator, and 
    produces each number as an xor'd sum of four previous values.
    r_n = r_{n-A} \oplus r_{n-B} \oplus r_{n-C} \oplus r_{n-D}
    Ziff (ref below) notes that ``it is now widely known'' that two-tap
    registers (such as R250, which is described below)
    have serious flaws, the most obvious one being the three-point
    correlation that comes from the definition of the generator.  Nice
    mathematical properties can be derived for GFSR's, and numerics bears
    out the claim that 4-tap GFSR's with appropriately chosen offsets are as
    random as can be measured, using the author's test.

    This implementation uses the values suggested the example on p392 of
    Ziff's article: A=471, B=1586, C=6988, D=9689.

    If the offsets are appropriately chosen (such as the one ones in this
    implementation), then the sequence is said to be maximal; that means
    that the period is 2^D - 1, where D is the longest lag.
    (It is one less than 2^D because it is not permitted to have all
    zeros in the ra array.)  For this implementation with
    D=9689 that works out to about 10^2917.

    Note that the implementation of this generator using a 32-bit
    integer amounts to 32 parallel implementations of one-bit
    generators.  One consequence of this is that the period of this
    32-bit generator is the same as for the one-bit generator.
    Moreover, this independence means that all 32-bit patterns are
    equally likely, and in particular that 0 is an allowed random
    value.  (We are grateful to Heiko Bauke for clarifying for us these
    properties of GFSR random number generators.)

    For more information see,
    Robert M. Ziff, ``Four-tap shift-register-sequence random-number 
    generators'', Computers in Physics, 12(4), Jul/Aug
    1998, pp 385--392.")

(def-rng-type +mrg+			; FDL
    "Multiple recursive random number generator
   This is a fifth-order multiple recursive generator by L'Ecuyer, Blouin
   and Coutre.  Its sequence is
   x_n = (a_1 x_{n-1} + a_5 x_{n-5}) mod m
   with a_1 = 107374182, a_2 = a_3 = a_4 = 0, a_5 = 104480 and m = 2^31 - 1.
   The period of this generator is about 10^46.  It uses 5 words
   of state per generator.  More information can be found in the following
   paper,
    P. L'Ecuyer, F. Blouin, and R. Coutre, ``A search for good multiple
    recursive random number generators'', ACM Transactions on Modeling and
    Computer Simulation 3, 87--98 (1993).")

(def-rng-type +mt19937+			; FDL
    "The MT19937 generator of Makoto Matsumoto and Takuji Nishimura is a
    variant of the twisted generalized feedback shift-register algorithm,
    and is known as the ``Mersenne Twister'' generator.  It has a Mersenne
    prime period of 2^19937 - 1 (about 10^6000) and is
    equi-distributed in 623 dimensions.  It has passed the diehard
    statistical tests.  It uses 624 words of state per generator and is
    comparable in speed to the other generators.  The original generator used
    a default seed of 4357 and choosing s equal to zero in #'rng-set
    reproduces this.   For more information see,
    Makoto Matsumoto and Takuji Nishimura, ``Mersenne Twister: A
    623-dimensionally equidistributed uniform pseudorandom number
    generator'' ACM Transactions on Modeling and Computer
    Simulation, Vol. 8, No. 1 (Jan. 1998), Pages 3--30
    This generator uses the second revision of the
    seeding procedure published by the two authors above in 2002.  The
    original seeding procedures could cause spurious artifacts for some seed
    values. They are still available through the alternative generators")
									 
(def-rng-type +mt19937-1999+		; FDL
    "Previous version of mt19937.")

(def-rng-type +mt19937-1998+		; FDL
    "Previous version of mt19937.")

(def-rng-type +ran0+)

(def-rng-type +ran1+)

(def-rng-type +ran2+)

(def-rng-type +ran3+)

(def-rng-type +ranlux+			; FDL
    "The ranlux generator is an implementation of the original
    algorithm developed by Lüscher.  It uses a
    lagged-fibonacci-with-skipping algorithm to produce ``luxury random
    numbers''.  It is a 24-bit generator, originally designed for
    single-precision IEEE floating point numbers.  This implementation is
    based on integer arithmetic, while the second-generation versions
    ranlxs and ranlxd described above provide floating-point
    implementations which will be faster on many platforms.
    The period of the generator is about
    10^171.  The algorithm has mathematically proven properties and
    it can provide truly decorrelated numbers at a known level of
    randomness.  The default level of decorrelation recommended by Lüscher
    is provided by this generator, while *ranlux389*
    gives the highest level of randomness, with all 24 bits decorrelated.
    Both types of generator use 24 words of state per generator.
    For more information see,
    M. Lüscher, ``A portable high-quality random number generator for
    lattice field theory calculations'', Computer Physics
    Communications, 79 (1994) 100--110.
    F. James, ``RANLUX: A Fortran implementation of the high-quality
    pseudo-random number generator of Lüscher'', Computer Physics
    Communications, 79 (1994) 111--114.")

(def-rng-type +ranlux389+)

(def-rng-type +ranlxs0+
    "This generator is a second-generation version of the
    *ranlux* algorithm of Lüscher, which produces ``luxury random
    numbers''.  This generator provides single precision output (24 bits) at
    three luxury levels *ranlxs0*, *ranlxs1* and *ranlxs2*.
    It uses double-precision floating point arithmetic internally and can be
    significantly faster than the integer version of *ranlux*,
    particularly on 64-bit architectures.  The period of the generator is
    about 10^171.  The algorithm has mathematically proven properties and
    can provide truly decorrelated numbers at a known level of randomness.
    The higher luxury levels provide increased decorrelation between samples
    as an additional safety margin.")

(def-rng-type +ranlxs1+)

(def-rng-type +ranlxs2+)

(def-rng-type +ranlxd1+			; FDL
    "Produce double precision output (48 bits) from the
    *ranlxs* generator.  The library provides two luxury levels,
    *ranlxd1* and *ranlxd2*.")

(def-rng-type +ranlxd2+			; FDL
    "Produce double precision output (48 bits) from the
    *ranlxs* generator.  The library provides two luxury levels,
    *ranlxd1* and *ranlxd2*.")

(def-rng-type +taus+			; FDL
    "Tausworthe random number generator
     This is a maximally equidistributed combined Tausworthe generator by
     L'Ecuyer.  The sequence is x_n = (s^1_n \oplus s^2_n \oplus s^3_n) 
     s1_{n+1} = (((s1_n 4294967294)<<12)^^(((s1_n<<13)^^s1_n)>>19))
     s2_{n+1} = (((s2_n 4294967288)<< 4)^^(((s2_n<< 2)^^s2_n)>>25))
     s3_{n+1} = (((s3_n 4294967280)<<17)^^(((s3_n<< 3)^^s3_n)>>11))
     computed modulo 2^32.  In the formulas above ^^}
     denotes ``exclusive-or''.  Note that the algorithm relies on the properties
     of 32-bit unsigned integers and has been implemented using a bitmask
     of 0xFFFFFFFF to make it work on 64 bit machines.
     The period of this generator is 2^88 (about
     10^26).  It uses 3 words of state per generator.  For more
     information see,
     P. L'Ecuyer, ``Maximally Equidistributed Combined Tausworthe
     Generators'', Mathematics of Computation, 65, 213 (1996), 203--213.")

(def-rng-type +taus2+			; FDL
    "The same algorithm as *taus* but with an improved seeding procedure
     described in the paper,
     P. L'Ecuyer, ``Tables of Maximally Equidistributed Combined LFSR
     Generators'', Mathematics of Computation, 68, 225 (1999), 261--269
     The generator +taus2+ should now be used in preference to +taus+.")

(def-rng-type +taus113+)

;;;;****************************************************************************
;;;; UNIX random number generators
;;;;****************************************************************************

(def-rng-type +rand+			; FDL
    "The BSD rand() generator.  Its sequence is
   x_{n+1} = (a x_n + c) mod m with a = 1103515245, 
   c = 12345 and m = 2^31.
   The seed specifies the initial value,  x_1.
   The period of this generator is 2^31, and it uses
   1 word of storage per generator.")

(def-rng-type +rand48+			; FDL
    "The Unix rand48 generator.  Its sequence is
     x_{n+1} = (a x_n + c) mod m
     defined on 48-bit unsigned integers with 
     a = 25214903917, c = 11 and m = 2^48. 
     The seed specifies the upper 32 bits of the initial value, x_1,
     with the lower 16 bits set to 0x330E.  The function
     #'get-random-number returns the upper 32 bits from each term of the
     sequence.  This does not have a direct parallel in the original
     rand48 functions, but forcing the result to type long int
     reproduces the output of mrand48.  The function
     #'uniform uses the full 48 bits of internal state to return
     the double precision number x_n/m, which is equivalent to the
     function drand48.  Note that some versions of the GNU C Library
     contained a bug in mrand48 function which caused it to produce
     different results (only the lower 16-bits of the return value were set).")

(def-rng-type +random128_bsd+)
(def-rng-type +random128_glibc2+)
(def-rng-type +random128_libc5+)
(def-rng-type +random256_bsd+)
(def-rng-type +random256_glibc2+)
(def-rng-type +random256_libc5+)
(def-rng-type +random32_bsd+)
(def-rng-type +random32_glibc2+)
(def-rng-type +random32_libc5+)
(def-rng-type +random64_bsd+)
(def-rng-type +random64_glibc2+)
(def-rng-type +random64_libc5+)
(def-rng-type +random8_bsd+)
(def-rng-type +random8_glibc2+)
(def-rng-type +random8_libc5+)
(def-rng-type +random_bsd+)
(def-rng-type +random_glibc2+)
(def-rng-type +random_libc5+)

;;;;****************************************************************************
;;;; Obsolete random number generators
;;;;****************************************************************************

;;; FDL
;;; Other random number generators

;;; The generators in this section are provided for compatibility with
;;; existing libraries.  If you are converting an existing program to use GSL
;;; then you can select these generators to check your new implementation
;;; against the original one, using the same random number generator.  After
;;; verifying that your new program reproduces the original results you can
;;; then switch to a higher-quality generator.

;;; Note that most of the generators in this section are based on single
;;; linear congruence relations, which are the least sophisticated type of
;;; generator.  In particular, linear congruences have poor properties when
;;; used with a non-prime modulus, as several of these routines do (e.g.
;;; with a power of two modulus, 2^31 or 2^32).  This
;;; leads to periodicity in the least significant bits of each number,
;;; with only the higher bits having any randomness.  Thus if you want to
;;; produce a random bitstream it is best to avoid using the least
;;; significant bits.

(def-rng-type +ranf+			; FDL
    "Obsolete, use only for compatibility.")

(def-rng-type +ranmar+			; FDL
    "Obsolete, use only for compatibility.")

(def-rng-type +r250+			; FDL
    "Obsolete, use only for compatibility.
     The shift-register generator of Kirkpatrick and Stoll.  The
     sequence is based on the recurrence
     x_n = x_{n-103} \oplus x_{n-250} where \oplus
     denotes ``exclusive-or'', defined on 32-bit words.
     The period of this generator is about 2^250 and it
     uses 250 words of state per generator.
     For more information see,
     S. Kirkpatrick and E. Stoll, ``A very fast shift-register sequence random
     number generator'', Journal of Computational Physics}, 40, 517--526
     (1981)")

(def-rng-type +tt800+			; FDL
    "Obsolete, use only for compatibility.
     An earlier version of the twisted generalized feedback
     shift-register generator, and has been superseded by the development of
     MT19937.  However, it is still an acceptable generator in its own
     right.  It has a period of 2^800 and uses 33 words of storage
     per generator.
     For more information see,
     Makoto Matsumoto and Yoshiharu Kurita, ``Twisted GFSR Generators
     II'', ACM Transactions on Modelling and Computer Simulation,
     Vol.: 4, No.: 3, 1994, pages 254--266.")

;;; The following generators are included only for historical reasons, so
;;; that you can reproduce results from old programs which might have used
;;; them.  These generators should not be used for real simulations since
;;; they have poor statistical properties by modern standards.

(def-rng-type +vax+			; FDL
    "Obsolete, use only for compatibility.")

(def-rng-type +transputer+		; FDL
    "Obsolete, use only for compatibility.")

(def-rng-type +randu+			; FDL
    "Obsolete, use only for compatibility.")

(def-rng-type +minstd+			;FDL
    "Obsolete, use only for compatibility.
    Park and Miller's ``minimal standard'' minstd generator, a
    simple linear congruence which takes care to avoid the major pitfalls of
    such algorithms.  Its sequence is x_{n+1} = (a x_n) mod m
    with a = 16807 and m = 2^31 - 1 = 2147483647. 
    The seed specifies the initial value, x_1.  The period of this
    generator is about 2^31.
    This generator is used in the IMSL Library (subroutine RNUN) and in
    MATLAB (the RAND function).  It is also sometimes known by the acronym
    ``GGL'' (I'm not sure what that stands for).
    For more information see
    Park and Miller, ``Random Number Generators: Good ones are hard to find'',
    Communications of the ACM, October 1988, Volume 31, No 10, pages
    1192--1201.")

(def-rng-type +uni+			; FDL
    "Obsolete, use only for compatibility.")

(def-rng-type +uni32+			; FDL
    "Obsolete, use only for compatibility.")

(def-rng-type +slatec+ 			; FDL
    "Obsolete, use only for compatibility.")

(def-rng-type +zuf+			; FDL
    "Obsolete, use only for compatibility.
    The ZUFALL lagged Fibonacci series generator of Peterson.  Its
    sequence is
        t = u_{n-273} + u_{n-607}
        u_n  = t - floor(t)
    The original source code is available from NETLIB.  For more information
    see
    W. Petersen, ``Lagged Fibonacci Random Number Generators for the NEC
    SX-3'', International Journal of High Speed Computing (1994).")

(def-rng-type +borosh13+		; FDL
    "Obsolete, use only for compatibility.
    The Borosh-Niederreiter random number generator. It is taken
    from Knuth's Seminumerical Algorithms, 3rd Ed., pages
    106--108. Its sequence is x_{n+1} = (a x_n) mod m
    with a = 1812433253 and m = 2^32.
    The seed specifies the initial value, x_1.")

(def-rng-type +coveyou+			; FDL
    "Obsolete, use only for compatibility.
     The Coveyou random number generator, taken from Knuth's
     Seminumerical Algorithms, 3rd Ed., Section 3.2.2. Its sequence
     is x_{n+1} = (x_n (x_n + 1)) mod m with m = 2^32.
     The seed specifies the initial value, x_1.")

(def-rng-type +fishman18+		; FDL
    "Obsolete, use only for compatibility.
     The Fishman, Moore III random number generator. It is taken from
     Knuth's Seminumerical Algorithms, 3rd Ed., pages 106--108. Its
     sequence is x_{n+1} = (a x_n) mod m with a = 62089911 and 
     m = 2^31 - 1.  The seed specifies the initial value, 
     x_1.")

(def-rng-type +fishman20+		; FDL
    "Obsolete, use only for compatibility.
     The Fishman random number generator. It is taken from Knuth's
     Seminumerical Algorithms, 3rd Ed., page 108. Its sequence is
     x_{n+1} = (a x_n) mod m with a = 48271 and 
     m = 2^31 - 1.  The seed specifies the initial value, 
     x_1.")

(def-rng-type +fishman2x+		; FDL
    "Obsolete, use only for compatibility.
     The L'Ecuyer--Fishman random number generator. It is taken from
     Knuth's Seminumerical Algorithms, 3rd Ed., page 108.
     Its sequence is z_{n+1} = (x_n - y_n) mod m with
     m = 2^31 - 1.
     x_n and y_n are given by the fishman20 and lecuyer21 algorithms.
     The seed specifies the initial value, x_1.")

(def-rng-type +knuthran+		; FDL
    "Obsolete, use only for compatibility.
    A second-order multiple recursive generator described by Knuth
    in Seminumerical Algorithms, 3rd Ed., Section 3.6.  Knuth
    provides its C code.")

(def-rng-type +knuthran2002+		; FDL
    "Obsolete, use only for compatibility.
    A second-order multiple recursive generator described by Knuth
    in Seminumerical Algorithms, 3rd Ed., Section 3.6.  Knuth
    provides its C code.  From the revised 9th printing and corrects
    some weaknesses in the earlier version, which is implemented as
    +knuthran+." "gsl_rng_knuthran2002" (1 9))

(def-rng-type +knuthran2+		; FDL
    "Obsolete, use only for compatibility.
     A second-order multiple recursive generator described by Knuth
     in Seminumerical Algorithms, 3rd Ed., page 108.  Its sequence is
     x_n = (a_1 x_{n-1} + a_2 x_{n-2}) mod m
     with a_1 = 271828183, a_2 = 314159269, and 
     m = 2^31 - 1.")

(def-rng-type +lecuyer21+		; FDL
    "Obsolete, use only for compatibility.
     The L'Ecuyer random number generator, taken from Knuth's
     Seminumerical Algorithms, 3rd Ed., page 106--108.
     Its sequence is x_{n+1} = (a x_n) mod m
     with a = 40692 and m = 2^31 - 249.
     The seed specifies the initial value, x_1.")

(def-rng-type +waterman14+		; FDL
    "Obsolete, use only for compatibility.
     The Waterman random number generator. It is taken from Knuth's
     Seminumerical Algorithms, 3rd Ed., page 106--108.
     Its sequence is x_{n+1} = (a x_n) mod m with
     a = 1566083941 and m = 2^32.
     The seed specifies the initial value, x_1.")
