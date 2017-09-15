;; Regression test EXPONENTIAL-FUNCTIONS for GSLL
;;
;; Copyright 2009, 2010 Liam M. Healy
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

(in-package :gsl)

(defconstant +exp-x+ (* 0.8d0 +log-dbl-max+))
(defconstant +ln2+ 0.69314718055994530941723212146d0)

;; assert-to-tolerance probably not doing any good here
(lisp-unit:define-test exponential-functions
  ;; From specfunc/test_sf.c.
  (assert-to-tolerance (gsl-exp -10.0d0) (exp -10.0d0) +test-tol0+)
  (assert-to-tolerance (gsl-exp 10.0d0) (exp 10.0d0) +test-tol0+)
  (assert-sf-scale (exp-scaled 1.0d0) (exp 1.0d0) 0 +test-tol0+ +test-tol1+)
  (assert-sf-scale (exp-scaled 2000.0d0) 3.88118019428363725d0 868 +test-tol3+ +test-tol5+)
  (assert-to-tolerance (exp-err -10.0d0 +test-tol1+) (exp -10.0d0) +test-tol1+)
  (assert-to-tolerance (exp-err 10.0d0 +test-tol1+) (exp 10.0d0) +test-tol1+)
  (assert-sf-scale
   (exp-err-scaled 1.0d0 +test-sqrt-tol0+) (exp 1.0d0) 0 +test-tol1+ (* 32 +test-sqrt-tol0+))
  (assert-sf-scale
   (exp-err-scaled 2000.0d0 1.0d-10) 3.88118019428363725d0 868 +test-tol3+ 1.0d-7)
  (assert-to-tolerance (exp-mult -10.0d0 1.0d-06) (* 1.0d-06 (exp -10.0d0)) +test-tol0+)
  (assert-to-tolerance (exp-mult -10.0d0 2.0d0) (* 2.0d0 (exp -10.0d0)) +test-tol0+)
  (assert-to-tolerance (exp-mult -10.0d0 -2.0d0) (* -2.0d0 (exp -10.0d0)) +test-tol0+)
  (assert-to-tolerance (exp-mult 10.0d0 1.0d-06) (* 1.0d-06 (exp 10.0d0)) +test-tol0+)
  (assert-to-tolerance (exp-mult 10.0d0 -2.0d0) (* -2.0d0 (exp 10.0d0)) +test-tol0+)
  (assert-to-tolerance
   (exp-mult +exp-x+ 1.00001d0) (* 1.00001d0 (exp +exp-x+)) +test-tol3+)
  (assert-to-tolerance
   (exp-mult +exp-x+ 1.000001d0) (* 1.000001d0 (exp +exp-x+)) +test-tol3+)
  (assert-to-tolerance
   (exp-mult +exp-x+ 1.0000001d0) (* 1.0000001d0 (exp +exp-x+)) +test-tol3+)
  (assert-to-tolerance
   (exp-mult +exp-x+ 100.0d0) (* 100.0d0 (exp +exp-x+)) +test-tol3+)
  (assert-to-tolerance
   (exp-mult +exp-x+ 1.0d20) (* 1.0d20 (exp +exp-x+)) +test-tol3+)
  (assert-to-tolerance
   (exp-mult +exp-x+ (* (exp (- +exp-x+)) (exp +ln2+))) 2.0d0 +test-tol4+)
  (assert-sf-scale
   (exp-mult-scaled 1.0d0 1.0d0) (exp 1.0d0) 0 +test-tol0+ +test-tol2+)
  (assert-sf-scale
   (exp-mult-scaled 1000.0d0 1.0d200) 1.970071114017046993888879352d0
   634 +test-tol3+ 1.0d-11)
  (assert-sf-scale
   (exp-mult-err-scaled 1.0d0 +test-tol0+ 1.0d0 +test-tol0+)
   (exp 1.0d0) 0 +test-tol0+ +test-tol2+)
  (assert-sf-scale
   (exp-mult-err-scaled 1000.0d0 1.0d-12 1.0d200 1.0d190) 1.9700711140165661d0
   634 +test-tol3+ 1.0d-9)
  (assert-sf-scale
   (exp-mult-scaled 10000.0d0 1.0d0) 8.806818225662921587261496007d0 4342 +test-tol5+)
  (assert-sf-scale
   (exp-mult-scaled 100.0d0 1.0d0) 2.688117141816135448412625551d43 0 +test-tol2+)
  (assert-sf-scale
   (exp-scaled 100.0d0) 2.688117141816135448412625551d43 0 +test-tol2+)
  (assert-sf-scale
   (exp-scaled 1000.0d0) 1.970071114017046993888879352d0 434 +test-tol3+)
  (assert-sf-scale
   (exp-scaled -100.0d0) 3.720075976020835962959695803d-44 0 +test-tol2+)
  (assert-sf-scale
   (exp-scaled -1000.0d0) 5.075958897549456765291809479d0 -435 +test-tol3+)
  (ASSERT-TO-TOLERANCE (EXPM1 -10.0d0) (1- (EXP -10.0d0)) +test-tol0+)
  (ASSERT-TO-TOLERANCE (EXPM1 -0.001d0) -0.00099950016662500845d0
		       +TEST-TOL0+)
  (assert-to-tolerance (expm1 -1.0d-8) (+ -1.0d-8 0.5d-16) +test-tol0+)
  (assert-to-tolerance (expm1 1.0d-8) (+ 1.0d-8 0.5d-16) +test-tol0+)
  (assert-to-tolerance (expm1 0.001d0) 0.0010005001667083417d0 +test-tol0+)
  (assert-to-tolerance (expm1 10.0d0) (1- (exp 10.0d0)) +test-tol0+)
  (ASSERT-TO-TOLERANCE (EXPREL -10.0d0) 0.0999954600070237515d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL -0.001d0) 0.9995001666250084d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL -1.0d-8) (- 1.0d0 0.5d-8) +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL 1.0d-8) (+ 1.0d0 0.5d-8) +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL 0.001d0) 1.0005001667083417d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL 10.0d0) 2202.5465794806716517d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-2 -10.0d0) 0.18000090799859524970d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-2 -0.001d0) 0.9996667499833361107d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-2 -1.0d-8) 0.9999999966666666750d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-2 1.0d-8) 1.0000000033333333417d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-2 0.001d0) 1.0003334166833361115d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-2 10.0d0) 440.3093158961343303d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 3 -1000.0d0) 0.00299400600000000000d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 3 -100.0d0) 0.02940600000000000000d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 3 -10.0d0) 0.24599972760042142509d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 3 -3.0d0) 0.5444917625849191238d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 3 -0.001d0) 0.9997500499916678570d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 3 -1.0d-8) 0.9999999975000000050d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 3 1.0d-8) 1.0000000025000000050d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 3 0.001d0) 1.0002500500083345240d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 3 3.0d0) 2.5745637607083706091d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 3 3.1d0) 2.6772417068460206247d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 3 10.0d0) 131.79279476884029910d0 +TEST-TOL1+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 3 100.0d0) 1.6128702850896812690d+38 +TEST-TOL2+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 50 -1000.0d0) 0.04766231609253975959d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 50 -100.0d0) 0.3348247572345889317d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 50 -10.0d0) 0.8356287051853286482d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 50 -3.0d0) 0.9443881609152163615d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 50 -1.0d0) 0.980762245565660617d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 50 -1.0d-8) (- 1.0d0 (/ 1.0d-8 51.0d0)) +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 50 1.0d-8) (+ 1.0d0 (/ 1.0d-8 51.0d0)) +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 50 1.0d0) 1.01999216583666790d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 50 3.0d0) 1.0624205757460368307d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 50 48.0d0) 7.499573876877194416d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 50 50.1d0) 9.311803306230992272d0 +TEST-TOL4+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 50 100.0d0) 8.175664432485807634d+07 +TEST-TOL4+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 50 500.0d0) 4.806352370663185330d+146 +TEST-TOL3+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 500 -1000.0d0) 0.3334815803127619256d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 500 -100.0d0) 0.8335646217536183909d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 500 -10.0d0) 0.9804297803131823066d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 500 -3.0d0) 0.9940475488850672997d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 500 -1.0d0) 0.9980079602383488808d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 500 -1.0d-8) (- 1.0d0 (/ 1.0d-8 501.0d0)) +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 500 1.0d-8) (+ 1.0d0 (/ 1.0d-8 501.0d0)) +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 500 1.0d0) 1.0019999920160634252d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 500 3.0d0) 1.0060240236632444934d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 500 48.0d0) 1.1059355517981272174d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 500 100.0d0) 1.2492221464878287204d0 +TEST-TOL1+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 500 500.0d0) 28.363019877927630858d0 +TEST-TOL2+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 500 1000.0d0) 2.4037563160335300322d+68 +TEST-TOL4+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 500 1600.0d0) 7.899293535320607403d+226 +TEST-TOL4+)
  (ASSERT-TO-TOLERANCE (EXPREL-N 1263131 1261282.3637d0)
		       545.0113107238425900305428360d0 +TEST-TOL4+)
  ;; Nothing for this ported yet
  ;; gsl_sf_exprel_n_CF_e, (6.315655e+05, 6.302583168053568806e+05, &r), 385.425369029433473098652465720, TEST_TOL4, GSL_SUCCESS)
  )
