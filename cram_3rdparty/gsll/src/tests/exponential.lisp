;; Regression test EXPONENTIAL for GSLL, automatically generated
;;
;; Copyright 2009 Liam M. Healy
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

(LISP-UNIT:DEFINE-TEST EXPONENTIAL
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    (LIST 0.0025828444588394794d0 18.145581427987647d0
	  12.636598054339759d0 0.5424387252062355d0
	  14.624994234158105d0 7.236607929535993d0
	  0.4345362449683603d0 2.95303920904529d0
	  6.161052939065796d0 3.011686333539114d0
	  2.7451079819355364d0))
   (MULTIPLE-VALUE-LIST
    (LET ((RNG (MAKE-RANDOM-NUMBER-GENERATOR +MT19937+ 0)))
      (LOOP FOR I FROM 0 TO 10 COLLECT
	   (sample rng :exponential :mu 10.0d0)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL (LIST 0.1d0)
				     (MULTIPLE-VALUE-LIST
				      (EXPONENTIAL-PDF
				       0.0d0 10.0d0)))
  ;; From cdf/test.c test_exponential
  (ASSERT-TO-TOLERANCE (EXPONENTIAL-P 0.1d0 0.7d0) 0.13312210024981838d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPONENTIAL-P 1.d-32 0.7d0) 1.4285714285714287d-32 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPONENTIAL-P 1000.0d0 0.7d0) 1.0d0 +TEST-TOL6+)
  (ASSERT-TO-TOLERANCE (EXPONENTIAL-Q 0.1d0 0.7d0) 0.8668778997501816d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPONENTIAL-Q 1.d-32 0.7d0) 1.0d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPONENTIAL-Q 1000.0d0 0.7d0) 0.0d0 +TEST-TOL6+)
  ;; From cdf/test.c test_exponentialinv
  (ASSERT-TO-TOLERANCE (EXPONENTIAL-PINV 0.13d0 0.7d0) 0.09748344713345536d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPONENTIAL-PINV 1.42d-32 0.7d0) 9.94d-33 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPONENTIAL-QINV 0.86d0 0.7d0) 0.10557602281420854d0 +TEST-TOL0+)
  (ASSERT-TO-TOLERANCE (EXPONENTIAL-QINV 0.99999d0 0.7d0) 7.000035000233335d-6 +TEST-TOL6+))
