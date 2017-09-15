;; Regression test GAUSSIAN-BIVARIATE for GSLL, automatically generated
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

(LISP-UNIT:DEFINE-TEST GAUSSIAN-BIVARIATE
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST
    (LIST -0.06509716124488897d0 -1.5733207749096374d0
	  0.27942740172325414d0 1.2021528358889673d0
	  -0.6041530626907894d0 0.07582702719413444d0
	  -0.5446229412165632d0 -0.6592026841613081d0
	  -0.11029516610819164d0 0.17931840412143885d0
	  2.1025104980291696d0))
   (MULTIPLE-VALUE-LIST
    (LET ((RNG (MAKE-RANDOM-NUMBER-GENERATOR +MT19937+ 0)))
      (LOOP FOR I FROM 0 TO 10 COLLECT
	   (sample
	    rng :bivariate-gaussian
	    :sigma-x 1.0d0 :sigma-y 0.75d0 :rho 0.25d0)))))
  (LISP-UNIT::ASSERT-NUMERICAL-EQUAL
   (LIST 0.5548265557970462d0)
   (MULTIPLE-VALUE-LIST
    (BIVARIATE-GAUSSIAN-PDF 0.25d0 0.5d0 0.25d0 0.4d0
			    0.2d0))))

