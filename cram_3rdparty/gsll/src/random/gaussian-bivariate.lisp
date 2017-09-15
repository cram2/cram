;; Gaussian bivariate distribution
;; Liam Healy, Sat Sep  2 2006 - 16:32
;; Time-stamp: <2010-01-17 10:15:54EST gaussian-bivariate.lisp>
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


(in-package :gsl)

;;; /usr/include/gsl/gsl_randist.h

(defmfun sample
    ((generator random-number-generator) (type (eql :bivariate-gaussian))
     &key sigma-x sigma-y rho)
  "gsl_ran_bivariate_gaussian"
  (((mpointer generator) :pointer) (sigma-x :double) (sigma-y :double) (rho :double)
   (x (:pointer :double)) (y (:pointer :double)))
  :definition :method
  :c-return :void
  :documentation			; FDL
  "Generate a pair of correlated Gaussian variates, with
   mean zero, correlation coefficient rho and standard deviations
   sigma_x and sigma_y in the x and y directions.
   The probability distribution for bivariate Gaussian random variates is,
   p(x,y) dx dy
   = {1 \over 2 \pi \sigma_x \sigma_y \sqrt{1-\rho^2}}
   \exp \left(-{(x^2/\sigma_x^2 + y^2/\sigma_y^2 - 2 \rho x y/(\sigma_x\sigma_y))
   \over 2(1-\rho^2)}\right) dx dy
   for x,y in the range -\infty to +\infty.  The
   correlation coefficient rho should lie between 1 and -1.")

(defmfun bivariate-gaussian-pdf (x y sigma-x sigma-y rho)
  "gsl_ran_bivariate_gaussian_pdf"
  ((x :double) (y :double) (sigma-x :double) (sigma-y :double) (rho :double))
  :c-return :double
  :documentation			; FDL
  "The probability density p(x,y) at
   (x,y) for a bivariate Gaussian distribution with standard
   deviations sigma_x, sigma_y and correlation coefficient
   rho, using the formula given for #'sample :bivariate-gaussian.")

;;; Examples and unit test
(save-test gaussian-bivariate
  (let ((rng (make-random-number-generator +mt19937+ 0)))
      (loop for i from 0 to 10
	    collect
	    (sample
	     rng :bivariate-gaussian
	     :sigma-x 1.0d0 :sigma-y 0.75d0 :rho 0.25d0)))
  (bivariate-gaussian-pdf 0.25d0 0.5d0 0.25d0
			   0.4d0 0.2d0))
