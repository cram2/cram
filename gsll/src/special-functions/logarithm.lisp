;; Logarithm
;; Liam Healy, Sun Apr 30 2006 - 22:08
;; Time-stamp: <2009-12-27 10:10:01EST logarithm.lisp>
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

(defgeneric gsl-log (x)
  (:documentation			; FDL
   "The natural logarithm of x, log(x), for x > 0."))

(defmfun gsl-log ((x float))
  "gsl_sf_log_e"
  ((x :double) (ret sf-result))
  :definition :method
  :export t)

(defmfun gsl-log ((x complex))
  "gsl_sf_complex_log_e"
  (((realpart x) :double) ((imagpart x) :double)
   (re-ret sf-result) (im-ret sf-result))
  :definition :method
  :return
  ((complex (val re-ret) (val im-ret)) (complex (err re-ret) (err im-ret)))
  :documentation			; FDL
  "Results are returned as lnr, theta such that
  exp(lnr + i \theta) = z_r + i z_i, where theta lies in the range [-\pi,\pi].")

(defmfun log-abs (x)
  "gsl_sf_log_abs_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "The natural logarithm of the magnitude of x, log(|x|), for x ne 0.")

(defmfun log-1+x (x)
  "gsl_sf_log_1plusx_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "log(1 + x) for x > -1 using an algorithm that is accurate for small x.")

(defmfun log-1+x-m1 (x)
  "gsl_sf_log_1plusx_mx_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "log(1 + x) - x for x > -1 using an algorithm that is accurate for small x.")

;;; Examples and unit test

(save-test logarithm
  (gsl-log 2.0d0)
  (gsl-log #C(1.0d0 1.0d0))
  (log-abs -2.0d0)
  (log-1+x 1.d-4)
  (log-1+x-m1 1.d-4))

