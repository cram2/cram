;; Integer powers
;; Liam Healy, Sun Apr 30 2006 - 22:46
;; Time-stamp: <2009-12-27 10:10:00EST power.lisp>
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

(defmfun pow (x n)
  "gsl_sf_pow_int_e" ((x :double) (n :int) (ret sf-result))
  :documentation			; FDL
  "The power x^n for integer n.  The
  power is computed using the minimum number of multiplications. For
  example, x^8 is computed as ((x^2)^2)^2, requiring only 3
  multiplications.  For reasons of efficiency, these functions do not
  check for overflow or underflow conditions.")

(save-test power
  (pow 3.5d0 5))
