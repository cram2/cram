;; Elementary functions
;; Liam Healy, Mon Mar 20 2006 - 21:43
;; Time-stamp: <2009-12-27 10:10:05EST elementary.lisp>
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

(defmfun multiply (x y)
  "gsl_sf_multiply_e"
  ((x :double) (y :double) (ret sf-result))
  :documentation			; FDL
  "Multiplies x and y returning the product and associated error.")

(defmfun multiply-err (x dx y dy)
    "gsl_sf_multiply_err_e"
  ((x :double) (dx :double) (y :double)  (dy :double) (ret sf-result))
  :documentation			; FDL
  "Multiplies x and y with associated absolute
   errors dx and dy.  The product xy +/- xy \sqrt((dx/x)^2 +(dy/y)^2)
   is returned.")

(save-test elementary
	   (multiply 3.0d0 2.0d0)
	   (multiply-err 3.0d0 0.1d0 2.0d0 0.1d0))

