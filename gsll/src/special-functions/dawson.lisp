;; Dawson function
;; Liam Healy, Sun Mar 19 2006 - 14:31
;; Time-stamp: <2009-12-27 10:10:06EST dawson.lisp>
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

#|
;;; FDL
The Dawson integral is defined by \exp(-x^2) \int_0^x dt
\exp(t^2).  A table of Dawson's integral can be found in Abramowitz &
Stegun, Table 7.5.  The Dawson functions are declared in the header file
gsl_sf_dawson.h.
|#

(in-package :gsl)

(defmfun dawson (x)
  "gsl_sf_dawson_e" ((x :double) (ret sf-result))
  :documentation			; FDL
  "Dawson's integral for x.")

(save-test dawson (dawson 1.0d0))


