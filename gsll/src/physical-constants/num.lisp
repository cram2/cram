;; Dimensionless constants
;; Sumant Oemrawsingh 2011-08-20 20:10:23UTC num.lisp
;; Time-stamp: <2011-08-20 20:10:23UTC num.lisp>
;;
;; Copyright 2011 Sumant Oemrawsingh, Liam M. Healy
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

#+linux
(define "_GNU_SOURCE")

;;; When installed through Mac Ports, GSL .h files will be found
;;; in /opt/local/include.
#+darwin
(cc-flags #.(gsl-config "--cflags"))

(include "gsl/gsl_const_num.h")

(constant (+num-fine-structure+ GSL_CONST_NUM_FINE_STRUCTURE) :type double-float)
(constant (+num-avogadro+ GSL_CONST_NUM_AVOGADRO) :type double-float)
(constant (+num-yotta+ GSL_CONST_NUM_YOTTA) :type double-float)
(constant (+num-zetta+ GSL_CONST_NUM_ZETTA) :type double-float)
(constant (+num-exa+ GSL_CONST_NUM_EXA) :type double-float)
(constant (+num-peta+ GSL_CONST_NUM_PETA) :type double-float)
(constant (+num-tera+ GSL_CONST_NUM_TERA) :type double-float)
(constant (+num-giga+ GSL_CONST_NUM_GIGA) :type double-float)
(constant (+num-mega+ GSL_CONST_NUM_MEGA) :type double-float)
(constant (+num-kilo+ GSL_CONST_NUM_KILO) :type double-float)
(constant (+num-milli+ GSL_CONST_NUM_MILLI) :type double-float)
(constant (+num-micro+ GSL_CONST_NUM_MICRO) :type double-float)
(constant (+num-nano+ GSL_CONST_NUM_NANO) :type double-float)
(constant (+num-pico+ GSL_CONST_NUM_PICO) :type double-float)
(constant (+num-femto+ GSL_CONST_NUM_FEMTO) :type double-float)
(constant (+num-atto+ GSL_CONST_NUM_ATTO) :type double-float)
(constant (+num-zepto+ GSL_CONST_NUM_ZEPTO) :type double-float)
(constant (+num-yocto+ GSL_CONST_NUM_YOCTO) :type double-float)
