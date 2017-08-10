;; Constants specifying limits of floating point calculations in hardware 
;; Liam Healy
;; Time-stamp: <2014-01-21 12:41:48EST machine.lisp>
;;
;; Copyright 2010, 2014 Liam M. Healy
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

(include "gsl/gsl_machine.h")

(constant (+DBL-EPSILON+ "GSL_DBL_EPSILON") :type double-float)
(constant (+SQRT-DBL-EPSILON+ "GSL_SQRT_DBL_EPSILON") :type double-float)
(constant (+ROOT3-DBL-EPSILON+ "GSL_ROOT3_DBL_EPSILON") :type double-float)
(constant (+ROOT4-DBL-EPSILON+ "GSL_ROOT4_DBL_EPSILON") :type double-float)
(constant (+ROOT5-DBL-EPSILON+ "GSL_ROOT5_DBL_EPSILON") :type double-float)
(constant (+ROOT6-DBL-EPSILON+ "GSL_ROOT6_DBL_EPSILON") :type double-float)
(constant (+LOG-DBL-EPSILON+ "GSL_LOG_DBL_EPSILON") :type double-float)

(constant (+DBL-MIN+ "GSL_DBL_MIN") :type double-float)
(constant (+SQRT-DBL-MIN+ "GSL_SQRT_DBL_MIN") :type double-float)
(constant (+ROOT3-DBL-MIN+ "GSL_ROOT3_DBL_MIN") :type double-float)
(constant (+ROOT4-DBL-MIN+ "GSL_ROOT4_DBL_MIN") :type double-float)
(constant (+ROOT5-DBL-MIN+ "GSL_ROOT5_DBL_MIN") :type double-float)
(constant (+ROOT6-DBL-MIN+ "GSL_ROOT6_DBL_MIN") :type double-float)
(constant (+LOG-DBL-MIN+ "GSL_LOG_DBL_MIN") :type double-float)

(constant (+DBL-MAX+ "GSL_DBL_MAX") :type double-float)
(constant (+SQRT-DBL-MAX+ "GSL_SQRT_DBL_MAX") :type double-float)
(constant (+ROOT3-DBL-MAX+ "GSL_ROOT3_DBL_MAX") :type double-float)
(constant (+ROOT4-DBL-MAX+ "GSL_ROOT4_DBL_MAX") :type double-float)
(constant (+ROOT5-DBL-MAX+ "GSL_ROOT5_DBL_MAX") :type double-float)
(constant (+ROOT6-DBL-MAX+ "GSL_ROOT6_DBL_MAX") :type double-float)
(constant (+LOG-DBL-MAX+ "GSL_LOG_DBL_MAX") :type double-float)

(constant (+FLT-EPSILON+ "GSL_FLT_EPSILON") :type double-float)
(constant (+SQRT-FLT-EPSILON+ "GSL_SQRT_FLT_EPSILON") :type double-float)
(constant (+ROOT3-FLT-EPSILON+ "GSL_ROOT3_FLT_EPSILON") :type double-float)
(constant (+ROOT4-FLT-EPSILON+ "GSL_ROOT4_FLT_EPSILON") :type double-float)
(constant (+ROOT5-FLT-EPSILON+ "GSL_ROOT5_FLT_EPSILON") :type double-float)
(constant (+ROOT6-FLT-EPSILON+ "GSL_ROOT6_FLT_EPSILON") :type double-float)
(constant (+LOG-FLT-EPSILON+ "GSL_LOG_FLT_EPSILON") :type double-float)

(constant (+FLT-MIN+ "GSL_FLT_MIN") :type double-float)
(constant (+SQRT-FLT-MIN+ "GSL_SQRT_FLT_MIN") :type double-float)
(constant (+ROOT3-FLT-MIN+ "GSL_ROOT3_FLT_MIN") :type double-float)
(constant (+ROOT4-FLT-MIN+ "GSL_ROOT4_FLT_MIN") :type double-float)
(constant (+ROOT5-FLT-MIN+ "GSL_ROOT5_FLT_MIN") :type double-float)
(constant (+ROOT6-FLT-MIN+ "GSL_ROOT6_FLT_MIN") :type double-float)
(constant (+LOG-FLT-MIN+ "GSL_LOG_FLT_MIN") :type double-float)

(constant (+FLT-MAX+ "GSL_FLT_MAX") :type double-float)
(constant (+SQRT-FLT-MAX+ "GSL_SQRT_FLT_MAX") :type double-float)
(constant (+ROOT3-FLT-MAX+ "GSL_ROOT3_FLT_MAX") :type double-float)
(constant (+ROOT4-FLT-MAX+ "GSL_ROOT4_FLT_MAX") :type double-float)
(constant (+ROOT5-FLT-MAX+ "GSL_ROOT5_FLT_MAX") :type double-float)
(constant (+ROOT6-FLT-MAX+ "GSL_ROOT6_FLT_MAX") :type double-float)
(constant (+LOG-FLT-MAX+ "GSL_LOG_FLT_MAX") :type double-float)

(constant (+SFLT-EPSILON+ "GSL_SFLT_EPSILON") :type double-float)
(constant (+SQRT-SFLT-EPSILON+ "GSL_SQRT_SFLT_EPSILON") :type double-float)
(constant (+ROOT3-SFLT-EPSILON+ "GSL_ROOT3_SFLT_EPSILON") :type double-float)
(constant (+ROOT4-SFLT-EPSILON+ "GSL_ROOT4_SFLT_EPSILON") :type double-float)
(constant (+ROOT5-SFLT-EPSILON+ "GSL_ROOT5_SFLT_EPSILON") :type double-float)
(constant (+ROOT6-SFLT-EPSILON+ "GSL_ROOT6_SFLT_EPSILON") :type double-float)
(constant (+LOG-SFLT-EPSILON+ "GSL_LOG_SFLT_EPSILON") :type double-float)

(constant (+SQRT-MACH-EPS+ "GSL_SQRT_MACH_EPS") :type double-float)
(constant (+ROOT3-MACH-EPS+ "GSL_ROOT3_MACH_EPS") :type double-float)
(constant (+ROOT4-MACH-EPS+ "GSL_ROOT4_MACH_EPS") :type double-float)
(constant (+ROOT5-MACH-EPS+ "GSL_ROOT5_MACH_EPS") :type double-float)
(constant (+ROOT6-MACH-EPS+ "GSL_ROOT6_MACH_EPS") :type double-float)
(constant (+LOG-MACH-EPS+ "GSL_LOG_MACH_EPS") :type double-float)

