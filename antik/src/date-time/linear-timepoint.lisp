;; Linear timepoint
;; Liam Healy 2014-01-13 21:52:20EST linear-timepoint.lisp
;; Time-stamp: <2014-01-13 21:53:05EST linear-timepoint.lisp>

;; Copyright 2011, 2013, 2014 Liam M. Healy
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

(in-package :antik)

(defparameter *midnight-2000* (read-timepoint "2000-01-01T00:00:00"))

(defun linear-timepoint
    (timepoint &optional (reference-point *midnight-2000*) (unit :year))
  "Compute the timepoint on a linear (number) scale."
  (cl:+ (with-nf-options (:system-of-units (make-sysunits (list unit) t))
	  (pqval (antik:- timepoint reference-point)))
	(if (eq unit :year)
	    (timeparse-part (timeparse reference-point) unit)
	    0)))
