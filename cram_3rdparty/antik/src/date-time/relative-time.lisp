;; Times relative to one another.
;; Liam Healy Mon Apr 25 2005 - 12:19
;; Time-stamp: <2011-08-10 23:24:08EDT relative-time.lisp>

;; Copyright 2011 Liam M. Healy
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

(export '(relative-time time-limits))

(defun relative-time (base-time time &optional absolute)
  "Compute a time using base-time as a reference.
   Either can be a timepoint or time pdq, and the result
   can be chosen either way by specifying absolute T or NIL
   respectively.  The returned value is the time specified by
   the second argument, but referenced to base-time."
  (etypecase time
    (null base-time)	       ; if time=NIL, return base-time
    (timepoint		       ; if a timepoint, return it or interval
       (if absolute time (antik:- time base-time)))
    ((or physical-quantity number)	  ; if a pdq or number,
       (antik:+ (make-pq time 'time)	  ; add the time pq
		(if (typep base-time 'timepoint) ; if base-time is a timepoint,
		    (if absolute base-time ; if absolute, to base-time
			0)		   ; if not, to 0
		    (if absolute ; can't do absolute w/o timepoint ref
			(error "No reference timepoint given.")
			(antik:- base-time))))))) ; not absolute, difference from base-time

(defun time-limits (time1 time2 &optional (reference now))
  "Make sense of time1 and time2 in absolute terms. If one is absolute,
   and the other relative, it is presumed to be relative to the other. 
   If they are both relative, they are presumed to be relative to reference."
  (let ((t1 (if (typep time1 'timepoint)
		time1
		(if (typep time2 'timepoint)
		    (relative-time time2 time1 t)
		    (relative-time reference time1 t))))
	(t2 (if (typep time2 'timepoint)
		time2
		(if (typep time1 'timepoint)
		    (relative-time time1 time2 t)
		    (relative-time reference time2 t)))))
    (values (antik:min t1 t2)
	    (antik:max t1 t2))))
