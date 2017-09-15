;; Functions on angles
;; Liam Healy Fri Mar 26 1999 - 13:48
;; Time-stamp: <2015-01-01 13:20:21EST angle.lisp>

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

(export '(norm-denorm-angle angle-as-time dms-angle time-as-angle angle-dms write-dms))

;;;;****************************************************************************
;;;; Basic angle definitions
;;;;****************************************************************************

(setf *radian* (make-pq 1 'radian))

(defun radian (angle)
  "The numerical value of the angle in radians."
  (antik::pq-magnitude angle))

;;; This will be obsolete
(defun norm-denorm-angle (angle function &optional positive)
  "Normalize an angle, process it, and denormalize it back to the
   original cycle count.  This is useful for e.g. Kepler's equation,
   where the normalized angle is needed to for the algorithm to
   converge properly, but the full cycles are needed for f & g functions."
  (multiple-value-bind (normalized revs)
      (normalize-angle angle positive)
    (antik:+ (make-pq revs 'revolution)	; must be first to get pdq if fn is 0
	     (funcall function normalized))))

;;; Not exactly what I'm looking for -- would like a tangent lock to the
;;; other angle.
(defmacro angle-find
    (&key sine-angle cosine-angle
       sign-sine-angle sign-cosine-angle
       angle-clone)
  "Find the angle, with accurate quadrant, from just about anything.
   Returns a value between -pi and pi.
   sine-angle: The sine of the angle
   cosine-angle: The cosine of the angle
   sign-sine-angle: If sine-angle is not supplied, the sign of sine
     will be assumed to be the sign of this number.
   sign-cosine-angle: If cosine-angle is not supplied, the sign of cosine
     will be assumed to be the sign of this number.
   angle-clone: If the sine-angle or the cosine-angle is not supplied,
     its sign will be the same as the sign of that function applied
     to this angle."
  (cond ((and sine-angle cosine-angle) `(cl:atan ,sine-angle ,cosine-angle))
	((and sine-angle sign-cosine-angle)
	 `(cl:atan ,sine-angle (sign ,sign-cosine-angle)))
	((and cosine-angle sign-sine-angle)
	 `(cl:atan (sign ,sign-sine-angle) ,cosine-angle))
	((and sine-angle angle-clone)
	 `(cl:atan ,sine-angle (sign (cl:cos ,angle-clone))))
	((and cosine-angle angle-clone)
	 `(cl:atan (sign (cl:sin ,angle-clone)) ,cosine-angle))
	(t (error "Can't determine angle from information given."))))

;;;;****************************************************************************
;;;; Angle relations
;;;;****************************************************************************

;;; To print an angle as time in the conventional hh:mm:ss.sss
;;; (setf *time-print-format* 3)
;;; (angle-as-time (sidereal-time now #_-77_deg))
(defun angle-as-time (angle &optional (normalize t))
  "Convert the angle to a time (interval), where one revolution is a
   complete day."
  (antik:* (make-pq 1 'day/revolution)
	   (normalize-angle angle normalize)))

(defvar *check-angle-limits* t
  "Whether to check that the angle pieces are within the normal
   range of numbers, e.g., number of seconds is < 60.")

;;; (angle-dms 21 48 0 t)
;;; 327.0°
;;; (angle-as-time *)
;;; PT21h48m0.000s
(defun angle-dms (degrees-or-hours minutes seconds &optional hours)
  "Read the angle as degrees (or hours), minutes and seconds.
   With hours=T computes the hour angle, where a minute is 1/60 of
   15 degrees instead of 1/60 of 1 degree.  This option is the inverse of 
   angle-as-time."
  (check-type degrees-or-hours integer)
  (check-type minutes number)
  (check-type seconds number)
  (when *check-angle-limits*
    (assert (cl:< (abs minutes) 60))
    (assert (cl:< (abs seconds) 60)))
  (assert (or (typep minutes 'integer) (cl:zerop seconds)))
  (let ((decimal
	 (* (if (or (minusp degrees-or-hours)
		    (and (cl:zerop degrees-or-hours) (cl:minusp minutes))
		    (and (cl:zerop degrees-or-hours) (cl:zerop minutes) (cl:minusp minutes)))
		-1 1)
	    (cl:+ (cl:abs degrees-or-hours)
		  (cl:/ (cl:abs minutes) +minutes-per-hour+)
		  (cl:/ (cl:abs seconds) +seconds-per-hour+)))))
    (if hours
	(antik:* (make-pq decimal 'hours) (make-pq 1.0 'revolution/day))
	(make-pq decimal 'degree))))

(defun time-as-angle (time-string)
  "Compute the angle from the time,
   the major field is hours with 24 hours = 1 revolution."
  ;; To get this to parse deg:arcmin:arcsec
  ;; would require hacking on iso8601-time-parse
  ;; to accept more than two digits in the major field.
  (let ((tp (iso8601-parse time-string)))
    (angle-dms (timeparse-hour tp)
	       (timeparse-minute tp)
	       (timeparse-second tp)
	       t)))

(defun dms-angle (angle)
  "The angle as a list degrees, minutes, seconds."
  (let ((*print-circle* nil))
    (multiple-value-bind (degrees dfrac)
	(cl:floor (with-system-of-units (degree) (pqval (antik:abs angle))))
      (multiple-value-bind (mins mfrac)
	  (cl:floor (cl:* 60 dfrac))
	(list
	 (cl:* (cl:round (cl:signum (pqval angle))) degrees)
	 mins
	 (cl:* 60 mfrac))))))

(defun write-dms
    (angle
     &optional (stream t)
       (first-sep (format nil "~a " *degree-symbol*))
       (second-sep "' ") (final "\""))
  (let ((dms (dms-angle angle)))
    (format
     stream
     "~2,'0d~a~2,'0d~a~6,3,,,'0f~a"
     (first dms) first-sep (second dms) second-sep (third dms) final)))

(defun format-dms (angle stream)
  "Hook for #'nf to format as degrees minutes seconds when option :degrees is specified as :dms."
  (write-dms
   angle stream
   (make-pq-string 'degree (nf-option :style))
   (if texstyle "^{\\prime}" "'")
   (if texstyle "^{\\prime\\prime}" "\"")))
