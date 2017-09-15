;; Specification and formatting of datimes.
;; Liam Healy Fri Oct 12 2001 - 10:04
;; Time-stamp: <2013-03-24 18:51:52EDT dtspec.lisp>

;; Copyright 2011, 2013 Liam M. Healy
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

(export '(dtspec timeparse iso8601 day-only
	  *datime-readable* make-dtspec julian-day-number
	  dtspec-from-julian-day-number
	  datime-relation
	  datime= datime< datime<= datime> datime>=
	  +seconds-per-minute+
	  +minutes-per-hour+
	  +hours-per-day+
	  +days-per-month+
	  +months-per-year+
	  +seconds-per-hour+
	  +seconds-per-day+
	  +seconds-per-month+
	  +seconds-per-year+
	  timeparse-year timeparse-month timeparse-day
	  timeparse-hour timeparse-minute timeparse-second
	  make-timeparse-majord make-timeparse-minord))

;;; A "dtspec" is the specification of a date/time.
;;; It includes the structure and format only, but no scale, so it does
;;; not represent a particular point in time.  For example,
;;; 2001-10-17T10:41:17 does not represent a particular time because
;;; it is not stated whether this is UTC, TAI, EDT, etc.
;;; Even when the specification has no scale, some mathematics
;;; can still be performed. 
;;; nf option:
;;; dtspec-linear to linearize time.


;;;****************************************************************************
;;; dtspec object
;;;****************************************************************************

(defclass dtspec ()
  ((iso8601 :initarg :iso8601 :reader iso8601)
   (timeparse :initarg :timeparse :reader timeparse)
   (day-only :initarg :day-only :reader day-only)) ; default to nil?
  (:documentation "Specification of date and time, without an assigned scale."))

(defparameter *datime-readable* nil
  "Whether to print datimes readably.")

(defmethod print-object ((object dtspec) stream)
  (if *datime-readable*
      (cl-readable-nf (nf object stream))
      (nf object stream)))

(defun make-dtspec (&key timeparse (day-only nil) (type 'dtspec))
  "Make the dtspec arguments."
  (make-instance
   type
   :iso8601 (iso8601-string timeparse nil)
   :timeparse timeparse :day-only day-only))

(defun read-dtspec (string)
  "Read the ISO8601 specification and make a dtspec."
  ;; This is used only in the definiton of *utc-leap-second-table*.
  (multiple-value-bind (tp components-specified)
      (iso8601-parse string t)
    (make-dtspec
     :timeparse tp
     :day-only (not (member :time components-specified)))))

(defmethod creation-form ((dt dtspec))
  (creation-form-readably
   dt
   `(make-dtspec
     :timeparse ',(timeparse dt) :day-only ',(day-only dt))))

; Sat Apr 12 2003 - 22:25 (def-make-load-form dtspec)

;;; For "time-interval by duration only", the follow relations hold exactly:
(defconstant +seconds-per-minute+ 60)
(defconstant +minutes-per-hour+ 60)
(defconstant +hours-per-day+ 24)
(defconstant +days-per-month+ 30)
(defconstant +months-per-year+ 12)
(defconstant +seconds-per-hour+ (cl:* +minutes-per-hour+ +seconds-per-minute+))
(defconstant +seconds-per-day+ (cl:* +seconds-per-hour+ +hours-per-day+))
(defconstant +seconds-per-month+ (cl:* +seconds-per-day+ +days-per-month+))
(defconstant +seconds-per-year+ (cl:* +seconds-per-month+ +months-per-year+))

;;;****************************************************************************
;;; Convert 2-digit year
;;;****************************************************************************

(defparameter *default-two-digit-year-reference-year* nil
  "If NIL, use the current year as the reference year for deciding in which century
   a two-digit year belongs.  Otherwise, use the specified year.")

(defun convert-two-digit-year
    (year &optional
	  (reference-year
	   (or *default-two-digit-year-reference-year*
	       (nth-value 5 (decode-universal-time (get-universal-time))))))
  "Convert two-digit year to four digit by taking the year that
   is closest to the reference year.  Reference year defaults
   to the current year."
  (if (< (abs year) 100)		; is it actually two digits?
      (let* ((delta (- year (mod reference-year 100))))
	(cl:+ delta reference-year
	   (if (cl:< (abs delta) 50)
	       0
	     (if (cl:plusp delta)
		 -100
	       100))))
    ;; if not two digits, just give it back
    year))

;;;****************************************************************************
;;; Timeparse
;;;****************************************************************************

(defun timeparse-part (timeparse part)
    (nth (position part '(:second :minute :hour :day :month :year))
	 timeparse))

(defun timeparse-year (timeparse) (timeparse-part timeparse :year))
(defun timeparse-month (timeparse) (timeparse-part timeparse :month))
(defun (setf timeparse-month) (value timeparse) (setf (fifth timeparse) value))
(defun timeparse-day (timeparse) (timeparse-part timeparse :day))
(defun (setf timeparse-day) (value timeparse) (setf (fourth timeparse) value))
(defun timeparse-hour (timeparse) (timeparse-part timeparse :hour))
(defun timeparse-minute (timeparse) (timeparse-part timeparse :minute))
(defun timeparse-second (timeparse) (timeparse-part timeparse :second))

(defun make-timeparse-majord
    (&optional (year 0) (month 0) (day 0) (hour 0) (minute 0) (second 0))
  (list second minute hour day month year))
(defun make-timeparse-minord
    (&optional (second 0) (minute 0) (hour 0) (day 0) (month 0) (year 0))
  (list second minute hour day month year))

(defun fn-major-timeparse (timeparse function)
  "Apply the function to the most major non-zero component of the timeparse."
  ;; Warning! destructively modifies timeparse
  (let ((pos (position 0 timeparse :from-end t :test-not #'=)))
    (when pos
      (setf (nth pos timeparse)
	(funcall function (nth pos timeparse))))
    timeparse))

;;;****************************************************************************
;;; Comparison
;;;****************************************************************************

;;; These do not yet take >2 dtspec arguments

(defun relative-datime (dtspec1 dtspec2 &optional day-only)
  "Find the largest time unit where the two dtspecs differ, and
   return the difference of that unit and the unit name.
   If the dtspecs are the same, return 0 :second."
  (let ((tp1 (timeparse dtspec1))
	(tp2 (timeparse dtspec2))
	(tod-meaningful
	 (if day-only nil
	   (not (or (day-only dtspec1) (day-only dtspec2))))))
    (let ((relyr (cl:- (timeparse-year tp1) (timeparse-year tp2))))
      (if (cl:zerop relyr)
	  (let ((relmo (cl:- (timeparse-month tp1) (timeparse-month tp2))))
	    (if (cl:zerop relmo)
		(let ((relday (cl:- (timeparse-day tp1) (timeparse-day tp2))))
		  (if (and (cl:zerop relday) tod-meaningful)
		      (let ((relhr (cl:- (timeparse-hour tp1) (timeparse-hour tp2))))
			(if (cl:zerop relhr)
			    (let ((relmin
				   (cl:- (timeparse-minute tp1) (timeparse-minute tp2))))
			      (if (cl:zerop relmin)
				  (values
				   (cl:- (timeparse-second tp1) (timeparse-second tp2))
				   :second)
				(values relmin :minute)))
			  (values relhr :hour)))
		    (values relday :day)))
	      (values relmo :month)))
	(values relyr :year)))))

(defun datime-relation (relation args)
  "Apply the relation to all the datime arguments and return
   what it returns."
  (loop for (x y) on args
      always (or (null y) (funcall relation (relative-datime x y) 0))))

(defun datime> (&rest args) (datime-relation 'cl:> args))
(defun datime>= (&rest args) (datime-relation 'cl:>= args))
(defun datime< (&rest args) (datime-relation 'cl:< args))
(defun datime<= (&rest args) (datime-relation 'cl:<= args))
(defun datime= (&rest args) (datime-relation 'cl:= args))

;;; Make database-lookup work properly on datimes.
(setq *type-equality* (acons 'dtspec #'datime= *type-equality*))

;;;****************************************************************************
;;; Numerica format
;;;****************************************************************************

(defmethod nf
    ((object dtspec) &optional (stream *standard-output*))
  (format stream "~@[~a~]~a"
	  (when (nf-readably) "#d")
	  (if (and (day-only object) (not (nf-option :ignore-day-only)))
	      (subseq (iso8601 object) 0 10)
	      (iso8601-string
	       (timeparse object)
	       nil
	       (if (nf-readably)
		   nil
		   (or (nf-option :fracpart-digits)
		       *iso8601-fractional-seconds-digits*))
	       (or (nf-option :date-time-separator)
		   (when (nf-readably) T))))))

;;;****************************************************************************
;;; Julian day number
;;;****************************************************************************

(defun julian-day-number (dtspec &optional modified fractional-day)
  "Julian day number at noon of the day, or modified Julian day at
   the start of the day, independent of timescale.
   Algorithm given in
   http://www.pauahtun.org/CalendarFAQ/cal/node3.html#SECTION003151000000000000000."
;;; incorrectly says `noon UTC'; JD calculation is independent of time scale.
  (check-type dtspec dtspec)
  (let* ((tp (timeparse dtspec))
	 (a (floor (cl:- 14 (timeparse-month tp)) 12))
	 (y (cl:+ (timeparse-year tp) 4800 (cl:- a)))
	 (m (cl:+ (timeparse-month tp) (cl:* 12 a) -3)))
    (cl:+ (timeparse-day tp)
	  (cl:floor (cl:+ (cl:* 153 m) 2) 5)
	  (cl:* 365 y)
	  (cl:floor y 4)
	  (cl:- (cl:floor y 100))
	  (cl:floor y 400)
	  -32045
	  (if fractional-day
	      (cl:/ (cl:+ (cl:* +seconds-per-hour+ (timeparse-hour tp))
		    (cl:* +seconds-per-minute+ (timeparse-minute tp))
		    (timeparse-second tp))
		 (seconds-per-day dtspec))
	      0)
	  (if modified -2400001 -1/2))))

(defun dtspec-from-julian-day-number (julian-day-number &optional modified)
  "Find the dtspec for the given (modified) Julian day.
   Algorithm given in
   http://www.pauahtun.org/CalendarFAQ/cal/node3.html#SECTION003151000000000000000."
  (check-type julian-day-number integer)
  (let* ((a (cl:+ julian-day-number 32044 (if modified 2400001 0)))
	 (b (floor (cl:+ (cl:* 4 a) 3) 146097))
	 (c (cl:- a (floor (cl:* 146097 b) 4)))
	 (d (floor (cl:+ (cl:* 4 c) 3) 1461))
	 (e (cl:- c (floor (cl:* 1461 d) 4)))
	 (m (floor (cl:+ (cl:* 5 e) 2) 153)))
    (make-dtspec :timeparse
		 (list 0 0 (if modified 0 12) ; sec, min, hour
		       (cl:+ e (cl:- (floor (cl:+ (cl:* 153 m) 2) 5)) 1) ; day
		       (cl:+ m 3 (cl:* -12 (cl:floor m 10))) ; month
		       (cl:+ (cl:* 100 b) d -4800 (cl:floor m 10)))))) ; year
