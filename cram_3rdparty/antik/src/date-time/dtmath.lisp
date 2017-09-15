;; Mathematics on timepoints.
;; Liam Healy Fri Oct 12 2001 - 16:32
;; Time-stamp: <2014-10-06 23:10:23EDT dtmath.lisp>

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

(export '(timepoint-j2000sec make-timepoint-J2000sec
	  datime+ datime- days+ days-
	  start-of-day seconds-per-day datime-J2000day))

;;; Mathematics can only be done on timepoints, not on dtspecs,
;;; because the scale must be known first.
;;; Keep in mind that CL's *-universal-time are handy for
;;; linearizing and delinearizing a specification to do math,
;;; but they carry no scale; they are simply a dtspec.

(defvar *j2000-epoch* (read-timepoint "2000-01-01T12:00:00"))

(defvar *clut-j2000-tai*
    (timeparse-to-clut
     (timeparse (convert-time-scale *j2000-epoch* :tai)))
  "J2000 epoch in CL's `universal-time' time linearization, in the TAI scale.")

(defun timepoint-j2000sec (tp)
  "Seconds from the J2000 epoch, 2000-01-01T12:00:00UTC."
  (flet ((tp-to-clut (tp)
	   (timeparse-to-clut (timeparse tp))))
    (check-type tp timepoint)
    (cl:- (tp-to-clut (convert-time-scale tp :tai)) *clut-j2000-tai*)))

(defun make-timepoint-J2000sec (J2000sec &optional day-only)
  "Make a timepoint from J2000sec."
  (convert-time-scale
   (make-timepoint
    :timeparse
    (clut-to-timeparse (cl:+ *clut-j2000-tai* J2000sec))
    :scale :tai
    :day-only day-only)
   :utc))

;;;****************************************************************************
;;; Whole days
;;;****************************************************************************

(defun start-of-day
    (datime &optional (relative-day 0) (day-only (day-only datime)))
  "From the given datime, return the start of the given day,
   relative-day = 0: this day, -1: day before, +1 next day, etc."
  (let ((answer
	 (dtspec-from-julian-day-number
	  (cl:+ (julian-day-number datime t) relative-day)
	  t)))
    (change-class answer (class-of datime))
    (if (typep datime 'timepoint)
	(setf (slot-value answer 'scale) (scale datime)))
    (if (or day-only (day-only datime))
	(setf (slot-value answer 'day-only) day-only))
    answer))

;;; (seconds-per-day #d1998-12-31) -> 86401
(defun seconds-per-day (datime)
  "Compute the number of seconds in the given day."
  (pq-magnitude
   (datime- (start-of-day datime 1) (start-of-day datime))))

(defun datime-J2000day (datime)
  "Julian Day from J2000 epoch, and seconds from the start of the day."
  (values
    ;; Days from J2000 epoch to the start of the UTC day; always integer + 0.5.
    (cl:- (days- datime (MAKE-TIMEPOINT :TIMEPARSE '(0 0 12 1 1 2000) :DAY-ONLY NIL :SCALE :UTC))
	  1/2)
    ;; Seconds from the start of the UTC day to the current timepoint.
    ;; No timescale is required because this is
    ;; only a difference from the start of the day.
    (cl:- (timeparse-to-clut (timeparse datime)) ;(timepoint-J2000sec datime)
	  (timeparse-to-clut (timeparse (start-of-day datime))))))

(defun days- (datime1 datime2)
  "Integer number of days separating the days.  Differs from datime-
   in that the time of day doesn't matter, just the day on which the two 
   datimes occur."
  (cl:- (julian-day-number datime1) (julian-day-number datime2)))

(defun days+ (datime &optional (increment 1))
  "Add the number of days to the given datime."
  (start-of-day datime increment))

;;;****************************************************************************
;;; Addition, subtraction
;;;****************************************************************************

(defun datime+ (tp time)
  "Add the time to the timepoint.  If time is a number,
   it is assumed to represent seconds."
  (check-type tp timepoint)
  (with-units
      (with-pq ((time time))
	(make-timepoint-j2000sec
	 (cl:+ (timepoint-J2000sec tp)
	       (if (numberp time)
		   time
		   (pq-magnitude time)))
	 (day-only tp)))))

;;; N.B.:
;;; (datime- #d1999-01-01T12:00:00 #d1998-12-31T12:00:00)
;;; 86401.0 second
(defun datime- (datime1 b)
  "The difference of the datimes, or subtract a time from a datime."
  (if (typep b 'timepoint)
      ;; add conditional to check if either are day-only
      ;; if so, just count the number of days.
      (if (or (day-only datime1) (day-only b))
	  (make-pq (days- datime1 b) 'day)
	  (make-pq		 ; time difference between two datimes
	   (cl:- (timepoint-J2000sec datime1)
		 (timepoint-J2000sec b))
	   'second))
      (datime+ datime1 (antik:- b))))	; subtract a time
	  
(defmethod +i ((a dtspec) b)
  (datime+ a b))

(defmethod +i (a (b dtspec))
  (datime+ b a))

(defmethod -i ((a dtspec) b)
  (datime- a b))

;;;****************************************************************************
;;; Relational
;;;****************************************************************************

(defun timepoint-relation (relation x y)
  "Do the two timepoints obey the relation?"
  (datime-relation
   relation
   (multiple-value-bind (converted error)
       (ignore-errors (convert-time-scale y (scale x)))
     (if (typep error 'error)		; can't convert y to x type?
	 ;; then convert x to y type
	 (list (convert-time-scale x (scale y)) y)
       (list x (if (eq (scale x) (scale y)) y converted))))))

;;; define <, etc.
(defmethod <i ((x timepoint) (y timepoint)) (timepoint-relation '< x y))
(defmethod >i ((x timepoint) (y timepoint)) (timepoint-relation '> x y))
(defmethod <=i ((x timepoint) (y timepoint)) (timepoint-relation '<= x y))
(defmethod >=i ((x timepoint) (y timepoint)) (timepoint-relation '>= x y))

(defmethod =i ((x timepoint) (y timepoint))
  (if (timepoint-relation '= x y) x))
