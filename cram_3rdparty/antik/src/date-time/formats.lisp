;; Datime formats, compatibility with non-iso8601.
;; Liam Healy Fri Oct 12 2001 - 16:27
;; Time-stamp: <2015-09-07 10:02:16EDT formats.lisp>

;; Copyright 2011, 2013, 2014, 2015 Liam M. Healy
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

(export '(to-yyddd from-yyddd from-jd jd-table))

;;;****************************************************************************
;;; Year and Day of year (doy)
;;;****************************************************************************

(defun doy-year (datime)
  "Return the doy and year for this datime."
  (let* ((tp (timeparse datime))
	 (sy-dtspec (copy-list tp)))
    (setf (timeparse-month sy-dtspec) 1 (timeparse-day sy-dtspec) 1)
    (let ((yearstart (make-dtspec :timeparse sy-dtspec)))
      (values
       (cl:+ 1 (days- datime yearstart)
	     ;; doy is counted starting from 1, not 0.
	     (with-system-of-units (day)
	       (pqval (antik:- datime (start-of-day datime)))))
       (timeparse-year (timeparse datime))))))

(defun to-yyddd (datime)
  "Generate the twoline element form of datime.  Note this
   is not year-2000 compliant!"
  (multiple-value-bind (doy year)
      (doy-year datime)
    (cl:+ (cl:* (mod year 100) 1000)
       (coerce doy 'double-float))))

(defun from-yyddd
    (yy-or-yyddd &key ddd (scale :utc) (hours 0) (minutes 0) (seconds 0))
  "Read datime from yyddd.ddddd or yy, ddd.ddddd format."
  ;; Specifying both a fractional part of the day and nonzero
  ;; hours/minutes/seconds will cause them to be added,
  ;; which doesn't make much sense.
  (let (year doy)
    (if ddd
	(setf year yy-or-yyddd doy ddd)
      (multiple-value-setq (year doy) (floor yy-or-yyddd 1000)))
    (multiple-value-bind (intday fracday) (floor doy)
      (let* ((zerohrtp
	      (timeparse
	       (dtspec-from-julian-day-number
		(cl:+ (julian-day-number
		    (make-dtspec
		     :timeparse
		     (make-timeparse-majord (convert-two-digit-year year) 1 1 0 0 0)))
		   -1/2
		   intday))))
	     (at-hms
	      (make-timepoint :timeparse
			      (make-timeparse-majord
			       (timeparse-year zerohrtp)
			       (timeparse-month zerohrtp)
			       (timeparse-day zerohrtp)
			       hours minutes seconds)
			      :scale scale)))
	(datime+
	 at-hms
	 (make-pq (cl:* fracday (seconds-per-day at-hms)) 'second))))))

(defun from-jd (jd &optional modified (scale :utc))
  "Find the timepoint from the Julian date.
   Assumes a fixed number of seconds per day (86400)."
  (multiple-value-bind (jdint jdfrac)
      (cl:floor jd)
    (datime+
     (make-timepoint
      :dtspec (dtspec-from-julian-day-number jdint modified)
      :scale scale)
     (make-pq jdfrac 'day))))

(defparameter *month-names*
    '("Jan" "Feb" "Mar" "Apr" "May" "Jun"
      "Jul" "Aug" "Sep" "Oct" "Nov" "Dec"))

(defparameter *month-names-full*
    '("January" "February" "March" "April" "May" "June"
      "July" "August" "September" "October" "November" "December"))

(defun jd-table (start-year end-year &optional (format :org))
  "Write out a J2000 day number table. Output format is org-mode table (:org), LaTeX (:tex), or comma-separated values (:csv)."
  (when (eq format :tex)
    (format t
	    "\\begin{tabular}{c|~a}~&"
	    (make-sequence 'string (1+ (- end-year start-year)) :initial-element #\c)))
  (format t (case format (:org "| Month\\Year") (t " Month")))
  (loop for year from start-year to end-year
	do
	   (format t
		   (ecase format
		     (:csv ", ~d")
		     (:org "| ~d")
		     (:tex "& ~d"))
		   year))
  (case format
    (:org (format t "|~&|-"))
    (:tex (format t "\\\\ \\hline")))
  (loop for month from 1 to 12
	do
	   (format t
		   (ecase format
		     (:csv "~& ~a")
		     (:org "~&| ~a")
		     (:tex  (if (eq month 1) "~& ~a " "\\\\ ~& ~a ")))
		   (elt (if (eq format :org) *month-names-full* *month-names*) (1- month)))
	   (loop for year from start-year to end-year
		 do
		    (format t
			    (ecase format
			      (:csv ", ~d")
			      (:org "| ~d")
			      (:tex " & $~d$"))
			    (- (datime-j2000day
				(make-timepoint
				 :timeparse
				 (make-timeparse-majord year month 1)
				 :scale :utc))
			       3/2)))
	   (when (eq format :org) (princ "|")))
  (when (eq format :tex) (format t "~&\\end{tabular}")))
