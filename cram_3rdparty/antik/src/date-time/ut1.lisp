;; UT1 timescale
;; Liam Healy 2015-01-03 11:13:34EST ut1.lisp
;; Time-stamp: <2015-12-24 15:55:13EST ut1.lisp>

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

;;; Only user definition is parameter *real-ut1-utc*
;;; Ref. [[id:ff2e9f1e-c9d2-4f3f-8435-59ea8eb42c1b][Astronomical time systems]]

(in-package :antik)

(export '*real-ut1-utc*)
(defparameter *real-ut1-utc* t "If NIL, user wants UT1 to be the same as UTC.")

(push "ut1" *timescales*)

(defparameter *ut1-utc-table* nil
  "Hash table of differences UT1-UTC by J2000 day number.
   Use (read-op-file) to load.")

(defparameter *ut1-utc-first-estimated* nil
  "First estimated UT1-UTC value in *ut1-utc-table*.
   This serves as a marker for the age of the eop.dat file.")

(defparameter *eop-data-fetch* nil
  "Bound to a thunk when drakma is loaded.")

(defun read-eop-file ()
  "Read the earth orientation parameters from USNO."
  (unless *eop-data-fetch*
    #+quicklisp
    (progn
      (cerror "Load science-data" "To fetch earth orientation data, science-data system needs to be loaded.")
      (ql:quickload "science-data"))
    #-quicklisp
    (error "To fetch earth orientation data, science-data system needs to be loaded."))
  (let ((eop-data (funcall *eop-data-fetch*)))
    (unless (and (> (length eop-data) 10000)
		 (equal (subseq eop-data 0 2) "73"))
      ;; Drakma doesn't signal errors for HTTP failure, just puts the
      ;; error or whatever is returned. So we just measure the length
      ;; and assume that if it's too short or if the first two
      ;; characters aren't "73", it's not the actual data.
      (warn "Failed to successfully download and read earth orientation data.")
      (return-from read-eop-file *ut1-utc-table*))
    (with-input-from-string
	(stream eop-data)
      (loop for line cl:= (read-line stream nil nil)
	    with table cl:= (make-hash-table :test #'eql :size 2000)
	    and estimated and date
	    while line
	    do
	       (setf
		date
		(make-timepoint
		 :timeparse
		 (list 0 0 0
		       ;; see ftp://maia.usno.navy.mil/ser7/readme.finals
		       ;; for column description
		       (read-from-string line nil nil :start 4 :end 6)
		       (read-from-string line nil nil :start 2 :end 4)
		       (convert-two-digit-year
			(read-from-string line nil nil :start 0 :end 2)))
		 :scale :utc
		 :day-only t)
		(gethash (cl:+ 1/2 (datime-j2000day date)) table)
		;; see ftp://maia.usno.navy.mil/ser7/readme.finals
		;; for column description
		(read-from-string line nil nil :start 58 :end 68))
	       (if (and (not estimated) (equal (subseq line 79 86) "       "))
		   (setf estimated date))
	    finally (setf *ut1-utc-table* table
			  *ut1-utc-first-estimated* estimated))
      *ut1-utc-table*)))

(defun convert-utc-ut1 (timespec &optional to-ut1)
  "Convert UTC time scale to or from UT1 by adding/subtracting the appropriate fractional second. Returns two values: the timespec, and whether conversion was successful."
  (unless *real-ut1-utc* (return-from convert-utc-ut1 (values timespec nil)))
  (flet ((no-data ()
	   (cerror "Take UT1 and UTC as the same from now on."
		   "Unable to find UT1-UTC table, or to find ~a in it."
		   timespec)
	   (setf *real-ut1-utc* nil)
	   (return-from convert-utc-ut1 (values timespec nil))))
    (unless (or *ut1-utc-table* (read-eop-file)) ; Read EOP file if unread
      (no-data))
    ;; Remove staleness check [[id:bc965ced-8e10-405f-becd-08d60f19a520][Split out system]]
    ;; Do the addition/subtraction
    (values 
     (add-uniform-time
      timespec
      (antik:* (if to-ut1 +1 -1)
	       (or (gethash (cl:+ 1/2 (datime-j2000day timespec)) *ut1-utc-table*)
		   (no-data))))
     t)))
;;; Round trip doesn't quite always come out even
;;; (convert-utc-ut1 (convert-utc-ut1 #D2005-01-01T00:00:00 t))
;;; #d2004-12-31T23:59:59.999556064605710
;;; because it is using different days in lookup table.

(setf *UTC-to-UT1* #'convert-utc-ut1)
