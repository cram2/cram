;; ISO 8601:2000(E) time and date representation
;; Liam Healy Sat Aug  7 1999 - 18:07
;; Time-stamp: <2015-04-25 12:32:19EDT iso8601.lisp>

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

;;; This file parses and generates ISO8601:2000(E) representations of
;;; the date, time, and intervals, to and from a timeparse list.
;;; As currently implemented, it is not complete, but much of the more
;;; basic specifications are present.
;;; A timeparse list looks like
;;;   (second minute hour day month year)
;;; For time only the lists are
;;; (second minute hour), for day only (day month year).
;;; Note that defaults are always 0 and years are literally given:
;;; a two-digit year is represented as such.

;;; ISO8601:2000(E) ISO/TC 154 N362
;;; "Data elements and interchange formats --- Information interchange ---
;;; Representation of dates and times Second edition 2000-12-15"
;;; http://dmoz.org/Science/Reference/Standards/Individual_Standards/ISO_8601/
;;; links to
;;; http://www.qsl.net/g1smd/temp/sites.html
;;; which gives locations of PDF copies of the standard.
;;; http://www.pvv.ntnu.no/~nsaa/ISO8601.html
;;; http://www.pvv.ntnu.no/~nsaa/8601v2000.pdf


(in-package :antik)

;;;****************************************************************************
;;; Parsing 
;;;****************************************************************************

;;; (iso8601-time-parse "103020")
;;; (20 30 10)
;;; (iso8601-time-parse "10:30:20")
;;; (20 30 10)
;;; (iso8601-time-parse "10:30:20.4333")
;;; (20.4333 30 10)
;;; (iso8601-time-parse "10:30")
;;; (0 30 10)
;;; (iso8601-time-parse "10")
;;; (0 0 10)
;;; (iso8601-time-parse "10H5M13.43S")
;;; (13.43 5 10)
;;; (iso8601-time-parse "10H5M")
;;; (0 5 10)
;;; (iso8601-time-parse "10H")
;;; (0 0 10)
;;; (iso8601-time-parse "10H13.43S")
;;; (13.43 0 10)
;;; (iso8601-time-parse "5M13.43S")
;;; (13.43 5 0)
(defun iso8601-time-parse (timespec)
  "Parse time specification extended format (Time of day, Sec 5.3.1),
   time-unit designator (Time intervals, Sec 5.5.4.2.1),
   or alternative format (Time intervals, Sec 5.5.4.2.2).  Returns
   a list of numbers (second minute hour)."
  (let ((ts (remove #\: (string-upcase timespec))))
    (flet ((rfs (start &optional end)
	     (let ((*read-default-float-format* 'double-float))
	       (read-from-string ts nil nil :start start :end end))))
      (cond ((zerop (length ts)) (list 0 0 0))
	    ((some #'alpha-char-p timespec)
	     ;; "Time-unit designator" format for time intervals, Section 5.5.4.2.1
	     (let* ((hpos (position #\H ts))
		    (mpos (position #\M ts))
		    (spos (position #\S ts)))
	       (list
		(if spos (rfs (1+ (or mpos hpos -1)) spos) 0) ; second
		(if mpos (rfs (1+ (or hpos -1)) mpos) 0)      ; minute
		(if hpos (rfs 0 hpos) 0))))		      ; hour
	    (t
	     (list
	      (if (cl:> (length ts) 4) (rfs 4) 0)   ; second
	      (if (cl:> (length ts) 2) (rfs 2 4) 0) ; minute
	      (rfs 0 2)))))))			    ; hour

;;; (iso8601-date-parse "2002-05-03")
;;; (3 5 2002)
;;; (iso8601-date-parse "20020503")
;;; (3 5 2002)
;;; (iso8601-date-parse "2002-05")
;;; (0 5 2002)
;;; (iso8601-date-parse "2002")
;;; (0 0 2002)
;;; (iso8601-date-parse "020503")
;;; (3 5 2)
;;; (iso8601-date-parse "02-05-03")
;;; (3 5 2)
;;; (iso8601-date-parse "16D")
;;; (16 0 0)
;;; (iso8601-date-parse "8M16D")
;;; (16 8 0)
;;; (iso8601-date-parse "2Y16D")
;;; (16 0 2)
;;; (iso8601-date-parse "2Y8M16D")
;;; (16 8 2)
(defun iso8601-date-parse (datespec &optional convert-two-digit-year)
  "Parse date specification in complete representation Section 5.2.1.1,
   reduced precision Section 5.2.1.2, truncated representation 5.2.1.3,
   or time-unit designator Section 5.5.4.2.1. Returns a list of numbers
   (day month year).  If a field is defaulted, it will return 0, even
   though this may make no sense for month or day."
  (let ((ds (remove #\- (string-upcase datespec))))
    (flet ((rfs (start &optional end)
	     (or (read-from-string ds nil nil :start start :end end)
		 ;; return zero if start=end
		 0)))
      (cond ((zerop (length ds)) (list 0 0 0))
	    ((some #'alpha-char-p datespec)
	     ;; "Time-unit designator" format for time intervals,
	     ;; Section 5.5.4.2.1
	     (let* ((ypos (position #\Y ds))
		    (mpos (position #\M ds))
		    (dpos (position #\D ds)))
	       (list
		(if dpos (rfs (1+ (or mpos ypos -1)) dpos) 0) ; day
		(if mpos (rfs (1+ (or ypos -1)) mpos) 0) ; month
		(if ypos (rfs 0 ypos) 0)))) ; year
	    (t 
	     (let* ((lenfull (length datespec))
		    (dashpos (loop for pos from 0 below lenfull
				 when (eq (cl:aref datespec pos) #\-)
				 collect pos))
		    (lents (length ds)))
	       (cond ((or (cl:= lenfull 6)
			  (and (cl:= lenfull 8) (eq (first dashpos) 2)))
		      ;; truncated representation 5.2.1.3a (two-digit year)
		      (list (rfs 4) (rfs 2 4)
			    (if convert-two-digit-year
				(convert-two-digit-year (rfs 0 2))
			      (rfs 0 2))))
		     ;; no support for other truncated reps yet,
		     ;; but mechanism is in place
		     (t
		      ;; complete representation 5.2.1.1,
		      ;; reduced precision 5.2.1.2 a, b
		      (list
		       (if (cl:> lents 6) (rfs 6) 0) ; day
		       (if (cl:> lents 4) (rfs 4 6) 0) ; month
		       (rfs 0 4)))))))))) ; year

(defun iso8601-parse (spec &optional convert-two-digit-year)
  "Parse combinations of date and time of the day, Section 5.4, and
   attempt to make sense of either date or time as well.  Returns
   a list of 6 numbers (second minute hour day month year), and
   a list of one of :date, :time, or both, indicating which was
   specified."
  (let ((time-designator-pos
	 (or (position #\T (string-upcase spec))
	     (position #\space spec))))
    (cond (time-designator-pos
	   ;; "T" or space found separating date and time
	   (values 
	    (append (iso8601-time-parse (subseq spec (1+ time-designator-pos)))
		    (iso8601-date-parse (subseq spec 0 time-designator-pos)
					convert-two-digit-year))
	    (list :date :time)))
	  ;; colon found indicating time only
	  ((find #\: spec)
	   (values
	    (append (iso8601-time-parse spec) (list 0 0 0))
	    (list :time)))
	  ;; 14 chars, all numbers: date/time all run together
	  ((and (every #'digit-char-p spec) (cl:= (length spec) 14))
	   (values
	    (append (iso8601-time-parse (subseq spec 8))
		    (iso8601-date-parse (subseq spec 0 8)))
	    (list :date :time)))
	  ;; assume date only
	  (t (values
	      (append (list 0 0 0) (iso8601-date-parse spec))
	      (list :date))))))

;;;****************************************************************************
;;; Create ISO8601 string
;;;****************************************************************************

(defun show-datime-field (format &rest args)
  "The first of args is to be shown if it is not zero (:tud format) or
   args are not all zeros (nil format)."
  (unless
      (if (eq format :tud)
	  (zerop (first args))
	(every (lambda (x) (and (numberp x) (cl:zerop x))) args))
    ;; If (first args) is T, it means 0, but we want to print
    ;; the zero.
    (if (eq (first args) t) 0 (first args))))

(defparameter *iso8601-fractional-seconds-digits*
    3					; print all milliseconds
  "Default number of digits to print for fractional part of seconds.")

;;; (iso8601-string '(34.54 13 17 4 9 2003))
;;; "2003-09-04T17:13:34.540000"
;;; (iso8601-string '(34.54 13 17 4 9 2003) :tud)
;;; "2003Y9M4DT17H13M34.540000S"
;;; (iso8601-string '(34.54 0 17 0 1 0))
;;; "01-00T17:00:34.540000"
;;; (iso8601-string '(34.54 0 17 0 1 0) :tud)
;;; "1MT17H34.540000S" 
;;; (iso8601-string '(34.54 0 17 0 0 0) :tud)
;;; "T17H34.540000S"
;;; (iso8601-string '(34.54 13 17 0 0 0) :tud)
;;; "T17H13M34.540000S"
(defun iso8601-string
    (timeparse
     &optional
       format
       (fractional-seconds-digits *iso8601-fractional-seconds-digits*)
       (T-separator (if texstyle " " t)))
  "If format is :TUD use the time-unit designator format; if nil,
   use the extended format as described in 5.5.4.2.2.  If
   fractional-seconds-digits
   is in an integer, always display that many digits in the
   seconds fraction, even if zero.  T-separator specifies the character
   used for separating the date part from the time part; if T, it is #\T,
   if NIL, a space, if a character, that character."
  (let ((year (timeparse-year timeparse))
	(month (timeparse-month timeparse))
	(day (timeparse-day timeparse))
	(hour (timeparse-hour timeparse))
	(minute (timeparse-minute timeparse))
	(second (timeparse-second timeparse))
	(separator (if (or (characterp T-separator) (stringp T-separator))
		       T-separator
		       (if T-separator #\T #\space))))
    (format nil
	    (if (eq format :tud)
		;; Per 5.5.3.1(c) we may drop the number and designator if the
		;; number is zero.
		"~@[~dy~]~@[~dm~]~@[~dd~]~a~@[~dh~]~@[~dm~]~@[~as~]"
		;; Per 5.2.1.3 (d) and (f) truncated representation leave the dash
		;; for omitted fields. 
		(if (show-datime-field format day month year)
		    "~:[--~;~:*~4,'0d-~]~@[~2,'0d~]-~@[~2,'0d~]~a~
                              ~@[~2,'0d:~]~@[~2,'0d:~]~a"
		    ;; Though the standard doesn't say that
		    ;; the date portion can be omitted entirely
		    ;; if they are all zero, I am doing this.
		    "~4*~@[~2,'0d:~]~@[~2,'0d:~]~a"))
	    (show-datime-field format year)
	    (show-datime-field format month year)
	    (show-datime-field format day month year)
	    separator
	    (show-datime-field format hour day month year)
	    (show-datime-field format minute hour day month year)
	    (unless (and (zerop second) (eq format :tud))
	      (if (integerp second)
		  (format nil "~2,'0d" second)
		  (format-float
		   second
		   :significant-figures nil
		   :intpart-digits (if (eq format :tud) 1 2)
		   :fracpart-digits
		   (if (numberp fractional-seconds-digits) fractional-seconds-digits)
		   :intpart-fill
		   #\0))))))
;;; It would be nice to have iso8601-string
;;; be able to generate "date only".  Maybe this should be split in two,
;;; the same way parsing is?
