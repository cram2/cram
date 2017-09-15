;; Read timepoint specifications.
;; Liam Healy, Sat Feb  4 2006 - 12:17
;; Time-stamp: <2015-01-03 11:17:30EST read-time.lisp>

;; Copyright 2011, 2014 Liam M. Healy
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

(export '(read-timepoint
	  read-timepoint-iso8601
	  read-time read-us-date write-us-date
	  *timescales*
	  *prehistoric*))

;;; Read many different kinds of date and time formats and
;;; create the correct timepoint.
;;; There are two alternatives for reading iso8601 strings,
;;; via read-timepoint and via read-timepoint-iso8601.
;;; They each can read strings the other can't,
;;; but the former is what read-time and thus #d macro reader use.

;;; Note: currently will not read specifications with letters.

;;; ISO8601 variants
;;; (read-timepoint "1999-03-30T12:33:45")
;;; (read-timepoint "1999-03-30 12:33:45")
;;; (read-timepoint "1999?03/30    % 12-33-45")
;;; (read-timepoint "1999-03-30T12:33:45EST")
;;; (read-timepoint "1999-03-30T12:33:45-05")
;;; (read-timepoint "1999-03-30")

;;; Others
;;; RINEX files
;;; (read-timepoint "03  6  6  9 59 44.0")
;;; (read-timepoint "03     6  6  9 59 44.0" nil :utc)
;;; OCEAN files (read-integrator-step)
;;; ?
;;; US customary, SSR files
;;; (read-us-date "01/15/2006")

(defun read-timepoint (string &optional pos-ymdhms (scale :utc))
  "Read a timepoint from a string with specification for the position in the string of each component; pos-ymdhms is a list of year, month, day, hour, minute, and second as sequence numbers for the integers in the string.  Scale is the timescale (zone) as a string or symbol.  If pos-ymdhms has only three components, or only a date is provided, the timepoint created will be specifed as day-only.  The default reads an ISO8601 string like 1999-03-30T12:33:45."
  (let* ((pcs				; list of (month day year)
	  (mapcar #'read-from-string
		  (split-sequence:split-sequence-if
		   (complement (lambda (c) (or (digit-char-p c) (eql c #\.))))
		   string :remove-empty-subseqs t)))
	 (lpcs (length pcs))
	 (poss (or pos-ymdhms '(0 1 2 3 4 5))))
    (macrolet
	((ymdhms (n)
	   `(let ((indx (nth ,n poss)))
	      (if (and (numberp indx) (> lpcs ,n))
		  (nth indx pcs)
		  ,(when (cl:< n 3) `(error "Insufficient number of components."))))))
      (if (cl:<= lpcs 2)     ; yymmdd as one number, hhmmss as another
	  (make-timepoint
	   :timeparse
	   (make-timeparse-majord
	    (convert-two-digit-year (floor (ymdhms 0) 10000))  ; year
	    (mod (floor (ymdhms 0) 100) 100)		       ; month
	    (mod (ymdhms 0) 100)			       ; month
	    (if (ymdhms 1) (floor (ymdhms 1) 10000) 0)	       ; hour
	    (if (ymdhms 1) (mod (floor (ymdhms 1) 100) 100) 0) ; minute
	    (if (ymdhms 1) (mod (ymdhms 1) 100) 0)) ; second
	   :scale scale
	   :day-only (= lpcs 1))
	  (let ((timespecp (ymdhms 3)))
	    (make-timepoint
	     :timeparse
	     (make-timeparse-majord
	      (convert-two-digit-year (ymdhms 0)) ; year
	      (ymdhms 1)			  ; month
	      (ymdhms 2)			  ; day
	      (or (ymdhms 3) 0)			  ; hour
	      (or (ymdhms 4) 0)			  ; minute
	      (or (ymdhms 5) 0))		  ; second
	     :scale
	     (or
	      (and timespecp
		   (read-timescale
		    (subseq string
			    (1+ (position-if #'digit-char-p string :from-end t)))))
	      scale)
	     :day-only (or (cl:<= (length poss) 3) (not timespecp))))))))

(defparameter *timezone-offsets*
  '((:acdt . 10.5)
    (:acst . 9.5)
    (:act . 8)
    (:adt . -3)
    (:aedt . 11)
    (:aest . 10)
    (:aft . 4.5)
    (:akdt . -8)
    (:akst . -9)
    (:amst . 5)
    (:amt . 4)
    (:art . -3)
    (:arab . 3)
    (:arabian . 4)
    (:awdt . 9)
    (:awst . 8)
    (:azost . -1)
    (:bdt . 8)
    (:biot . 6)
    (:bit . -12)
    (:bot . -4)
    (:brt . -3)
    (:bangladesh . 6)
    (:bst . 1)
    (:btt . 6)
    (:cat . 2)
    (:cdt . -5)
    (:cedt . 2)
    (:cest . 2)
    (:cet . 1)
    (:chadt . 13.75)
    (:chast . 12.75)
    (:cist . -8)
    (:ckt . -10)
    (:clt . -4)
    (:cost . 4)
    (:cot . -5)
    (:cst . -6)
    (:china . 8)
    (:centaust . 9.5)
    (:ct . 8)
    (:cvt . -1)
    (:cxt . 7)
    (:chst . 10)
    (:east . -6)
    (:eat . 3)
    (:ect . -4)			; carribean
    (:ecuador . -5)
    (:edt . -4)
    (:eedt . 3)
    (:eet . 2)
    (:est . -5)
    (:fkst . -3)
    (:fkt . -4)
    (:galt . -6)
    (:get . 4)
    (:gft . -3)
    (:gilt . 12)
    (:git . -9)
    (:gmt . 0)
    (:gst . -2)
    (:gyt . -4)
    (:hadt . -9)
    (:hast . -10)
    (:hkt . 8)
    (:hmt . 5)
    (:hst . -10)
    (:ict . 7)
    (:idt . 3)
    (:irkt . 8)
    (:irst . 3.5)
    (:ist . 5.5)		; Indian summer time
    (:irish . 1)		; Irish summer time
    (:israel . 2)		; Israel standard time
    (:jst . 9)
    (:krat . 7)
    (:kst . 9)
    (:lhst . 10.5)
    (:lint . 14)
    (:magt . 11)
    (:mdt . -6)
    (:mit . 9.5)
    (:msd . 4)
    (:msk . 3)
    (:mst . -7)
    (:malaysia . 8)
    (:myanmar . 6.5)
    (:mut . 4)
    (:myt . 8)
    (:ndt . -2.5)
    (:nft . 11.5)
    (:npt . 5.75)
    (:nst . -3.5)
    (:nzdt . 13)
    (:nzst . 12)
    (:omst . 6)
    (:pdt . -7)
    (:pett . 12)
    (:phot . 13)
    (:pkt . 5)
    (:pst . -8)
    (:phillipine . 8)
    (:ret . 4)
    (:samt . 4)
    (:sast . 2)
    (:sbt . 11)
    (:sct . 4)
    (:sgt . 8)
    (:slt . 5.5)
    (:sst . -11)
    (:singapore . 8)
    (:taht . -10)
    (:tha . 7)
    (:uyst . -2)
    (:uyt . -3)
    (:vet . -4.5)
    (:vlat . 10)
    (:wat . 1)
    (:wedt . 1)
    (:west . 1)
    (:wet . 0)
    (:wst . 8)
    (:yakt . 9)
    (:yekt . 5))
  ;; from http://en.wikipedia.org/wiki/List_of_time_zone_abbreviations
  "Timezones and number of hours from UTC, used to input timepoints.")

(defparameter *timescales*
    (append '("utc" "tai" "gps") (mapcar #'first *timezone-offsets*))
  "All known time scales.")

(defun read-timescale (string)
  (let ((ts (first (member string *timescales* :test #'string-equal))))
    (if ts
	;; a named timescale
	(alexandria:make-keyword (string-upcase ts))
	(if (and (cl:>= (length string) 3) ; long enough to include time part
		 (member (cl:aref string 0) '(#\+ #\-))) ; and a signed integer
	    (read-from-string string)))))

(defun read-timepoint-iso8601 (string &optional (scale :utc))
  "Read the timepoint specified as an ISO8601 string.  In contrast
   to #'read-timepoint, this accepts the various rearrangements
   permitted by ISO8601 (see documentation for #'iso8601-parse,
   but does not accept miscellaneous separator symbols."
  ;; Provided for completeness.
  (make-timepoint 
   :dtspec (read-dtspec string)
   :scale scale))

;;; (read-us-date "12/20/2000 3:41:12")
;;; #d2000-12-20T03:41:12.000000000000000
(defun read-us-date (string &optional day-only)
  "Read dates and times in customary US format MM/DD/YYYY; times may
   be included as well if day-only is nil."
  (if day-only
      (read-timepoint string '(2 0 1))
      (read-timepoint string '(2 0 1 3 4 5))))

(defun write-us-date (datime)
  "Write dates and times in customary US format MM/DD/YYYY."
  (let ((tp (timeparse datime)))
    (format nil "~a/~a/~a"
	    (timeparse-month tp)
	    (timeparse-day tp)
	    (timeparse-year tp))))

(defun read-time (string)
  "Parse the datime or time interval string and create a timepoint object."
  (if (char-equal (cl:aref string 0) #\p)
	(iso8601-parse-time-interval (subseq string 1))
	(read-timepoint string)))

#|
(defun numeric-month (string)
  "Find the month number from the English month name."
  (1+
   (find
    string
    *month-names*
    :test
    (lambda (x) (string-equal x "unsigned" :end1 3)))))

;; (split-sequence #\space "Fri Sep  2 14:00:00 2011 EDT" :remove-empty-subseqs t)
;; ("Fri" "Sep" "2" "14:00:00" "2011" "EDT")

;;; use read-timepoint, don't need to split-sequence
;;; map from month name to number in read-timepoint
(defun read-unix-date (string &optional day-only)
  "Read dates and times in UNIX format, e.g. Fri Sep  2 14:00:00 2011 EDT."
  (let ((parsed
	  (split-sequence:split-sequence
	   #\space
	   string :remove-empty-subseqs t)))
    (if day-only
      (read-timepoint string '(2 0 1))
      (read-timepoint string '(2 0 1 3 4 5)))))
|#

(defun read-datime (stream subchar arg)
  (declare (ignore subchar arg))
  (let* ((string
	   (stream-to-string
	    stream
	    :terminate-if
	    (lambda (ch)
	      (not (or (alphanumericp ch)
		       (member ch '(#\: #\- #\.) :test #'char-equal)))))))
    (unless *read-suppress*
      (read-time string))))

;;; Read timepoint with reader macro, e.g.  #d1999-03-30T12:33:45.
;;; Is there any way to get this on additional listeners?
(set-dispatch-macro-character
 #\# #\d 'read-datime
 (named-readtables:find-readtable :antik))

(defparameter *prehistoric* (read-timepoint "1900-01-01T00:00:00"))
