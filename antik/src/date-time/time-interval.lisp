;; Time interval parsing and generating.
;; Liam Healy Thu May  2 2002 - 16:34
;; Time-stamp: <2015-04-25 13:07:07EDT time-interval.lisp>

;; Copyright 2011, 2013, 2015 Liam M. Healy
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

;;; ISO8601:2000(E) Sec. 5.5.4.2 "Representation of time interval by duration only"

(in-package :antik)

(export '(timeparse-time-interval iso8601-time-interval time-precision-seconds
	  time-interval-timeparse iso8601-parse-time-interval))

;;;;****************************************************************************
;;;; Output 
;;;;****************************************************************************

(defun timeparse-time-interval
    (time &optional (fractional-seconds-digits *iso8601-fractional-seconds-digits*))
  "Convert the time interval to a timeparse."
  (with-pq ((time time))
    (let* ((timemag (pqval time))
	   (rounded-time
	     ;; Use the time in seconds to the formatted precision
	     ;; to prevent something like
	     ;; (iso8601-time-interval #_35999.9999999998_sec :tud)
	     ;; "PT9h59m60.000s"
	     (if (rationalp timemag)
		 (abs timemag)
		 (let ((*read-default-float-format* 'double-float))
		   (read-from-string 
		    (format-float
		     (abs timemag)
		     :significant-figures nil
		     :fracpart-digits fractional-seconds-digits)))))
	   (tseconds rounded-time))
      (multiple-value-bind
	    (tminutes seconds) (cl:floor tseconds +seconds-per-minute+)
	(multiple-value-bind
	      (thours minutes) (cl:floor tminutes +minutes-per-hour+)
	  (multiple-value-bind
		(tdays hours) (cl:floor thours +hours-per-day+)
	    (multiple-value-bind
		  (tmonths days) (cl:floor tdays +days-per-month+)
	      (multiple-value-bind
		    (years months) (cl:floor tmonths +months-per-year+)
		(make-timeparse-minord seconds minutes hours days months years)))))))))

(defun iso8601-time-interval
    (time
     &optional (format :tud)
       (fractional-seconds-digits *iso8601-fractional-seconds-digits*))
  "Represent the time interval in ISO8601:2000(E) Section 5.5.4.2.1 format, representation of time-interval by duration only."
  (mkstr (if texstyle "" "P")		; interval marker
	 (iso8601-string
	  (fn-major-timeparse
	   (timeparse-time-interval time fractional-seconds-digits)
	   (if (minusp (cl:signum (pq-magnitude time))) #'cl:- #'identity))
	  format
	  fractional-seconds-digits)))

(defun time-precision-seconds (time-interval &optional (float-format 'double-float))
  "Approximate inaccuracy in seconds of the time interval; e.g. if 1, than the seconds digit is inaccurate."
  (with-si-units
    (* (pqval time-interval)
       (symbol-value (alexandria:ensure-symbol (mkstr float-format "-EPSILON"))))))

;;;;****************************************************************************
;;;; Input
;;;;****************************************************************************

(defun time-interval-timeparse (timeparse)
  "Convert the timeparse to a time interval."
  (make-pq
   (cl:+ (timeparse-second timeparse)
	 (cl:* +seconds-per-minute+ (timeparse-minute timeparse))
	 (cl:* +seconds-per-hour+ (timeparse-hour timeparse))
	 (cl:* +seconds-per-day+ (timeparse-day timeparse))
	 (cl:* +seconds-per-month+ (timeparse-month timeparse))
	 (cl:* +seconds-per-year+ (timeparse-year timeparse)))
   'second))

(defun iso8601-parse-time-interval (string)
  "Parse the time interval in ISO8601:2000(E) Section 5.5.4.2.1 format, representation of time-interval by duration only."
  (time-interval-timeparse (iso8601-parse string)))

;;;;****************************************************************************
;;;; Format using nf
;;;;****************************************************************************

(define-parameter nf :time
  :value :tud :type (or null (member :tud :alternate))
  :documentation
  "How to format time intervals:
   :tud = ISO8601 time-unit designator (like P10H7M23.726979085813582S)
   :alternative = ISO8601 alternate (like P10:07:23.726979085813582)
   nil = with the unit for time given by :system-of-units.")

(defun time-interval-format (pq &optional (stream t))
  "Format the time interval ISO8601."
  (when (and (nf-option :time)
	     (not (grid:gridp (pq-magnitude pq)))
	     (check-dimension pq 'time nil nil))
    (unless texstyle (when (nf-readably) (princ "#d" stream)))
    (princn (iso8601-time-interval pq (nf-option :time))
	    stream)))

;;; Set hook for method (nf physical-quantity) to format time intervals using ISO8601 if the option is selected.
(setf *time-interval-format-function* #'time-interval-format)
