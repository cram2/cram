;; Tests for date-time 
;; Liam Healy 2014-01-13 22:00:52EST tests.lisp
;; Time-stamp: <2014-12-26 15:37:08EST time-interval-tests.lisp>

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
(named-readtables:in-readtable :antik)

;;;****************************************************************************
;;; Tests
;;;****************************************************************************

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

;;;; From before:

;;; Input formats automatically recognized
;;; Output formats should be specified with
;:calendar-date, :ordinal-date,:calendar-week-day, :calendar-week
;:day-of-week, :calendar-month, :calendar-year

;:extended-format, :basic-format

;;; parsed format, with all possible specifications
;;; (:year 1999 :month 8 :day 7 :hour 20 :minute 19 :second 12 
;;;  :year-of-century 99 :century 19 
;;;  :day-of-week 6 :week-of-year 33 :day-of-year 155)
;;;  :second and :day-of-year may have a fractional component;
;;;  the rest are integers.

(defmacro piti (string)
  `(ignore-errors (iso8601-parse-time-interval ,string)))

(lisp-unit:define-test time-interval
    (lisp-unit:assert-numerical-equal
     #dPT22h
     (piti "T22"))			; 22 hours
  (lisp-unit:assert-numerical-equal
   #dPT22.000s
   (piti "T22S"))			; 22 seconds
  (lisp-unit:assert-numerical-equal
   #dPT14m22s
   (piti "T14M22S"))			; 14 minutes, 22 seconds
  (lisp-unit:assert-numerical-equal
   #dPT14h22m
   (piti "T14:22"))			; 14 hours, 22 minutes
  (lisp-unit:assert-numerical-equal
   #dPT6h14m22.000s
   (piti "T6H14M22S"))		     ; 6 hours, 14 minutes, 22 seconds
  (lisp-unit:assert-numerical-equal
   #dPT6h14m22.000s
   (piti "T06:14:22"))		     ; 6 hours, 14 minutes, 22 seconds
  (lisp-unit:assert-numerical-equal
   #dP16dT6h14m22.000s
   (piti "16DT6H14M22S"))   ; 16 days, 6 hours, 14 minutes, 22 seconds
  (lisp-unit:assert-numerical-equal
   #dP8m16dT6h14m22.000s
   (piti "8M16DT6H14M22S"))		; 8 months, 16 days, ...
  (lisp-unit:assert-numerical-equal
   #dP8mT6h14m22.000s
   (piti "8MT6H14M22S"))		; 8 months, 0 days, ...
  (lisp-unit:assert-numerical-equal
   #dP2y8m16dT6h14m22.000s
   (piti "2Y8M16DT6H14M22S"))		; 2 years, 8 months, ...
  (lisp-unit:assert-numerical-equal
   #dP2y8m16dT6h14m22.000s
   (piti "0002-08-16T06:14:22")))	; 2 years, 8 months, ...

;; not permitted yet
;;(piti "--16T06:14:22")		; 16 days, 6 hours, 14 minutes, 22 seconds
;; not permitted yet
;;(piti "P-08-16T06:14:22")		; 8 months, 16 days, ...
     

