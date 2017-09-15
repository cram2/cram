;; Analysis of interface status
;; Liam Healy 2008-11-16 16:17:14EST analysis.lisp
;; Time-stamp: <2009-12-27 09:50:35EST analysis.lisp>
;;
;; Copyright 2008, 2009 Liam M. Healy
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

;;; These definitions are to monitor the status of the GSLL interface
;;; with respect to the GSL library for the purpose of directing
;;; development of GSLL.  Therefore, they are not loaded in the
;;; system.  They require the port system.

(in-package :gsl)

(defparameter *gsl-library-path*
  (namestring
   (cffi::foreign-library-handle (cffi::get-foreign-library 'cl-user::libgsl))))

(defparameter *gsl-library-symbols*
  (with-open-stream
      (instream
       (port:pipe-input			; linux commands
	"/usr/bin/nm" "-D" "--defined-only" "-g" *gsl-library-path*))
    (loop for symbol = (read-line instream nil nil)
       while symbol
       collect (subseq symbol 19)))
  "All the external symbols in the GSL library.")

(defun in-gsl (string)
  "The symbol occurs in the GSL library."
  (find string *gsl-library-symbols* :test 'string-equal))

(defparameter *gsll-defined-symbols*
  (loop for gslsymb being the hash-key of *gsl-symbol-equivalence* collect gslsymb)
  "All the exported definitions in GSLL.")

(defun name-special-function-no-error (string)
  "Names a function for a special function that does not return an error estimate."
  (and (search "gsl_sf_" string :test 'string-equal)
       (not (string-equal (subseq string (- (length string) 2)) "_e"))
       (in-gsl (concatenate 'string string "_e"))))

(defun defined-without-strings (defined-list &rest strings)
  "A list of definitions in GSL where none of the given strings occurs."
  (loop for string in strings
     for from = defined-list then list
     for list = (remove-if (lambda (x) (search string x :test 'string-equal)) from)
     finally (return list)))

(defun gsll-defined-with-string (string)
  "A list of definitions in GSLL where the given strings occurs."
  (remove-if-not
   (lambda (x) (search string x :test 'string-equal))
   *gsll-defined-symbols*))

;;; Step through the list of GSL definitions, removing those things
;;; we're not going to port.

(defparameter *target-port-1*
  (defined-without-strings *gsl-library-symbols* "fscan" "fread" "print" "view"))

(defparameter *target-port-2*
  (remove-if 'name-special-function-no-error *target-port-1*))

;;; Step through the list of GSLL definitions, removing those things
;;; that we're not advertising we've ported, but did port anyway for
;;; internal use or for whatever reason.

(defparameter *have-port-1*
  (defined-without-strings *gsll-defined-symbols* "fscan" "fread" "print" "view"))
