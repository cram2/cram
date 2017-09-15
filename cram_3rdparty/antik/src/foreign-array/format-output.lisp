;; Format of foreign (C or C-compatible) arrays
;; Liam Healy 2008-12-28 10:44:22EST foreign-array.lisp
;; Time-stamp: <2014-01-08 22:58:25EST print-foreign-array.lisp>
;;
;; Copyright 2013, 2014 Liam M. Healy
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

(in-package :grid)

(export '(*print-foreign-array-readably*))

(defparameter *print-foreign-array-readably* t
  "Print the contents of the foreign-array with the #m reader macro.")

(defmethod print-object :around ((object foreign-array) stream)
  (if (and *print-foreign-array-readably* *print-contents*)
      (antik::cl-readable-nf (antik::nf-grid object stream))
      (call-next-method)))

(defmethod antik:nf ((object foreign-array) &optional (stream *standard-output*))
  (antik::nf-grid object stream))
