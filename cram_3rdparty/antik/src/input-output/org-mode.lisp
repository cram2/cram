;; Exchange data through org-mode, particularly org-babel.
;; Liam Healy 2013-01-21 12:35:10EST org-mode.lisp
;; Time-stamp: <2013-05-26 18:45:04EDT org-mode.lisp>

;; Copyright 2013 Liam M. Healy
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

;;; This is experimental, interface is likely to change.
(export '(export-value format-value val *code-block-eval-nf-options* org-table-line org-table))
;;; No documentation yet.

;;;;****************************************************************************
;;;; Input
;;;;****************************************************************************

#| 
   + This is an example of how to read a table to define parameters.  First, it is necessary to define the table with at least some subset of the columns
     - Name
     - Value
     - Description
     - Type
     - Category
   + Each column should be labeled in the header (first) line, but they can be in any order.
   + Not all columns are required; if a column is missing, the parameter of an optional column will receive a default value.
   + Additional columns may be included, and they will be ignored.
   + Example of a table; note that the last line (all blank) defines a parameter (:PAR0004) with defaults:
    #+TBLNAME: test-table
    |------+-------+-------------+--------------+----------|
    | Name | Value | Description | Type         | Category |
    |------+-------+-------------+--------------+----------|
    | A    |  3.21 | Length      | double-float | testcat  |
    | B    |   4.5 | Width       | double-float | testcat  |
    | C    |   5.0 | Height      | double-float | testcat  |
    |      |       |             |              |          |
    |------+-------+-------------+--------------+----------|
   + In a lisp source section, define the name with variable=tablename, where tablename is the #+TBLNAME given to the table, and 'variable is the CL variable name
   + Example of Lisp code
   #+name: anexample(parameters=test-table)
   #+BEGIN_SRC lisp
   (make-parameters-from-table parameters)
   (* (parameter-value :testcat :a) (parameter-value :testcat :b) (parameter-value :testcat :c))
   #+END_SRC
   #+RESULTS: anexample
   : 72.225
   + These are the definitions of each parameter.
     #+BEGIN_QUOTE
     ANTIK-USER> (parameter-help :testcat)
     Parameters in TESTCAT: A, B, C,  and PAR0004.
     NIL
     ANTIK-USER> (parameter-help :testcat :a)
     A: Length
     Type is DOUBLE-FLOAT, default value is 3.21, 
     current value is 3.21.
     NIL
     ANTIK-USER> (parameter-help :testcat :b)
     B: Width
     Type is DOUBLE-FLOAT, default value is 4.5, 
     current value is 4.5.
     NIL
     ANTIK-USER> (parameter-help :testcat :c)
     C: Height
     Type is DOUBLE-FLOAT, default value is 5.0, 
     current value is 5.0.
     NIL
     ANTIK-USER> (parameter-help :testcat :par0004)
     PAR0004: No description.
     Type is DOUBLE-FLOAT, default value is 0.0, 
     current value is 0.0.
     #+END_QUOTE
|#

;;;;****************************************************************************
;;;; Output
;;;;****************************************************************************

(defparameter *code-block-eval-nf-options* '(:significant-figures 5)
  "NF options for LaTeX output generated from parameters.")

(defmacro export-value (form &optional (nf-options *code-block-eval-nf-options*))
  "The form value formatted for tex.  Useful in embedded org-mode code block evaluation as e.g.
   src_lisp[:results raw]{(export-value xyz)}."
  `(with-nf-options (:style :tex ,@nf-options)
     (nf-string ,form)))

(defmacro val
    (category variable &optional (nf-options *code-block-eval-nf-options*))
  "The parameter value formatted for tex.  Useful in embedded org-mode code block evaluation as e.g.
   src_lisp[:results raw]{(val P1020 B)}."
  `(export-value (parameter-value ,category ,variable) ,nf-options))

;;; Currently very simple, doesn't discern the dimension, for lists only.
;;; Eventually, hook this into its own :style, or nil.

(defun org-table-line (list &optional (stream t))
  "Format the list for a line of an org-mode table."
  (format stream "|~{ ~a | ~}" (mapcar 'nf-string list)))

(defun org-table (list-of-lists &optional (stream t))
  "Format the table."
  (dolist (line list-of-lists)
    (org-table-line line stream)
    (fresh-line stream)))
