;; Global/local parameters to pass to functions
;; Liam Healy 2013-02-16 20:51:14HST variable-metadata.lisp
;; Time-stamp: <2014-01-08 22:26:33EST parameters.lisp>

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

(export '(make-parameters-from-table))

;;;;****************************************************************************
;;;; Make parameters
;;;;****************************************************************************

;;; This function is designed to read an org-mode table that is parsed and passed to CL by org-babel.
;;; See the file input-output/org-mode.lisp for documentation and examples
(defun make-parameters-from-table
    (table
     &key (headerp t) (category (first *parameters*)) (type grid::*default-element-type*) (prefix :par))
  "From the list of lists, define the parameters.  The optional header should have column names.  These column names include 'category 'name 'value 'type 'description ('value and 'default mean the same thing).  Any column names not specified will receive a default value.  Any values in the cells that are empty will receive a default value.  Any columns given with a header not on the list will be ignored.  Category must already exist."
  (let* ((header (when headerp (first table)))
	 (tbl (if headerp (rest table) table))
	 (catpos (position :category header :test 'string-equal))
	 (nampos (position :name header :test 'string-equal))
	 (valpos (or (position :default header :test 'string-equal)
		     (position :value header :test 'string-equal)))
	 (typepos (position :type header :test 'string-equal))
	 (descpos (position :description header :test 'string-equal))
	 (counter 0))
    ;; Should the type be 
    (dolist (row tbl)
      (incf counter)
      (let ((valtype
	      (find-symbol
	       (string-upcase
		(if (and typepos (stringp (nth typepos row)) (plusp (length (nth typepos row))))
		    (nth typepos row)
		    type)))))
	(make-parameter
	 ;; Category
	 (alexandria:make-keyword
	  (string-upcase
	   (if (and catpos (stringp (nth catpos row)) (plusp (length (nth catpos row))))
	       (nth catpos row)
	       category)))
	 ;; Name
	 (alexandria:make-keyword
	  (if (and nampos (stringp (nth nampos row)) (plusp (length (nth nampos row))))
	      (string-upcase (nth nampos row))
	      (alexandria:symbolicate prefix (format nil "~4,'0,d" counter))))
	 :value				; (default, or initial)
	 (let ((val (when (and valpos (not (equal (nth valpos row) ""))) (nth valpos row))))
	   (etypecase val
	     (number val)
	     (string (read-from-string val))
	     (null (antik:coerce grid::*default-numerical-value* valtype))))
	 :type valtype
	 :documentation
	 (if (and descpos (plusp (length (nth descpos row))))
	     (nth descpos row)
	     "No documentation."))))))
