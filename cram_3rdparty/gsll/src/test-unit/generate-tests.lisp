;; Make tests from examples.
;; Liam Healy 2008-09-07 21:00:48EDT generate-tests.lisp
;; Time-stamp: <2009-12-27 10:12:12EST generate-tests.lisp>
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

;;; Througout the GSLL interface definition files are #'save-test
;;; forms.  These serve to define both examples and tests.  Getting an
;;; example involves calling (examples) to get a list of names, then
;;; the calling the function with a particular name to get the
;;; examples, e.g.
;;; (examples)
;;; (examples 'matrix-add)

;;; To do all the tests,
;;; (lisp-unit:run-tests)

;;; The files that define the tests are in tests/.  These files are
;;; generated automatically and checked into the repository; they
;;; shouldn't be changed very often.  Rarely, it may be necessary to
;;; generate such a file.  In this case, #'write-test-to-file recreates
;;; the file, e.g.
;;; (write-test-to-file 'matrix-add "test/")

(in-package :gsl)

(defun numerical-serialize (form)
  (if (typep form 'list)
      (cons 'list (mapcar #'numerical-serialize form))
      (if (typep form 'mobject)
	  (make-load-form form)
	  form)))

;;; (make-test '(legendre-conicalP-half 3.5d0 10.0d0))
(defun make-test (form &optional answer)
  "Make a test for lisp-unit.  If the answer is known separately from
   evaluation of the form, it may be supplied in 'answer; note that
   this only accommodates a single value at present."
  (let ((vals (multiple-value-list (or answer (ignore-errors (eval form))))))
    (if (typep (second vals) 'condition)
	`(lisp-unit::assert-error
	  ',(type-of (second vals))
	  ,form)
	`(lisp-unit::assert-numerical-equal
	  ,(numerical-serialize vals)
	  (multiple-value-list ,form)))))

(defun create-test (test-name &optional answers)
  "Find the saved test by name and create it, with the generated results."
  (append
   `(lisp-unit:define-test ,test-name)
   (let ((test-set (getf *all-generated-tests* test-name)))
     (mapcar #'make-test test-set
	     (or answers (make-list (length test-set) :initial-element nil))))))

(defun write-test-to-file (test path &optional answers)
  "Write the test to a file with the same name under path.
   Use this function with caution; it will replace an existing
   test file and thus the opportunity for regression test will be lost."
  (let ((pathname (merge-pathnames (format nil "~(~a~).lisp" test) path))
	(*read-default-float-format* 'single-float))
    (with-open-file
	(stream pathname :direction :output :if-exists :rename)
      (format
       stream
       ";; Regression test ~a for GSLL, automatically generated~%~%" test)
      (format stream "(in-package :gsl)~%~%")
      (format t "Writing test ~a to file ~a~&" test pathname)
      (format stream "~s~%~%" (create-test test answers)))))


;;; This is commented out because it shouldn't normally be run.  It
;;; will regenerate all tests, so there will be no regression tests to
;;; previous versions.  DON'T FORGET THE TRAILING SLASH ON THE PATH
;;; (write-tests "/home/liam/mathematics/gsl/tests/")
#+(or)
(defun write-tests (path)
  "Write all the tests to the appropriate file."
  (iter:iter
    (iter:for (key val) on *all-generated-tests* by #'cddr)
    (write-test-to-file key path)))
