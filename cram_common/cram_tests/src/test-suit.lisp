;;;
;;; Copyright (c) 2022, Arthur Niedzwiecki <niedzwiecki@uni-bremen.de>
;;;
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors
;;;       may be used to endorse or promote products derived from this software
;;;       without specific prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :cram-tests)

;; All functionality in this file is meant to be used with scripts/test.sh
;; but is also works on it's own. That's what the default filenames are for.

(defparameter *result* nil
  "The result of lisp-unit's test execution.")
(defparameter *report* nil
  "`report' pushes strings to the `*report*' list, `return-report' reverses and creates a string.")
(defparameter *default-report-file*
  (concatenate 'string
               (namestring (ros-load:ros-package-path "cram_tests"))
               "reports/test-report-"
               (format nil "~d" (sb-posix:time))))

(defun report (data)
  "Pushes a new `data' line to the report."
  (push data *report*))

(defun return-report ()
  "Returns the report in reversed order, because of how `report' works."
  (format NIL "~{~a~^~%~}" (reverse *report*)))

(defun run-package-tests (package &optional (report-file-path *default-report-file*))
  (declare (type keyword package)
           (type string report-file-path))
  "Run with a specific `package' that is available via asdf.
If that package is not loaded yet, it will be.
Exectues all lisp-unit tests in that package, formats it and
exports the report into a file in the `report-file-path'."
  (setf *report* nil)
  (if (not (asdf:find-system package NIL))
      (format NIL "System ~a doesn't exist." package)
      (progn
        (unless (find-package package)
          (asdf:load-system package))
        (setf btr-belief:*spawn-debug-window* nil)
        (unless (eq (roslisp:node-status) :RUNNING)
          ;; doing it here for all the tests without a proper init
          (roslisp-utilities:startup-ros))
        (setf *result* (lisp-unit:run-tests :all package))
        (format-result package)
        (export-report :filename report-file-path)))
  (print (return-report))
  (print (format nil "Report written to ~a." report-file-path)))

(defun format-result (package &optional (result *result*))
  (declare (type keyword package))
  "Formats the `*result*' into a readable report."
  (report "----------")
  (if (not *result*)
      (report (format nil "Package: ~a - No tests defined" package))
      (let ((passd (lisp-unit::pass result))
            (faild (lisp-unit::fail result))
            (errord (lisp-unit::exerr result)))
        (report (format nil "Package: ~a" package))
        (report (format nil "  Assertions Passed: ~a, Failed: ~a, Error'd: ~a" passd faild errord))
        (report "")
        (report "  Failed Tests:")
        (if (lisp-unit::failed-tests result)
            (loop for failed-test-name in (lisp-unit::failed-tests result)
                  and failed-test = NIL do
                    (setf failed-test (gethash failed-test-name (lisp-unit::database result)))
                    (report (format nil "    ~a" failed-test-name))
                    (loop for failure in (lisp-unit::fail failed-test)
                          do (report (format nil "      Test: ~a" (lisp-unit::test failure)))
                             (report (format nil "        Form: ~a" (lisp-unit::form failure)))
                             (report (format nil "        Expected: ~a" (lisp-unit::expected failure)))
                             (report (format nil "        Actual: ~a" (lisp-unit::actual failure))))
                    (report ""))
            (report "    NONE"))
        (report "  Error'd Tests:")
        (if (lisp-unit::error-tests result)
            (loop for error-test-name in (lisp-unit::error-tests result)
                  and error-test = NIL do
                    (setf error-test (gethash error-test-name (lisp-unit::database result)))
                    (report (format nil "    ~a" error-test-name))
                    (report (format nil "      Error: ~a~%" (lisp-unit::exerr error-test))))
            (report "    NONE")))))

(defun export-report (&key (filename *default-report-file*) (report (reverse *report*)))
  (declare (type string filename)
           (type list report))
  "Exports the report into a file named `filename'."
  (with-open-file (report-file filename
                               :direction :output
                               :if-exists :append
                               :if-does-not-exist :create)
    (dolist (line report)
      (write-line line report-file))
    (write-line "" report-file)))
