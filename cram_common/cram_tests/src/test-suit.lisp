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

(defparameter *packages-under-test*
  '(;; cram-process-module-tests
    ;; cram-math-tests
    bullet-visualization-test
    cram-btr-spatial-relations-costmap-tests
    cram-urdf-environment-manipulation-tests
    cram-beginner-tutorial-tests
    cram-manipulation-interfaces-tests
    cram-common-failures-tests
    cram-task-tree-export-tests
    cram-utilities-tests
    cram-prolog-tests
    cram-projection-tests
    cram-fetch-deliver-plans-tests
    cram-bullet-reasoning-tests
    cram-pr2-pick-place-demo-tests
    cram-designators-tests
    cram-bullet-reasoning-belief-state-tests
    ))

(defparameter *results* nil)

(defun run-all-tests ()
  (setf *results* nil) ;; make a hash-table from that to update results, not throwing them away
  (unless (eq (roslisp:node-status) :RUNNING)
    (roslisp-utilities:startup-ros))
  (loop for package in *packages-under-test* do
    (format NIL "---~a START---" package)
    (push `(,package . ,(run-tests)) *results*))
  (format-results))

(defun format-results ()
  "Formats the `*results*' into a readable string."
  (let* ((no-results (mapcar #'car (remove-if #'cdr *results*)))
         (results (remove-if-not #'cdr *results*))
         report-list) ;; every new line is pushed into the report, later returned in reverse
    (flet ((report (data) (push data report-list)))
      (report (format nil "~%~%-------------TEST REPORT-------------"))
      (report (format nil "Packages under test:~%~{  ~a~^~%~}~%" *packages-under-test*))
      (report (format nil "Missing testdata for the following packages:~%~{  ~a~^~%~}"
                      (or no-results '(none))))
      (loop for (package . result) in results do
        (report (format nil "---~%Package: ~a" package))
        (report (format nil "  Assertions Passed: ~a, Failed: ~a, Error'd: ~a"
                        (lisp-unit::pass result)
                        (lisp-unit::fail result)
                        (lisp-unit::exerr result)))
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
                             (report (format nil "        Actual: ~a" (lisp-unit::actual failure)))))
            (report "    NONE"))
        (report "  Error'd Tests:")
        (if (lisp-unit::error-tests result)
            (loop for error-test-name in (lisp-unit::error-tests result)
                  and error-test = NIL do
                    (setf error-test (gethash error-test-name (lisp-unit::database result)))
                    (report (format nil "    ~a" error-test-name))
                    (report (format nil "      Error: ~a" (lisp-unit::exerr error-test))))
            (report "    NONE")))
      (report "-------------END REPORT-------------")
      (let ((passd (reduce #'+ *results* :key (alexandria:compose #'lisp-unit::pass #'cdr)))
            (faild (reduce #'+ *results* :key (alexandria:compose #'lisp-unit::fail #'cdr)))
            (errord (reduce #'+ *results* :key (alexandria:compose #'lisp-unit::exerr #'cdr))))
        (report (format nil "Total passed: ~:d, failed: ~:d, error'd: ~:d"
                        passd faild errord))))
    (format t "~{~a~^~%~}" (reverse report-list))
    *results*))

