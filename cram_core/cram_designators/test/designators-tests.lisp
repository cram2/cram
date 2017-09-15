;;; Copyright (c) 2012, Georg Bartels <georg.bartels@cs.tum.edu>
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

(in-package :cram-designators-tests)

;;; main function to call to run tests

(defun run-designators-tests ()
  (run-tests))

;;; auxiliary generator and validation functions for testing

(defun generator-1 (designator)
  (declare (ignore designator))
  (list :foo))

(defun generator-2 (designator)
  (declare (ignore designator))  
  (list :bar))

(defun generator-3 (designator)
  (declare (ignore designator))  
  (loop for i from 1 to 51 collecting :foo))

(defun validation-1 (designator solution)
  (declare (ignore designator))
  (when (or (eq solution :bar) (eq solution :foo))
    :accept))

(defun validation-2 (designator solution)
  (declare (ignore designator))
  (when (eq solution :foo)
    :accept))

(defun validation-3 (designator solution)
  (declare (ignore designator))
  (when (eq solution :bar)
    :accept))

;;; auxiliary functions

(defun cleanup-generators-and-validators ()
  (delete-location-generator-function 'generator-1)
  (delete-location-generator-function 'generator-2)
  (delete-location-generator-function 'generator-3)
  (delete-location-validation-function 'validation-1)
  (delete-location-validation-function 'validation-2)
  (delete-location-validation-function 'validation-3))

;;; auxiliary macros

(defmacro with-location-designators ((&rest names) &body body)
  `(let ,(loop for n in names collect `(,n (make-designator :location nil)))
     ,@body))

;;; actual test cases

(define-test register-location-generator
  (with-location-designators (desig0 desig1 desig2)
    ;; test if call for reference in empty setting yields error
    (cleanup-generators-and-validators)
    (assert-error 'designator-error (reference desig0))
    ;; test basic registration functionality with one generator
    (register-location-generator 1 generator-1)
    (assert-eql :foo (reference desig1))
    (assert-false (next-solution desig1))
    ;; see if caching after first reference worked
    (register-location-generator 2 generator-2)
    (assert-eql :foo (reference desig1))
    (assert-false (next-solution desig1))
    ;; test registration functionality with two generators
    (assert-eql :foo (reference desig2))
    (assert-true (next-solution desig2))
    (assert-eql :bar (reference (next-solution desig2)))
    (assert-false (next-solution (next-solution desig2)))))

(define-test register-location-validation-function
  (with-location-designators (desig1 desig2 desig3 desig4)
    (cleanup-generators-and-validators)
    ;; check basic functionality with one validation function
    (register-location-generator 1 generator-1)
    (register-location-generator 2 generator-2)
    (register-location-validation-function 2 validation-2)
    (assert-eql :foo (reference desig1))
    (assert-false (next-solution desig1))
    ;; check basic functionality with two validation functions
    (register-location-validation-function 1 validation-1)
    (assert-eql :foo (reference desig2))
    (assert-false (next-solution desig2))
    ;; check for multiple results
    (delete-location-validation-function 'validation-2)
    (assert-eql :foo (reference desig3))
    (assert-true (next-solution desig3))
    (assert-eql :bar (reference (next-solution desig3)))
    (assert-false (next-solution (next-solution desig3)))
    ;; check if caching after first reference works
    (assert-eql :foo (reference desig1))
    (assert-false (next-solution desig1))
    ;; check if excluding validation functions lead to error
    (delete-location-validation-function 'validation-1)
    (register-location-validation-function 2 validation-2)
    (register-location-validation-function 3 validation-3)
    (assert-error 'designator-error (reference desig4))))

(define-test location-generator-max-retries
  (with-location-designators (desig)
    (cleanup-generators-and-validators)
    ;; check if error comes after too many trials
    (register-location-generator 3 generator-3)
    (register-location-validation-function 3 validation-3)
    (assert-error 'designator-error (reference desig))))

(defclass test-designator (designator) ())
(register-designator-class :test test-designator)

(defmethod reference ((designator test-designator) &optional role)
  (declare (ignore role))
  (or (slot-value designator 'data)
      (setf (slot-value designator 'data)
            (funcall (properties designator)))))

(define-test synchronized-reference
  (let ((designator (make-designator :test (lambda ()
                                             (sleep 0.2)
                                             (random 1.0))))
        (value-1 nil)
        (value-2 nil))
    (let ((thread-1 (sb-thread:make-thread
                     (lambda ()
                       (setf value-1 (reference designator)))))
          (thread-2 (sb-thread:make-thread
                     (lambda ()
                       (sleep 0.1)
                       (setf value-2 (reference designator))))))
      (sb-thread:join-thread thread-1)
      (sb-thread:join-thread thread-2))
    (assert-eq value-1 value-2)))

(define-test default-location-generators
  (cleanup-generators-and-validators)
  (let* ((pose-desig (make-designator :location '((:pose (1 2 3)))))
         (object-desig (make-designator :object `((:at ,pose-desig))))
         (object-loc-desig (make-designator :location `((:of ,object-desig)))))
    (assert-equal '(1 2 3) (reference pose-desig))
    (assert-equal '(1 2 3) (reference object-loc-desig))))
