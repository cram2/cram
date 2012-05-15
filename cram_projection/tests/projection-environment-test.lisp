;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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


(in-package :cram-projection-tests)

(defvar *test-variable* :a)

;; We need to redefine projection environments as task variables here
;; to make WITH-TEST-ENVIRONMENT work correctly with TOP-LEVEL.
(cpl-impl:define-task-variable cram-projection::*projection-environments*)
(cpl-impl:define-task-variable cram-projection::*special-projection-variables*)

(defmacro with-test-environment (&body body)
  `(let ((cram-projection::*projection-environments* nil)
         (cram-projection::*special-projection-variables* nil))
     ,@body))

(define-test special-variable-environment-rebinding
  (with-test-environment
    (define-projection-environment test-projection-environment
        :special-variable-initializers ((*test-variable* :b)))

    (assert-eq :a *test-variable*)
    (let ((value nil))
      (cpl-impl:top-level
        (with-projection-environment test-projection-environment
          (setf value *test-variable*)))
      (assert-eq :b value))
    (assert-eq :a *test-variable*)))

(define-test special-projection-variable-rebinding
  (with-test-environment
    (define-projection-environment test-projection-environment)
    (define-special-projection-variable *test-variable* :b)

    (assert-eq :a *test-variable*)
    (let ((value nil))
      (cpl-impl:top-level
        (with-projection-environment test-projection-environment
          (setf value *test-variable*)))
      (assert-eq :b value))
    (assert-eq :a *test-variable*)))
