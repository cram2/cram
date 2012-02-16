;;; Copyright (c) 2011, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :btr-desig)

(defun float-equal (lhs rhs &optional (threshold 1e-6))
  (< (abs (- lhs rhs))
     threshold))

(defun float-equal-and-print (actual expected &optional (threshold 1e-6))
  (cond ((float-equal actual expected threshold)
         (format t "Floats equal: Expected: ~a, got: ~a~%" expected actual)
         t)
        (t (format t "Floats not equal: Expected: ~a, got: ~a~%" expected actual))))

(sb-rt:deftest aligned-orientation-test
    (let* ((object-pose (cl-transforms:make-identity-pose))
           (reference-object-pose (cl-transforms:make-identity-pose))
           (orientation-generator (make-aligned-orientation-generator
                                   reference-object-pose object-pose))
           (test-inputs '((1 0.9 0)  (1 0 0)  (1 -0.9 0)
                          (0 0.9 0)           (0 -0.9 0)
                          (-1 0.9 0) (-1 0 0) (-1 -0.9 0)))
           (expected-results (list pi pi pi
                                   (/ pi -2) (/ pi 2)
                                   0 0 0)))
      (every #'identity
             (mapcar #'float-equal-and-print
                     (mapcar (lambda (input) (cl-transforms:normalize-angle
                                              (cl-transforms:get-yaw (apply orientation-generator input))))
                             test-inputs)
                     expected-results)))
  t)