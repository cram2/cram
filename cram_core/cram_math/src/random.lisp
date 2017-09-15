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

(in-package :cma)

(define-condition invalid-probability-distribution (error) ())

(defun sample (var-fun p-fun &optional (max-retries 10000000))
  "Draw a variable from a probability distripution.

   `var-fun' is a function that maps a value within the interval [0;1)
   to a random variable.
   
   `p-fun' is the probability distripution that maps a variable to a
   probability within the interval [0;1]."
  (when (<= max-retries 0)
    (error 'simple-error
           :format-control "Could not draw a sample. Maximal retries exeeded."))
  (let* ((t-val (random 1.0d0))
         (u-val (random 1.0d0))
         (var (funcall var-fun t-val)))
    (if (<= u-val (funcall p-fun var))
        var
        (sample var-fun p-fun (- max-retries 1)))))

(defun sample-discrete (var-probs)
  "Draws a sample from a discrete probability distribution.

  `var-probs' is a sequence with values of the form (VAR
  . PROBABILITY) with VAR being of arbitrary type and PROBABILITY
  being a probability within the interval [0;1]."
  (let* ((n-vars (length var-probs))
         (vars (make-array n-vars))
         (probs (make-hash-table :test 'equal)))
    (loop for (var . prob) in var-probs
          for i from 0 do
       (setf (aref vars i) var)
       (setf (gethash var probs) prob))
    (flet ((var-fun (x)
             (aref vars (truncate (* x n-vars))))
           (prob-fun (var)
             (gethash var probs)))
      (sample #'var-fun #'prob-fun))))

(defun sample-from-distribution-matrix (probability-distribution-matrix)
  "Draws a random sample from a probability distribution represented
  by a DOUBLE-MATRIX and returns a list with two values: 

  (COLUMN ROW)"
  (declare (type double-matrix probability-distribution-matrix))
  (let ((rand (random 1.0))
        (counter 0.0))
    (dotimes (row (height probability-distribution-matrix))
      (dotimes (col (width probability-distribution-matrix))
        (setf counter (+ counter (aref probability-distribution-matrix
                                       row col)))
        (when (> counter rand)
          (return-from sample-from-distribution-matrix
            (list col row))))))
  (error 'invalid-probability-distribution))
