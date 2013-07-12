;;; Copyright (c) 2013, Georg Bartels <georg.bartels@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;; * Redistributions of source code must retain the above copyright
;;; notice, this list of conditions and the following disclaimer.
;;; * Redistributions in binary form must reproduce the above copyright
;;; notice, this list of conditions and the following disclaimer in the
;;; documentation and/or other materials provided with the distribution.
;;; * Neither the name of the Institute for Artificial Intelligence/
;;; Universitaet Bremen nor the names of its contributors may be used to 
;;; endorse or promote products derived from this software without specific 
;;; prior written permission.
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

(in-package :pr2-fccl-process-module)

(defun fccl-controller-finished-p (constraints-state current-movement-id)
  "Checks whether all weight entries in 'constraints-state' are smaller than 1.0 AND whether the movement-id in 'msg' corresponds to 'current-movement-id'. If yes T is return, else nil."
  (when (and constraints-state current-movement-id)
    (assert (typep constraints-state 'cram-feature-constraints:feature-constraint-state))
    (assert (numberp current-movement-id))
    ;; extract the current information from the state object
    (let ((weights (cram-feature-constraints:current-weights constraints-state))
          (movement-id (cram-feature-constraints:movement-id constraints-state)))
      ;; check if the movement-ids match
      (when (eql movement-id current-movement-id)
        ;; calculate maximum weight over all constraints
        (let ((max-weight (loop for i from 0 below (length weights)
                                for weight = (elt weights i)
                                maximizing weight into max-weight
                                finally (return max-weight))))
          ;; return T if max-weight is greater then 1.0, else 'when' returns nil.
          (when (< max-weight 1.0) t))))))