;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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

(in-package :boxy-desig)

(defun calculate-pose-from-direction (distance)
  (let* ((left-pose
           (cl-transforms-stamped:make-pose-stamped
            cram-tf:*robot-left-tool-frame*
            0.0
            (cl-transforms:make-3d-vector 0.0 0.0 distance)
            (cl-transforms:make-identity-rotation))))
    left-pose))

(def-fact-group boxy-motion-designators (desig:motion-grounding)

  (<- (desig:motion-grounding ?designator (move-tcp-wiggle ?arm ?pose))
    (property ?designator (:type :wiggling-tcp))
    (property ?designator (:arm ?arm))
    (property ?designator (:target ?location-designator))
    (desig:designator-groundings ?location-designator ?poses)
    (member ?pose ?poses))

  (<- (desig:motion-grounding ?designator (move-tcp-wiggle :left ?pose))
    (property ?designator (:type :wiggling-tcp))
    ;; (property ?designator (:arm ?arm))
    ;; (property ?designator (:direction ?direction-keyword))
    ;; (property ?designator (:frame ?reference-frame))
    (property ?designator (:arm :left))
    (property ?designator (:direction :forward))
    (property ?designator (:distance ?distance))
    (lisp-fun calculate-pose-from-direction ?distance ;; ?arm ?direction-keyword ?reference-frame
              ?pose)))
