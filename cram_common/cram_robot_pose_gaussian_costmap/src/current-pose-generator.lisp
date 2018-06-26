;;;
;;; Copyright (c) 2015, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
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

(in-package :gaussian-costmap)

(defun robot-current-pose-tf-generator (desig)
  (when (or (cram-robot-interfaces:reachability-designator-p desig)
            (cram-robot-interfaces:visibility-designator-p desig))
    (when cram-tf:*transformer*
      (handler-case
          (list (cram-tf:robot-current-pose))
        (cl-transforms-stamped:transform-stamped-error () nil)))))

(desig:register-location-generator
 5 robot-current-pose-tf-generator
 "We should move the robot only if we really need to move. Try the
 current robot pose as a first solution.")

;; (defun robot-current-pose-bullet-generator (desig)
;;   (when (or (cram-robot-interfaces:reachability-designator-p desig)
;;             (cram-robot-interfaces:visibility-designator-p desig))
;;     (handler-case
;;         (cut:var-value '?pose
;;                        (car (prolog `(and (btr:bullet-world ?w)
;;                                           (cram-robot-interfaces:robot ?robot-name)
;;                                           (btr:object-pose ?w ?robot-name ?pose)))))
;;       (error () nil))))
