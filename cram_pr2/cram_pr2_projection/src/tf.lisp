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

(in-package :pr2-proj)

(defun set-tf-from-bullet (&key (transformer cram-tf:*transformer*)
                             (fixed-frame cram-tf:*fixed-frame*)
                             (odom-frame cram-tf:*odom-frame*)
                             (base-frame cram-tf:*robot-base-frame*)
                             (time (cut:current-timestamp)))
  "Sets the transform from fixed frame to odom and then robot and all robot link transforms"
  ;; get the robot bullet object instance
  (cut:with-vars-bound (?robot-instance)
      (cut:lazy-car
       (prolog:prolog `(and (cram-robot-interfaces:robot ?robot)
                            (btr:bullet-world ?world)
                            (btr:%object ?world ?robot ?robot-instance))))
    (assert (not (cut:is-var ?robot-instance)))

    (let* ((robot-pose-in-map (btr:link-pose ?robot-instance base-frame))
         (reference-transform-inv (cl-transforms:transform-inv
                                   (cl-transforms:reference-transform robot-pose-in-map))))

    ;; tell the tf transformer that the global fixed frame and the odom frame are the same
    (cl-tf:set-transform
     transformer
     (cl-tf:make-transform-stamped
      fixed-frame odom-frame time
      (cl-transforms:make-identity-vector)
      (cl-transforms:make-identity-rotation))
     :suppress-callbacks t)

    ;; tell the tf transformer where the robot is in the odometry frame
    (cl-tf:set-transform
     transformer
     (cl-tf:transform->transform-stamped
      odom-frame base-frame time
      (cl-transforms:pose->transform robot-pose-in-map))
     :suppress-callbacks t)

    ;; tell the tf transformer the current configuration of robot's joints
    (dolist (link (btr:link-names ?robot-instance))
      (unless (equal link base-frame)
        (let ((transform (cl-transforms:transform*
                          reference-transform-inv
                          (cl-transforms:reference-transform
                           (btr:link-pose ?robot-instance link)))))
          (cl-tf:set-transform
           transformer
           (cl-tf:make-transform-stamped
            base-frame link time
            (cl-transforms:translation transform)
            (cl-transforms:rotation transform))
           :suppress-callbacks t))))

    ;; execute the TF update callbacks
    (cl-tf:execute-changed-callbacks transformer))))

(defmethod cram-occasions-events:on-event
    update-tf ((event cram-plan-occasions-events:robot-state-changed))
  (when (eql cram-projection:*projection-environment*
           'cram-pr2-projection::pr2-bullet-projection-environment)
    (set-tf-from-bullet)))

(defmethod cram-robot-interfaces:compute-iks
    :before (pose-stamped &key link-name arm robot-state seed-state
                            pose-stamped-frame tcp-in-ee-pose)
  (declare (ignore pose-stamped link-name arm robot-state seed-state
                   pose-stamped-frame tcp-in-ee-pose))
  (set-tf-from-bullet))

