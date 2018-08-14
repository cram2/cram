;;;
;;; Copyright (c) 2014, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :bullet-reasoning-utilities)

(defun visualize-designator-costmaps (designator)
  (declare (type desig:location-designator designator))
  "Visualizes costmaps associated with a location designator."
  (format t "[BTR-UTILS CM] Visualizing costmaps of ~a~%" designator)
  (mapcar
   (lambda (costmap-pair)
     (let* ((costmap (cdar costmap-pair))
            (functions (car (location-costmap:cost-functions costmap))))
       (if functions
           (progn
             (format t "[BTR-UTILS CM] Costmap named ~a~%"
                     (location-costmap:generator-name functions))
             (location-costmap:get-cost-map costmap)
             (sleep 3))
           (format t "[BTR-UTILS CM] No cost functions registered~%"))))
   (force-ll (prolog `(and (location-costmap:desig-costmap ,designator ?cm)))))
  (format t "[BTR-UTILS CM] Combined costmap~%")
  (desig:reference designator))

(defun visualize-gripper (pose arm)
  (let ((tool-frame (ecase arm
                      (:left cram-tf:*robot-left-tool-frame*)
                      (:right cram-tf:*robot-right-tool-frame*))))
    (btr-utils:spawn-object
     (ecase arm
       (:left :left-gripper-grasp)
       (:right :right-gripper-grasp))
     :gripper
     :mass 0.0
     :color (ecase arm
              (:left '(0 1 0 0.5))
              (:right '(0 0 1 0.5)))
     :pose (cl-transforms:transform->pose
            (cram-tf:multiply-transform-stampeds
             cram-tf:*fixed-frame*
             tool-frame
             (cram-tf:pose-stamped->transform-stamped
              (cram-tf:robot-current-pose)
              cram-tf:*robot-base-frame*)
             (cram-tf:multiply-transform-stampeds
              cram-tf:*robot-base-frame*
              tool-frame
              (cram-tf:pose-stamped->transform-stamped
               pose
               tool-frame)
              (cram-tf:transform-stamped-inv
               (cl-transforms-stamped:transform->transform-stamped
                tool-frame
                tool-frame
                0.0
                (cut:var-value
                 '?transform
                 (car (prolog:prolog
                       `(and (cram-robot-interfaces:robot ?robot)
                             (cram-robot-interfaces:standard-to-particular-gripper-transform
                              ?robot ?transform)))))))))))))
