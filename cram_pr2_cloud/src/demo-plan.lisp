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

(in-package :pr2-cloud)

(defun init ()
  (kr-cloud:initialize-cloud-connection)
  (cpl:sleep 2)
  (kr-cloud:load-episodes '(16)))

(defun strip-transform-stamped (transform-stamped)
  (cl-transforms-stamped:make-pose-stamped
   (cl-transforms-stamped:frame-id transform-stamped)
   (cl-transforms-stamped:stamp transform-stamped)
   (cl-transforms-stamped:translation transform-stamped)
   (cl-transforms:rotation transform-stamped)))

(defun copy-transform-stamped (transform-stamped &key frame-id child-frame-id stamp
                                                   translation rotation)
  (cl-transforms-stamped:make-transform-stamped
   (or frame-id (cl-transforms-stamped:frame-id transform-stamped))
   (or child-frame-id (cl-transforms-stamped:child-frame-id transform-stamped))
   (or stamp (cl-transforms-stamped:stamp transform-stamped))
   (or translation (cl-transforms-stamped:translation transform-stamped))
   (or rotation (cl-transforms-stamped:rotation transform-stamped))))

(defun pose-stamped->transform-stamped (pose-stamped child-frame-id)
  (cl-transforms-stamped:make-transform-stamped
   (cl-transforms-stamped:frame-id pose-stamped)
   child-frame-id
   (cl-transforms-stamped:stamp pose-stamped)
   (cl-transforms-stamped:translation pose-stamped)
   (cl-transforms-stamped:rotation pose-stamped)))

(defun calculate-robust-handle-and-joint-transform (handle-transform joint-transform)
  ;; (return-from calculate-robust-handle-and-joint-transform
  ;;   (values handle-transform joint-transform))
  (let* ((handle->joint-vector
           (cl-transforms:v-
            (cl-transforms-stamped:translation joint-transform)
            (cl-transforms-stamped:translation handle-transform)))
         (handle->joint-vector-projected-onto-xy
           (cl-transforms:copy-3d-vector handle->joint-vector :z 0))
         (y-axis handle->joint-vector-projected-onto-xy)
         (z-axis (cl-transforms:make-3d-vector 0 0 1))
         (x-axis (cl-transforms:cross-product y-axis z-axis))
         (orientation (cl-transforms:column-vectors->quaternion x-axis y-axis z-axis)))
    (values
     (copy-transform-stamped
      handle-transform
      :rotation orientation)
     (copy-transform-stamped
      joint-transform
      :rotation orientation))))

(defun visualize-trajectory (poses)
  (btr-utils:kill-all-objects)
  (let ((i 0))
    (dolist (pose poses)
      (btr-utils:spawn-object
       (intern (format nil "fork-~a" i) :keyword)
       :fork
       :pose pose)
      (btr-utils:spawn-object
       (intern (format nil "apple-~a" i) :keyword)
       :apple
       :pose pose)
      (incf i))))

(defun move-in-projection-to-fridge ()
  (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
    (cpl:top-level
      (exe:perform
       (let ((?pose (cl-transforms-stamped:make-pose-stamped
                     "map" 0.0
                     (cl-transforms:make-3d-vector 0.5 -0.8 0)
                     (cl-transforms:make-identity-rotation))))
         (desig:a motion (type going) (target (desig:a location (pose ?pose)))))))))


(defun cloud-handle-to-robot-transform ()
  (let ((map-to-handle (kr-cloud:semantic-map-object-transform "IAIFridgeDoorHandle"))
        (map-to-joint (kr-cloud:semantic-map-object-transform "HingedJoint"))
        (map-to-robot (kr-cloud:robot-pose-before-action "PreGraspPose")))
    (multiple-value-bind (map-to-handle map-to-joint)
        (calculate-robust-handle-and-joint-transform
         map-to-handle
         map-to-joint)
      (cram-tf:multiply-transform-stampeds
       (cl-transforms-stamped:child-frame-id map-to-handle)
       cram-tf:*robot-base-frame*
       (cram-tf:transform-stamped-inv map-to-handle)
       map-to-robot))))

(defun cloud-joint-to-robot-transform (&optional (before-action "PreGraspPose"))
  (let ((map-to-handle (kr-cloud:semantic-map-object-transform "IAIFridgeDoorHandle"))
        (map-to-joint (kr-cloud:semantic-map-object-transform "HingedJoint"))
        (map-to-robot (kr-cloud:robot-pose-before-action before-action)))
    (multiple-value-bind (map-to-handle map-to-joint)
        (calculate-robust-handle-and-joint-transform
         map-to-handle
         map-to-joint)
       (cram-tf:multiply-transform-stampeds
        (cl-transforms-stamped:child-frame-id map-to-joint)
        cram-tf:*robot-base-frame*
        (cram-tf:transform-stamped-inv map-to-joint)
        map-to-robot))))

(defun cloud-handle-to-gripper-transform ()
  (let ((map-to-handle (kr-cloud:semantic-map-object-transform "IAIFridgeDoorHandle"))
        (map-to-joint (kr-cloud:semantic-map-object-transform "HingedJoint"))
        (map-to-gripper-list
          (kr-cloud::gripper-trajectory-during-action :right "MoveFridgeHandle")))
    ;; (btr-utils:spawn-object 'cloud-handle-original :mug
    ;;                         :pose (strip-transform-stamped map-to-handle))
    (multiple-value-bind (map-to-handle map-to-joint)
        (calculate-robust-handle-and-joint-transform
         map-to-handle
         map-to-joint)
      ;; (btr-utils:spawn-object 'cloud-handle :mug :pose (strip-transform-stamped map-to-handle))
      (mapcar (lambda (map-to-gripper)
                (cram-tf:multiply-transform-stampeds
                 (cl-transforms-stamped:child-frame-id map-to-handle)
                 cram-tf:*robot-right-tool-frame*
                 (cram-tf:transform-stamped-inv map-to-handle)
                 map-to-gripper))
              map-to-gripper-list))))




(defun local-robot-pose-in-map (&optional (before-action "ReachAndOpenFridgeDoor"))
  (let ((local-map-to-handle (kr-cloud:local-semantic-map-object-transform "IAIFridgeDoorHandle"))
        (local-map-to-joint (kr-cloud:local-semantic-map-object-transform "HingedJoint"))
        (local-map-to-robot (cram-tf:pose->transform-stamped
                             cram-tf:*fixed-frame*
                             cram-tf:*robot-base-frame*
                             0.0
                             (btr:object-pose 'cram-pr2-description:pr2)))
        (joint-to-robot (cloud-joint-to-robot-transform before-action)))
    (multiple-value-bind (local-map-to-handle local-map-to-joint)
        (calculate-robust-handle-and-joint-transform
         local-map-to-handle
         local-map-to-joint)
      (cram-tf:multiply-transform-stampeds
       cram-tf:*fixed-frame*
       cram-tf:*robot-base-frame*
       local-map-to-joint
       joint-to-robot
       :result-as-pose-or-transform :pose))))

(defun local-gripper-trajectory-in-map ()
  (let ((local-map-to-handle (kr-cloud:local-semantic-map-object-transform "IAIFridgeDoorHandle"))
        (local-map-to-joint (kr-cloud:local-semantic-map-object-transform "HingedJoint"))
        (map-to-handle (kr-cloud:semantic-map-object-transform "IAIFridgeDoorHandle"))
        (handle-to-gripper-list (cloud-handle-to-gripper-transform)))
    ;; (visualize-trajectory (mapcar (lambda (transform)
    ;;                                 (cram-tf:multiply-transform-stampeds
    ;;                                  cram-tf:*fixed-frame*
    ;;                                  cram-tf:*robot-right-tool-frame*
    ;;                                  map-to-handle
    ;;                                  transform
    ;;                                  :result-as-pose-or-transform :pose))
    ;;                               handle-to-gripper-list))
    (multiple-value-bind (local-map-to-handle local-map-to-joint)
        (calculate-robust-handle-and-joint-transform
         local-map-to-handle
         local-map-to-joint)
      (mapcar (lambda (handle-to-gripper)
                (cram-tf:multiply-transform-stampeds
                  cram-tf:*fixed-frame*
                  cram-tf:*robot-right-tool-frame*
                  local-map-to-handle
                  handle-to-gripper
                  :result-as-pose-or-transform :pose))
              handle-to-gripper-list))))

(defun calculate-handle-to-gripper-transforms (map-to-handle map-to-joint
                                               &optional (theta-max
                                                          (cma:degrees->radians 60)))
  (let ((joint-to-handle
          (cram-tf:multiply-transform-stampeds
           (cl-transforms-stamped:child-frame-id map-to-joint)
           (cl-transforms-stamped:child-frame-id map-to-handle)
           (cram-tf:transform-stamped-inv map-to-joint)
           map-to-handle)))
    (mapcar (lambda (joint-to-circle-point)
              (cram-tf:multiply-transform-stampeds
               (cl-transforms-stamped:frame-id map-to-joint)
               (cl-transforms-stamped:child-frame-id joint-to-circle-point)
               map-to-joint
               joint-to-circle-point))
            (loop for theta = 0.0 then (+ 0.1 theta)
                  while (< theta theta-max)
                  collect
                  (let ((rotation
                          (cl-tf:axis-angle->quaternion
                           (cl-transforms:make-3d-vector 0 0 1) theta)))
                    (cl-transforms-stamped:make-transform-stamped
                     (cl-transforms-stamped:frame-id joint-to-handle)
                     cram-tf:*robot-right-tool-frame*;; (format nil "cirlce_point~a" (* 2 theta))
                     (cl-transforms-stamped:stamp joint-to-handle)
                     (cl-transforms:rotate rotation (cl-transforms:translation joint-to-handle))
                     (cl-transforms:rotation joint-to-handle))) ))))

(defun local-gripper-trajectory-in-robot-base-from-radius ()
  (let ((local-map-to-handle (kr-cloud:local-semantic-map-object-transform "IAIFridgeDoorHandle"))
        (local-map-to-joint (kr-cloud:local-semantic-map-object-transform "HingedJoint"))
        (local-map-to-robot (cram-tf:pose->transform-stamped
                             cram-tf:*fixed-frame*
                             cram-tf:*robot-base-frame*
                             0.0
                             (btr:object-pose 'cram-pr2-description:pr2))))
    (multiple-value-bind (local-map-to-handle local-map-to-joint)
        (calculate-robust-handle-and-joint-transform
         local-map-to-handle
         local-map-to-joint)
      (let ((trajectory-in-map
              (calculate-handle-to-gripper-transforms local-map-to-handle local-map-to-joint)))
        (mapcar (lambda (map-to-gripper)
                  (cram-tf:multiply-transform-stampeds
                   cram-tf:*robot-base-frame*
                   cram-tf:*robot-right-tool-frame*
                   (cram-tf:transform-stamped-inv local-map-to-robot)
                   map-to-gripper
                   :result-as-pose-or-transform :pose))
                trajectory-in-map)))))




(defun test-circle ()
  (move-in-projection-to-fridge)
  (cram-process-modules:with-process-modules-running
      (pr2-pms::pr2-perception-pm pr2-pms::pr2-base-pm pr2-pms::pr2-arms-pm
                                  pr2-pms::pr2-grippers-pm pr2-pms::pr2-ptu-pm)
    (cpl:top-level
      (mapcar (lambda (?pose)
                (exe:perform
                 (desig:a motion
                          (type moving-tcp)
                          (right-target (desig:a location (pose ?pose))))))
              (local-gripper-trajectory-in-robot-base-from-radius)))))


;;; arms down
;; (exe:perform (desig:an action
;;                             (to move-arm-motion)
;;                             (left ((0.09611d0 0.68d0 0.35466d0)
;;                                    (-0.45742778331019085d0
;;                                     0.3060123951483878d0
;;                                     0.3788581151804847d0
;;                                     0.744031427853262d0)))
;;                             (right ((0.0848d0 -0.712d0 0.35541d0)
;;                                     (-0.061062529688043946d0
;;                                      -0.6133522138254498d0
;;                                      0.197733462359113d0
;;                                      -0.7622151317882601d0)))))
