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

(defmacro with-projected-robot (&body body)
  `(proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
      (cpl:top-level
        ,@body)))

(defun visualize-cloud-handle-and-joint ()
  (btr-utils:spawn-object 'cloud-handle-original :mug
                          :pose (strip-transform-stamped
                                 (kr-cloud:semantic-map-object-transform "IAIFridgeDoorHandle"))
                          :color '(1 0 0))
  (btr-utils:spawn-object 'cloud-handle :mug
                          :pose (strip-transform-stamped (cloud-handle-transform)))
  (btr-utils:spawn-object 'cloud-joint-original :mondamin
                          :pose (strip-transform-stamped
                                 (kr-cloud:semantic-map-object-transform "HingedJoint"))
                          :color '(1 0 0))
  (btr-utils:spawn-object 'cloud-joint :mondamin
                          :pose (strip-transform-stamped (cloud-joint-transform))))

(defun visualize-trajectory (poses)
  (btr-utils:kill-all-objects)
  (let ((i 0))
    (dolist (pose poses)
      ;; (btr-utils:spawn-object
      ;;  (intern (format nil "fork-~a" i) :keyword)
      ;;  :fork
      ;;  :pose pose)
      (btr-utils:spawn-object
       (intern (format nil "apple-~a" i) :keyword)
       :cereal
       :pose pose)
      (incf i))))


(defun local-gripper-trajectory-in-map-from-radius ()
  (let* ((local-map-to-handle (local-handle-transform))
         (local-handle-to-gripper-list
           (calculate-handle-to-gripper-transforms local-map-to-handle
                                                   (local-joint-transform))))
    (mapcar (lambda (local-handle-to-gripper)
              (apply-transform local-map-to-handle local-handle-to-gripper))
            local-handle-to-gripper-list)))

(defun local-gripper-trajectory-in-base-from-radius ()
  (let* ((local-map-to-gripper-list (local-gripper-trajectory-in-map-from-radius))
         (local-map-to-robot (cram-tf:pose->transform-stamped
                              cram-tf:*fixed-frame*
                              cram-tf:*robot-base-frame*
                              0.0
                              (btr:object-pose 'cram-pr2-description:pr2))))
    (mapcar (lambda (local-map-to-gripper)
              (apply-transform (cram-tf:transform-stamped-inv local-map-to-robot)
                               local-map-to-gripper))
            local-map-to-gripper-list)))


(defun move-in-projection-to-fridge ()
  (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
    (cpl:top-level
      (exe:perform
       (let ((?pose (cl-transforms-stamped:make-pose-stamped
                     "map" 0.0
                     (cl-transforms:make-3d-vector 0.5 -0.8 0)
                     (cl-transforms:make-identity-rotation))))
         (desig:a motion (type going) (target (desig:a location (pose ?pose))))))
      (exe:perform
       (desig:a motion (type moving-torso) (joint-angle 0.3))))))

(defun execute-trajectory-in-projection ()
  (move-in-projection-to-fridge)
  (let ((trajectory-in-base (local-gripper-trajectory-in-base-from-radius)))
    (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
      (cpl:top-level
        (mapc (lambda (via-point-transform)
                (let ((?pose (strip-transform-stamped via-point-transform)))
                 (exe:perform
                  (desig:a motion
                           (type moving-tcp)
                           (right-target (desig:a location (pose ?pose)))))))
              trajectory-in-base)))))
