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

(defvar *cloud-handle-transform* nil)
(defvar *cloud-joint-transform* nil)


(defun init ()
  (kr-cloud::initialize-iai-cloud-connection)
  (cpl:sleep 2)
  (kr-cloud:load-episodes '(1) :old-db-or-new :new))


(defun cloud-handle-transform ()
  (or *cloud-handle-transform*
      (multiple-value-bind (handle joint)
          (robust-cloud-handle-and-joint-transform)
        (setf *cloud-joint-transform* joint)
        (setf *cloud-handle-transform* handle))))

(defun cloud-joint-transform ()
  (or *cloud-joint-transform*
      (multiple-value-bind (handle joint)
          (robust-cloud-handle-and-joint-transform)
        (setf *cloud-handle-transform* handle)
        (setf *cloud-joint-transform* joint))))

(defun robust-cloud-handle-and-joint-transform ()
  (calculate-robust-handle-and-joint-transform
   (kr-cloud:semantic-map-object-transform "IAIFridgeDoorHandle")
   (kr-cloud:semantic-map-object-transform "HingedJoint")))


(defun cloud-handle-to-robot-transform (&optional (before-action "MoveFridgeHandle"))
  (let ((cloud-map-to-handle (cloud-handle-transform))
        (cloud-map-to-robot (kr-cloud:robot-pose-before-action before-action)))
    (apply-transform (cram-tf:transform-stamped-inv cloud-map-to-handle)
                     cloud-map-to-robot)))

(defun cloud-joint-to-robot-transform (&optional (before-action "MoveFridgeHandle"))
  (let ((cloud-map-to-joint (cloud-joint-transform))
        (cloud-map-to-robot (kr-cloud:robot-pose-before-action before-action)))
    (apply-transform (cram-tf:transform-stamped-inv cloud-map-to-joint)
                     cloud-map-to-robot)))

(defun cloud-handle-to-gripper-transforms (&optional (action "MoveFridgeHandle"))
  (let ((cloud-map-to-handle (cloud-handle-transform))
        (cloud-map-to-gripper-list
          (kr-cloud::gripper-trajectory-during-action :right action)))
    (mapcar (lambda (cloud-map-to-gripper)
              (apply-transform (cram-tf:transform-stamped-inv cloud-map-to-handle)
                               cloud-map-to-gripper))
            cloud-map-to-gripper-list)))
