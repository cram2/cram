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

(in-package :cl-user)

(defpackage cram-manipulation-interfaces
  (:use #:common-lisp #:cram-prolog)
  (:nicknames #:man-int)
  (:export
   ;; object-designator-interfaces
   #:get-object-transform
   #:get-object-pose
   #:get-object-transform-in-map
   #:get-object-pose-in-map
   ;; prolog
   #:object-type-subtype
   #:object-type-direct-subtype
   #:robot-free-hand
   ;; manipulation-interfaces
   #:reasoning-engine-for-method
   #:get-action-gripping-effort
   #:get-action-gripper-opening
   #:get-action-trajectory
   #:get-action-grasps
   ;; grasps
   #:calculate-object-faces
   #:calculate-face-vector
   #:object-type-grasp->robot-grasp
   #:robot-grasp->object-type-grasp
   #:object-rotationally-symmetric
   #:orientation-matters
   ;; trajectories
   #:make-traj-segment
   #:traj-segment-label
   #:traj-segment-poses
   #:make-empty-trajectory
   #:get-traj-poses-by-label
   #:calculate-gripper-pose-in-base
   ;;
   #:get-object-type-to-gripper-transform
   #:get-object-type-to-gripper-pregrasp-transform
   #:get-object-type-to-gripper-2nd-pregrasp-transform
   #:get-object-type-to-gripper-lift-transform
   #:get-object-type-to-gripper-2nd-lift-transform
   #:def-object-type-to-gripper-transforms
   #:get-object-grasping-poses
   ;;
   #:get-object-type-in-other-object-transform
   #:get-object-placement-transform
   #:def-object-type-in-other-object-transform
   #:get-object-look-from-pose
   ;; standard-grasps
   #:*x-across-z-grasp-rotation*
   #:*-x-across-z-grasp-rotation*
   #:*x-across-y-grasp-rotation*
   #:*-x-across-y-grasp-rotation*
   #:*y-across-z-grasp-rotation*
   #:*-y-across-z-grasp-rotation*
   #:*y-across-x-grasp-rotation*
   #:*-y-across-x-grasp-rotation*
   #:*z-across-x-grasp-rotation*
   #:*z-across-y-grasp-rotation*
   #:*z-diagonal-grasp-rotation*
   #:*-z-across-x-grasp-rotation*
   ;; standard-rotations
   #:*rotation-around-z-90-matrix*
   #:*rotation-around-z+90-matrix*
   #:*identity-matrix*
   #:*rotation-around-x+90-list*
   #:*rotation-around-x-90-list*
   #:*rotation-around-y+90-list*
   #:*rotation-around-y-90-list*
   #:*rotation-around-z+90-list*
   #:*rotation-around-z-90-list*))
