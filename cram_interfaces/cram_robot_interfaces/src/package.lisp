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

(in-package :cl-user)

(defpackage cram-robot-interfaces
  (:use #:common-lisp #:cram-prolog #:cram-designators)
  (:nicknames #:rob-int)
  (:export
   ;; robot
   #:*robot-description-parameter* #:*robot-urdf*
   #:set-robot-name #:get-robot-name
   #:robot #:robot-odom-frame #:robot-base-frame #:robot-base-link
   #:robot-torso-link-joint
   #:robot-joint-states
   #:robot-pose
   #:arms #:arms-that-are-not-neck
   ;; arms
   #:arm #:required-arms #:available-arms
   #:arm-joints #:arm-links
   #:hand-links #:hand-link #:hand-finger-link #:gripper-joint
   #:end-effector-link #:robot-tool-frame
   #:gripper-meter-to-joint-multiplier
   #:gripper-minimal-position #:gripper-convergence-delta
   #:standard<-particular-gripper-transform
   #:tcp-in-ee-pose
   ;; designator utils
   #:compute-iks
   #:reachability-designator #:designator-reach-pose #:visibility-designator
   #:reachability-designator-p #:visibility-designator-p
   #:trajectory-desig? #:constraints-desig?
   ;; neck
   #:camera-frame #:camera-minimal-height #:camera-maximal-height
   #:camera-horizontal-angle #:camera-vertical-angle
   #:neck #:robot-neck-links #:robot-neck-joints
   #:robot-neck-pan-joint-forward-facing-axis-sign
   #:robot-neck-tilt-joint-forward-facing-axis-sign
   #:robot-neck-base-link
   #:camera-in-neck-ee-pose
   #:neck-camera-z-offset #:neck-camera-pose-unit-vector-multiplier
   #:neck-camera-resampling-step
   #:neck-camera-x-axis-limit #:neck-camera-y-axis-limit #:neck-camera-z-axis-limit
   ;; trajectories
   #:trajectory-point
   ;; urdf
   #:get-joint-type #:get-joint-lower-limit #:get-joint-upper-limit
   #:get-joint-axis #:get-joint-origin #:get-joint-parent #:get-joint-child
   #:joint-lower-limit #:joint-upper-limit #:joint-type #:joint-axis #:joint-origin
   #:joint-parent-link #:joint-child-link
   ;; environment
   #:*environment-description-parameter* #:*environment-urdf*
   #:set-environment-name #:get-environment-name #:environment-name))
