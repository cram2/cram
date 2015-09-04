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
  (:export
   ;; trajectories
   #:trajectory-point
   ;; robot
   #:robot
   ;; ptu
   #:camera-frame #:camera-minimal-height #:camera-maximal-height
   #:robot-pan-tilt-links #:robot-pan-tilt-joints
   ;; arms
   #:arm #:required-arms #:available-arms
   #:end-effector-link #:gripper-link #:planning-group
   #:robot-arms-parking-joint-states #:end-effector-parking-pose
   #:robot-pre-grasp-joint-states
   ;; grasps
   #:def-grasp #:def-tool #:get-grasp #:get-grasps #:get-grasp-names
   #:calculate-bounding-box-tool-length #:get-tool-direction-vector
   #:get-tool-length #:get-tool-vector #:calculate-tool
   #:grasp #:side #:object-type-grasp #:object-designator-grasp
   #:object-type-tool-length #:object-designator-tool-length
   ;; objects
   #:orientation-matters
   ;; reachbility prolog utils
   #:compute-ik #:side->ik-group-name
   #:reachability-designator #:designator-reach-pose
   #:reachability-designator-p #:visibility-designator-p))
