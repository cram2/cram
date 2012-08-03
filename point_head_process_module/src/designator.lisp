;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;

(in-package :point-head-process-module)

(defun pose-stamped->point-stamped-msg (ps)
  (roslisp:make-message
   "geometry_msgs/PointStamped"
   (stamp header) (tf:stamp ps)
   (frame_id header) (tf:frame-id ps)
   (x point) (cl-transforms:x
              (cl-transforms:origin ps))
   (y point) (cl-transforms:y
              (cl-transforms:origin ps))                
   (z point) (cl-transforms:z
              (cl-transforms:origin ps))))

(defun ensure-pose-stamped (pose)
  (etypecase pose
    (tf:pose-stamped pose)
    (tf:stamped-transform
       (tf:make-pose-stamped
        (tf:frame-id pose) (tf:stamp pose)
        (cl-transforms:translation pose)
        (cl-transforms:rotation pose)))
    (location-designator (reference pose))))

(defun make-action-goal (pose-stamped)
  (let ((pose-stamped (ensure-pose-stamped pose-stamped)))
    (unless (tf:wait-for-transform
             *tf* :source-frame (tf:frame-id pose-stamped)
                  :target-frame "base_link"
                  :time (tf:stamp pose-stamped)
                  :timeout 1.0)
      (error 'simple-error
             :format-control "Could not transform ~a into base_link. Invalid time stamp."
             :format-arguments (list pose-stamped)))
    (let* ((transformed-pose-stamped (tf:transform-pose *tf* :pose pose-stamped :target-frame "/base_link"))
           (point-stamped-msg (pose-stamped->point-stamped-msg transformed-pose-stamped)))
      (roslisp:make-message
       "pr2_controllers_msgs/PointHeadGoal"
       max_velocity 10
       min_duration 0.3
       pointing_frame "/high_def_frame"
       (x pointing_axis) 1.0
       (y pointing_axis) 0.0
       (z pointing_axis) 0.0
       target point-stamped-msg))))

(def-fact-group point-head (action-desig)

  (<- (action-desig ?desig (point ?act))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to see))
    (desig-prop ?desig (pose ?pose))
    (lisp-fun make-action-goal ?pose ?act))

  (<- (action-desig ?desig (point ?act))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to see))
    (desig-prop ?desig (location ?loc))
    (loc-desig-location ?loc ?pose)
    (lisp-fun make-action-goal ?pose ?act))

  (<- (action-desig ?desig (point ?act))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to see))
    (desig-prop ?desig (obj ?obj))
    (obj-desig-location ?obj ?pose)
    (lisp-fun make-action-goal ?pose ?act))  
    
  (<- (action-desig ?desig (follow ?act))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to follow))
    (desig-prop ?desig (pose ?pose))
    (lisp-fun make-action-goal ?pose ?act))

  (<- (action-desig ?desig (follow ?act))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to follow))
    (desig-prop ?desig (location ?loc))
    (loc-desig-location ?loc ?pose)
    (lisp-fun make-action-goal ?pose ?act))

  (<- (action-desig ?desig (follow ?act))
    (trajectory-desig? ?desig)
    (desig-prop ?desig (to follow))
    (desig-prop ?desig (obj ?obj))
    (obj-desig-location ?obj ?pose)
    (lisp-fun make-action-goal ?pose ?act)))
