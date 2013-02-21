;;; Copyright (c) 2012, Georg Bartels <georg.bartels@cs.uni-bremen.de>
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

(in-package :pr2-manip-pm)
;; Conversion functions to create the parts of the InitScene service-request
(defun make-itasc-robot-msg (itasc-robot)
  (when itasc-robot
    (roslisp:make-msg
     "task_msgs/Robot"
     :name (robot-name itasc-robot)
     :type (robot-type itasc-robot))))

;; Conversion functions to create the 'objects' part of the goal message
(defun make-itasc-object-msg (itasc-object)
  (when itasc-object
    (roslisp:make-msg
     "task_msgs/Object"
     name (object-name itasc-object)
     type (object-type itasc-object)
     object_frames (map 'vector #'identity (attached-frames itasc-object)))))

(defun assemble-itasc-object-msg-list (object-name-list)
  ;; create the corresponding object-msg for every object-name
  (mapcar (lambda (object-name)
            (let ((itasc-object (find-itasc-object object-name)))
              (cond (itasc-object
                     (make-itasc-object-msg itasc-object))
                    (t (cpl-impl:fail 'manipulation-failed
                                      :format-control 
                                      (concatenate 'string
                                                   "Could not find object in database. Object: "
                                                   object-name))))))
          object-name-list))

(defun assemble-itasc-object-msg-vector (object-name-list)
  (map 'vector #'identity 
       (assemble-itasc-object-msg-list object-name-list)))

;;; Conversions to create the 'joint_weights' part of the goal message
(defun make-robot-joint-weight-msg (robot-joint-weight)
  (roslisp:make-msg
   "task_msgs/RobotJointWeight"
   joint_name (joint-name robot-joint-weight)
   weight (weight robot-joint-weight)))

(defun make-robot-joint-weights-msg-vector (robot-joint-weights)
  (map 'vector #'identity 
       (mapcar #'make-robot-joint-weight-msg robot-joint-weights)))

;;; Conversions to create the "tasks" part of the goal message
(defun assemble-tasks-msg-vector (task-names-list)
  (when task-names-list
    (map 'vector #'identity
         (mapcar (lambda (task-name)
                   (let ((task (find-itasc-task task-name)))
                     (cond (task
                            (make-itasc-task-msg task))
                           (t (cpl-impl:fail 'manipulation-failed
                                             :format-control "Could not find task in database: ~a."
                                             :format-arguments (list task-name))))))
                 task-names-list))))

(defun make-itasc-task-msg (task)
  (when task
    (roslisp:make-msg
     "task_msgs/ItascTask"
     output_type (roslisp-msg-protocol:symbol-code
                  'task_msgs-msg:itasctask
                  (output-type task))
     name (name task)
     kinematic_chain (make-itasc-vkc-msg
                      (vkc task))
     joint_constraints (assemble-constraints-msg-vector
                        (joint-constraints task))
     feature_constraints (assemble-constraints-msg-vector 
                          (feature-constraints task))
     priority (priority task))))

(defun make-itasc-vkc-msg (vkc)
  (when vkc
    (roslisp:make-msg
     "task_msgs/VirtualKinematicChain"
     object_1 (make-itasc-object-frame-msg (object1 vkc))
     object_2 (make-itasc-object-frame-msg (object2 vkc))
     chain_joints (map 'vector #'identity
                       (mapcar #'make-itasc-chain-joint-msg
                               (chain-joints vkc))))))

(defun assemble-constraints-msg-vector (constraints)
  (when constraints
    (map 'vector #'identity
         (mapcar #'make-itasc-constraint-msg constraints))))

(defun make-itasc-chain-joint-msg (chain-joint)
  (when chain-joint
    (roslisp:make-msg
     "task_msgs/ChainJoint"
     type (roslisp-msg-protocol:symbol-code
           'task_msgs-msg:chainjoint
           (joint-type chain-joint))
     name (joint-name chain-joint))))

(defun make-itasc-object-frame-msg (object-frame)
  (when object-frame
    (roslisp:make-msg
     "task_msgs/Object_frame"
     object_name (object-name object-frame)
     frame_name (frame-name object-frame))))

(defun make-itasc-constraint-msg (constraint)
  (when constraint
    (roslisp:make-msg
     "task_msgs/Constraint"
     constraint_type (roslisp-msg-protocol:symbol-code
                      'task_msgs-msg:constraint
                      (constraint-type constraint))
     name (constraint-name constraint)
     referred_joint (referred-joint constraint)
     constraint_operator (roslisp-msg-protocol:symbol-code
                          'task_msgs-msg:constraint
                          (operator constraint))
     constraint_value (value constraint)
     lower_tracking_boundary (lower-boundary constraint)
     upper_tracking_boundary (upper-boundary constraint)
     weight (weight constraint)
     controller (roslisp:make-msg
                 "task_msgs/Controller"
                 type (roslisp-msg-protocol:symbol-code
                       'task_msgs-msg:controller
                       (controller constraint)))
     trajectory_generator (roslisp:make-msg
                           "task_msgs/TrajectoryGenerator"
                           type (roslisp-msg-protocol:symbol-code
                                 'task_msgs-msg:trajectorygenerator
                                 (trajectory-type constraint))
                           execution_time (trajectory-duration constraint)
                           velocity_boundary (trajectory-velocity constraint)))))
