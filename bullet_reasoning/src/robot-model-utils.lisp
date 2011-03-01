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
;;;

(in-package :btr)

(defun set-robot-state-from-tf (tf robot &optional (reference-frame "/map"))
  (loop for name being the hash-keys in  (slot-value robot 'links) do
    (let ((tf-name (if (eql (elt name 0) #\/) name (concatenate 'string "/" name))))
      (handler-case
          (setf (link-pose robot name) (cl-transforms:transform->pose
                                        (tf:lookup-transform tf :source-frame tf-name :target-frame reference-frame)))
        (tf:tf-connectivity-error ()
          nil)))))

(defun set-robot-state-from-joints (joint-states robot)
  (roslisp:with-fields ((names name)
                (positions position))
      joint-states
    (map nil (lambda (name state)
               (setf (joint-state robot name) state))
         names positions)))

(defun make-robot-joint-state-msg (robot &key joint-names (time 0))
  (let ((joint-names (map 'vector #'identity (or joint-names
                                                 (joint-names robot)))))
    (roslisp:make-msg "sensor_msgs/JointState"
                      (stamp header) time
                      name joint-names
                      position (map 'vector (curry #'joint-state robot) joint-names)
                      velocity (make-array (length joint-names)
                                           :element-type 'float
                                           :initial-element 0.0)
                      effort (make-array (length joint-names)
                                         :element-type 'float
                                         :initial-element 0.0))))

(defun get-link-chain (robot start end)
  "Returns the chain of links from the link named `start' to the link
  named `end'"
  (labels ((walk-tree (curr end &optional path)
             (let ((from-joint (cl-urdf:from-joint curr)))
               (cond ((eq curr end)
                      (cons curr path))
                     ((not from-joint)
                      nil)
                     ((eq (cl-urdf:joint-type from-joint) :fixed)
                      (walk-tree (cl-urdf:parent (cl-urdf:from-joint curr)) end path))
                     (t
                      (walk-tree (cl-urdf:parent (cl-urdf:from-joint curr))
                                 end (cons curr path)))))))
    (let ((start-link (gethash start (cl-urdf:links robot)))
          (end-link (gethash end (cl-urdf:links robot))))
      (assert start-link nil "Link `~a' unknown" start)
      (assert end-link nil "Link `~a' unknown" end)
      (walk-tree end-link start-link))))

(defun get-joint-chain (robot start end)
  "Returns the chain of joints from the link named `start' to the link
  named `end'"
  (labels ((walk-tree (curr end &optional path)
             (let ((from-joint (cl-urdf:from-joint curr)))
               (cond ((eq curr end)
                      path)
                     ((not from-joint)
                      nil)
                     ((eq (cl-urdf:joint-type from-joint) :fixed)
                      (walk-tree (cl-urdf:parent (cl-urdf:from-joint curr)) end path))
                     (t
                      (walk-tree (cl-urdf:parent (cl-urdf:from-joint curr))
                                 end (cons from-joint path)))))))
    (let ((start-link (gethash start (cl-urdf:links robot)))
          (end-link (gethash end (cl-urdf:links robot))))
      (assert start-link nil "Link `~a' unknown" start)
      (assert end-link nil "Link `~a' unknown" end)
      (walk-tree end-link start-link))))

(defun set-tf-from-robot-state (tf robot &optional (base-frame "base_footprint"))
  (let ((reference-transform-inv (cl-transforms:transform-inv
                                  (cl-transforms:reference-transform
                                   (link-pose robot base-frame)))))
    (dolist (link (link-names robot) tf)
      (unless (equal link base-frame)
        (let ((transform (cl-transforms:transform*
                          reference-transform-inv
                          (cl-transforms:reference-transform
                           (link-pose robot link)))))
          (tf:set-transform tf (tf:make-stamped-transform
                                base-frame link (roslisp:ros-time)
                                (cl-transforms:translation transform)
                                (cl-transforms:rotation transform))))))))

(defun make-seed-states (robot joint-names &optional (steps 4))
  "Returns a sequence of possible seed states. The first seed state is
the current state represented by `robot' the other states are
generated by dividing the interval between the lower and the upper
limit of the joint into 1 + `steps' segments and using the corresponding
joint positions as seeds."
  (flet ((init-limits (urdf lower upper joint-name)
             (let* ((joint (or (gethash joint-name (cl-urdf:joints urdf))
                               (error 'simple-error
                                      :format-control "Unknown joint `~a'"
                                      :format-arguments (list joint-name))))
                    (joint-limits (and (slot-boundp joint 'cl-urdf:limits)
                                       (cl-urdf:limits joint))))
               (case (cl-urdf:joint-type joint)
                 ((:revolute :prismatic)
                    (setf (gethash joint-name lower)
                          (cl-urdf:lower joint-limits))
                    (setf (gethash joint-name upper)
                          (cl-urdf:upper joint-limits)))
                 (:continuous
                    (setf (gethash joint-name lower)
                          (cl-urdf:lower joint-limits))
                    (setf (gethash joint-name upper)
                          (* pi 2)))
                 (t (setf (gethash joint-name lower) 0.0)
                    (setf (gethash joint-name upper) 0.0))))))
    (let ((lower-limits (make-hash-table :test 'equal))
          (upper-limits (make-hash-table :test 'equal))
          (joint-names (map 'vector #'identity joint-names)))
      (map 'nil (curry #'init-limits (urdf robot) lower-limits upper-limits) joint-names)
      (cons
       (make-robot-joint-state-msg robot :joint-names joint-names)
       (loop for i from 1 to steps collecting
         (roslisp:make-msg
          "sensor_msgs/JointState"
          (stamp header) 0
          name joint-names
          position (map 'vector
                        (lambda (name)
                          (+ (gethash name lower-limits)
                             (* i (/ (- (gethash name upper-limits)
                                        (gethash name lower-limits))
                                     (+ 1 steps)))))
                        joint-names)
          velocity (make-array (length joint-names)
                               :element-type 'float
                               :initial-element 0.0)
          effort (make-array (length joint-names)
                             :element-type 'float
                             :initial-element 0.0)))))))

(defun get-weighted-ik (robot pose-stamped &key
                        (tool-frame (cl-transforms:make-pose
                                     (cl-transforms:make-3d-vector 0 0 0)
                                     (cl-transforms:make-quaternion 0 0 0 1)))
                        (ik-namespace (error "Namespace of IK service has to be specified"))
                        (ik-base-link "torso_lift_link")
                        (base-frame "base_footprint"))
  (let* ((tf (set-tf-from-robot-state (make-instance 'tf:transformer)
                                      robot base-frame))
         (pose (tf:transform-pose tf :pose pose-stamped :target-frame ik-base-link))
         (joint-names (kinematics_msgs-msg:joint_names 
                       (kinematics_msgs-srv:kinematic_solver_info
                        (roslisp:call-service
                         (concatenate 'string ik-namespace "/get_ik_solver_info")
                         'kinematics_msgs-srv:getkinematicsolverinfo))))
         (seed-states (make-seed-states robot joint-names 4)))
    (dolist (seed seed-states)
      (roslisp:with-fields ((solution solution)
                            (error-code (val error_code)))
          (roslisp:call-service
           (concatenate 'string ik-namespace "/get_weighted_ik")
           'kdl_arm_kinematics-srv:getweightedik
           :pose (tf:pose-stamped->msg pose)
           :tool_frame (tf:pose->msg tool-frame)
           :ik_seed seed)
        (when (eql error-code (roslisp-msg-protocol:symbol-code
                               'motion_planning_msgs-msg:ArmNavigationErrorCodes
                               :success))
          (return solution))))))
