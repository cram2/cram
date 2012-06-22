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

(defvar *ik-solver-info-cache* nil
  "An alist of namespace names to GetKinematicSolverInfo messages.")

(defvar *persistent-ik-services* nil
  "An alist that maps from ROS namespaces to persistent service
  handles.")

(defun set-robot-state-from-tf (tf robot &optional (reference-frame "/map"))
  (loop for name being the hash-keys in  (slot-value robot 'links) do
    (let ((tf-name (if (eql (elt name 0) #\/) name (concatenate 'string "/" name))))
      ;; We wait transforms instead of calling can-transform because
      ;; in particular at initialization time, the TF tree might not
      ;; be fully received yet.
      (when (tf:wait-for-transform
             tf :timeout 0.5 :source-frame tf-name
                :target-frame reference-frame)
        (handler-case
            (setf (link-pose robot name)
                  (cl-transforms:transform->pose
                   (tf:lookup-transform tf :source-frame tf-name :target-frame reference-frame)))
          (tf:tf-lookup-error ()
            nil))))))

(defgeneric set-robot-state-from-joints (joint-states robot)
  (:method ((joint-states sensor_msgs-msg:jointstate) (robot robot-object))
    "Sets the joints of `robot' to the values specified in the
sensor_msgs/JointStates message."
    (roslisp:with-fields ((names name)
                          (positions position))
        joint-states
      (map nil (lambda (name state)
                 (setf (joint-state robot name) state))
           names positions)))
  (:method ((joint-states list) (robot robot-object))
    "Sets the joint states of `robot' to the values specifies in the
    list `joint-states'. `joint-states' is a list of the form:
     
      ([(name value)]*)"
    (loop for (name value) in joint-states do
      (setf (joint-state robot name) value))))

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

(defun make-joint-state-message (joint-states &key (time-stamp 0))
  "`joint-states is a list of lists with two elements, the name of the
  joint and its position. This function returns a message of type
  sensor_msgs/JointState that is filled with these values."
  (roslisp:make-msg "sensor_msgs/JointState"
                    (stamp header) time-stamp
                    name (map 'vector #'car joint-states)
                    position (map 'vector #'cadr joint-states)
                    velocity (make-array (length joint-states)
                                         :element-type 'float
                                         :initial-element 0.0)
                    effort (make-array (length joint-states)
                                       :element-type 'float
                                       :initial-element 0.0)))

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

(defun set-tf-from-robot-state (tf robot
                                &key (base-frame "base_footprint")
                                  (time (roslisp:ros-time)))
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
                                base-frame link time
                                (cl-transforms:translation transform)
                                (cl-transforms:rotation transform))))))))

(defun make-seed-states (robot joint-names &optional (steps 3))
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
       (cut:lazy-mapcar (lambda (joint-states)
                          (roslisp:make-msg
                           "sensor_msgs/JointState"
                           (stamp header) 0
                           name joint-names
                           position (reverse
                                     (map 'vector #'identity joint-states))
                           velocity (make-array (length joint-names)
                                                :element-type 'float
                                                :initial-element 0.0)
                           effort (make-array (length joint-names)
                                              :element-type 'float
                                              :initial-element 0.0)))
                        (apply #'lazy-cross-product
                               (reverse
                                (loop for name across joint-names collecting
                                  (loop for i from 0 below steps collecting
                                    (+ (gethash name lower-limits)
                                       (* i (/ (- (gethash name upper-limits)
                                                  (gethash name lower-limits))
                                               (- steps 1)))))))))))))

(defun calculate-tool-pose (pose &key (tool (cl-transforms:make-pose
                                             (cl-transforms:make-3d-vector 0 0 0)
                                             (cl-transforms:make-quaternion 0 0 0 1))))
  (let ((goal-trans (cl-transforms:transform*
                     (cl-transforms:reference-transform pose)
                     (cl-transforms:transform-inv
                      (cl-transforms:reference-transform tool)))))
    (tf:make-pose-stamped
     (tf:frame-id pose) (tf:stamp pose)
     (cl-transforms:translation goal-trans)
     (cl-transforms:rotation goal-trans))))

(defun get-ik-solver-info (ik-namespace)
  (or (cdr (assoc ik-namespace *ik-solver-info-cache* :test #'equal))
      (let ((solver-info (roslisp:call-service
                          (concatenate 'string ik-namespace "/get_ik_solver_info")
                          'kinematics_msgs-srv:getkinematicsolverinfo)))
        (push (cons ik-namespace solver-info) *ik-solver-info-cache*)
        solver-info)))

(defun get-persistent-ik-service (ik-namespace)
  (let ((service (cdr (assoc ik-namespace *persistent-ik-services*
                             :test #'equal))))
    (unless (and service (roslisp:persistent-service-ok service))
      (setf service (make-instance 'roslisp:persistent-service
                      :service-name (concatenate
                                     'string ik-namespace "/get_ik")
                      :service-type "kinematics_msgs/GetPositionIK"))
      (setf *persistent-ik-services*
            (cons (cons ik-namespace service)
                  (remove ik-namespace *persistent-ik-services*
                          :key #'car :test #'equal))))
    service))

(defun get-ik (robot pose-stamped
               &key 
                 (tool-frame (cl-transforms:make-pose
                              (cl-transforms:make-3d-vector 0 0 0)
                              (cl-transforms:make-quaternion 0 0 0 1)))
                 (ik-namespace (error "Namespace of IK service has to be specified"))
                 (fixed-frame "map")
                 (robot-base-frame "base_footprint")
                 seed-state)
  (let ((tf (make-instance 'tf:transformer))
        (time (roslisp:ros-time)))
    (set-tf-from-robot-state tf robot
                             :base-frame robot-base-frame
                             :time time)
    (tf:set-transform tf (tf:transform->stamped-transform
                          fixed-frame robot-base-frame time
                          (cl-transforms:pose->transform (pose robot))))
    (roslisp:with-fields ((joint-names (joint_names kinematic_solver_info))
                          (link-names (link_names kinematic_solver_info)))
        (get-ik-solver-info ik-namespace)
      (let* ((ik-base-frame (cl-urdf:name
                             (cl-urdf:parent
                              (gethash (elt joint-names 0)
                                       (cl-urdf:joints (urdf robot))))))
             (pose (tf:transform-pose tf :pose (tf:copy-pose-stamped pose-stamped :stamp 0)
                                         :target-frame ik-base-frame)))
        (roslisp:with-fields ((solution (joint_state solution))
                              (error-code (val error_code)))
            (roslisp:call-persistent-service
             (get-persistent-ik-service ik-namespace)
             :ik_request
             (roslisp:make-msg
              "kinematics_msgs/PositionIKRequest"
              ;; we assume that the last joint in JOINT-NAMES is the end
              ;; of the chain which is what we want for ik_link_name.
              :ik_link_name (elt link-names 0)
              :pose_stamped (tf:pose-stamped->msg
                             (calculate-tool-pose pose :tool tool-frame))
              :ik_seed_state (roslisp:make-msg
                              "arm_navigation_msgs/RobotState"
                              joint_state
                              (or seed-state
                                  (make-robot-joint-state-msg
                                   robot :joint-names joint-names))))
             :timeout 1.0)
          (when (eql error-code (roslisp-msg-protocol:symbol-code
                                 'arm_navigation_msgs-msg:ArmNavigationErrorCodes
                                 :success))
            (list solution)))))))

(defun calculate-pan-tilt (robot pan-link tilt-link pose)
  "Calculates values for the pan and tilt joints so that they pose on
  `pose'. Returns (LIST PAN-VALUE TILT-VALUE)"
  (let* ((pan-transform (cl-transforms:reference-transform
                         (link-pose robot pan-link)))
         (tilt-transform (cl-transforms:reference-transform
                          (link-pose robot tilt-link)))
         (pose-trans (etypecase pose
                       (cl-transforms:3d-vector
                          (cl-transforms:make-transform
                           pose (cl-transforms:make-quaternion 0 0 0 1)))
                       (cl-transforms:pose (cl-transforms:reference-transform pose))
                       (cl-transforms:transform pose)))
         (pose-in-pan (cl-transforms:transform*
                       (cl-transforms:transform-inv pan-transform)
                       pose-trans))
         (pose-in-tilt (cl-transforms:transform*
                        (cl-transforms:transform-inv tilt-transform)
                        pose-trans))
         (pan-joint-name (cl-urdf:name
                          (cl-urdf:from-joint
                           (gethash pan-link (cl-urdf:links (urdf robot))))))
         (tilt-joint-name (cl-urdf:name
                           (cl-urdf:from-joint
                            (gethash tilt-link (cl-urdf:links (urdf robot)))))))
    (list
     (+ (joint-state robot pan-joint-name)
        (if (= (cl-transforms:x (cl-transforms:translation pose-in-pan)) 0)
            0.0
            (atan (cl-transforms:y (cl-transforms:translation pose-in-pan))
                  (cl-transforms:x (cl-transforms:translation pose-in-pan)))))
     (+ (joint-state robot tilt-joint-name)
        (if (= (cl-transforms:x (cl-transforms:translation pose-in-tilt)) 0)
            0.0
            (atan (- (cl-transforms:z (cl-transforms:translation pose-in-tilt)))
                  (+ (expt (cl-transforms:y (cl-transforms:translation pose-in-tilt)) 2)
                     (expt (cl-transforms:x (cl-transforms:translation pose-in-tilt)) 2))))))))
