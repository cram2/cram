;;; Copyright (c) 2013, Georg Bartels <georg.bartels@cs.uni-bremen.de>
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

(in-package :pr2-manip-pm)

;; auxiliary data structures
(defclass cylindrical-shape ()
  ((name :reader name :initarg :name)
   (height :reader height :initarg :height)
   (radius :reader radius :initarg :radius)
   (frame-id :reader frame-id :initarg :frame-id)
   (pose-stamped :reader pose-stamped :initarg :pose-stamped)))

;; auxiliary data-slots
(defvar *sample-cylinder* nil)

(defun init-feature-controller-tests ()
  "Inits the internal datastructures for testing the feature constraints controller."
  (setf *sample-cylinder* 
        (make-instance
         'cylindrical-shape
         :name "ketchup bottle"
         :height 0.2
         :radius 0.04
         :frame-id "/ketchup_frame"
         :pose-stamped (cl-tf:make-pose-stamped
                         "/base_link"
                         (roslisp:ros-time)
                         (cl-transforms:make-3d-vector 0.6 0.0 0.8)
                         (cl-transforms:make-identity-rotation)))))

(register-ros-init-function init-feature-controller-tests)

;; functions to create relevant features of external objects
(defgeneric calc-main-axis-feature (object))

(defmethod calc-main-axis-feature ((cylinder cylindrical-shape))
  "Calculates the feature representation of the main axis of a cylindrical object."
  ; With Cylindrical objects the convention is to have the main
  ; axis along the z-axis of the frame-id.
  (let ((cylinder-name (name cylinder))
        (cylinder-frame-id (frame-id cylinder)))
    (declare (type string cylinder-name cylinder-frame-id))
    (cram-feature-constraints:make-line-feature
     (concatenate 'string cylinder-name " main axis")
     cylinder-frame-id
     :direction (cl-transforms:make-3d-vector 0.0 0.0 0.1))))

(defgeneric calc-main-planes-features (object))

(defmethod calc-main-planes-features ((cylinder cylindrical-shape))
  "Calculates the feature representation of the main planes of a cylindrical object."
  ;TODO(Georg): it seems Daniel was right with his notion of relational
  ; feature functions: I'm just introducing all these planes to setup
  ; 'left-of 'in-front-of 'above-of etc. constraints. this approach implicitedly
  ; depends on the fact that our observation frame-id and the cylinder frame-id
  ; are aligned. refactoring of the controller to have feature functions with three
  ; entities (object, tool, observer) could solve this inconvenience...
  ;CONTINUTATION(GEORG): on second thought, relative feature functions
  ; seem to remove the need for virtual planes to setup of certain correspondences.
  ; however, there will be a need for another feature 'to be right off'.
  ; this could be a side patch of the object which raises the question of
  ; how to choose this...
  (let ((cylinder-name (name cylinder))
        (cylinder-frame-id (frame-id cylinder)))
    (declare (type string cylinder-name cylinder-frame-id))
    (let ((above-below-plane 
            (cram-feature-constraints:make-plane-feature
             (concatenate 'string "above-below-plane " cylinder-name)
             cylinder-frame-id
             :normal (cl-transforms:make-3d-vector 0.0 0.0 0.1)))
          (left-right-plane 
            (cram-feature-constraints:make-plane-feature
             (concatenate 'string "left-right-plane " cylinder-name)
             cylinder-frame-id
             :normal (cl-transforms:make-3d-vector 0.0 0.1 0.0)))
          (back-front-plane 
            (cram-feature-constraints:make-plane-feature
             (concatenate 'string "back-front-plane " cylinder-name)
             cylinder-frame-id
             :normal (cl-transforms:make-3d-vector 0.1 0.0 0.0))))
      (values above-below-plane left-right-plane back-front-plane))))

;; functions to generate features for parts of the robot
;; TODO(Georg): this should probably move into pr2_manipulation_knowledge
(defun get-gripper-main-axis (side)
  "Returns the feature representation of the main axis of gripper 'side'."
  (let ((pre (ecase side
                   (:left "/l_")
                   (:right "/r_")))
        (verbose-pre (ecase side
                       (:left "left ")
                       (:right "right "))))
    (cram-feature-constraints:make-line-feature
     (concatenate 'string verbose-pre "gripper main axis")
     (concatenate 'string pre "gripper_tool_frame")
     :direction (cl-transforms:make-3d-vector 0.1 0.0 0.0))))
        
(defun get-gripper-plane (side)
  "Returns the feature representation of the main plane of the 'side' gripper."
  (let ((pre (ecase side
                   (:left "/l_")
                   (:right "/r_")))
        (verbose-pre (ecase side
                       (:left "left ")
                       (:right "right "))))
    (cram-feature-constraints:make-plane-feature
     (concatenate 'string verbose-pre "gripper plane")
     (concatenate 'string pre "gripper_tool_frame")
     :normal (cl-transforms:make-3d-vector 0.0 0.0 0.1))))
     
;; functions to generate relevant constraints to grasp an object
(defgeneric calc-grasping-constraints (object side))

(defmethod calc-grasping-constraints ((cylinder cylindrical-shape) side)
  "Calculates the constraints to grasp the cylindrical object 'cylinder' with arm 'side'. The constraints are returned as a list."
  ;; get object features
  (let ((object-main-axis (calc-main-axis-feature cylinder)))
    (multiple-value-bind (object-above-below-plane 
                          object-left-right-plane 
                          object-back-front-plane)
        (calc-main-planes-features cylinder)
      (declare (ignore object-back-front-plane))
      ;; get robot features
      (let ((gripper-main-axis (get-gripper-main-axis side))
            (gripper-plane (get-gripper-plane side)))
        (let* (; semantics:have _gripper (plane)_ _very_ _perpendicular_ to _(main axis of)object_
               (gripper-cylinder-perpendicular-constraint
                 (cram-feature-constraints:make-perpendicular-constraint
                  "gripper cylinder vertical"
                  gripper-plane object-main-axis
                  0.95 1.5)) ;typical for very perpendicular
               ; semenatics: have _gripper (main axis)_ _pointing at_ _(center of)object_ _very much_
               (gripper-pointing-at-cylinder-constraint
                 (cram-feature-constraints:make-pointing-at-constraint
                  "gripper pointing at cylinder"
                  gripper-main-axis object-above-below-plane
                  -0.05 0.05)) ;typical for very pointing at
               ; semantics: _position_ _gripper (plane)_ _in the middle_ of the _object_
               (gripper-in-middle-constraint
                 (cram-feature-constraints:make-height-constraint
                  "gripper along cylinder"
                  gripper-plane object-above-below-plane
                  (- (/ (height cylinder) 2))
                  (/ (height cylinder) 2)))
               ; semantics: _position_ _gripper (plane)_ _somewhat away from_ the _object_
               (gripper-distance-constraint
                 (cram-feature-constraints:make-distance-constraint
                  "gripper distance from cylinder"
                  gripper-plane object-above-below-plane
                  0.1 0.2)) ;this is some pre-pose-distance
               (correct-side-lower-boundary (ecase side
                                              (:left 0.02)
                                              (:right -2.0)))
               (correct-side-upper-boundary (ecase side
                                              (:left 2.0)
                                              (-0.02))) ; these are typical for 'binary' constraints
               ; semantics: _position_ _left/right gripper (plane)_ _left/right_ of the _object_
               (gripper-correct-side-constraint
                 (cram-feature-constraints:make-height-constraint
                  "gripper left/right of cylinder"
                  gripper-plane object-left-right-plane
                  correct-side-lower-boundary correct-side-upper-boundary)))
          (list gripper-cylinder-perpendicular-constraint
                gripper-pointing-at-cylinder-constraint
                gripper-in-middle-constraint
                gripper-distance-constraint
                gripper-correct-side-constraint))))))

; some auxiliary functions to get tf-frames into tf
(defgeneric publish-object-frames-in-tf (object))

(defmethod publish-object-frames-in-tf ((cylinder cylindrical-shape))
  "Extracts the frame defined in the 'cylinder' and publishes the given transform through a broadcaster with an own thread to TF. The thread is returned."
  (let* ((pose-stamped (pose-stamped cylinder))
         (target-frame (frame-id cylinder))
         (source-frame (cl-tf:frame-id pose-stamped))
         (stamp (cl-tf:stamp pose-stamped))
         (translation (cl-transforms:origin pose-stamped))
         (origin (cl-transforms:orientation pose-stamped)))
    (let ((stamped-transform 
            (cl-tf:make-stamped-transform source-frame target-frame stamp
                                          translation origin)))
      (cl-tf:send-static-transform *tf-broadcaster*
                                   stamped-transform
                                   :interval 0.02))))

; some grasping code using the feature constraints controller
(defgeneric grasp-object-with-constraints (object side))

(defmethod grasp-object-with-constraints ((cylinder cylindrical-shape) side)
  (let ((tf-thread (publish-object-frames-in-tf cylinder))
        (constraints (calc-grasping-constraints cylinder side)))
    (execute-constraints-motion constraints side t)
    (sb-thread:terminate-thread tf-thread)))

;executive code of the test
(defun test-feature-constraints-controller-grasping-hardcoded ()
  (format t "Using regular pr2-manipulation to get left arm to pre-defined joint space goal.")
  (joint-move-to-nice-config)
  (format t "Using feature constraints controller to get to pre-pose to grasp a ketchup bottle with the left arm.")
  (grasp-object-with-constraints *sample-cylinder* :left))

(defun joint-move-to-nice-config ()
 "Moves the left arm to a non-singular pre-configuration to have a defined start for testing."
  (let ((left-joint-goal '(1.05 0.01 0.61 -0.44 -5.6 -0.86 0.16))
        (right-joint-goal '(-1.05 -0.01 -0.61 -0.44 5.6 -0.86 0.16))
        (left-joint-names '("l_shoulder_pan_joint" "l_shoulder_lift_joint" "l_upper_arm_roll_joint" "l_elbow_flex_joint" "l_forearm_roll_joint" "l_wrist_flex_joint" "l_wrist_roll_joint"))
        (right-joint-names '("r_shoulder_pan_joint" "r_shoulder_lift_joint" "r_upper_arm_roll_joint" "r_elbow_flex_joint" "r_forearm_roll_joint" "r_wrist_flex_joint" "r_wrist_roll_joint"))
        (trajectory-duration 5.0))
    (let ((left-trajectory (roslisp:make-message
                            "trajectory_msgs/JointTrajectory"
                            (stamp header) (roslisp:ros-time)
                            joint_names (map 'vector #'identity left-joint-names)
                            points (vector
                               (roslisp:make-message
                                "trajectory_msgs/JointTrajectoryPoint"
                                positions (map 'vector #'identity left-joint-goal)
                                velocities (map 'vector #'identity
                                                (make-list (length left-joint-goal)
                                                           :initial-element 0.0))
                                accelerations (map 'vector #'identity
                                                   (make-list (length left-joint-goal)
                                                              :initial-element 0.0))
                                time_from_start trajectory-duration))))
          (right-trajectory (roslisp:make-message
                            "trajectory_msgs/JointTrajectory"
                            (stamp header) (roslisp:ros-time)
                            joint_names (map 'vector #'identity right-joint-names)
                            points (vector
                               (roslisp:make-message
                                "trajectory_msgs/JointTrajectoryPoint"
                                positions (map 'vector #'identity right-joint-goal)
                                velocities (map 'vector #'identity
                                                (make-list (length right-joint-goal)
                                                           :initial-element 0.0))
                                accelerations (map 'vector #'identity
                                                   (make-list (length right-joint-goal)
                                                              :initial-element 0.0))
                                time_from_start trajectory-duration)))))
      (switch-controller '("l_arm_controller" "r_arm_controller") 
                         '("l_arm_vel" "r_arm_vel"))
      (cpl:par
        (execute-arm-trajectory :left left-trajectory)
        (execute-arm-trajectory :right right-trajectory)))))