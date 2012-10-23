;;; Copyright (c) 2011, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(defvar *collision-object-pub* nil)
(defvar *attached-object-pub* nil)

(defvar *known-collision-objects* (tg:make-weak-hash-table :weakness :key))

(defun init-collision-environment ()
  (setf *collision-object-pub*
        (roslisp:advertise "/collision_object" "arm_navigation_msgs/CollisionObject" :latch t))
  (setf *attached-object-pub*
        (roslisp:advertise "/attached_collision_object" "arm_navigation_msgs/AttachedCollisionObject" :latch t)))

(register-ros-init-function init-collision-environment)

(defun get-collision-object-name (desig)
  "returns the name of a known collision object or NIL"
  (let ((obj (find-desig-collision-object desig)))
    (when obj
      (roslisp:with-fields (id) obj
        id))))

(defun find-desig-collision-object (desig)
  (labels ((find-collision-object (curr)
             (when curr
               (or (gethash curr *known-collision-objects*)
                   (find-collision-object (parent curr))))))
    (find-collision-object (current-desig desig))))

(defun desig-allowed-contacts (desig links &optional (penetration-depth 0.1))
  "returns allowed contact specifications for `desig' if desig has
  been added as a collision object. Returns NIL otherwise"
  (let ((obj (find-desig-collision-object desig))
        (i 0))
    (when obj
      (roslisp:with-fields ((id id)
                            (frame-id (frame_id header))
                            (shapes shapes)
                            (poses poses))
          obj
        (map 'vector (lambda (shape pose)
                       (incf i)
                       (roslisp:make-msg
                        "motion_planning_msgs/AllowedContactSpecification"
                        name (format nil "~a-~a" id i)
                        shape shape
                        (frame_id header pose_stamped) frame-id
                        (pose pose_stamped) pose
                        link_names (map 'vector #'identity links)
                        penetration_depth (float penetration-depth 0.0d0)))
             shapes poses)))))

(defun desig-bounding-box (desig)
  "Returns the bounding box of the object that is bound to `desig' if
  the object is a point cloud. Otherwise, returns NIL. The result is
  of type CL-TRANSFORMS:3D-VECTOR"
  (let ((obj (find-desig-collision-object desig)))
    (when obj
      (roslisp:with-fields (shapes)
          obj
        ;; This operation only makes sense if we have only one shape
        ;; in the collision object. Otherwise we return NIL
        (unless (> (length shapes) 1)
          (roslisp:with-fields (type dimensions)
              (elt shapes 0)
            ;; We need a BOX
            (when (eql type 1)
              (apply #'cl-transforms:make-3d-vector
                     (map 'list #'identity dimensions)))))))))

(defun register-collision-object (designator &key (padding -1))
  "Registers the object referenced by `designator' in the collision
environment."
  (declare (type object-designator designator))
  (when (desig-prop-values designator 'collision-parts)
    (let* ((obj-pose-stamped (designator-pose designator))
           (obj-trans (tf:pose->transform obj-pose-stamped))
           (collision-parts (desig-prop-values designator 'collision-part))
           (collision-shape-msgs
             (map 'vector 'identity
                  (mapcar (lambda (collision-part)
                            (roslisp:make-msg
                             "arm_navigation_msgs/Shape"
                             type (ecase (desig-prop-value collision-part 'shape)
                                    (:cylinder (roslisp-msg-protocol:symbol-code
                                                'arm_navigation_msgs-msg:shape
                                                :cylinder)))
                             dimensions (ecase (desig-prop-value collision-part 'shape)
                                          (:cylinder
                                           (vector
                                            (desig-prop-value collision-part 'radius)
                                            (desig-prop-value collision-part 'length))))))
                          collision-parts)))
           (collision-pose-msgs
             (map 'vector 'identity
                  (mapcar (lambda (collision-part)
                            (tf:pose->msg
                             ;; Pose of collision parts w.r.t. pose of object, therefore
                             ;; we transform the collision part pose
                             (cl-transforms:transform-pose
                              obj-trans
                              (reference
                               (desig-prop-value collision-part 'at)))))
                          collision-parts)))
           (object-name (desig-prop-value designator 'name))
           (collision-msg (roslisp:make-msg
                           "arm_navigation_msgs/CollisionObject"
                           (seq header) 0
                           (stamp header) (tf:stamp obj-pose-stamped)
                           (frame_id header) (tf:frame-id obj-pose-stamped)
                           id object-name
                           padding padding
                           (operation operation) (roslisp-msg-protocol:symbol-code
                                                  'arm_navigation_msgs-msg:collisionobjectoperation
                                                  :add)
                           shapes collision-shape-msgs
                           poses collision-pose-msgs)))
      (roslisp:publish *collision-object-pub* collision-msg))))

(defun remove-collision-object (desig)
  (let ((collision-object (find-desig-collision-object desig)))
    (when collision-object
      (roslisp:with-fields (id) collision-object
        (roslisp:publish *collision-object-pub*
                         (roslisp:make-msg
                          "arm_navigation_msgs/CollisionObject"
                          (frame_id header) "/base_footprint"
                          (stamp header) (roslisp:ros-time)
                          id id
                          (operation operation) 1))))))

(defun clear-collision-objects ()
  (roslisp:publish *collision-object-pub*
                   (roslisp:make-msg
                    "arm_navigation_msgs/CollisionObject"
                    (frame_id header) "/base_footprint"
                    (stamp header) (roslisp:ros-time)
                    id "all"
                    (operation operation) 1)))

(defun attach-collision-object (side desig)
  (let ((collision-object (find-desig-collision-object desig)))
    (when collision-object
      (let ((attach-object (roslisp:modify-message-copy
                            collision-object
                            (operation operation) (roslisp:symbol-code
                                                   'arm_navigation_msgs-msg:CollisionObjectOperation
                                                   :attach_and_remove_as_object))))
        (roslisp:publish *attached-object-pub*
                         (roslisp:make-msg
                          "arm_navigation_msgs/AttachedCollisionObject"
                          link_name (ecase side
                                      (:right "r_gripper_r_finger_tip_link")
                                      (:left "l_gripper_r_finger_tip_link"))
                          touch_links (map 'vector #'identity
                                           (ecase side
                                             (:right (roslisp:get-param
                                                      "/hand_description/right_arm/hand_touch_links"
                                                      '("r_gripper_palm_link"
                                                        "r_gripper_r_finger_link"
                                                        "r_gripper_l_finger_link")))
                                             (:left (roslisp:get-param
                                                     "/hand_description/left_arm/hand_touch_links"
                                                     '("l_gripper_palm_link"
                                                       "l_gripper_r_finger_link"
                                                       "l_gripper_l_finger_link")))))
                          object attach-object))))))

(defun detach-collision-object (side desig)
  (let ((collision-object (find-desig-collision-object desig)))
    (when collision-object
      (let ((detach-object (roslisp:modify-message-copy
                            collision-object
                            (operation operation) (roslisp:symbol-code
                                                   'arm_navigation_msgs-msg:CollisionObjectOperation
                                                   :detach_and_add_as_object))))
        (roslisp:publish *attached-object-pub*
                         (roslisp:make-msg
                          "arm_navigation_msgs/AttachedCollisionObject"
                          link_name (ecase side
                                      (:right "r_gripper_r_finger_tip_link")
                                      (:left "l_gripper_r_finger_tip_link"))
                          touch_links (ecase side
                                        (:right (vector
                                                 "r_gripper_palm_link"
                                                 "r_gripper_r_finger_link"
                                                 "r_gripper_l_finger_link"))
                                        (:left (vector
                                                "l_gripper_palm_link"
                                                "l_gripper_r_finger_link"
                                                "l_gripper_l_finger_link")))
                          object detach-object))))))

(defun point->msg (point &optional (msg-type "geometry_msgs/Point"))
  (declare (type cl-transforms:3d-vector point))
  (roslisp:make-msg
   msg-type
   x (cl-transforms:x point)
   y (cl-transforms:y point)
   z (cl-transforms:z point)))

(defun points->point-cloud (pose points)
  (let ((pose-tf (cl-transforms:reference-transform pose)))
    (roslisp:make-msg
     "sensor_msgs/PointCloud"
     (stamp header) (tf:stamp pose)
     (frame_id header) (tf:frame-id pose)
     points (map 'vector
                 (lambda (p)
                   (roslisp:with-fields (x y z) p
                     (point->msg
                      (cl-transforms:transform-point
                       pose-tf (cl-transforms:make-3d-vector x y z))
                      "geometry_msgs/Point32")))
                 points))))

(defun collision-environment-set-laser-period ()
  "Sets the tilting laser period to work best with collision
  environment"
  (roslisp:call-service
   "/laser_tilt_controller/set_periodic_cmd" "pr2_msgs/SetPeriodicCmd"
   :command (roslisp:make-msg
             "pr2_msgs/PeriodicCmd"
             (stamp header) (roslisp:ros-time)
             profile "linear"
             period 3
             amplitude 0.75
             offset 0.25)))
