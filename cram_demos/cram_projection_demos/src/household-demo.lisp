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

(in-package :demos)

(defparameter *demo-object-spawning-poses*
  '((:bowl
     "sink_area_left_middle_drawer_main"
     ((0.10 -0.0505 -0.062256) (0 0 -1 0)))
    (:cup
     ;; "sink_area_left_bottom_drawer_main"
     ;; ((0.11 0.12 -0.0547167) (0 0 -1 0))
     "kitchen_island_left_upper_drawer_main"
     ((0.11 0.08 -0.026367) (0 0 -1 0)))
    (:spoon
     ;; "oven_area_area_middle_upper_drawer_main"
     "sink_area_left_upper_drawer_main"
     ((0.125 0 -0.0136) (0 -0.053938 -0.998538 -0.003418)))
    ;; So far only this orientation works
    (:breakfast-cereal
     "oven_area_area_right_drawer_board_3_link"
     ((0.10 -0.03 0.11) (0.0087786 0.005395 -0.838767 -0.544393)))
    ;; ((:breakfast-cereal . ((1.398 1.490 1.2558) (0 0 0.7071 0.7071)))
    ;; (:breakfast-cereal . ((1.1 1.49 1.25) (0 0 0.7071 0.7071)))
    (:milk
     ;; "iai_fridge_main_middle_level"
     ;; ((0.10355 0.022 0.094) (0.00939 -0.00636 -0.96978 -0.2437))
     "iai_fridge_door_shelf1_bottom"
     ((-0.01 -0.05 0.094) (0 0 0 1)))))

(defparameter *object-grasps*
  '((:cup . (:left-side :right-side :back :front))
    ;; PR2 cannot grasp the cereal from the top on the oven shelf
    ;; (Boxy can though.)
    (:breakfast-cereal . (:front :back :front-flipped :back-flipped))))

(defparameter *furniture-offsets-original-kitchen*
  '(("sink_area_footprint_joint"
     ((1.855d0 1.3d0 0.0d0) (0 0 1 0)))
    ("oven_area_footprint_joint"
     ((1.845d0 2.5d0 0.0d0) (0 0 1 0)))
    ("kitchen_island_footprint_joint"
     ((-1.363d0 0.59d0 0.0d0) (0 0 0 1)))
    ("fridge_area_footprint_joint"
     ((1.845d0 -0.73d0 0.0d0) (0 0 1 0)))
    ("table_area_main_joint"
     ((-2.4d0 -1.5d0 0.0d0) (0 0 1 0)))
    ("dining_area_footprint_joint"
     ((-3.38d0 0.28d0 0.0d0) (0.0d0 0.0d0 0.7071067811848163d0 0.7071067811882787d0)))))

(defparameter *furniture-offsets-offset-kitchen*
  '(("sink_area_footprint_joint"
     ((1.855d0 2.9d0 0.0d0) (0 0 1 0)))
    ("oven_area_footprint_joint"
     ((1.65d0 0.35d0 0.0d0) (0 0 0.7 0.3)))
    ("kitchen_island_footprint_joint"
     ((-3.6d0 0.7d0 0.0d0) (0 0 0 1)))
    ("fridge_area_footprint_joint"
     ((-1.4d0 3.05d0 0.0d0) (0 0 -0.7 0.7)))
    ("table_area_main_joint"
     ((0.95d0 -0.95d0 0.0d0) (0 0 0.3 0.7)))
    ("dining_area_footprint_joint"
     ((-2.5d0 -0.55d0 0.0d0) (0 0 1 0)))))


(defun spawn-objects-on-fixed-spots (&key
                                       (spawning-poses-relative
                                        *demo-object-spawning-poses*)
                                       (object-types
                                        '(:breakfast-cereal :cup :bowl :spoon :milk)))
  ;; clean up
  (kill-and-detach-all)

  ;; spawn objects at default poses
  (let* ((spawning-poses-absolute
           (make-poses-list-relative spawning-poses-relative))
         (objects (mapcar (lambda (object-type)
                            (btr-utils:spawn-object
                             (intern (format nil "~a-1" object-type) :keyword)
                             object-type
                             :pose (cdr (assoc object-type
                                               spawning-poses-absolute))))
                          object-types)))
    ;; stabilize world
    ;; (btr:simulate btr:*current-bullet-world* 100)
    objects)

  ;; attach objects to world
  (mapcar (lambda (object-type)
            (btr:attach-object (btr:get-environment-object)
                               (btr:object btr:*current-bullet-world*
                                           (intern (format nil "~a-1" object-type) :keyword))
                               :link (second (find object-type
                                                   spawning-poses-relative
                                                   :key #'car))))
          object-types))


(defun park-robot (&optional (?nav-pose
                              (cl-transforms-stamped:make-pose-stamped
                               cram-tf:*fixed-frame*
                               0.0
                               (cl-transforms:make-identity-vector)
                               (cl-transforms:make-identity-rotation))))
  (cpl:with-failure-handling
      ((cpl:plan-failure (e)
         (declare (ignore e))
         (return)))
    (cpl:par
      (exe:perform
       (desig:an action
                 (type positioning-arm)
                 (left-configuration park)
                 (right-configuration park)))
      (exe:perform
       (desig:an action
                 (type moving-torso)
                 (joint-angle upper-limit)))
      (exe:perform
       (desig:an action
                 (type going)
                 (target (desig:a location
                                  (pose ?nav-pose)))))
      (exe:perform (desig:an action (type opening-gripper) (gripper (left right))))
      (exe:perform (desig:an action (type looking) (direction forward))))))

(defun initialize ()
  (sb-ext:gc :full t)

  ;;(when ccl::*is-logging-enabled*
  ;;    (setf ccl::*is-client-connected* nil)
  ;;    (ccl::connect-to-cloud-logger)
  ;;    (ccl::reset-logged-owl))

  ;; (setf proj-reasoning::*projection-checks-enabled* t)

  (kill-and-detach-all)
  (setf (btr:joint-state (btr:get-environment-object)
                         "sink_area_left_upper_drawer_main_joint")
        0.0
        (btr:joint-state (btr:get-environment-object)
                         "sink_area_left_middle_drawer_main_joint")
        0.0
        (btr:joint-state (btr:get-environment-object)
                         "sink_area_left_bottom_drawer_main_joint")
        0.0
        (btr:joint-state (btr:get-environment-object)
                         "iai_fridge_door_joint")
        0.0
        (btr:joint-state (btr:get-environment-object)
                         "sink_area_dish_washer_door_joint")
        0.0
        (btr:joint-state (btr:get-environment-object)
                         "sink_area_dish_washer_tray_main")
        0.0
        (btr:joint-state (btr:get-environment-object)
                         "oven_area_area_right_drawer_main_joint")
        0.0
        (btr:joint-state (btr:get-environment-object)
                         "sink_area_trash_drawer_main_joint")
        0.0
        (btr:joint-state (btr:get-environment-object)
                         "kitchen_island_left_upper_drawer_main_joint")
        0.0)
  (btr-belief::publish-environment-joint-state
   (btr:joint-states (btr:get-environment-object)))

  (setf desig::*designators* (tg:make-weak-hash-table :weakness :key))

  ;; (coe:clear-belief)

  (btr:clear-costmap-vis-object))

(defun finalize ()
  ;; (setf proj-reasoning::*projection-reasoning-enabled* nil)

  ;;(when ccl::*is-logging-enabled*
  ;;  (ccl::export-log-to-owl "ease_milestone_2018.owl")
  ;;  (ccl::export-belief-state-to-owl "ease_milestone_2018_belief.owl"))
  (sb-ext:gc :full t))


(defun household-demo (&key (object-list '(:bowl :breakfast-cereal :milk :cup :spoon))
                         varied-kitchen)
  (urdf-proj:with-simulated-robot

    (if varied-kitchen
        (btr-belief:vary-kitchen-urdf *furniture-offsets-offset-kitchen*)
        (btr-belief:vary-kitchen-urdf *furniture-offsets-original-kitchen*))
    (if (> (cl-transforms:x
            (cl-transforms:origin
             (btr:pose
              (btr:rigid-body (btr:get-environment-object)
                              :|IAI-KITCHEN.fridge_area|))))
           0)
        ;; if the fridge is in front of robot, current kitchen is original
        (when varied-kitchen
          (setf btr:*current-bullet-world* (make-instance 'btr:bt-reasoning-world))
          (btr-belief:spawn-world))
        ;; if the fridge is behind the robot, current kitchen is varied
        (unless varied-kitchen
          (setf btr:*current-bullet-world* (make-instance 'btr:bt-reasoning-world))
          (btr-belief:spawn-world)))
    (initialize)
    (setf btr:*visibility-threshold* 0.7)
    (when cram-projection:*projection-environment*
      (spawn-objects-on-fixed-spots
       :object-types object-list
       :spawning-poses-relative *demo-object-spawning-poses*))
    (park-robot)

    ;; set the table
    (dolist (?object-type object-list)
      (exe:perform
       (desig:an action
                 (type transporting)
                 (object (desig:an object (type ?object-type)))
                 (context table-setting))))

    ;; clean up
    ;; (when cram-projection:*projection-environment*
    ;;   (spawn-objects-on-fixed-spots
    ;;    :object-types object-list
    ;;    :spawning-poses-relative *delivery-poses-relative*))

    (dolist (?object-type (reverse object-list))
      (let ((?grasps (cdr (assoc ?object-type *object-grasps*))))
        (exe:perform
         (desig:an action
                   (type transporting)
                   (object (desig:an object (type ?object-type)))
                   (context table-cleaning)
                   (grasps ?grasps)))))))





;;;;;; THE STUFF BELOW IS FOR HOUSEHOLD-DEMO-RANDOM

(defparameter *object-spawning-poses*
  '("sink_area_surface"
    (:breakfast-cereal . ((0.2 -0.15 0.1) (0 0 0 1)))
    (:cup . ((0.2 -0.35 0.1) (0 0 0 1)))
    (:bowl . ((0.18 -0.55 0.1) (0 0 0 1)))
    (:spoon . ((0.15 -0.4 -0.05) (0 0 0 1)))
    (:milk . ((0.07 -0.35 0.1) (0 0 0 1))))
  "Relative poses on sink area")

(defparameter *object-placing-poses*
  '((:breakfast-cereal . ((-0.78 0.9 0.95) (0 0 1 0)))
    (:cup . ((-0.79 1.35 0.9) (0 0 0.7071 0.7071)))
    (:bowl . ((-0.76 1.19 0.88) (0 0 0.7071 0.7071)))
    (:spoon . ((-0.78 1.5 0.86) (0 0 0 1)))
    (:milk . ((-0.75 1.7 0.95) (0 0 0.7071 0.7071))))
  "Absolute poses on kitchen_island.")

(defparameter *delivery-poses-relative*
  `((:bowl
     "kitchen_island_surface"
     ((0.24 -0.5 0.0432199478149414d0)
      (0.0 0.0 0.33465 0.94234)))
    (:cup
     "kitchen_island_surface"
     ((0.21 -0.20 0.06)
      (0.0 0.0 0.33465 0.94234)))
    (:spoon
     "kitchen_island_surface"
     ((0.26 -0.32 0.025)
      (0.0 0.0 1 0)))
    (:milk
     "kitchen_island_surface"
     ((0.25 0 0.0983174006144206d0)
      (0.0 0.0 -0.9 0.7)))
    (:breakfast-cereal
     "kitchen_island_surface"
     ((0.32 -0.9 0.1) (0 0 0.6 0.4)))))

(defparameter *delivery-poses*
  `((:bowl . ((-0.8399440765380859d0 1.2002920786539713d0 0.8932199478149414d0)
              (0.0 0.0 0.33465 0.94234)))
    (:cup . ((-0.8908212025960287d0 1.4991984049479166d0 0.9027448018391927d0)
             (0.0 0.0 0.33465 0.94234)))
    (:spoon . ((-0.8409400304158529d0 1.38009208679199219d0 0.8673268000284831d0)
               (0.0 0.0 1 0)))
    (:milk . ((-0.8495257695515951d0 1.6991498311360678d0 0.9483174006144206d0)
              (0.0 0.0 -0.9 0.7)))
    (:breakfast-cereal . ((-0.78 0.8 0.95) (0 0 0.6 0.4)))))

(defparameter *object-colors*
  '((:spoon . "black")
    (:breakfast-cereal . "yellow")
    (:milk . "blue")
    (:bowl . "red")
    (:cup . "red")))

(defparameter *object-cad-models*
  '(;; (:cup . "cup_eco_orange")
    ;; (:bowl . "edeka_red_bowl")
    ))

;; (defparameter *cleaning-deliver-poses*
;;   `((:bowl . ((1.45 -0.4 1.0) (0 0 0 1)))
;;     (:cup . ((1.45 -0.4 1.0) (0 0 0 1)))
;;     (:spoon . ((1.45 -0.4 1.0) (0 0 0 1)))
;;     (:milk . ((1.2 -0.5 0.8) (0 0 1 0)))
;;     (:breakfast-cereal . ((1.15 -0.5 0.8) (0 0 1 0)))))

;; (defparameter *object-grasping-arms*
;;   '((:breakfast-cereal . :right)
;;     (:cup . :left)
;;     (:bowl . :right)
;;     (:spoon . :right)
;;     (:milk . :right)))

(defun spawn-objects-on-sink-counter (&key
                                        (object-types
                                         '(:breakfast-cereal
                                           :cup
                                           :bowl
                                           :spoon
                                           :milk))
                                        (spawning-poses-relative
                                         *object-spawning-poses*)
                                        (random
                                         NIL))
  ;; make sure mesh paths are known, kill old objects and destroy all attachments
  (kill-and-detach-all)

  ;; spawn objects
  (let* ((spawning-poses-absolute
           (make-poses-relative spawning-poses-relative))
         (objects
           (mapcar
            (lambda (object-type)
              (let* (;; generate object name of form :TYPE-1
                     (object-name
                       (intern (format nil "~a-1" object-type) :keyword))
                     ;; spawn object in Bullet World at default pose
                     (object
                       (btr-utils:spawn-object object-name object-type))
                     ;; calculate new pose: either random or from input list
                     (object-pose
                       (if random
                           ;; generate a pose on a surface
                           (let* ((aabb-z
                                    (cl-transforms:z
                                     (cl-bullet:bounding-box-dimensions
                                      (btr:aabb object)))))
                             (cram-tf:translate-pose
                              (desig:reference
                               (if (eq object-type :spoon)
                                   (desig:a location
                                            (in (desig:an object
                                                          (type drawer)
                                                          (urdf-name
                                                           sink-area-left-upper-drawer-main)
                                                          (part-of iai-kitchen)))
                                            (side front)
                                            (range 0.2)
                                            (range-invert 0.12))
                                   (desig:a location
                                            (on (desig:an object
                                                          (type counter-top)
                                                          (urdf-name sink-area-surface)
                                                          (part-of iai-kitchen)))
                                            ;; below only works for knowrob sem-map
                                            ;; (centered-with-padding 0.1)
                                            (side left)
                                            (side front))))
                              :z (/ aabb-z 2.0)))
                           ;; take the pose from the function input list
                           (cdr (assoc object-type spawning-poses-absolute))))
                     ;; rotate new pose randomly around Z
                     (rotated-object-pose
                       (cram-tf:rotate-pose object-pose
                                            :z (/ (* 2 pi) (random 10.0)))))
                ;; move object to calculated pose on surface
                (btr-utils:move-object object-name rotated-object-pose)
                ;; return object
                object))
                   object-types)))

    ;; make sure generated poses are stable, especially important for random ones
    ;; TDOO: if unstable, call itself

     ;; attach spoon to the drawer
    (when (btr:object btr:*current-bullet-world* :spoon-1)
      (btr:attach-object (btr:get-environment-object)
                         (btr:object btr:*current-bullet-world* :spoon-1)
                         :link "sink_area_left_upper_drawer_main"))

    ;; stabilize world
    (btr:simulate btr:*current-bullet-world* 100)

    ;; return list of BTR objects
    objects))

(defun household-demo-random (&optional
                                (random
                                 nil)
                                (list-of-objects
                                 '(:bowl :spoon :cup :milk :breakfast-cereal)))

  (urdf-proj:with-simulated-robot

    (initialize)
    (when cram-projection:*projection-environment*
      (spawn-objects-on-sink-counter :random random))

    (park-robot)

    ;; (an object
    ;;     (obj-part "drawer_sinkblock_upper_handle"))

    (dolist (?object-type list-of-objects)
      (let* ( ;; (?arm-to-use
             ;;   (cdr (assoc ?object-type *object-grasping-arms*)))
             (?cad-model
               (cdr (assoc ?object-type *object-cad-models*)))
             (?color
               (cdr (assoc ?object-type *object-colors*)))
             (?object-to-fetch
               (desig:an object
                         (type ?object-type)
                         (desig:when ?cad-model
                           (cad-model ?cad-model))
                         (desig:when ?color
                           (color ?color)))))

        (cpl:with-failure-handling
            ((common-fail:high-level-failure (e)
               (roslisp:ros-warn (pp-plans demo) "Failure happened: ~a~%Skipping..." e)
               (return)))

          (exe:perform
           (desig:an action
                     (type transporting)
                     (context table-setting-counter)
                     (object ?object-to-fetch)
                     ;; (desig:when ?arm-to-use
                     ;;   (arms (?arm-to-use)))
                     )))

        ;; (setf proj-reasoning::*projection-reasoning-enabled* nil)
        ))

    ;; (setf proj-reasoning::*projection-reasoning-enabled* nil)

    (park-robot)

    (finalize)

   cpl:*current-path*))




;;;;;;;; THE STUFF BELOW IS FOR THE POURING EXPERIMENT

(defun pour-demo ()
  (spawn-objects-on-fixed-spots
   :spawning-poses-relative *delivery-poses-relative*)
  (urdf-proj:with-simulated-robot
    (pp-plans::add
     (desig:an object
               (type milk)
               (location (desig:a location
                                  (on (desig:an object
                                                (type counter-top)
                                                (urdf-name kitchen-island-surface)
                                                (part-of iai-kitchen))))))
     (desig:an object
               (type bowl)
               (location (desig:a location
                                  (on (desig:an object
                                                (type counter-top)
                                                (urdf-name kitchen-island-surface)
                                                (part-of iai-kitchen)))))))))




#+stuff-for-testing-different-projection-stuff-in-household-domain-also-old-stuff
(
 (defparameter *sink-nav-goal*
   (cl-transforms-stamped:make-pose-stamped
    "map"
    0.0
    (cl-transforms:make-3d-vector 0.75d0 0.70d0 0.0)
    (cl-transforms:make-identity-rotation)))
 (defparameter *island-nav-goal*
   (cl-transforms-stamped:make-pose-stamped
    "map"
    0.0
    (cl-transforms:make-3d-vector -0.2d0 1.5d0 0.0)
    (cl-transforms:make-quaternion 0 0 1 0)))
 (defparameter *look-goal*
   (cl-transforms-stamped:make-pose-stamped
    "base_footprint"
    0.0
    (cl-transforms:make-3d-vector 0.5d0 0.0d0 1.0d0)
    (cl-transforms:make-identity-rotation)))


 (defun go-to-sink-or-island (&optional (sink-or-island :sink))
   (let ((?navigation-goal (ecase sink-or-island
                             (:sink *sink-nav-goal*)
                             (:island *island-nav-goal*)))
         (?ptu-goal *look-goal*))
     (cpl:par
       (exe:perform (desig:an action
                              (type parking-arms)))
       (exe:perform (desig:a motion
                             (type going)
                             (pose ?navigation-goal))))
     (exe:perform (desig:a motion
                           (type looking)
                           (pose ?ptu-goal)))))

 (defun pick-object (&optional (?object-type :breakfast-cereal) (?arm :right))
   (go-to-sink-or-island :sink)
   (let* ((?object-desig
            (desig:an object (type ?object-type)))
          (?perceived-object-desig
            (exe:perform (desig:an action
                                   (type detecting)
                                   (object ?object-desig)))))
     (cpl:par
       (exe:perform (desig:an action
                              (type looking)
                              (object ?perceived-object-desig)))
       (exe:perform (desig:an action
                              (type picking-up)
                              (arm ?arm)
                              (object ?perceived-object-desig))))))

 (defun place-object (?target-pose &optional (?arm :right))
   (go-to-sink-or-island :island)
   (cpl:par
     (exe:perform (desig:a motion
                           (type looking)
                           (pose ?target-pose)))
     (exe:perform (desig:an action
                            (type placing)
                            (arm ?arm)
                            (target (desig:a location
                                             (pose ?target-pose)))))))

 (defun demo-hard-coded ()
   (spawn-objects-on-sink-counter)

   (urdf-proj:with-simulated-robot

     (dolist (object-type '(:breakfast-cereal :cup :bowl :spoon :milk))

       (let ((placing-target
               (cl-transforms-stamped:pose->pose-stamped
                "map" 0.0
                (cram-bullet-reasoning:ensure-pose
                 (cdr (assoc object-type *object-placing-poses*)))))
             (arm-to-use
               (cdr (assoc object-type *object-grasping-arms*))))

         (pick-object object-type (or arm-to-use :left))
         (place-object placing-target (or arm-to-use :left))))))



 (defun test-projection ()
   (proj:with-projection-environment urdf-proj:urdf-bullet-projection-environment
     (cpl:top-level
       (exe:perform
        (let ((?pose (cl-transforms-stamped:make-pose-stamped
                      cram-tf:*robot-base-frame* 0.0
                      (cl-transforms:make-3d-vector -0.5 0 0)
                      (cl-transforms:make-identity-rotation))))
          (desig:a motion (type going) (pose ?pose))))
       (exe:perform
        (desig:a motion (type moving-torso) (joint-angle 0.3)))
       (exe:perform
        (desig:a motion (type opening-gripper) (gripper left)))
       (exe:perform
        (desig:a motion (type looking) (direction forward)))
       (exe:perform
        (let ((?pose (cl-transforms-stamped:make-pose-stamped
                      cram-tf:*robot-base-frame* 0.0
                      (cl-transforms:make-3d-vector 0.7 0.3 0.85)
                      (cl-transforms:make-identity-rotation))))
          (desig:a motion (type moving-tcp) (left-pose ?pose)))))))

 (defun test-desigs ()
   (let ((?pose
           (desig:reference
            (desig:a location
                     (on "CounterTop")
                     (name "iai_kitchen_meal_table_counter_top")))))
     (desig:reference
      (desig:a location
               (to see)
               (object (desig:an object (at (desig:a location (pose ?pose)))))))))

 (defun spawn-bottle ()
   (add-objects-to-mesh-list)
   (btr-utils:kill-all-objects)
   (btr-utils:spawn-object :bottle-1 :bottle :color '(1 0.5 0))
   (btr-utils:move-object :bottle-1 (cl-transforms:make-pose
                                     (cl-transforms:make-3d-vector -2 -1.0 0.861667d0)
                                     (cl-transforms:make-identity-rotation)))
   ;; stabilize world
   (btr:simulate btr:*current-bullet-world* 100))

 (defun spawn-objects ()
   (let ((object-types (add-objects-to-mesh-list)))
     ;; spawn at default location
     (let ((objects (mapcar (lambda (object-type)
                              (btr-utils:spawn-object
                               (intern (format nil "~a-1" object-type) :keyword)
                               object-type))
                            object-types)))
       ;; move on top of counter tops
       (mapcar (lambda (btr-object)
                 (let* ((aabb-z
                          (cl-transforms:z
                           (cl-bullet:bounding-box-dimensions
                            (btr:aabb btr-object))))
                        (new-pose
                          (cram-tf:translate-pose
                           (desig:reference
                            (desig:a location
                                     (on "CounterTop")
                                     (name "iai_kitchen_meal_table_counter_top")))
                           :z (/ aabb-z 2.0))))
                   (btr-utils:move-object (btr:name btr-object) new-pose)))
               objects)
       ;; bottle gets special treatment
       (btr-utils:move-object
        :bottle-1
        (cl-transforms:make-pose
         (cl-transforms:make-3d-vector -2 -1.0 0.861667d0)
         (cl-transforms:make-identity-rotation)))))
   ;; stabilize world
   (btr:simulate btr:*current-bullet-world* 100))


 (defparameter *meal-table-left-base-pose*
   (cl-transforms-stamped:make-pose-stamped
    "map"
    0.0
    (cl-transforms:make-3d-vector -1.12d0 -0.42d0 0.0)
    (cl-transforms:axis-angle->quaternion
     (cl-transforms:make-3d-vector 0 0 1) (/ pi -2))))
 (defparameter *meal-table-right-base-pose*
   (cl-transforms-stamped:make-pose-stamped
    "map"
    0.0
    (cl-transforms:make-3d-vector -2.0547d0 -0.481d0 0.0d0)
    (cl-transforms:axis-angle->quaternion
     (cl-transforms:make-3d-vector 0 0 1) (/ pi -2))))
 (defparameter *meal-table-left-base-look-pose*
   (cl-transforms-stamped:make-pose-stamped
    "base_footprint"
    0.0
    (cl-transforms:make-3d-vector 0.75d0 -0.12d0 1.11d0)
    (cl-transforms:make-identity-rotation)))
 (defparameter *meal-table-right-base-look-pose*
   (cl-transforms-stamped:make-pose-stamped
    "base_footprint"
    0.0
    (cl-transforms:make-3d-vector 0.65335d0 0.076d0 0.758d0)
    (cl-transforms:make-identity-rotation)))
 (defparameter *meal-table-left-base-look-down-pose*
   (cl-transforms-stamped:make-pose-stamped
    "base_footprint"
    0.0
    (cl-transforms:make-3d-vector 0.7d0 -0.12d0 0.7578d0)
    (cl-transforms:make-identity-rotation)))


 (defun prepare ()
   (cpl:with-failure-handling
       ((common-fail:low-level-failure (e)
          (roslisp:ros-warn (demo step-0) "~a" e)
          (return)))

     (let ((?navigation-goal *meal-table-right-base-pose*)
           (?ptu-goal *meal-table-right-base-look-pose*))
       (cpl:par
         (exe:perform
          (desig:an action
                    (type parking-arms)))
         (exe:perform (desig:a motion
                               (type going)
                               (pose ?navigation-goal))))
       (exe:perform (desig:a motion
                             (type looking)
                             (pose ?ptu-goal))))))
 (defun test-pr2-plans ()
   (proj:with-projection-environment urdf-proj:urdf-bullet-projection-environment
     (cpl:top-level
       (prepare))))

 (defun test-projection-perception ()
   (spawn-objects)
   (test-pr2-plans)
   (cpl:sleep 1)
   (proj:with-projection-environment urdf-proj:urdf-bullet-projection-environment
     (cpl:top-level
       (exe:perform
        (let ((?object-designator
                (desig:an object (type bottle))))
          (desig:a motion
                   (type detecting)
                   (object ?object-designator)))))))
 )
