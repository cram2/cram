;;;
;;; Copyright (c) 2023, Arthur Niedzwiecki <aniedz@cs.uni-bremen.de>
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

(defparameter *eurobin-object-spawning-poses*
  '((:eurobin-package
     "some_origin_frame"
     ((0.20 0.05 0.08) (1 0 0 0)))))

(defparameter *robot-base-pose-door*
  (cl-tf:make-pose-stamped "map" 0.0
                           (cl-tf:make-3d-vector 10.0d0 4.0d0 0.0d0)
                           (cl-tf:euler->quaternion :az (/ pi 2))))

(defun initialize-eurobin ()
  (sb-ext:gc :full t)

  ;;(when ccl::*is-logging-enabled*
  ;;    (setf ccl::*is-client-connected* nil)
  ;;    (ccl::connect-to-cloud-logger)
  ;;    (ccl::reset-logged-owl))

  ;; (setf proj-reasoning::*projection-checks-enabled* t)

  (kill-and-detach-all)
  (setf (btr:joint-state (btr:get-environment-object)
                         "window4_right_joint")
        0.0
        ;; (btr:joint-state (btr:get-environment-object)
        ;;                  "cabinet7_door_bottom_left_joint")
        ;; 0.025
        ;; (btr:joint-state (btr:get-environment-object)
        ;;                  "dishwasher_drawer_middle_joint")
        ;; 0.0
        )
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


(defun eurobin-demo (&key (step 0))
  ;;urdf-proj:with-simulated-robot
  (setf proj-reasoning::*projection-checks-enabled* nil)
  (setf btr:*visibility-threshold* 0.7)

  ;;; ---
  ;;; Step 0: Init apartment, reset joints, desigs and objects
  (when (<= step 0)
    (initialize-eurobin)
    ;; (btr-belief:vary-kitchen-urdf '(("handle_cab1_top_door_joint"
    ;;                                  ((-0.038d0 -0.5d0 -0.08d0)
    ;;                                   (0.706825181105366d0 0.0d0
    ;;                                    0.0d0 0.7073882691671998d0)))))
    ;; (setf btr:*current-bullet-world* (make-instance 'btr:bt-reasoning-world))
    ;; (btr-belief:spawn-world)

    ;; (when cram-projection:*projection-environment*
    ;;   (spawn-objects-on-fixed-spots
    ;;    :object-types '(:jeroen-cup :cup)
    ;;    :spawning-poses-relative *apartment-object-spawning-poses*))

    ;; (park-robot (cl-transforms-stamped:make-pose-stamped
    ;;              cram-tf:*fixed-frame*
    ;;              0.0
    ;;              (cl-transforms:make-3d-vector 1.5 1.5 0.0)
    ;;              (cl-transforms:make-quaternion 0 0 1 0)))
    )

  ;; Door trajectory
  ;; (let ((door-traj
  ;;         (urdf-proj:with-simulated-robot
  ;;           (man-int:get-action-trajectory :opening :right :right-side NIL
  ;;                                          (list (desig:AN OBJECT
  ;;                                                          (TYPE CUPBOARD)
  ;;                                                          (URDF-NAME window4-right)
  ;;                                                          (PART-OF APARTMENT)))
  ;;                                                   :opening-distance 0.0))))
  ;;   (mapcan (lambda (pose)
  ;;             (btr:add-vis-axis-object pose)
  ;;             (sleep 0.5))
  ;;          (append  ;; (man-int:get-traj-poses-by-label door-traj :reaching)
  ;;                   (man-int:get-traj-poses-by-label door-traj :grasping)
  ;;                   ;;(man-int:get-traj-poses-by-label door-traj :opening)
  ;;                   ;;(man-int:get-traj-poses-by-label door-traj :retracting)
  ;;                   )))

  
  (let* (
         ;; Initialize desigs for objects and locations
         (?accessing-window-base-pose
           (cl-transforms-stamped:make-pose-stamped
            "map" (roslisp:ros-time)
            (cl-tf:make-3d-vector 9.7d0 4.3d0 0.0d0)
            (cl-tf:euler->quaternion :az (* pi 0.75))))
         (?accessing-window-base-pose-2
           (cl-transforms-stamped:make-pose-stamped
            "map" (roslisp:ros-time)
            (cl-tf:make-3d-vector 9.5d0 4.3d0 0.0d0)
            (cl-tf:euler->quaternion :az (* pi 0.25))))
         )

    ;;; ---
    ;;; Go and open the door
    (when (<= step 1)
      (exe:perform
       (desig:AN ACTION
                 (TYPE going)
                 (target (desig:a location
                                  (pose ?accessing-window-base-pose)))))

      (exe:perform
          (desig:AN ACTION
                    (TYPE opening)
                    (ARM right)
                    (distance 0.0)
                    (GRASPS (left-side))
                    (OBJECT (desig:AN OBJECT
                                      (TYPE cupboard)
                                      (URDF-NAME window4-right)
                                      (PART-OF APARTMENT)))))
      ;; (exe:perform
      ;;  (desig:AN ACTION
      ;;            (TYPE going)
      ;;            (target (desig:a location
      ;;                             (pose ?accessing-window-base-pose-2)))))
      ;; (exe:perform
      ;;     (desig:AN ACTION
      ;;               (TYPE opening)
      ;;               (ARM left)
      ;;               (distance 1.5)
      ;;               (GRASPS (front))
      ;;               (OBJECT (desig:AN OBJECT
      ;;                                 (TYPE cupboard)
      ;;                                 (URDF-NAME window4-right)
      ;;                                 (PART-OF APARTMENT)))))
      )

    ;; (btr-utils:spawn-object :package :package :pose '((9.95 5.30 1.2)(0 0 0 1)))

    ;;; ---
    ;;; Take the package with both hands and carry it to the table
    (when (<= step 2))

    ;;; ---
    ;;; Cut the package open along its tape
    (when (<= step 3))

    ;;; ---
    ;;; Open the package lid
    (when (<= step 4))

    ;;; ---
    ;;; Pick items out of the package and place
    (when (<= step 5))

    )

  (finalize))
