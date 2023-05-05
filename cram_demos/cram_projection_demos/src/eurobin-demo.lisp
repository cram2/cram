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

(defun initialize-eurobin ()
  (sb-ext:gc :full t)
  ;; (setf proj-reasoning::*projection-checks-enabled* t)
  (kill-and-detach-all)
  (setf (btr:joint-state (btr:get-environment-object)
                         "window4_right_joint")
        0.0)
  (btr-belief::publish-environment-joint-state
   (btr:joint-states (btr:get-environment-object)))
  (setf desig::*designators* (tg:make-weak-hash-table :weakness :key))
  (giskard:reset-collision-scene)
  ;; (coe:clear-belief)
  (btr:clear-costmap-vis-object))


(defun eurobin-demo (&key (step 0))
  ;;urdf-proj:with-simulated-robot
  (setf proj-reasoning::*projection-checks-enabled* nil)
  (setf btr:*visibility-threshold* 0.7)

  (let* (;; Initialize desigs for objects and locations
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
         (?accessing-window-base-front-pose
           (cl-transforms-stamped:make-pose-stamped
            "map" (roslisp:ros-time)
            (cl-tf:make-3d-vector 9.3d0 4.2d0 0.0d0)
            (cl-tf:euler->quaternion :az (* pi 0.5))))
         (?picking-up-package-base-pose
           (cl-transforms-stamped:make-pose-stamped
            "map" (roslisp:ros-time)
            (cl-tf:make-3d-vector 9.5d0 4.6d0 0.0d0)
            (cl-tf:euler->quaternion :az (* pi 0.5))))
         (?placing-package-base-pose
           (cl-transforms-stamped:make-pose-stamped
            "map" (roslisp:ros-time)
            (cl-tf:make-3d-vector 15.4d0 2.0d0 0.0d0)
            (cl-tf:euler->quaternion :az (* pi 0.0))))
         (?box-delivery-pose
           (cl-transforms-stamped:make-pose-stamped
            "map" (roslisp:ros-time)
            (cl-tf:make-3d-vector 16.05d0 1.87d0 0.56d0)
            (cl-tf:euler->quaternion :az (* pi 0.0))))
         (?item-delivery-base-pose
           (cl-transforms-stamped:make-pose-stamped
            "map" (roslisp:ros-time)
            (cl-tf:make-3d-vector 15.5d0 3.0d0 0.0d0)
            (cl-tf:euler->quaternion :az (* pi 0.0))))
         (?item-delivery-pose
           (cl-transforms-stamped:make-pose-stamped
            "map" (roslisp:ros-time)
            (cl-tf:make-3d-vector 16.2d0 3.0d0 0.48d0)
            (cl-tf:euler->quaternion :az (* pi 0.0)))))

    ;;; ---
    ;;; Step 0: Init apartment, reset joints, desigs and objects
    (when (<= step 0)
      (initialize-eurobin))

    ;;; ---
    ;;; Go and open the door
    (when (<= step 1)
      (park-robot ?accessing-window-base-front-pose)
      ;; (exe:perform
      ;;  (desig:an action
      ;;            (type going)
      ;;            (target (desig:a location
      ;;                             (pose ?accessing-window-base-pose-2)))))
      )
    (when (<= step 2)
      (exe:perform
          (desig:an action
                    (type opening)
                    (arm right)
                    (distance 1.55)
                    (grasps (door-angled))
                    ;; (link window4-left-handle)
                    (object (desig:an object
                                      (type cupboard)
                                      (urdf-name window4-right)
                                      (part-of apartment))))))


    ;; (when (<= step 3)
    ;;   (exe:perform
    ;;    (desig:an action
    ;;              (type going)
    ;;              (target (desig:a location
    ;;                               (pose ?accessing-window-base-pose-2))))))

    ;; (when (<= step 4)
    ;;   (exe:perform
    ;;       (desig:an action
    ;;                 (type opening)
    ;;                 (arm right)
    ;;                 (distance 1.2)
    ;;                 (grasps (door-left))
    ;;                 (object (desig:an object
    ;;                                   (type cupboard)
    ;;                                   (urdf-name window4-right)
    ;;                                   (part-of apartment))))))

    (unless (btr:object btr:*current-bullet-world* :package-stand)
      (btr:add-object btr:*current-bullet-world*
                      :cylinder
                      :package-stand
                      '((9.95 5.40 0.35)(0 0 0 1))
                      :size '(0.3 0.3 0.7)
                      :mass 1.0))
    (btr-utils:spawn-object :open-box :open-box :pose '((9.95 5.40 0.8) (0 0 0 1)))

    ;;; ---
    ;;; Take the package and carry it to the table
    (when (<= step 5)
      ;; go to open door
      (exe:perform
       (desig:an action
                 (type going)
                 (target (desig:a location
                                  (pose ?picking-up-package-base-pose)))))
      ;; look at package
      (let ((?door-poi
               (cl-tf:make-pose-stamped
                "base_footprint" 0.0
                (cl-tf:make-3d-vector 1.0 -0.5 0.5)
                (cl-tf:make-identity-rotation))))
        (exe:perform
         (desig:an action (type looking)
                   (target (desig:a location
                                    (pose ?door-poi))))))
      ;; perceive package
      (let ((?package-desig (exe:perform
                             (desig:an action
                                       (type detecting)
                                       (object (desig:an object
                                                         (type open-box)))))))
        ;; pick-up the package
        (exe:perform
         (desig:an action
                   (type picking-up)
                   (object ?package-desig)
                   (park-arms nil))))

      ;; going to placing location
      (exe:perform
       (desig:an action
                 (type going)
                 (target (desig:a location
                                  (pose ?placing-package-base-pose)))))

      ;; place box
      (exe:perform
       (desig:an action
                 (type placing)
                 (target (desig:a location
                                  (pose ?box-delivery-pose))))))

    ;;; ---
    ;;; Pick items out of the package and place
    (when (<= step 3)
      (btr-utils:spawn-object :jeroen-cup :jeroen-cup :pose '((16.05d0 1.87d0 0.56d0)(0 0 0 1)))

      (let ((?package-desig (exe:perform
                             (desig:an action
                                       (type detecting)
                                       (object (desig:an object
                                                         (type jeroen-cup)))))))
        ;; pick-up the package
        (exe:perform
         (desig:an action
                   (type picking-up)
                   (object ?package-desig)
                   (grasp top))))

      (exe:perform
       (desig:an action
                 (type going)
                 (target (desig:a location
                                  (pose ?item-delivery-base-pose)))))

      (exe:perform
       (desig:an action
                 (type placing)
                 (target (desig:a location
                                  (pose ?item-delivery-pose)))))))
  (finalize))


;; visualize the gripper frames for opening a container
;; (let ((door-traj
;;         (urdf-proj:with-simulated-robot
;;           (man-int:get-action-trajectory :opening :left :door-left NIL
;;                                          (list (desig:AN OBJECT
;;                                                          (TYPE CUPBOARD)
;;                                                          (URDF-NAME window4-right)
;;                                                          (PART-OF APARTMENT)))
;;                                          :opening-distance 0.1))))
;;   (mapcan (lambda (pose)
;;             (btr:add-vis-axis-object pose)
;;             (sleep 0.5))
;;           (append  ;; (man-int:get-traj-poses-by-label door-traj :reaching)
;;            (man-int:get-traj-poses-by-label door-traj :grasping)
;;            ;; (man-int:get-traj-poses-by-label door-traj :opening)
;;            ;; (man-int:get-traj-poses-by-label door-traj :retracting)
;;            )))
