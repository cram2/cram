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

(in-package :demo)

(defparameter *object-spawning-poses*
  '((:popcorn-pot . ((-0.875 -1.45 0.195) (0 0 0 1)))
    (:popcorn-pot-lid . ((-0.78 -1.53 0.54) (0 0 0 1)))
    (:ikea-bowl-ww . ((-0.95 -1.65 0.54) (0 0 0 1)))
    (:salt . ((0.5 -1.6 0.78) (0 0 0 1)))
    (:ikea-plate . ((-0.18 -0.35 0.1) (0 0 0 1))))
  "Absolute poses in map.")

(defparameter *look-goal*
  (cl-transforms-stamped:make-pose-stamped
   "base_footprint"
   0.0
   (cl-transforms:make-3d-vector 0.5d0 0.0d0 1.0d0)
   (cl-transforms:make-identity-rotation)))


(defun spawn-objects (&key
                        (object-types '(:popcorn-pot
                                        :popcorn-pot-lid
                                        :salt
                                        :ikea-bowl-ww
                                        :ikea-plate))
                        (spawning-poses-absolute *object-spawning-poses*))
  ;; make sure mesh paths are known, kill old objects and destroy all attachments
  (btr:add-objects-to-mesh-list "cram_pr2_popcorn_demo")
  (btr-utils:kill-all-objects)
  (btr:detach-all-objects (btr:get-robot-object))
  (btr:detach-all-objects (btr:get-environment-object))

  ;; spawn objects
  (let* ((objects
           (mapcar (lambda (object-type)
                     (let* (;; generate object name of form :TYPE-1
                            (object-name
                              (intern (format nil "~a-1" object-type) :keyword))
                            ;; spawn object in Bullet World at default pose
                            (object
                              (btr-utils:spawn-object object-name object-type))
                            ;; calculate new pose: either random or from input list
                            (object-pose
                              ;; take the pose from the function input list
                              (cdr (assoc object-type spawning-poses-absolute))))
                       ;; move object to calculated pose on surface
                       (btr-utils:move-object object-name object-pose)
                       ;; return object
                       object))
                   object-types)))

    ;; make sure generated poses are stable
    ;; TODO: if unstable, call itself

    ;; stabilize world
    (btr:simulate btr:*current-bullet-world* 100)

    ;; attach the objects to the links in the kitchen
    (btr:attach-object (btr:object btr:*current-bullet-world* :kitchen)
                       (btr:object btr:*current-bullet-world* :popcorn-pot-1)
                       :link "iai_popcorn_table_right_grid")
    (btr:attach-object (btr:object btr:*current-bullet-world* :kitchen)
                       (btr:object btr:*current-bullet-world* :popcorn-pot-lid-1)
                       :link "iai_popcorn_table_drawer_right_main")
    (btr:attach-object (btr:object btr:*current-bullet-world* :kitchen)
                       (btr:object btr:*current-bullet-world* :ikea-bowl-ww-1)
                       :link "iai_popcorn_table_drawer_right_main")
    (btr:attach-object (btr:object btr:*current-bullet-world* :kitchen)
                       (btr:object btr:*current-bullet-world* :salt-1)
                       :link "iai_popcorn_table_surface")
    (btr:attach-object (btr:object btr:*current-bullet-world* :kitchen)
                       (btr:object btr:*current-bullet-world* :ikea-plate-1)
                       :link "iai_popcorn_table_drawer_right_main")

    ;; return list of BTR objects
    objects))



(defun go-to-pose (?navigation-goal)
  (let ((?ptu-goal *look-goal*))
    (cpl:par
      (exe:perform (desig:an action
                             (type positioning-arm)
                             (left-configuration park)
                             (right-configuration park)))
      (exe:perform (desig:a motion
                            (type going)
                            (pose ?navigation-goal))))
    (exe:perform (desig:a motion
                          (type looking)
                          (pose ?ptu-goal)))))

(defun perceive-object (?object-type)
  (let ((?object-desig
          (desig:an object (type ?object-type))))
    (exe:perform (desig:an action
                           (type perceiving)
                           (object ?object-desig)
                           (counter 0)
                           (occluding-names T)))))

(defun pick-object (?object-type ?arm)
  (let* ((?perceived-object-desig (perceive-object ?object-type)))
    (cpl:seq
     (exe:perform (desig:an action
                            (type looking)
                            (object ?perceived-object-desig)))
     (exe:perform (desig:an action
                            (type picking-up)
                            (arm ?arm)
                            (object ?perceived-object-desig))))))

(defun place-object (?target-pose ?arm &key ?object-placed-on ?object-to-place ?attachment)
  (cpl:par
    (exe:perform (desig:a motion
                          (type looking)
                          (pose ?target-pose)))
    (exe:perform 
     (desig:an action
               (type placing)
               (arm ?arm)
               (desig:when ?object-to-place
                 (object ?object-to-place))
               (target (desig:a location
                                (desig:when ?object-placed-on
                                  (on ?object-placed-on))
                                (desig:when ?object-to-place
                                  (for ?object-to-place))
                                (desig:when ?attachment
                                  (attachment ?attachment))
                                (pose ?target-pose))))))
  ;; (btr:simulate btr:*current-bullet-world* 100)
  )

(defun open-drawer (drawer)
  (manipulate-drawer drawer :opening))

(defun close-drawer (drawer)
  (manipulate-drawer drawer :closing))

(defun manipulate-drawer (?drawer ?action)
  (let* ((?urdf-name (if (eq ?drawer :right)
                         'iai-popcorn-table-drawer-right-main
                         'iai-popcorn-table-drawer-left-main))
         (?drawer-obj (desig:an object
                                (type drawer)
                                (urdf-name ?urdf-name)
                                (part-of kitchen)))
         (?distance (if (eq ?action :opening)
                        0.4
                        0.0)))
    (exe:perform
     (desig:an action 
               (type ?action)
               (arm ?drawer) ;; since for opening the left drawer the
                             ;; robot should use the left arm.
               (object ?drawer-obj)
               (distance ?distance)))))

(defun move-arms (&key ?left-arm-pose ?right-arm-pose)
  (exe:perform
   (desig:a motion
            (type moving-tcp)
            (desig:when ?right-arm-pose
              (right-pose ?right-arm-pose))
            (desig:when ?left-arm-pose
              (left-pose ?left-arm-pose)))))

(defun park-arms ()
  (exe:perform
   (desig:an action
             (type positioning-arm)
             (left-configuration park)
             (right-configuration park))))


;; (defun test-projection ()
;;   (proj:with-projection-environment urdf-proj:urdf-bullet-projection-environment
;;     (cpl:top-level
;;       (exe:perform
;;        (let ((?pose (cl-tf:make-pose-stamped
;;                      cram-tf:*robot-base-frame* 0.0
;;                      (cl-transforms:make-3d-vector -0.5 0 0)
;;                      (cl-transforms:make-identity-rotation))))
;;          (desig:a motion (type going) (pose ?pose))))
;;       (exe:perform
;;        (desig:a motion (type moving-torso) (joint-angle 0.3)))
;;       (exe:perform
;;        (desig:a motion (type opening-gripper) (gripper left)))
;;       (exe:perform
;;        (desig:a motion (type looking) (direction forward)))
;;       (exe:perform
;;        (let ((?pose (cl-tf:make-pose-stamped
;;                      cram-tf:*robot-base-frame* 0.0
;;                      (cl-transforms:make-3d-vector 0.7 0.3 0.85)
;;                      (cl-transforms:make-identity-rotation))))
;;          (desig:a motion (type moving-tcp) (left-pose ?pose)))))))

;; (defun test-desigs ()
;;   (let ((?pose (desig:reference (desig:a location
;;                                          (on "CounterTop")
;;                                          (name "iai_kitchen_meal_table_counter_top")))))
;;     (desig:reference (desig:a location
;;                               (to see)
;;                               (object (desig:an object (at (desig:a location (pose ?pose)))))))))

;; (defun spawn-bottle ()
;;   (add-objects-to-mesh-list)
;;   (btr-utils:kill-all-objects)
;;   (btr-utils:spawn-object :bottle-1 :bottle :color '(1 0.5 0))
;;   (btr-utils:move-object :bottle-1 (cl-transforms:make-pose
;;                                     (cl-transforms:make-3d-vector -2 -1.0 0.861667d0)
;;                                     (cl-transforms:make-identity-rotation)))
;;   ;; stabilize world
;;   (btr:simulate btr:*current-bullet-world* 100))

;; (defun spawn-objects ()
;;   (let ((object-types (add-objects-to-mesh-list)))
;;     ;; spawn at default location
;;     (let ((objects (mapcar (lambda (object-type)
;;                              (btr-utils:spawn-object
;;                               (intern (format nil "~a-1" object-type) :keyword)
;;                               object-type))
;;                            object-types)))
;;       ;; move on top of counter tops
;;       (mapcar (lambda (btr-object)
;;                 (let* ((aabb-z (cl-transforms:z
;;                                 (cl-bullet:bounding-box-dimensions (btr:aabb btr-object))))
;;                        (new-pose (cram-tf:translate-pose
;;                                   (desig:reference
;;                                    (desig:a location
;;                                             (on "CounterTop")
;;                                             (name "iai_kitchen_meal_table_counter_top")))
;;                                   :z-offset (/ aabb-z 2.0))))
;;                   (btr-utils:move-object (btr:name btr-object) new-pose)))
;;               objects)
;;       ;; bottle gets special treatment
;;       (btr-utils:move-object :bottle-1 (cl-transforms:make-pose
;;                                         (cl-transforms:make-3d-vector -2 -1.0 0.861667d0)
;;                                         (cl-transforms:make-identity-rotation)))))
;;   ;; stabilize world
;;   (btr:simulate btr:*current-bullet-world* 100))

;; (defparameter *meal-table-left-base-pose*
;;   (cl-transforms-stamped:make-pose-stamped
;;    "map"
;;    0.0
;;    (cl-transforms:make-3d-vector -1.12d0 -0.42d0 0.0)
;;    (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi -2))))
;; (defparameter *meal-table-right-base-pose*
;;   (cl-transforms-stamped:make-pose-stamped
;;    "map"
;;    0.0
;;    (cl-transforms:make-3d-vector -2.0547d0 -0.481d0 0.0d0)
;;    (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi -2))))
;; (defparameter *meal-table-left-base-look-pose*
;;   (cl-transforms-stamped:make-pose-stamped
;;    "base_footprint"
;;    0.0
;;    (cl-transforms:make-3d-vector 0.75d0 -0.12d0 1.11d0)
;;    (cl-transforms:make-identity-rotation)))
;; (defparameter *meal-table-right-base-look-pose*
;;   (cl-transforms-stamped:make-pose-stamped
;;    "base_footprint"
;;    0.0
;;    (cl-transforms:make-3d-vector 0.65335d0 0.076d0 0.758d0)
;;    (cl-transforms:make-identity-rotation)))
;; (defparameter *meal-table-left-base-look-down-pose*
;;   (cl-transforms-stamped:make-pose-stamped
;;    "base_footprint"
;;    0.0
;;    (cl-transforms:make-3d-vector 0.7d0 -0.12d0 0.7578d0)
;;    (cl-transforms:make-identity-rotation)))


;; (defun prepare ()
;;   (cpl:with-failure-handling
;;           ((common-fail:low-level-failure (e)
;;              (roslisp:ros-warn (demo step-0) "~a" e)
;;              (return)))

;;         (let ((?navigation-goal *meal-table-right-base-pose*)
;;               (?ptu-goal *meal-table-right-base-look-pose*))
;;           (cpl:par
;; (exe:perform
;;  (desig:an action
;;            (type positioning-arm)
;;            (left-configuration park)
;;            (right-configuration park)))
;;             (exe:perform (desig:a motion
;;                                   (type going)
;;                                   (pose ?navigation-goal))))
;;           (exe:perform (desig:a motion
;;                                 (type looking)
;;                                 (pose ?ptu-goal))))))
;; (defun test-pr2-plans ()
;;   (proj:with-projection-environment urdf-proj:urdf-bullet-projection-environment
;;     (cpl:top-level
;;       (prepare))))

;; (defun test-projection-perception ()
;;   (spawn-objects)
;;   (test-pr2-plans)
;;   (cpl:sleep 1)
;;   (proj:with-projection-environment urdf-proj:urdf-bullet-projection-environment
;;     (cpl:top-level
;;       (exe:perform
;;        (let ((?object-designator
;;                (desig:an object (type bottle))))
;;          (desig:a motion
;;                   (type detecting)
;;                   (object ?object-designator)))))))

;; (defun test-grasp-and-place-object (&optional (?object-type :bottle) (?arm :right))
;;   (let ((proj-result
;;           (proj:with-projection-environment urdf-proj:urdf-bullet-projection-environment
;;             (cpl:top-level
;;               (prepare))
;;             (cpl:top-level
;;               (let ((?bottle-desig (desig:an object (type ?object-type))))
;;                 (flet ((step-1-inner ()
;;                          (let ((?perceived-bottle-desig (pp-plans::perceive ?bottle-desig)))
;;                            (cpl:par
;;                              (exe:perform (desig:an action
;;                                                     (type looking)
;;                                                     (object ?perceived-bottle-desig)))
;;                              (exe:perform (desig:an action
;;                                                     (type picking-up)
;;                                                     (arm ?arm)
;;                                                     (object ?perceived-bottle-desig)))))))
;;                   (cpl:with-retry-counters ((bottle-grasp-tries 2))
;;                     (cpl:with-failure-handling
;;                         ((common-fail:low-level-failure (e)
;;                            (roslisp:ros-warn (demo step-1) "~a" e)
;;                            (cpl:do-retry bottle-grasp-tries
;;                              (roslisp:ros-warn (demo step-1) "~a" e)
;;                              (prepare)
;;                              (cpl:retry))))

;;                       (step-1-inner))))
;;                 (desig:current-desig ?bottle-desig))))))
;;     (cpl:sleep 1.0)
;;     (let ((?result-object (car (proj::projection-environment-result-result proj-result))))
;;       (proj:with-projection-environment urdf-proj:urdf-bullet-projection-environment
;;         (cpl:top-level
;;           (exe:perform (desig:an action
;;                                  (type placing)
;;                                  (arm ?arm)
;;                                  (object ?result-object))))))))

;; (defun test-place-bottle ()
;;   (proj:with-projection-environment urdf-proj:urdf-bullet-projection-environment
;;     (cpl:top-level
;;       (exe:perform (desig:an action
;;                              (type placing)
;;                              (arm right))))))
