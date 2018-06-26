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

(defun pose-list->desig (pose-list)
  (let ((?pose (cl-transforms-stamped:pose->pose-stamped
                "map" 0.0
                (btr:ensure-pose pose-list))))
    (desig:a location (pose ?pose))))

(defparameter *object-cad-models*
  '(;; (:cup . "cup_eco_orange")
    ;; (:bowl . "edeka_red_bowl")
    ))

(defparameter *object-colors*
  '((:spoon . "blue")))

(defmethod exe:generic-perform :before (designator)
  (format t "~%PERFORMING~%~A~%~%" designator))

(defmacro with-real-robot (&body body)
  `(cram-process-modules:with-process-modules-running
       (rs:robosherlock-perception-pm
        pr2-pms::pr2-base-pm pr2-pms::pr2-arms-pm
        pr2-pms::pr2-grippers-pm pr2-pms::pr2-ptu-pm)
     (cpl-impl::named-top-level (:name :top-level)
       ,@body)))

(cpl:def-cram-function initialize-or-finalize ()
  (cpl:with-failure-handling
      ((cpl:plan-failure (e)
         (declare (ignore e))
         (return)))
    (cpl:par
      (pp-plans::park-arms)
      (let ((?pose (cl-transforms-stamped:make-pose-stamped
                    cram-tf:*fixed-frame*
                    0.0
                    (cl-transforms:make-identity-vector)
                    (cl-transforms:make-identity-rotation))))
        (exe:perform
         (desig:an action
                   (type going)
                   (target (desig:a location
                                    (pose ?pose))))))
      (exe:perform (desig:an action (type opening) (gripper (left right))))
      (exe:perform (desig:an action (type looking) (direction forward))))))



(cpl:def-cram-function demo-random (&optional
                                    (random t)
                                    (list-of-objects '(:bowl :spoon :cup :milk :breakfast-cereal)))

  (when ccl::*is-logging-enabled*
      (setf ccl::*is-client-connected* nil)
      (ccl::connect-to-cloud-logger)
      (ccl::reset-logged-owl))

  (btr:detach-all-objects (btr:get-robot-object))
  (btr-utils:kill-all-objects)
  (setf (btr:joint-state (btr:object btr:*current-bullet-world* :kitchen)
                         "sink_area_left_upper_drawer_main_joint")
        0.0)
  (btr-belief::publish-environment-joint-state
   (btr:joint-states (btr:object btr:*current-bullet-world* :kitchen)))

  (setf pr2-proj-reasoning::*projection-reasoning-enabled* nil)

  (unless proj:*projection-environment*
    (json-prolog:prolog-simple "rdf_retractall(A,B,C,belief_state).")
    (btr-belief::call-giskard-environment-service :kill-all "attached")
    (cram-bullet-reasoning-belief-state::call-giskard-environment-service
     :add-kitchen
     "kitchen"
     (cl-transforms-stamped:make-pose-stamped
      "map"
      0.0
      (cl-transforms:make-identity-vector)
      (cl-transforms:make-identity-rotation))))

  (setf desig::*designators* (tg:make-weak-hash-table :weakness :key))

  (when (eql cram-projection:*projection-environment*
             'cram-pr2-projection::pr2-bullet-projection-environment)
    (if random
        (spawn-objects-on-sink-counter-randomly)
        (spawn-objects-on-sink-counter)))

  ;; (setf cram-robot-pose-guassian-costmap::*orientation-samples* 3)

  (initialize-or-finalize)

  (let ((object-fetching-locations
          `((:breakfast-cereal . ,(desig:a location
                                           (on (desig:an object
                                                         (type counter-top)
                                                         (urdf-name sink-area-surface)
                                                         (owl-name "kitchen_sink_block_counter_top")
                                                         (part-of kitchen)))
                                           (side left)
                                           (side front)
                                           (range 0.5)))
            (:cup . ,(desig:a location
                              (side left)
                              (on (desig:an object
                                            (type counter-top)
                                            (urdf-name sink-area-surface)
                                            (owl-name "kitchen_sink_block_counter_top")
                                            (part-of kitchen)))))
            (:bowl . ,(desig:a location
                               (on (desig:an object
                                             (type counter-top)
                                             (urdf-name sink-area-surface)
                                             (owl-name "kitchen_sink_block_counter_top")
                                             (part-of kitchen)))
                               (side left)
                               (side front)
                               (range-invert 0.5)))
            (:spoon . ,(desig:a location
                                (in (desig:an object
                                              (type drawer)
                                              (urdf-name sink-area-left-upper-drawer-main)
                                              (owl-name "drawer_sinkblock_upper_open")
                                              (part-of kitchen)))
                                (side front)))
            (:milk . ,(desig:a location
                               (side left)
                               (on;; in
                                (desig:an object
                                             (type counter-top)
                                             (urdf-name iai-fridge-main)
                                             (owl-name "drawer_fridge_upper_interior")
                                             (part-of kitchen)))))))
        (object-placing-locations
          `((:breakfast-cereal . ,(desig:a location
                                           (left-of (an object (type bowl)))
                                           (far-from (an object (type bowl)))
                                           (for (an object (type breakfast-cereal)))
                                           (on (desig:an object
                                                         (type counter-top)
                                                         (urdf-name kitchen-island-surface)
                                                         (owl-name "kitchen_island_counter_top")
                                                         (part-of kitchen)))
                                           (side back)))
            (:cup . ,(desig:a location
                              (right-of (an object (type bowl)))
                              ;; (behind (an object (type bowl)))
                              (near (an object (type bowl)))
                              (for (an object (type cup)))))
            (:bowl . ,(desig:a location
                               (on (desig:an object
                                             (type counter-top)
                                             (urdf-name kitchen-island-surface)
                                             (owl-name "kitchen_island_counter_top")
                                             (part-of kitchen)))
                               (context table-setting)
                               (for (an object (type bowl)))
                               (object-count 3)
                               (side back)
                               (side right)
                               (range-invert 0.5)))
            (:spoon . ,(desig:a location
                                (right-of (an object (type bowl)))
                                (near (an object (type bowl)))
                                (for (an object (type spoon)))))
            (:milk . ,(desig:a location
                               (left-of (an object (type bowl)))
                               (far-from (an object (type bowl)))
                               (for (an object (type milk))))))))

    ;; (an object
    ;;     (obj-part "drawer_sinkblock_upper_handle"))

    (dolist (?object-type list-of-objects)
      (let* ((?fetching-location
               (cdr (assoc ?object-type object-fetching-locations)))
             (?delivering-location
               (cdr (assoc ?object-type object-placing-locations)))
             (?arm-to-use
               (cdr (assoc ?object-type *object-grasping-arms*)))
             (?cad-model
               (cdr (assoc ?object-type *object-cad-models*)))
             (?color
               (cdr (assoc ?object-type *object-colors*)))
             (?object-to-fetch
               (desig:an object
                         (type ?object-type)
                         (location ?fetching-location)
                         (desig:when ?cad-model
                           (cad-model ?cad-model))
                         (desig:when ?color
                           (color ?color)))))

        (case ?object-type
          (:bowl
           (cpl:with-failure-handling
               ((common-fail:high-level-failure (e)
                  (roslisp:ros-warn (pp-plans demo) "Failure happened: ~a~%Skipping the search" e)
                  (return)))
             (let ((?loc (cdr (assoc :breakfast-cereal object-fetching-locations))))
               (exe:perform
                (desig:an action
                          (type searching)
                          (object (desig:an object (type breakfast-cereal)))
                          (location ?loc))))))
          (:spoon
           (pp-plans:park-arms)
           (break))
          (:breakfast-cereal
           (setf pr2-proj-reasoning::*projection-reasoning-enabled* t)))

        (cpl:with-failure-handling
            ((common-fail:high-level-failure (e)
               (roslisp:ros-warn (pp-plans demo) "Failure happened: ~a~%Skipping..." e)
               (return)))
          (if (eq ?object-type :bowl)
              (exe:perform
               (desig:an action
                         (type transporting)
                         (object ?object-to-fetch)
                         (arm right)
                         (location ?fetching-location)
                         (target ?delivering-location)))
              (exe:perform
               (desig:an action
                         (type transporting)
                         (object ?object-to-fetch)
                         ;; (arm ?arm-to-use)
                         (location ?fetching-location)
                         (target ?delivering-location)))))

        (setf pr2-proj-reasoning::*projection-reasoning-enabled* nil))))

  (setf pr2-proj-reasoning::*projection-reasoning-enabled* nil)

  (initialize-or-finalize)

  (when ccl::*is-logging-enabled*
    (ccl::export-log-to-owl "ease_milestone_2018.owl")
    (ccl::export-belief-state-to-owl "ease_milestone_2018_belief.owl"))

  cpl:*current-path*)
