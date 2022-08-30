;;;
;;; Copyright (c) 2022, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(defparameter *apartment-object-spawning-poses*
  '((:jeroen-cup
     "cabinet1_coloksu_level4"
     ((0.20 0.05 0.08) (1 0 0 0)))
    (:cup
     "cabinet1_coloksu_level4"
     ((0.15 -0.1 0.08) (0 0 -1 0)))))

(defun initialize-apartment ()
  (sb-ext:gc :full t)

  ;;(when ccl::*is-logging-enabled*
  ;;    (setf ccl::*is-client-connected* nil)
  ;;    (ccl::connect-to-cloud-logger)
  ;;    (ccl::reset-logged-owl))

  ;; (setf proj-reasoning::*projection-checks-enabled* t)

  (kill-and-detach-all)
  (setf (btr:joint-state (btr:get-environment-object)
                         "cabinet1_door_top_left_joint")
        0.0
        (btr:joint-state (btr:get-environment-object)
                         "cabinet7_door_bottom_left_joint")
        0.025
        (btr:joint-state (btr:get-environment-object)
                         "dishwasher_drawer_middle_joint")
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


(defun apartment-demo (&key (step 0))
  ;;urdf-proj:with-simulated-robot
  (setf proj-reasoning::*projection-checks-enabled* nil)
  (setf btr:*visibility-threshold* 0.7)
  (when (<= step 0)
    (initialize-apartment)
    ;; (btr-belief:vary-kitchen-urdf '(("handle_cab1_top_door_joint"
    ;;                                  ((-0.038d0 -0.5d0 -0.08d0)
    ;;                                   (0.706825181105366d0 0.0d0
    ;;                                    0.0d0 0.7073882691671998d0)))))
    ;; (setf btr:*current-bullet-world* (make-instance 'btr:bt-reasoning-world))
    ;; (btr-belief:spawn-world)

    (when cram-projection:*projection-environment*
      (spawn-objects-on-fixed-spots
       :object-types '(:jeroen-cup :cup)
       :spawning-poses-relative *apartment-object-spawning-poses*))
    (park-robot (cl-transforms-stamped:make-pose-stamped
                 cram-tf:*fixed-frame*
                 0.0
                 (cl-transforms:make-3d-vector 1.5 1.5 0.0)
                 (cl-transforms:make-quaternion 0 0 1 0))))

  (let* ((?object
           (an object
               (type jeroen-cup)
               (name jeroen-cup-1)))
         (?location-in-cupboard
           (a location
              (on (an object
                      (type shelf)
                      (urdf-name cabinet1-coloksu-level4)
                      (part-of apartment)
                      (location (a location
                                   (in (an object
                                           (type cupboard)
                                           (urdf-name cabinet1-door-top-left)
                                           (part-of apartment)))))))
              (side (back left))
              (range-invert 0.2)
              (range 0.25)
              (orientation upside-down)
              (for ?object)))
         (?location-on-island
           (a location
              (on (an object
                      (type surface)
                      (urdf-name island-countertop)
                      (part-of apartment)))
              (side back)
              (range 0.4)
              (range-invert 0.3)
              (for ?object)))
         (?location-in-dishwasher
           (a location
              (above (an object
                         (type drawer)
                         (urdf-name dishwasher-drawer-middle)
                         (part-of apartment)
                         (location (a location
                                      (in (an object
                                              (type dishwasher)
                                              (urdf-name cabinet7)
                                              (part-of apartment)))))))
              (for (desig:an object
                             (type jeroen-cup)
                             (name jeroen-cup-1)))
              (attachments (jeroen-cup-in-dishwasher-1 jeroen-cup-in-dishwasher-2))))
         ;; (?location-in-hand
         ;;   (a location
         ;;      (in (an object
         ;;              (type robot)
         ;;              (name tiago-dual)
         ;;              (part-of tiago-dual)))))
         ;; (?location-in-other-hand
         ;;   (a location
         ;;      (in (an object
         ;;              (type robot)
         ;;              (name tiago-dual)
         ;;              (part-of tiago-dual)))))
         (?location-on-island-upside-down
           (a location
              (on (an object
                      (type surface)
                      (urdf-name island-countertop)
                      (part-of apartment)))
              (side (right back))
              (range-invert 1.5)
              (orientation upside-down)
              (for (an object
                       (type jeroen-cup)
                       (name jeroen-cup-1)))))


         ;; hard-coded stuff for real-world demo
         (?accessing-cupboard-door-robot-pose
           (cl-transforms-stamped:make-pose-stamped
            "map"
            0.0
            (cl-transforms:make-3d-vector 1.3861795354091224 2.0873920232286345 0.0d0)
            (cl-transforms:make-quaternion 0.0d0 0.0d0 1 0)))
         (?accessing-cupboard-door-another-robot-pose
           (cl-transforms-stamped:make-pose-stamped
            "map"
            0.0
            (cl-transforms:make-3d-vector 1.23 2.0 0.0d0)
            (cl-transforms:make-quaternion 0.0d0 0.0d0 1 0)))
         (?accessing-cupboard-door-robot-poses
           (list ?accessing-cupboard-door-robot-pose
                 ?accessing-cupboard-door-another-robot-pose))
         (?detecting-cupboard-robot-pose
           (cl-transforms-stamped:make-pose-stamped
            "map"
            0.0
            (cl-transforms:make-3d-vector 1.2624905511520543d0 2.002821242371188d0 0.0d0)
            (cl-transforms:make-quaternion 0.0d0 0.0d0 -0.980183726892722d0 0.19809053873088875d0)))
         (?on-counter-top-cup-pose
           (cl-transforms-stamped:make-pose-stamped
            "map"
            0.0
            (cl-transforms:make-3d-vector 2.38 2.7 1.0126)
            (cl-transforms:make-quaternion 0 0 1 0)))
         (?delivering-counter-top-robot-pose
           (cl-transforms-stamped:make-pose-stamped
            "map"
            0.0
            (cl-transforms:make-3d-vector 1.6 2.2 0.0d0)
            (cl-transforms:make-quaternion 0.0d0 0.0d0 0.3 1)))
         (?sealing-cupboard-door-robot-pose
           (cl-transforms-stamped:make-pose-stamped
            "map"
            0.0
            (cl-transforms:make-3d-vector 1.6861833377510809 1.0873920171726708 0.0)
            (cl-transforms:make-quaternion 0.0d0 0.0d0 0.9238795325112867 0.3826834323650899)))
         (?accessing-dishwasher-door-robot-pose
           (cl-transforms-stamped:make-pose-stamped
            "map"
            0.0
            (cl-transforms:make-3d-vector 1.6 3.5 0.0d0)
            (cl-transforms:make-quaternion 0.0d0 0.0d0 0 1))))

    ;; bring cup from cupboard to table
    (when (<= step 1)
      (let ((?goal `(and (cpoe:object-at-location ,(an object
                                                       (type jeroen-cup)
                                                       (name jeroen-cup-1))
                                                  ,(a location
                                                      (pose ?on-counter-top-cup-pose)))
                         (cpoe:location-reset ,?location-in-cupboard))))
        (exe:perform
         (an action
             (type transporting)
             (object (an object
                         (type jeroen-cup)
                         (name jeroen-cup-1)
                         (location ?location-in-cupboard)))
             (access-search-outer-robot-location (a location
                                                    (poses ?accessing-cupboard-door-robot-poses)))
             (access-seal-search-outer-arms (left))
             (access-search-outer-grasps (right-side))
             (search-robot-location (a location
                                       (pose ?detecting-cupboard-robot-pose)))
             (fetch-robot-location (a location
                                      (pose ?detecting-cupboard-robot-pose)))
             (arms (left))
             (grasps (front))
             (target (a location
                        (pose ?on-counter-top-cup-pose))
                     ;; ?location-on-island
                     )
             (deliver-robot-location (a location
                                        (pose ?delivering-counter-top-robot-pose)))
             (seal-search-outer-robot-location (a location
                                                  (pose ?sealing-cupboard-door-robot-pose)))
             (seal-search-outer-grasps (right-side))
             (goal ?goal)))))

    ;; put cup from island into dishwasher
    ;; (when (<= step 2)
    ;;   (exe:perform
    ;;    (an action
    ;;        (type transporting)
    ;;        (object (an object
    ;;                    (type jeroen-cup)
    ;;                    (name jeroen-cup-1)
    ;;                    (location ?location-on-island)))
    ;;        (target ?location-in-dishwasher))))

    ;; put cup from dishwasher onto table upside-down
    ;; (when (<= step 3)
    ;;   ;; let ((?goal `(cpoe:object-at-location ,?object ,?location-in-hand)))
    ;;   (exe:perform
    ;;    (an action
    ;;        (type transporting)
    ;;        (object (an object
    ;;                    (type jeroen-cup)
    ;;                    (name jeroen-cup-1)
    ;;                    (location ?location-in-dishwasher)))
    ;;        (target ?location-on-island-upside-down))))

    ;; regrasp cup
    ;; (when (<= step 4)
    ;;   (exe:perform
    ;;    (an action
    ;;        (type transporting)
    ;;        (object (an object
    ;;                    (type jeroen-cup)
    ;;                    (name jeroen-cup-1)
    ;;                    (location ?location-in-hand)))
    ;;        (target ?location-in-other-hand))))

    ;; bring cup to cupboard
    ;; (when (<= step 5)
    ;;   (exe:perform
    ;;    (an action
    ;;        (type transporting)
    ;;        (object (an object
    ;;                    (type jeroen-cup)
    ;;                    (name jeroen-cup-1)
    ;;                    ;; (location ?location-on-island-upside-down)
    ;;                    (location ?location-on-island)))
    ;;        (search-robot-location (a location
    ;;                                  (pose ?delivering-counter-top-robot-pose)))
    ;;        (fetch-robot-location (a location
    ;;                                 (pose ?delivering-counter-top-robot-pose)))
    ;;        (arms (right))
    ;;        (grasps (front))
    ;;        (access-deliver-outer-robot-location (a location
    ;;                                                (poses ?accessing-cupboard-door-robot-poses)))
    ;;        (access-seal-search-outer-arms (left))
    ;;        (access-search-outer-grasps (right-side))

    ;;        (target ?location-in-cupboard)
    ;;        (deliver-robot-location (a location
    ;;                                   (pose ?delivering-counter-top-robot-pose)))
    ;;        (seal-search-outer-robot-location (a location
    ;;                                             (pose ?sealing-cupboard-door-robot-pose)))
    ;;        (seal-search-outer-grasps (right-side)))))
    )

  (finalize))
