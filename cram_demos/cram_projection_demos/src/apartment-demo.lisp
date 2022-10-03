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
     ((0.15 -0.1 0.08) (0 0 -1 0)))
    (:bowl
     "island_countertop"
     ((0.34 0.5 0.1) (0 0 0 -1)))))

(defparameter *apartment-object-spawning-poses-pouring*
  '((:jeroen-cup
     "island_countertop"
     ((0.34 -0.5 0.1) (0 0 0 -1)))
    (:bowl
     "island_countertop"
     ((0.34 0.5 0.1) (0 0 0 -1)))))

(defun initialize-apartment ()
  (sb-ext:gc :full t)

  ;;(when ccl::*is-logging-enabled*
  ;;    (setf ccl::*is-client-connected* nil)
  ;;    (ccl::connect-to-cloud-logger)
  ;;    (ccl::reset-logged-owl))

  ;; (setf proj-reasoning::*projection-checks-enabled* t)

  (kill-and-detach-all)
  (giskard:reset-collision-scene)
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
         (?location-in-cupboard-with-attachment
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
              (for ?object)
              (attachments (jeroen-cup-on-shelf))))
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
         ;;              (type robot)))))
         ;; (?location-in-other-hand
         ;;   (a location
         ;;      (in (an object
         ;;              (type robot)))))
         (?location-on-island-upside-down
           (a location
              (on (an object
                      (type surface)
                      (urdf-name island-countertop)
                      (part-of apartment)))
              (side (right back))
              (range 0.5)
              (orientation upside-down)
              (for (an object
                       (type jeroen-cup)
                       (name jeroen-cup-1)))))


         ;; hard-coded stuff for real-world demo
         (?initial-parking-pose
           (cl-transforms-stamped:make-pose-stamped
            cram-tf:*fixed-frame*
            0.0
            (cl-transforms:make-3d-vector 1.5 1.5 0.0)
            (cl-transforms:make-quaternion 0 0 0.5 0.5)))

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
            (cl-transforms:make-3d-vector 1.1 2.002821242371188d0 0.0d0)
            (cl-transforms:make-quaternion 0.0d0 0.0d0 -1 0)))
         ;; (?fetching-cupboard-right-hand-robot-pose
         ;;   (cl-transforms-stamped:make-pose-stamped
         ;;    "map"
         ;;    0.0
         ;;    (cl-transforms:make-3d-vector 1.3668892503154677d0 1.6951934636420103d0 0.0d0)
         ;;    (cl-transforms:make-quaternion 0.0d0 0.0d0 0.9298226390305793d0
         ;;                                   -0.36800561314986846d0)))
         (?sealing-cupboard-door-robot-pose
           (cl-transforms-stamped:make-pose-stamped
            "map"
            0.0
            (cl-transforms:make-3d-vector 1.6 1.0873920171726708 0.0)
            (cl-transforms:make-quaternion 0.0d0 0.0d0 -1 0)))
         (?sealing-cupboard-door-another-robot-pose
           (cl-transforms-stamped:make-pose-stamped
            "map"
            0.0
            (cl-transforms:make-3d-vector 1.7 1.0873920171726708 0.0)
            (cl-transforms:make-quaternion 0.0d0 0.0d0 -1 0)))
         (?sealing-cupboard-door-robot-poses
           (list ?sealing-cupboard-door-robot-pose
                 ?sealing-cupboard-door-another-robot-pose))

         (?on-counter-top-cup-pose
           (cl-transforms-stamped:make-pose-stamped
            "map"
            0.0
            (cl-transforms:make-3d-vector 2.37 2.6 1.0126)
            (cl-transforms:make-quaternion 0 0 1 0)))
         (?delivering-counter-top-robot-pose
           (cl-transforms-stamped:make-pose-stamped
            "map"
            0.0
            (cl-transforms:make-3d-vector 1.8 2.8 0.0d0)
            (cl-transforms:make-quaternion 0.0d0 0.0d0 -0.1 1)))
         (?fetching-counter-top-robot-pose
           (cl-transforms-stamped:make-pose-stamped
            "map"
            0.0
            (cl-transforms:make-3d-vector 1.7642620086669922d0 2.8088844299316404d0 0.0d0)
            (cl-transforms:make-quaternion 0.0d0 0.0d0 -0.145986869931221d0 0.9892865419387817d0)))
         (?on-counter-top-cup-upsidedown-pose
           (cl-transforms-stamped:make-pose-stamped
            "map"
            0.0
            (cl-transforms:make-3d-vector 2.37 2.6 1.0126)
            (cl-transforms:make-quaternion 0 1 0 0)))
         (?on-counter-top-cup-look-pose
           (cl-transforms-stamped:make-pose-stamped
            "map"
            0.0
            (cl-transforms:make-3d-vector 2.1 2.6 1.0126)
            (cl-transforms:make-quaternion 0 1 0 0)))

         (?accessing-dishwasher-door-robot-pose
           (cl-transforms-stamped:make-pose-stamped
            "map"
            0.0
            (cl-transforms:make-3d-vector 1.7 3.0 0.0d0)
            (cl-transforms:make-quaternion 0.0d0 0.0d0 0 1)))
         (?accessing-dishwasher-drawer-robot-pose
           (cl-transforms-stamped:make-pose-stamped
            "map"
            0.0
            (cl-transforms:make-3d-vector 1.48 3.71 0.0d0)
            (cl-transforms:make-quaternion 0.0d0 0.0d0 0.071 0.9975)))
         (?sealing-dishwasher-drawer-robot-pose
           (cl-transforms-stamped:make-pose-stamped
            "map"
            0.0
            (cl-transforms:make-3d-vector 1.419999599456787d0 3.619999408721924d0 0.0d0)
            (cl-transforms:make-quaternion 0.0d0 0.0d0 0.09318785493943212d0 0.9956485442623755d0)))
         (?deliver-dishwasher-robot-pose
           (cl-transforms-stamped:make-pose-stamped
            "map"
            0.0
            (cl-transforms:make-3d-vector 1.6 3.4182098388671873d0 0.0d0)
            (cl-transforms:make-quaternion 0.0d0 0.0d0 0.555830717086792d0 0.8312955498695374d0))))

    ;; park robot into the initial position
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
      (park-robot ?initial-parking-pose))

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
                         (color blue)
                         (name jeroen-cup-1)
                         (location ?location-in-cupboard)))
             (access-search-outer-robot-location (a location
                                                    (poses ?accessing-cupboard-door-robot-poses)))
             (access-seal-search-outer-arms (left))
             (access-search-outer-grasps (back))
             (search-robot-location (a location
                                       (pose ?detecting-cupboard-robot-pose
                                             ;; ?fetching-cupboard-right-hand-robot-pose
                                             )))
             (fetch-robot-location (a location
                                      (pose ?detecting-cupboard-robot-pose
                                            ;; ?fetching-cupboard-right-hand-robot-pose
                                            )))
             (arms (left
                    ;; right
                    ))
             (grasps (front))
             (target (a location
                        (pose ?on-counter-top-cup-pose))
                     ;; ?location-on-island
                     )
             (deliver-robot-location (a location
                                        (pose ?delivering-counter-top-robot-pose)))
             (seal-search-outer-robot-location (a location
                                                  (poses ?sealing-cupboard-door-robot-poses)))
             (seal-search-outer-grasps (back))
             (goal ?goal)))))

    ;; put cup from island into dishwasher
    (when nil ; (<= step 2)
      (exe:perform
       (an action
           (type transporting)
           (object (an object
                       (type jeroen-cup)
                       (color blue)
                       (name jeroen-cup-1)
                       (location ?location-on-island)))
           (target ?location-in-dishwasher)

           (access-deliver-robot-location (a location
                                             (pose ?accessing-dishwasher-drawer-robot-pose)))
           (seal-deliver-robot-location (a location
                                           (pose ?sealing-dishwasher-drawer-robot-pose)))
           (access-seal-deliver-arms (left))
           (access-seal-deliver-grasps (back))

           (access-deliver-outer-robot-location (a location
                                                   (pose ?accessing-dishwasher-door-robot-pose)))
           (seal-deliver-outer-robot-location (a location
                                                 (pose ?accessing-dishwasher-door-robot-pose)))
           (access-seal-deliver-outer-arms (left))
           (access-seal-deliver-outer-grasps (back))

           (search-robot-location (a location
                                     (pose ?fetching-counter-top-robot-pose)))
           (fetch-robot-location (a location
                                    (pose ?fetching-counter-top-robot-pose)))
           (deliver-robot-location (a location
                                      (pose ?deliver-dishwasher-robot-pose)))
           (arms (right))
           (grasps (front)))))

    ;; put cup from dishwasher onto table upside-down
    (when nil ; (<= step 3)
      ;; let ((?goal `(cpoe:object-at-location ,?object ,?location-in-hand)))
      (exe:perform
       (an action
           (type transporting)
           (object (an object
                       (type jeroen-cup)
                       (color blue)
                       (name jeroen-cup-1)
                       (location ?location-in-dishwasher)))
           (grasps (bottom))
           (arms (right))

           (deliver-robot-location (a location
                                      (pose ?delivering-counter-top-robot-pose)))
           ;; (target ?location-on-island-upside-down)
           (target (a location
                      (pose ?on-counter-top-cup-upsidedown-pose)))

           (access-search-robot-location (a location
                                            (pose ?accessing-dishwasher-drawer-robot-pose)))
           (seal-search-robot-location (a location
                                          (pose ?sealing-dishwasher-drawer-robot-pose)))
           (access-seal-search-arms (left))
           (access-seal-search-grasps (back))

           (access-search-outer-robot-location (a location
                                                  (pose ?accessing-dishwasher-door-robot-pose)))
           (seal-search-outer-robot-location (a location
                                                (pose ?accessing-dishwasher-door-robot-pose)))
           (access-seal-search-outer-arms (left))
           (access-search-outer-grasps (back))
           (seal-search-outer-grasps (back)))))

    ;; bring cup to cupboard
    (when nil ; (<= step 4)
      (exe:perform
       (an action
           (type transporting)
           (object (an object
                       (type jeroen-cup)
                       (name jeroen-cup-1)
                       (color blue)
                       ;; (location ?location-on-island-upside-down)
                       (location (a location
                                    (pose ;; ?on-counter-top-cup-upsidedown-pose
                                          ?on-counter-top-cup-look-pose)))))

           (access-deliver-outer-robot-location (a location
                                                   (poses ?accessing-cupboard-door-robot-poses)))
           (seal-deliver-outer-robot-location (a location
                                                 (poses ?sealing-cupboard-door-robot-poses)))
           (access-seal-deliver-outer-arms (left))
           (access-seal-deliver-outer-grasps (back))

           (search-robot-location (a location
                                     (poses (?delivering-counter-top-robot-pose
                                             ?fetching-counter-top-robot-pose))))
           (fetch-robot-location (a location
                                    (poses (?delivering-counter-top-robot-pose
                                            ?fetching-counter-top-robot-pose))))
           (arms (left))
           (grasps (front))

           (target ?location-in-cupboard-with-attachment)
           (deliver-robot-location (a location
                                      (pose ?detecting-cupboard-robot-pose
                                            ;; ?initial-parking-pose
                                            )))))))

  ;; (finalize)
  )



(defun apartment-demo-pouring-only (&key (step 0))
  "copy and pasted stuff from apartment-demo above so i can later just add the pouring part in the real demo"
  ;;(urdf-proj:with-simulated-robot
  (setf proj-reasoning::*projection-checks-enabled* t)
  (setf btr:*visibility-threshold* 0.7)
  (let* ((?source-object
           (an object
               (type jeroen-cup)
               (name jeroen-cup-1)))

         (?location-on-island
           (a location
              (on (an object
                      (type surface)
                      (urdf-name island-countertop)
                      (part-of apartment)))
              (side back)
              (range 0.4)
              (range-invert 0.3)
              (for ?source-object)))

         (?target-object
           (an object
               (type bowl)
               (name bowl-1)))

         (?location-on-island-target
           (a location
              (on (an object
                      (type surface)
                      (urdf-name island-countertop)
                      (part-of apartment)))
              (side back)
              (range 0.4)
              (range-invert 0.3)
              (for ?target-object)))

         ;; hard-coded stuff for real-world demo
         (?initial-parking-pose
           (cl-transforms-stamped:make-pose-stamped
            cram-tf:*fixed-frame*
            0.0
            (cl-transforms:make-3d-vector 1.6 3.3 0.0)
            (cl-transforms:make-quaternion 0 0 0 1)))

         (?second-cup-park-pose
           (cl-transforms-stamped:make-pose-stamped
            cram-tf:*fixed-frame*
            0.0
            (cl-transforms:make-3d-vector 1.6 2.4 0.0)
            (cl-transforms:make-quaternion 0 0 0 1)))

         (?third-cup-park-pose
           (cl-transforms-stamped:make-pose-stamped
            "map"
            0.0
            (cl-transforms:make-3d-vector 1.6867786693573 2.1811975717544556 0)
            (cl-transforms:make-quaternion 0 0 0 1)))

         (?on-counter-top-source-cup-look-pose
           (cl-transforms-stamped:make-pose-stamped
            "map"
            0.0
            (cl-transforms:make-3d-vector 2.49 3.0 1.0126)
            (cl-transforms:make-quaternion 0 1 0 0)))

         (?on-counter-top-target-cup-look-pose
           (cl-transforms-stamped:make-pose-stamped
            "map"
            0.0
            (cl-transforms:make-3d-vector 2.49 2.2 1.0126)
            (cl-transforms:make-quaternion 0 1 0 0))))

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
         :object-types '(:jeroen-cup :bowl)
         :spawning-poses-relative *apartment-object-spawning-poses-pouring*))
      (park-robot ?initial-parking-pose))

    ;; get all the object designators
    ;;we pour from red to blue so red=source blue=target
    (when (<= step 1)
      (park-robot ?initial-parking-pose)
      (exe:perform
       (desig:an action
                 (type going)
                 (target (desig:a location
                                  (pose ?initial-parking-pose)))))

      (exe:perform (desig:a motion
                            (type looking)
                            (pose ?on-counter-top-source-cup-look-pose)))

      (let* ((?source-object-desig
               (desig:an object
                         (type jeroen-cup)
                         (color blue)
                         (name jeroen-cup-1)
                         (location ?location-on-island)))

             (?source-perceived-object-desig
               (exe:perform (desig:an action
                                      (type detecting)
                                      (object ?source-object-desig)))))

        ;;TODO: Pick-up has parking in the end this need to be changed
        ;;or with constraints
        (cpl:with-retry-counters ((giskardside-retries 3))
          (cpl:with-failure-handling
              (((or common-fail:manipulation-low-level-failure
                    common-fail:manipulation-goal-not-reached
                    common-fail:gripper-closed-completely) (e)
                 (roslisp:ros-warn (pp-plans pour-reach)
                                   "Manipulation messed up: ~a~%Failing."
                                   e)

                 (cpl:do-retry giskardside-retries
                   (cpl:retry))
                 (return)))

            (exe:perform (desig:an action
                                   (type picking-up)
                                   (arm :right)
                                   (grasp :front)
                                   (object ?source-perceived-object-desig)
                                   (park-arms NIL)))))))

    ;; (exe:perform (desig:an action
    ;;                        (type fetching)
    ;;                        (arm :left)
    ;;                        (grasp :front)
    ;;                        (object ?source-perceived-object-desig)
    ;;                        ))
    ;;)
    ;;)
    (when (<= step 2)
      (exe:perform
       (desig:an action
                 (type positioning-arm)
                 (left-configuration park)
                 (right-configuration park)))

      ;; (exe:perform
      ;;  (desig:an action
      ;;            (type going)
      ;;            (target (desig:a location
      ;;                             (pose ?second-cup-park-pose)))))

      (exe:perform
       (desig:an action
                 (type going)
                 (target (desig:a location
                                  (pose ?third-cup-park-pose))))))

    (when (<= step 3)
      (exe:perform (desig:an action
                             (type moving-torso)
                             (joint-angle lower-limit)))

      (exe:perform (desig:a motion
                            (type looking)
                            (pose ?on-counter-top-target-cup-look-pose)))

      (let* ((?target-object-desig
               (desig:an object
                         (type bowl)
                         (color red)
                         (name bowl-1)
                         (location ?location-on-island-target)))

             (?target-perceived-object-desig
               (exe:perform (desig:an action
                                      (type detecting)
                                      (object ?target-object-desig)))))
        (exe:perform (desig:an action
                               (type moving-torso)
                               (joint-angle upper-limit)))

        (exe:perform
         (desig:an action
                   (type pouring)
                   ;; (object ?source-perceived-object-desig)
                   (on-object ?target-perceived-object-desig)
                   (arm :right)
                   (sides (:top-right))
                   (wait-duration 5)))))))

;;
;; (when (<= step 2)

;;      (?target-object-desig
;;        (desig:an object
;;                  (type jeroen-cup)
;;                  (color blue)
;;                  (name jeroen-cup-1)
;;                  (location ?location-on-island)))
;;         (?target-perceived-object-desig
;;        (exe:perform (desig:an action
;;                               (type detecting)
;;                               (object ?target-object-desig)))))
;;   (exe:perform (desig:a motion
;;                         (type looking)
;;                         (pose ?on-counter-top-bowl-look-pose)))


;; (let* ((?target-object-desig
;;          (desig:an object
;;                    (type jeroen-cup)
;;                    (color blue)
;;                    (name jeroen-cup-1)
;;                    (location ?location-on-island)))
;;        (?target-perceived-object-desig
;;          (exe:perform (desig:an action
;;                                 (type detecting)
;;                                 (object ?target-object-desig)))))
;;   ?perceived-object-desig



(defun apartment-demo-merged (&key (step 0))
  ;;urdf-proj:with-simulated-robot
  (setf proj-reasoning::*projection-checks-enabled* nil)
  (setf btr:*visibility-threshold* 0.7)

  (let* ((?on-counter-top-cup-pose
           (cl-transforms-stamped:make-pose-stamped
            "map"
            0.0
            (cl-transforms:make-3d-vector 2.37 2.6 1.0126)
            (cl-transforms:make-quaternion 0 0 1 0)))

         ;;hardcoded pouring stuff

         (?source-object
           (an object
               (type jeroen-cup)
               (name jeroen-cup-1)))

         (?location-on-island
           (a location
              (on (an object
                      (type surface)
                      (urdf-name island-countertop)
                      (part-of apartment)))
              (side back)
              (range 0.4)
              (range-invert 0.3)
              (for ?source-object)))

         (?target-object
           (an object
               (type bowl)
               (name bowl-1)))

         (?location-on-island-target
           (a location
              (on (an object
                      (type surface)
                      (urdf-name island-countertop)
                      (part-of apartment)))
              (side back)
              (range 0.4)
              (range-invert 0.3)
              (for ?target-object)))

         ;; hard-coded stuff for real-world demo
         (?percieve-blue-cup-pose-pouring
           (cl-transforms-stamped:make-pose-stamped
            cram-tf:*fixed-frame*
            0.0
            (cl-transforms:make-3d-vector 1.6 2.5 0.0)
            (cl-transforms:make-quaternion 0 0 0 1)))

         (?on-counter-top-source-cup-look-pose
           (cl-transforms-stamped:make-pose-stamped
            "map"
            0.0
            (cl-transforms:make-3d-vector 2.49 2.6 1.0126)
            (cl-transforms:make-quaternion 0 1 0 0)))

         (?on-counter-top-target-cup-look-pose
           (cl-transforms-stamped:make-pose-stamped
            "map"
            0.0
            (cl-transforms:make-3d-vector 2.49 2.2 1.0126)
            (cl-transforms:make-quaternion 0 1 0 0))))

    (apartment-demo :step step)

    ;;pick-up-cup
    (when (<= step 2)

      (exe:perform
       (desig:an action
                 (type going)
                 (target (desig:a location
                                  (pose ?percieve-blue-cup-pose-pouring)))))

      (exe:perform (desig:a motion
                            (type looking)
                            (pose ?on-counter-top-source-cup-look-pose)))


      (let* ((?source-object-desig
               (desig:an object
                         (type jeroen-cup)
                         (color blue)
                         (name jeroen-cup-1)
                         (location ?location-on-island)))

             (?source-perceived-object-desig
               (exe:perform (desig:an action
                                      (type detecting)
                                      (object ?source-object-desig))))


             (?target-object-desig
               (desig:an object
                         (type bowl)
                         (color red)
                         (name bowl-1)
                         (location ?location-on-island-target)))

             (?target-perceived-object-desig
               (cpl::seq
                 (exe:perform (desig:a motion
                                       (type looking)
                                       (pose ?on-counter-top-target-cup-look-pose)))

                 (exe:perform (desig:an action
                                        (type detecting)
                                        (object ?target-object-desig))))))


        ;;TODO: Pick-up has parking in the end this need to be changed
        ;;or with constraints
        (cpl:with-retry-counters ((giskardside-retries 3))
          (cpl:with-failure-handling
              (((or common-fail:manipulation-low-level-failure
                    common-fail:manipulation-goal-not-reached
                    common-fail:gripper-closed-completely) (e)
                 (roslisp:ros-warn (pp-plans pour-reach)
                                   "Manipulation messed up: ~a~%Failing."
                                   e)

                 (cpl:do-retry giskardside-retries
                   (cpl:retry))
                 (return)))


            (exe:perform (desig:an action
                                   (type picking-up)
                                   (arm :right)
                                   (grasp :front)
                                   (object ?source-perceived-object-desig)
                                   (park-arms NIL)))))

        (exe:perform
         (desig:an action
                   (type pouring)
                   (on-object ?target-perceived-object-desig)
                   (arm :right)
                   (sides (:top-right))
                   (wait-duration 5)))))

    (exe:perform
     (desig:an action
               (type placing)
               (arm right)
               (object ?source-object)
               (target (a location
                          (pose ?on-counter-top-cup-pose)))))))

