;;;
;;; Copyright (c) 2022, Vanessa Hassouna <hassouna@uni-bremen.de>
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
  '((:popcorn-pot
     "cabinet1_drawer_middle"
     ((0.0 0.0 -0.02) (0 0 0 1)))))

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

(defun apartment-popcorn-demo (&key (step 0))
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
       :object-types '(:popcorn-pot)
       :spawning-poses-relative *apartment-object-spawning-poses*))
    (park-robot (cl-transforms-stamped:make-pose-stamped
                 cram-tf:*fixed-frame*
                 0.0
                 (cl-transforms:make-3d-vector 1.5 1.5 0.0)
                 (cl-transforms:make-quaternion 0 0 0.5 0.5))))
    
   (let* ((?object
           (an object
               (type jeroen-cup)
               (name jeroen-cup-1)))
         (?location-in-cupboard
           (a location
              (on (an object
                      (type shelf)
                      (urdf-name cabinet1_drawer_middle)
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
	  )
     
    ;; bring the pot to the stove
    (when (<= step 1)
      (let ((?goal `(and (cpoe:object-at-location ,(an object
                                                       (type popcorn-pot)
                                                       (name popcorn-pot-1))
                                                  ,(a location
                                                      (pose ?on-counter-top-cup-pose)))
                         (cpoe:location-reset ,?location-in-cupboard))))
        (exe:perform
         (an action
             (type transporting)
             (object (an object
                                (type popcorn-pot)
                                (name popcorn-pot-1))
                         (location ?location-in-cupboard)))
             (access-search-outer-robot-location (a location
                                                    (poses ?accessing-cupboard-door-robot-poses)))
             (access-seal-search-outer-arms (left))
             (access-search-outer-grasps (back))
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
             (seal-search-outer-grasps (back))
             (goal ?goal)))))

