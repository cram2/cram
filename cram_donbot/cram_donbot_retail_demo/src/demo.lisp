;;;
;;; Copyright (c) 2018, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

   ;; perceive shelf system verhuetung base pose
   ;; #<CL-TRANSFORMS-STAMPED:TRANSFORM-STAMPED
   ;; FRAME-ID: "map", CHILD-FRAME-ID: "base_footprint", STAMP: 1.576594938305679d9
   ;; #<3D-VECTOR (2.2660367329915365d0 -0.16621163686116536d0 3.814697266402156d-9)>
   ;; #<QUATERNION (0.0d0 0.0d0 -0.020739689469337463d0 0.9997849464416504d0)>>


   ;; place on other shelf
   ;; #<CL-TRANSFORMS-STAMPED:TRANSFORM-STAMPED
   ;; FRAME-ID: "map", CHILD-FRAME-ID: "base_footprint", STAMP: 1.576596110886719d9
   ;; #<3D-VECTOR (2.6765769958496093d0 -0.13911641438802083d0 3.814697266402156d-9)>
   ;; #<QUATERNION (0.0d0 0.0d0 0.6886594891548157d0 0.7250849008560181d0)>>


(defun stuff-that-works ()

  (setf rob-int:*robot-urdf*
        (cl-urdf:parse-urdf
         (btr-belief::replace-all (roslisp:get-param btr-belief:*robot-parameter*) "\\" "  ")))


  (cram-process-modules:with-process-modules-running
      ( giskard:giskard-pm)
    (cpl-impl::named-top-level (:name :top-level)
      (exe:perform
       (let ((?pose (cl-tf:make-pose-stamped
                     "base_footprint" 0.0
                     (cl-transforms:make-3d-vector -0.27012088894844055d0 0.5643729567527771d0 1.25943687558174133d0)
                     (cl-tf:make-quaternion -0.4310053586959839d0 0.24723316729068756d0 0.752766489982605d0 0.4318017065525055d0 ))))
         (desig:a motion
                  (type moving-tcp)
                  (left-pose ?pose)
                  (collision-mode :allow-all))))))


  (cram-process-modules:with-process-modules-running
      (giskard:giskard-pm)
    (cpl-impl::named-top-level (:name :top-level)
      (exe:perform
       (desig:an action
                 (type positioning-arm)
                 (left-configuration park)))))

  (cram-process-modules:with-process-modules-running
      (giskard:giskard-pm)
    (cpl-impl::named-top-level (:name :top-level)
      (let ((?pose (cl-tf:pose->pose-stamped
                    "base_footprint" 0.0
                    (cl-transforms-stamped:make-identity-pose)
                    )))
        (exe:perform
         (desig:an action
                   (type going)
                   (target (desig:a location (pose ?pose))))))))

  (rs::call-robosherlock-service '((:shape :box) (:location "donbot_tray")) :quantifier :a)

  (rs::call-robosherlock-service '((:shape :box) (:location "shelf_system_verhuetung")) :quantifier :a)


  (cram-process-modules:with-process-modules-running
      (rs:robosherlock-perception-pm)
    (cpl-impl::named-top-level (:name :top-level)
      (let ((?robot-name (btr:get-robot-name)))
        (exe:perform
         (desig:an action
                   (type detecting)
                   (object (desig:an object
                                     (shape box)
                                     (location (desig:a location
                                                        (on (desig:an object
                                                                      (type
                                                                       ?robot-name)
                                                                      (urdf-name
                                                                       donbot-tray)
                                                                      (owl-name
                                                                       "shelf_system_verhuetung"))))))))))))





  (urdf-proj:with-simulated-robot
    (let* ((?robot-name (btr:get-robot-name))
           (?obj
             (perform (an action
                          (type detecting)
                          (object (desig:an object
                                            (type breakfast-cereal)
                                            (location (desig:a location
                                                               (on (desig:an object
                                                                             (type ?robot-name)))))))))))
      (sleep 0.4)
      (exe:perform (desig:an action
                             (type picking-up)
                             (grasp top)
                             (object ?obj)))))




  (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo")
  (btr-utils:spawn-object :my-box :breakfast-cereal :pose '((2.3 -0.2 0.8) (0 0 0 1)))
  (btr:simulate btr:*current-bullet-world* 50)
  (urdf-proj:with-simulated-robot
          (exe:perform
           (desig:an action
                     (type looking)
                     (direction down))))

  (let* ((?robot-name (btr:get-robot-name))
         (?obj
           (urdf-proj:with-simulated-robot
             (perform
              (an action
                  (type detecting)
                  (object (an object
                              (type breakfast-cereal)
                              (location (a location
                                           (on (an object
                                                   (type ?robot-name))))))))))))
    (sleep 0.4)
    (donbot-pm:with-real-robot
      (exe:perform (desig:an action
                             (type picking-up)
                             (grasp top)
                             (object ?obj))))))



#+everything-below-is-pr2-s-stuff-so-need-new-things-for-donbot
(
(defparameter *object-cad-models*
  '(;; (:cup . "cup_eco_orange")
    ;; (:bowl . "edeka_red_bowl")
    ))

(cpl:def-cram-function initialize-or-finalize ()
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
      (exe:perform (desig:an action (type opening-gripper) (gripper (left right))))
      (exe:perform (desig:an action (type looking) (direction forward))))))


(cpl:def-cram-function demo-random (&optional
                                    (random t)
                                    (list-of-objects '(:milk :cup :breakfast-cereal :bowl :spoon)))
  (btr:detach-all-objects (btr:get-robot-object))
  (btr-utils:kill-all-objects)

  ;; (setf proj-reasoning::*projection-reasoning-enabled* nil)
  (when (eql cram-projection:*projection-environment*
             'urdf-proj:urdf-bullet-projection-environment)
    (spawn-objects-on-sink-counter :random random))

  (setf cram-robot-pose-guassian-costmap::*orientation-samples* 1)

  (initialize-or-finalize)

  (dolist (?object-type list-of-objects)
    (let* ((?cad-model
             (cdr (assoc ?object-type *object-cad-models*)))
           (?object-to-fetch
             (desig:an object
                       (type ?object-type)
                       (desig:when ?cad-model
                         (cad-model ?cad-model))))
           (?fetching-location
             (desig:a location
                      (on "CounterTop")
                      (name "iai_kitchen_sink_area_counter_top")
                      (side left)))
           (?placing-target-pose
             (cl-transforms-stamped:pose->pose-stamped
              "map" 0.0
              (cram-bullet-reasoning:ensure-pose
               (cdr (assoc ?object-type *object-placing-poses*)))))
           (?arm-to-use
             (cdr (assoc ?object-type *object-grasping-arms*)))
           (?delivering-location
             (desig:a location
                      (pose ?placing-target-pose))))

      (cpl:with-failure-handling
          ((common-fail:high-level-failure (e)
             (roslisp:ros-warn (pp-plans demo) "Failure happened: ~a~%Skipping..." e)
             (return)))
        (exe:perform
         (desig:an action
                   (type transporting)
                   (object ?object-to-fetch)
                   ;; (arm ?arm-to-use)
                   (location ?fetching-location)
                   (target ?delivering-location))))))

  (initialize-or-finalize)

  cpl:*current-path*)

)
