;;;
;;; Copyright (c) 2016, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
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

(in-package :pr2-plans)

;; (an action
;;     (to pour)
;;     (source (an object))
;;     (target (an object))
;;   -  (theme (some stuff))
;;   -  (goal drink-poured)
;;   ?  (arm right)
;;     (phases (a motion (type approaching))
;;             (a motion
;;                (type tilt-down)
;;                (angle )
;;                ())
;;             (a motion (type tilt-back)
;;                       (constraints (source-top-behind-target-top 12 23)
;;                                    (source-top-left-target-top 0.04 0.57)
;;             (a motion (type retracting))))



;; Input: cup in base_link, bottle in base_link
  ;;  name: tilt_back
  ;;   constraints: 
  ;;     - 
  ;;       name: source_left_itself
  ;;       lower: 0.17160918768
  ;;       upper: 0.175651417877
  ;;     - 
  ;;       name: source_top_above_goal_target_top ; source_top_above_target_top
  ;;       lower: 0.0506713657833
  ;;       upper: 0.0601368737079
  ;;     - 
  ;;       name: source_above_itself            ; source_top_above_source_bottom
  ;;       lower: -0.0355860225202
  ;;       upper: -0.0228787381723
  ;;     - 
  ;;       name: source_behind_itself
  ;;       lower: -0.0435688405115
  ;;       upper: -0.0299401394795
  ;;     - 
  ;;       name: source_top_behind_goal_target_top
  ;;       lower: -0.0123281616118
  ;;       upper: -0.00443432333723
  ;;     - 
  ;;       name: source_top_left_goal_target_top
  ;;       lower: -0.0175524394121
  ;;       upper: -0.0129251489616
  ;; - 
  ;;   name: tilt_down
  ;;   constraints: 
  ;;     - 
  ;;       name: source_left_itself
  ;;       lower: 0.166978793623
  ;;       upper: 0.171221504047
  ;;     - 
  ;;       name: source_top_above_goal_target_top
  ;;       lower: 0.0351830346321
  ;;       upper: 0.0424454599667
  ;;     - 
  ;;       name: source_above_itself
  ;;       lower: -0.0565096525695
  ;;       upper: -0.0511978531642
  ;;     - 
  ;;       name: source_behind_itself
  ;;       lower: -0.0409957359998
  ;;       upper: -0.025745088615
  ;;     - 
  ;;       name: source_top_behind_goal_target_top
  ;;       lower: -0.00946009677249
  ;;       upper: -0.00202902833224
  ;;     - 
  ;;       name: source_top_left_goal_target_top
  ;;       lower: -0.0133496650731
  ;;       upper: -0.0102149032488
  ;; - 
  ;;   name: move_above
  ;;   constraints: 
  ;;     - 
  ;;       name: source_left_itself
  ;;       lower: 0.171364527114
  ;;       upper: 0.175911635164
  ;;     - 
  ;;       name: source_top_above_goal_target_top
  ;;       lower: 0.0562834368678
  ;;       upper: 0.0680672259882
  ;;     - 
  ;;       name: source_above_itself
  ;;       lower: -0.0347799722306
  ;;       upper: -0.0236473652741
  ;;     - 
  ;;       name: source_behind_itself
  ;;       lower: -0.0429654419749
  ;;       upper: -0.0277158142335
  ;;     - 
  ;;       name: source_top_behind_goal_target_top
  ;;       lower: -0.00826726491721
  ;;       upper: -0.000473181881838
  ;;     - 
  ;;       name: source_top_left_goal_target_top
  ;;       lower: -0.0168588836555
  ;;       upper: -0.0109408055036


;; soft-constraints:
;; - soft-constraint:
;;   - double-sub: [-0.036, mug-top-behind-maker]
;;   - double-sub: [0.0051, mug-top-behind-maker]
;;   - constraint-weight
;;   - mug-top-behind-maker
;; - soft-constraint:
;;   - double-sub: [-0.038, mug-top-left-maker]
;;   - double-sub: [-0.011, mug-top-left-maker]
;;   - constraint-weight
;;   - mug-top-left-maker
;; - soft-constraint:
;;   - double-sub: [0.1767, mug-top-above-maker]
;;   - double-sub: [0.2181, mug-top-above-maker]
;;   - constraint-weight
;;   - mug-top-above-maker
;; - soft-constraint:
;;   - double-sub: [-0.0768, mug-behind-itself] mug-top-behind-mug-bottom
;;   - double-sub: [-0.0354, mug-behind-itself]
;;   - constraint-weight
;;   - mug-behind-itself
;; - soft-constraint:
;;   - double-sub: [0.0698, mug-left-itself] 
;;   - double-sub: [0.0981, mug-left-itself]
;;   - constraint-weight
;;   - mug-left-itself
;; - soft-constraint:
;;   - double-sub: [-0.0394, mug-above-itself]
;;   - double-sub: [-0.0206, mug-above-itself]
;;   - constraint-weight
;;   - mug-above-itself

(defun top-level-pick-and-pour (&key (?arm :right))
  (move-pr2-arms-out-of-sight :arm :right)
  (let* ((?bottle-desig (desig:an object
                                  (type bottle)))
         (?perceived-bottle-desig (cram-plan-library:perform
                                   (desig:an action
                                             (to detect-motion)
                                             (object ?bottle-desig)))))
    (plan-lib:perform (desig:an action
                                (to my-pick-up)
                                (arm ?arm)
                                (object ?perceived-bottle-desig)))
    (move-pr2-arms-out-of-sight :arm :right)
    (let* ((?cup-desig (desig:an object
                                 ;; (type cup)
                                 (cad-model "cup_eco_orange")))
           (?perceived-cup-desig (cram-plan-library:perform
                                  (desig:an action
                                            (to detect-motion)
                                            (object ?cup-desig)))))
      (plan-lib:perform (desig:an action
                                  (to my-pour)
                                  (arm ?arm)
                                  (source ?perceived-bottle-desig)
                                  (destination ?perceived-cup-desig)))
      (plan-lib:perform (desig:an action
                                  (to my-pour)
                                  (arm ?arm)
                                  (source ?perceived-bottle-desig)
                                  (destination ?perceived-cup-desig)))
      (move-pr2-arms-out-of-sight :arm :right))))

(defun top-level-pour (&key (?arm :right))
  (move-pr2-arms-out-of-sight :arm :right)
  (let* ((?cup-desig (desig:an object
                               (cad-model "cup_eco_orange")
                               ;; (type cup)
                               ))
         (?perceived-cup-desig (cram-plan-library:perform
                                (desig:an action
                                          (to detect-motion)
                                          (object ?cup-desig))))
         (?bottle-desig (desig:an object
                                  (type bottle))))
    (plan-lib:perform (desig:an action
                                (to my-pour)
                                (arm ?arm)
                                (source ?bottle-desig)
                                (destination ?perceived-cup-desig)))
    (plan-lib:perform (desig:an action
                                (to my-pour)
                                (arm ?arm)
                                (source ?bottle-desig)
                                (destination ?perceived-cup-desig)))
    (move-pr2-arms-out-of-sight :arm :right)))

(defparameter *constraints* '((:approach
                               (name . (lower upper)) (name lower upper))
                              (:tilt
                               (constraint-1 23 34) (constraint-2 45 47))
                              (:tilt-back
                               (name 23 5))))

(defun georgs-function (yaml-string list-of-constraints)
  (declare (type string yaml-string)
           (type list list-of-constraints))
  "The list of constraints looks like the following:
'((constraint-1 23 34)
  (constraint-2 45 47))"
  yaml-string)

(defun put-together-yaml (phase constraints)
  (georgs-function
   (read-yaml-file phase)
   constraints))

(defun giskard-yaml (phase constraints)
  (pr2-ll::call-giskard-yaml-action
   :yaml-string (put-together-yaml phase constraints)
   :action-timeout 60.0))

(defun top-level-two-arm-pour ()
  ;; prepare
  (let ((?navigation-goal *meal-table-right-base-pose*)
        (?ptu-goal *meal-table-right-base-look-pose*))
    (cpl:par
      (move-pr2-arms-out-of-sight)
      (plan-lib:perform (desig:an action
                                  (to go-motion)
                                  (to ?navigation-goal)))
      (plan-lib:perform (desig:an action
                                  (to look-motion)
                                  (at ?ptu-goal)))))
  ;; 
  (let* ((?bottle-desig (desig:an object
                                  (type bottle)))
         (?perceived-bottle-desig (cram-plan-library:perform
                                   (desig:an action
                                             (to detect-motion)
                                             (object ?bottle-desig)))))
    (plan-lib:perform (desig:an action
                                (to my-pick-up)
                                (arm right)
                                (object ?perceived-bottle-desig)))
    (move-pr2-arms-out-of-sight :arm :right)
    (let* ((?cup-desig (desig:an object
                                 (type cup)
                                 (cad-model "cup_eco_orange")))
           (?perceived-cup-desig (cram-plan-library:perform
                                  (desig:an action
                                            (to detect-motion)
                                            (object ?cup-desig)))))
      (plan-lib:perform (desig:an action
                                  (to my-pick-up)
                                  (arm left)
                                  (object ?perceived-cup-desig)))
      (plan-lib:perform (desig:an action
                                  (to pour-activity)
                                  (arm (left right))
                                  (source ?perceived-bottle-desig)
                                  (target ?perceived-cup-desig)
                                  (pour-volume 0.0002)))
      (cpl:par
        (top-level-place :?arm :left :?object ?perceived-cup-desig)
        (top-level-place :?arm :right :?object ?perceived-bottle-desig))
      (move-pr2-arms-out-of-sight :arm :right))))


;; Example input designator for pouring (learned constraints server must be running):
;; (with-pr2-process-modules
;;              (reference (desig:an action
;;                      (to my-pour)
;;                      (arm (left right))
;;                      (source (desig:an object (type bottle) (pose ((pose *pr2-right-arm-out-of-sight-gripper-pose*)))))
;;                      (target (desig:an object (type cup) (pose ((pose *pr2-left-arm-out-of-sight-gripper-pose*))))))))
