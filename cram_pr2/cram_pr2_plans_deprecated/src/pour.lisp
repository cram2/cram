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
;;     (arm right)
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

(defun wait (duration)
  (roslisp:wait-duration duration))

(defun pour-activity (action-designator)
  (perform-phases-in-sequence action-designator))

(defun pick-and-pour-one-arm-plan (&key (?arm :right))
  (move-pr2-arms-out-of-sight :arm ?arm)
  (let* ((?bottle-desig (desig:an object
                                  (type bottle)))
         (?perceived-bottle-desig (exe:perform
                                   (desig:a motion
                                            (type detecting)
                                            (object ?bottle-desig)))))
    (exe:perform (desig:an action
                           (type picking-up)
                           (arm ?arm)
                           (object ?perceived-bottle-desig)))
    (move-pr2-arms-out-of-sight :arm :right)
    (let* ((?cup-desig (desig:an object
                                 (type cup)
                                 (cad-model "cup_eco_orange")))
           (?perceived-cup-desig (exe:perform
                                  (desig:a motion
                                           (type detecting)
                                           (object ?cup-desig)))))
      (exe:perform (desig:an action
                             (to pour-activity)
                             (arm ?arm)
                             (source ?perceived-bottle-desig)
                             (destination ?perceived-cup-desig)))
      (exe:perform (desig:an action
                             (to pour-activity)
                             (arm ?arm)
                             (source ?perceived-bottle-desig)
                             (destination ?perceived-cup-desig)))
      (move-pr2-arms-out-of-sight :arm ?arm))))

(defun pour-one-arm-plan (&key (?arm :right))
  (move-pr2-arms-out-of-sight :arm ?arm)
  (let* ((?cup-desig (desig:an object
                               (cad-model "cup_eco_orange")
                               (type cup)))
         (?perceived-cup-desig (exe:perform
                                (desig:a motion
                                         (type detecting)
                                         (object ?cup-desig))))
         (?bottle-desig (desig:an object
                                  (type bottle))))
    (exe:perform (desig:an action
                           (to pour-activity)
                           (arm ?arm)
                           (source ?bottle-desig)
                           (destination ?perceived-cup-desig)))
    (exe:perform (desig:an action
                           (to pour-activity)
                           (arm ?arm)
                           (source ?bottle-desig)
                           (destination ?perceived-cup-desig)))
    (move-pr2-arms-out-of-sight :arm ?arm)))

(defun giskard-yaml (phase constraints)
  ;; TODO: change this to (a motion (type moving-joints) (constraints ?yaml-string))
  ;; (pr2-ll::call-giskard-yaml-action
  ;;  :yaml-string (constraints-into-controller (read-yaml-file phase) constraints)
  ;;  :action-timeout 60.0)
  )

(defun go-to-initial-pouring-configuration-plan ()
  ;;  pouring initial configuration
  (cpl:with-failure-handling
      ((common-fail:low-level-failure (e)
         (declare (ignore e))
         (return)))
    (let ((?left-initial-configuration *pr2-left-arm-pouring-joint-positions*)
          (?right-initial-configuration *pr2-right-arm-pouring-joint-positions*))
      (exe:perform (desig:a motion
                            (type moving-joints)
                            (left-configuration ?left-initial-configuration)
                            (right-configuration ?right-initial-configuration))))))


;; Example input designator for pouring (learned constraints server must be running):
;; (with-pr2-process-modules
;;              (reference (desig:an action
;;                      (to my-pour)
;;                      (arm (left right))
;;                      (source (desig:an object (type bottle) (pose ((pose ?pr2-right-arm-out-of-sight-gripper-pose)))))
;;                      (target (desig:an object (type cup) (pose ((pose ?pr2-left-arm-out-of-sight-gripper-pose))))))))
