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

(defun step-0-prepare ()
  (cpl:with-failure-handling
      ((common-fail:low-level-failure (e)
         (roslisp:ros-warn (demo step-0) "~a" e)
         (return)))

    (let ((?navigation-goal *meal-table-right-base-pose*)
          (?ptu-goal *meal-table-right-base-look-pose*))
      (cpl:par
        (move-pr2-arms-out-of-sight)
        (exe:perform (desig:a motion
                              (type going)
                              (target (desig:a location (pose ?navigation-goal)))))
        (exe:perform (desig:a motion
                              (type looking)
                              (target (desig:a location (pose ?ptu-goal)))))))))

(defun step-1-grasp-bottle ()
  (flet ((step-1-inner ()
           (let* ((?bottle-desig (desig:an object
                                           (type bottle)))
                  (?perceived-bottle-desig (perceive ?bottle-desig)))
             (drive-and-pick-up-plan ?perceived-bottle-desig :?arm :right))))
    (cpl:with-retry-counters ((bottle-grasp-tries 2))
      (cpl:with-failure-handling
          ((common-fail:low-level-failure (e)
             (roslisp:ros-warn (demo step-1) "~a" e)
             (if (get-object-in-hand :right)
                 (return)
                 (cpl:do-retry bottle-grasp-tries
                   (roslisp:ros-warn (demo step-1) "~a" e)
                   (step-0-prepare)
                   (cpl:retry)))))

        (step-1-inner)))))

(defun step-2-grasp-first-cup ()
  (flet ((step-2-inner ()
           (move-pr2-arms-out-of-sight)
           (let* ((?cup-desig (desig:an object
                                        (type cup)
                                        (color "yellow")
                                        (cad-model "cup_eco_orange")))
                  (?first-cup-to-pour (perceive ?cup-desig
                                                :quantifier :all
                                                :object-chosing-function
                                                (lambda (desigs)
                                                  (car (last desigs))))))
             (drive-and-pick-up-plan ?first-cup-to-pour :?arm :left))))

    (cpl:with-retry-counters ((bottle-grasp-tries 2))
      (cpl:with-failure-handling
          ((common-fail:low-level-failure (e)
             (roslisp:ros-warn (demo step-2) "~a" e)
             (if (get-object-in-hand :left)
                 (return)
                 (cpl:do-retry bottle-grasp-tries
                   (roslisp:ros-warn (demo step-2) "Retrying")
                   (cpl:retry)))))

        (step-2-inner)))))

(defun step-3-pour-two-arms ()
  (cpl:with-failure-handling
      ((common-fail:low-level-failure (e)
         (roslisp:ros-warn (demo step-3) "~a" e)
         (return)))

    (go-to-initial-pouring-configuration-plan)
    (let ((?left-gripper
            (cut:with-vars-bound (?left-tool-frame)
                (cut:lazy-car
                 (prolog:prolog
                  `(and (cram-robot-interfaces:robot ?robot)
                        (cram-robot-interfaces:robot-tool-frame ?robot :left ?left-tool-frame))))
              (if (cut:is-var ?left-tool-frame)
                  (roslisp:ros-info (low-level giskard-init)
                                    "?left-tool-frame is unknown. ~
                                     Did you load a robot description package?")
                  ?left-tool-frame))))
      (exe:perform (desig:a motion
                            (type looking)
                            (frame ?left-gripper))))
    (exe:perform (desig:an action
                           (to pour-activity)
                           (arm (left right))
                           (source right)
                           (target left)
                           (pour-volume 2.77019964e-05)))
    (go-to-initial-pouring-configuration-plan)))

(defun step-4-place-cup ()
  (cpl:with-failure-handling
      ((common-fail:low-level-failure (e)
         (roslisp:ros-warn (demo step-4) "~a" e)
         (return)))

    (drive-and-place-plan :?arm :left)))

(defun step-5-drive-to-the-left ()
  (cpl:with-failure-handling
      ((common-fail:low-level-failure (e)
         (roslisp:ros-warn (demo step-5) "~a" e)
         (return)))

    (let ((?nav-goal *meal-table-left-base-pose*))
      (cpl:par
        (exe:perform (desig:a motion
                              (type going)
                              (target (desig:a location (pose ?nav-goal)))))
        (move-pr2-arms-out-of-sight)))))

(defun step-6-pour-into-second-cup ()
  (flet ((step-6-inner ()
           (let* ((?cup-desig (desig:an object
                                        (type cup)
                                        (color "yellow")))
                  (?second-cup-to-pour (perceive ?cup-desig
                                                 :quantifier :all
                                                 :object-chosing-function
                                                 #'chose-higher-cup)))
             (when (< (desig:desig-prop-value ?second-cup-to-pour :bb-dist-to-plane) 0.1)
               (cpl:fail 'common-fail:low-level-failure
                         :description "couldn't perceive cup in hand"))
             ;; drive towards second cup
             (drive-towards-object-plan ?second-cup-to-pour :?arm :right)
             ;; perform pouring using object-in-hand-s
             (cpl:par
               (exe:perform (desig:an action
                                      (type looking-at)
                                      (object ?second-cup-to-pour)))
               (exe:perform (desig:an action
                                      (to pour-activity)
                                      (arm right)
                                      (target ?second-cup-to-pour)
                                      (pour-volume 0.0002)))))))
    (cpl:with-retry-counters ((second-pour-tries 20))
      (cpl:with-failure-handling
          ((common-fail:low-level-failure (e)
             (roslisp:ros-warn (demo step-6) "~a" e)
             (cpl:do-retry second-pour-tries
               (roslisp:ros-warn (demo step-6) "Retrying")
               (cpl:retry))
             (return)))

        (step-6-inner)))))

(defun step-7-place-bottle ()
  (cpl:with-failure-handling
      ((common-fail:low-level-failure (e)
         (roslisp:ros-warn (demo step-5) "~a" e)
         (return)))

    (drive-and-place-plan :?arm :right)))

(defun step-8-finalize ()
  (cpl:with-failure-handling
      ((common-fail:low-level-failure (e)
         (declare (ignore e))
         (return)))
    (cpl:par
      (move-pr2-arms-out-of-sight)
      (exe:perform (desig:a motion
                            (type looking)
                            (target ((2 0 1) (0 0 0 1))))))))

#+this-is-commented-out-because-it-is-only-for-real-robot-not-simulation
(defun demo-plan (&optional (step 0))
  (cram-process-modules:with-process-modules-running
      (pr2-pms::pr2-perception-pm pr2-pms::pr2-base-pm pr2-pms::pr2-arms-pm
                                  pr2-pms::pr2-grippers-pm pr2-pms::pr2-ptu-pm)
    (cpl:top-level
      (with-pr2-process-modules
        (mapc (lambda (fun)
                (print fun)
                (funcall fun))
              (subseq (list
                       ;; prepare
                       #'step-0-prepare
                       ;; perceive and grasp bottle with the right arm
                       #'step-1-grasp-bottle
                       ;; perceive cups and grasp the rightmost cup
                       #'step-2-grasp-first-cup
                       ;; pour
                       #'step-3-pour-two-arms
                       ;; place the object in left hand
                       #'step-4-place-cup
                       ;; drive to the left part of table
                       #'step-5-drive-to-the-left
                       ;; perceive leftmost cup and pour into it
                       #'step-6-pour-into-second-cup
                       ;; drive to the right and put down the bottle
                       #'step-7-place-bottle
                       ;; finalize
                       #'step-8-finalize)
                      step))))))







;;; arms down
;; (exe:perform (desig:an action
;;                             (to move-arm-motion)
;;                             (left ((0.09611d0 0.68d0 0.35466d0)
;;                                    (-0.45742778331019085d0
;;                                     0.3060123951483878d0
;;                                     0.3788581151804847d0
;;                                     0.744031427853262d0)))
;;                             (right ((0.0848d0 -0.712d0 0.35541d0)
;;                                     (-0.061062529688043946d0
;;                                      -0.6133522138254498d0
;;                                      0.197733462359113d0
;;                                      -0.7622151317882601d0)))))
