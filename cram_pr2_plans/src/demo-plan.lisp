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
  (let ((?navigation-goal *meal-table-right-base-pose*)
        (?ptu-goal *meal-table-right-base-look-pose*))
    (cpl:par
      (move-pr2-arms-out-of-sight)
      (plan-lib:perform (desig:an action
                                  (to go-motion)
                                  (to ?navigation-goal)))
      (plan-lib:perform (desig:an action
                                  (to look-motion)
                                  (at ?ptu-goal))))))

(defun step-1-grasp-bottle ()
  (let* ((?bottle-desig (desig:an object
                                  (type bottle)))
         (?perceived-bottle-desig (logged-perceive ?bottle-desig)))
    (drive-and-pick-up-plan ?perceived-bottle-desig :?arm :right)))

(defun step-2-grasp-first-cup ()
  (move-pr2-arms-out-of-sight)
  (let* ((?cup-desig (desig:an object
                               (type cup)
                               (cad-model "cup_eco_orange")))
         (?first-cup-to-pour (logged-perceive ?cup-desig
                                              :quantifier :all
                                              :object-chosing-function
                                              (lambda (desigs)
                                                (car (last desigs))))))
    (drive-and-pick-up-plan ?first-cup-to-pour :?arm :left)))

(defun step-3-pour-two-arms ()
  ;(go-to-initial-pouring-configuration-plan)
  (plan-lib:perform (desig:an action
                              (to pour-activity)
                              (arm (left right))
                              (source right)
                              (target left)
                              (pour-volume 0.0002))))

(defun step-4-place-cup ()
  (place-plan :?arm :left))

(defun step-5-drive-to-the-left ()
  (let ((?nav-goal *meal-table-left-base-pose*))
    (cpl:par
      (plan-lib:perform (desig:an action
                                  (to go-motion)
                                  (to ?nav-goal)))
      (move-pr2-arms-out-of-sight))))

(defun step-6-pour-into-second-cup ()
  (let* ((?cup-desig (desig:an object
                               (type cup)
                               (cad-model "cup_eco_orange")))
         (?second-cup-to-pour (logged-perceive ?cup-desig
                                                 :quantifier :all
                                                 :object-chosing-function
                                                 #'car)))
    ;; drive towards second cup
    (drive-towards-and-look-at-object-plan ?second-cup-to-pour :?arm :right)
    ;; perform pouring using object-in-hand-s
    (plan-lib:perform (desig:an action
                                (to pour-activity)
                                (arm right)
                                (target ?second-cup-to-pour)
                                (pour-volume 0.0002)))))

(defun step-7-place-bottle ()
  (drive-and-place-plan :?arm :right))

(defun step-8-finalize ()
  (cpl:with-failure-handling
      ((pr2-ll:pr2-low-level-failure (e)
         (declare (ignore e))
         (return)))
    (cpl:par
      (move-pr2-arms-out-of-sight)
      (plan-lib:perform (desig:an action
                                  (to look-motion)
                                  (at ((2 0 1) (0 0 0 1))))))))

(defun demo-plan (&optional (step 0))
  (with-pr2-process-modules
    (mapc #'funcall
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
                  step))))

;;; arms down
;; (plan-lib:perform (desig:an action
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
