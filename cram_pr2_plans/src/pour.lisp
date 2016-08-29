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
;;     (destination (an object))
;;     (source (an object))
;;     (theme (some stuff))
;;     (goal drink-poured)
;;     (arm right)
;;     (phases (a motion (type approaching))
;;             (a motion
;;                (type tilting)
;;                (angle )
;;                ())
;;             (a motion (type tilting) (angle -x)
;;             (a motion (type retracting))))

;; - mug-top-behind-maker: {x-coord: mug-top-to-maker-top}
;; - mug-top-left-maker: {y-coord: mug-top-to-maker-top}
;; - mug-top-above-maker: {z-coord: mug-top-to-maker-top}
;; - mug-behind-itself: {x-coord: mug-top-to-mug-bottom}
;; - mug-left-itself: {y-coord: mug-top-to-mug-bottom}
;; - mug-above-itself: {z-coord: mug-top-to-mug-bottom}
;;
;; - {controllable-weight: 0.001}
;; - {constraint-weight: 10.001}
;; - {min-trans-vel: -0.1946}
;; - {max-trans-vel: 0.1946}
;; - {min-rot-vel: -1.2061}
;; - {max-rot-vel: 1.2061}
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
;;   - double-sub: [-0.0768, mug-behind-itself]
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
  (with-pr2-process-modules
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
        (move-pr2-arms-out-of-sight :arm :right)))))

(defun top-level-pour (&key (?arm :right))
  (with-pr2-process-modules
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
      (move-pr2-arms-out-of-sight :arm :right))))

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

(defun put-together-yaml (phase)
  (georgs-function
   (read-yaml-file phase)
   (cdr (assoc phase *constraints*))))

(defun pour-giskard ()
                                        ;with-pr2-process-modules
  (pr2-ll::call-giskard-yaml-action
   :yaml-string (put-together-yaml :approach)
   :action-timeout 20.0)
  (pr2-ll::call-giskard-yaml-action
   :yaml-string (put-together-yaml :tilt)
   :action-timeout 20.0)
  (pr2-ll::call-giskard-yaml-action
   :yaml-string (put-together-yaml :tilt-back)
   :action-timeout 20.0))

(defun top-level-two-arm-pour ()
  (with-pr2-process-modules
    (move-pr2-arms-out-of-sight)
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
                                   ;; (cad-model "cup_eco_orange")
                                   ))
             (?perceived-cup-desig (cram-plan-library:perform
                                    (desig:an action
                                              (to detect-motion)
                                              (object ?cup-desig)))))
        (plan-lib:perform (desig:an action
                                  (to my-pick-up)
                                  (arm left)
                                  (object ?perceived-cup-desig)))
        (pour-giskard)
        (cpl:par
          (top-level-place :?arm :left :?object ?perceived-cup-desig)
          (top-level-place :?arm :right :?object ?perceived-bottle-desig))
        (move-pr2-arms-out-of-sight :arm :right)))))


