;;;
;;; Copyright (c) 2020, Christopher Pollok <cpollok@uni-bremen.de>
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

(in-package :cram-integration-tests)

(defparameter *motions-executed* 0)

(defmethod exe:generic-perform :after ((designator motion-designator))
  (incf *motions-executed*))

(defun reset-motions-counter ()
  (setf *motions-executed* 0))

(defun executed-motions? ()
  (> *motions-executed* 0))

(defun initialize ()
  (sb-ext:gc :full t)
  (setf desig::*designators* (tg:make-weak-hash-table :weakness :key))
  (setf btr-belief:*spawn-debug-window* t)
  (coe:clear-belief)
  (reset-motions-counter))

(defun get-object-pose ()
  (let* ((map-T-surface
           (cl-transforms:pose->transform
            (btr:link-pose (btr:get-environment-object) "sink_area_surface")))
         (surface-T-object
           (cl-transforms:pose->transform
            (cram-tf:list->pose '((0.2 -0.15 0.1) (0 0 0 1)))))
         (map-T-object
           (cl-transforms:transform* map-T-surface surface-T-object)))
    (cl-transforms:transform->pose map-T-object)))

(defun spawn-object-on-sink-counter ()
  (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo")
  (btr-utils:kill-all-objects)
  (btr:detach-all-objects (btr:get-robot-object))
  (btr:detach-all-objects (btr:get-environment-object))
  (btr-utils:spawn-object :test-object :breakfast-cereal
                          :pose (get-object-pose)))

(define-test navigation-goal
  (initialize)
  (let* ((?pose
           (cl-transforms-stamped:make-pose-stamped
            "map" 0.0
            (cl-transforms:make-3d-vector 0.7 0.7 0)
            (cl-transforms:make-identity-rotation)))
         (?goal
           `(cpoe:robot-at-location ,(a location (pose ?pose))))
         (the-action
           (an action
               (type going)
               (target (a location (pose ?pose)))
               (goal ?goal)))
         (executed-motions-initially?
           nil))
    (urdf-proj:with-simulated-robot
      (perform the-action)
      (setf executed-motions-initially? (executed-motions?))
      (reset-motions-counter)
      (perform the-action))
    (lisp-unit:assert-true executed-motions-initially?))
  (lisp-unit:assert-false (executed-motions?)))

(define-test arms-positioned-goal
  (initialize)
  (let* ((?goal
           `(cpoe:arms-positioned-at :park :park))
         (the-action
           (an action
               (type positioning-arm)
               (left-configuration park)
               (right-configuration park)
               (goal ?goal)))
         (executed-motions-initially?
           nil))
    (urdf-proj:with-simulated-robot
      (perform
       (an action
           (type positioning-arm)
           (right-configuration carry-top)
           (left-configuration carry-top)))
      (perform the-action)
      (setf executed-motions-initially? (executed-motions?))
      (reset-motions-counter)
      (perform the-action))
    (lisp-unit:assert-true executed-motions-initially?))
  (lisp-unit:assert-false (executed-motions?)))

(define-test torso-at-goal
  (initialize)
  (let* ((?goal
           `(cpoe:torso-at :lower-limit))
         (the-action
           (an action
               (type moving-torso)
               (joint-angle lower-limit)
               (goal ?goal)))
         (executed-motions-initially?
           nil))
    (urdf-proj:with-simulated-robot
      (perform the-action)
      (setf executed-motions-initially? (executed-motions?))
      (reset-motions-counter)
      (perform the-action))
    (lisp-unit:assert-true executed-motions-initially?))
  (lisp-unit:assert-false (executed-motions?)))

(define-test gripper-joints-at-test
  (initialize)
  (let* ((?goal
           `(cpoe:gripper-joint-at :left 0.05 0.005))
         (the-action
           (an action
               (type setting-gripper)
               (gripper left)
               (position 0.05)
               (goal ?goal)))
         (executed-motions-initially?
           nil))
    (urdf-proj:with-simulated-robot
      (perform the-action)
      (setf executed-motions-initially? (executed-motions?))
      (reset-motions-counter)
      (perform the-action))
    (lisp-unit:assert-true executed-motions-initially?))
  (lisp-unit:assert-false (executed-motions?)))

(define-test gripper-opened-test
  (initialize)
  (let* ((?goal
           `(cpoe:gripper-opened :left 0.01))
         (the-action
           (an action
               (type opening-gripper)
               (gripper left)
               (goal ?goal)))
         (executed-motions-initially?
           nil))
    (urdf-proj:with-simulated-robot
      (perform the-action)
      (setf executed-motions-initially? (executed-motions?))
      (reset-motions-counter)
      (perform the-action))
    (lisp-unit:assert-true executed-motions-initially?))
  (lisp-unit:assert-false (executed-motions?)))

(define-test gripper-closed-test
  (initialize)
  (let* ((?goal
           `(cpoe:gripper-closed :left 0.01))
         (the-action
           (an action
               (type closing-gripper)
               (gripper left)
               (goal ?goal)))
         (executed-motions-initially?
           nil))
    (urdf-proj:with-simulated-robot
      (perform (an action
                   (type opening-gripper)
                   (gripper left)))
      (reset-motions-counter)
      (perform the-action)
      (setf executed-motions-initially? (executed-motions?))
      (reset-motions-counter)
      (perform the-action))
    (lisp-unit:assert-true executed-motions-initially?))
  (lisp-unit:assert-false (executed-motions?)))

(define-test look-at-goal
  (initialize)
  (let* ((?pose
           (cl-transforms-stamped:make-pose-stamped
            "map" 0.0
            (cl-transforms:make-3d-vector 0.7 0.7 1)
            (cl-transforms:make-identity-rotation)))
         (?look-pos
           (a location (pose ?pose)))
         (?goal
           `(cpoe:looking-at ,?look-pos))
         (the-action
           (an action
               (type looking)
               (target ?look-pos)
               (goal ?goal)))
         (executed-motions-initially? nil))
    (urdf-proj:with-simulated-robot
      (perform the-action)
      (setf executed-motions-initially? (executed-motions?))
      (reset-motions-counter)
      (perform the-action))
    (lisp-unit:assert-true executed-motions-initially?))
  (lisp-unit:assert-false (executed-motions?)))

(define-test container-state-open-goal
  (initialize)
  (let* ((?container
           (an object
               (type drawer)
               (urdf-name sink-area-left-upper-drawer-main)
               (part-of environment)))
         (?goal
           `(cpoe:container-state ,?container :open))
         (the-action
           (an action
               (type opening)
               (object ?container)
               (arm left)
               (goal ?goal)))
         (executed-motions-initially?
           nil))
    (urdf-proj:with-simulated-robot
      (btr-utils:move-robot '((.5 .4 0) (0 0 0 1)))
      (perform the-action)
      (setf executed-motions-initially? (executed-motions?))
      (reset-motions-counter)
      (perform the-action))
    (lisp-unit:assert-true executed-motions-initially?))
  (lisp-unit:assert-false (executed-motions?)))

(define-test container-state-close-goal
  (initialize)
  (let* ((?container
           (an object
               (type drawer)
               (urdf-name sink-area-left-upper-drawer-main)
               (part-of environment)))
         (?goal
           `(cpoe:container-state ,?container :closed))
         (the-action
           (an action
               (type closing)
               (object ?container)
               (arm left)
               (goal ?goal)))
         (executed-motions-initially?
           nil))
    (urdf-proj:with-simulated-robot
      (btr:set-robot-state-from-joints
       `((,"sink_area_left_upper_drawer_main_joint" ,0.4))
       (btr:object btr:*current-bullet-world* :environment))
      (btr-utils:move-robot '((.5 .4 0) (0 0 0 1)))
      (perform the-action)
      (setf executed-motions-initially? (executed-motions?))
      (reset-motions-counter)
      (perform the-action))
    (lisp-unit:assert-true executed-motions-initially?))
  (lisp-unit:assert-false (executed-motions?)))

(define-test tool-frames-at
  (initialize)
  (let* ((?tool-pose
           (btr:link-pose (btr:get-robot-object) cram-tf:*robot-right-tool-frame*))
         (?goal
           `(cpoe:tool-frames-at nil ,?tool-pose))
         (the-action
           (an action
               (type reaching)
               (right-poses (?tool-pose))
               (goal ?goal))))
    (urdf-proj:with-simulated-robot
     (perform the-action)))
  (lisp-unit:assert-false (executed-motions?)))

(define-test picking-up-goal
  (initialize)
  (spawn-object-on-sink-counter)
  (let ((?robot-pose
          (cl-transforms-stamped:pose->pose-stamped
           cram-tf:*fixed-frame* 0.0
           (cram-tf:list->pose '((.6 .6 0) (0 0 0 1)))))
        (?look-pose
          (cl-transforms-stamped:pose->pose-stamped
           cram-tf:*fixed-frame* 0.0
           (get-object-pose)))
        (executed-motions-initially?
          nil))
    (urdf-proj:with-simulated-robot
      (perform
       (a motion
          (type going)
          (pose ?robot-pose)))
      (perform
       (an action
           (type looking)
           (target (a location
                      (pose ?look-pose)))))
      (let* ((?object
               (perform
                (an action
                    (type perceiving)
                    (object (an object
                                (type breakfast-cereal))))))
             ;; We copy the object here because the original gets changed,
             ;; in a way that referencing the pick up designator doesn't work anymore.
             (?object-copy
               (desig:copy-designator ?object))
             (?goal
               `(cpoe:object-in-hand ,?object)))
        (perform
         (an action
             (type picking-up)
             (object ?object)
             (goal ?goal)))
        (setf executed-motions-initially? (executed-motions?))
        (reset-motions-counter)
        (perform
         (an action
             (type picking-up)
             (object ?object-copy)
             (goal ?goal)))))
    (lisp-unit:assert-true executed-motions-initially?))
  (lisp-unit:assert-false (executed-motions?)))
