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

(in-package :pr2-pp-plans)

(defun perceive (?object-designator
                 &key
                   (quantifier :a)
                   (object-chosing-function #'identity))
  (cpl:with-retry-counters ((perceive-retries 5))
    (cpl:with-failure-handling
        ((pr2-fail:perception-object-not-found (e)
           (cpl:do-retry perceive-retries
             (roslisp:ros-warn (pick-and-place perceive) "~a" e)
             (cpl:retry))))
      (let* ((resulting-designators
               (case quantifier
                 (:all (exe:perform
                        (desig:a motion
                                 (type detecting)
                                 (objects ?object-designator))))
                 (t (exe:perform
                     (desig:a motion
                              (type detecting)
                              (object ?object-designator))))))
             (resulting-designator
               (funcall object-chosing-function resulting-designators)))
        resulting-designator))))

;; (defun pose-to-reach-object (object-pose-in-map arm)
;;   "Hardcoded pose for reaching an object: take an x offset of 20 cm from object."
;;   (let* ((new-x-for-base (- (cl-transforms:x (cl-transforms:origin object-pose-in-map))
;;                             (case arm
;;                               (:left 0.2)
;;                               (:right -0.2)
;;                               (t 0.0))))
;;          (robot-pose-in-map (cram-tf:robot-current-pose))
;;          (?goal-for-base (cl-transforms-stamped:copy-pose-stamped
;;                           robot-pose-in-map
;;                           :origin (cl-transforms:copy-3d-vector
;;                                    (cl-transforms:origin robot-pose-in-map)
;;                                    :x new-x-for-base))))
;;     ?goal-for-base))
(defun drive-to-reach-pose (pose &key (?arm :right))
  (let* ((?pose-in-map
           (cram-tf:ensure-pose-in-frame pose cram-tf:*fixed-frame* :use-current-ros-time t))
         (?goal-for-base
           (desig:reference (desig:a location
                                     (to reach)
                                     (location (a location (pose ?pose-in-map)))
                                     (side ?arm)))))
    (exe:perform (desig:a motion
                          (type going)
                          (target (desig:a location (pose ?goal-for-base)))))))

(defun drive-towards-object-plan (object-designator &key (?arm :right))
  (let ((object-pose-in-base (get-object-pose object-designator)))
    (drive-to-reach-pose object-pose-in-base :?arm ?arm)))

(defun drive-and-pick-up-plan (?object-designator &key (?arm :right))
  ;; navigate to a better pose
  (drive-towards-object-plan ?object-designator :?arm ?arm)
  (cpl:par
    (exe:perform (desig:an action
                           (type looking-at)
                           (object ?object-designator)))
    (exe:perform (desig:an action
                           (type picking-up)
                           (arm ?arm)
                           (object ?object-designator)))))

(defun perceive-and-drive-and-pick-up-plan (?type &key (?arm '(:left :right))
                                                    ?color ?cad-model)
  (if (member ?type '(:fork :knife :cutlery))
      (move-pr2-arms-out-of-sight :flipped t)
      (move-pr2-arms-out-of-sight))
  (let ((object-description `((:type ,?type))))
    (when ?color
      (push `(:color ,?color) object-description))
    (when ?cad-model
      (push `(:cad-model ,?cad-model) object-description))
    (let* ((?object-desig (desig:make-designator :object object-description))
           (?updated-object-desig (perceive ?object-desig)))
      (drive-and-pick-up-plan ?updated-object-desig :?arm ?arm))))

(defun drive-and-place-plan (&key (?arm :right) ?target-location)
  (let* ((?object-designator (get-object-in-hand ?arm))
         (?driving-pose (or (when ?target-location (desig:reference ?target-location))
                            (get-object-pose ?object-designator))))
    (drive-to-reach-pose ?driving-pose :?arm ?arm)
    (cpl:par
      (exe:perform (desig:an action
                             (type looking-at)
                             (object ?object-designator)))
      (exe:perform (if ?target-location
                       (desig:an action
                                 (type placing)
                                 (arm ?arm)
                                 (target ?target-location))
                       (desig:an action
                                 (type placing)
                                 (arm ?arm)))))))

(defun pick-and-place-plan (?type &key (?arm :right) ?color ?cad-model)
  (perceive-and-drive-and-pick-up-plan ?type :?arm ?arm :?color ?color :?cad-model ?cad-model)
  (exe:perform (desig:an action
                         (type placing)
                         (arm ?arm))))
