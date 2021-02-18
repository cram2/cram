;;;
;;; Copyright (c) 2019, Amar Fayaz <amar@uni-bremen.de>
;;;               2020, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :cram-bullet-reasoning-belief-state)

(defparameter *artificial-perception-noise-factor* 0.0
  "Gives the maximum distance in meters of simulated perception noise.
Eg. A value 0.1 means the perceived object can be moved around up to 0.1 meters
when simulated perception noise is applied. 0 means no artificial noise.")

(defparameter *perception-instability-threshold* 0.07
  "The distance threshold in meters,
in which the simulation is expected to safely correct the perceived object pose.
Anything beyond this distance will cause a correction method to be triggered
to try and correct the unstable object.
If this threshold is NIL, no correction will happen.")

(defparameter *perception-noise-correction-offset* 0.01
  "First distance in meters, with which the unstable perceived object will be
counter-moved when it falls down upon simulation.")

(defparameter *perception-noise-correction-iterations* 5
  "Number of distances to try the correction approach with. In times.")

(defparameter *perception-noise-correction-step* 0.01
  "The step in meters that each iteration of the correction approach differs
from the correction offset.")

(defparameter *perception-noise-simulation-timeout* 3
  "In secs. How many seconds to simulate to compare initial vs simulated pose.")


(defun add-artificial-perception-noise (object-name object-pose)
  "Randomly moves around the object to simulate the errors in perception.
The range at which the object is moved is determined by
*artificial-perception-noise-factor* and setting it to zero will disable noise."
  (when (> *artificial-perception-noise-factor* 0.0)
    (let* ((offset-direction (nth (random 3) '(:x :y :z)))
           (offset-distance (- (random (* 2 *artificial-perception-noise-factor*))
                               *artificial-perception-noise-factor*))
           (new-pose (cram-tf:translate-pose object-pose offset-direction offset-distance)))
      (setf (btr:pose (btr:object btr:*current-bullet-world* object-name))
            new-pose))))

(defun calculate-perception-pose-correction (initial-pose simulated-pose)
  "Returns correction info to counteract the perception noise.
The correction only happens if the distance between the poses is larger than
*perception-instability-threshold*.
Returns a list of ((:axis-1 sign-1) (:axis-2 sign-2)...)."
  (loop with dist = (cl-transforms:v- (cl-transforms:origin initial-pose)
                                      (cl-transforms:origin simulated-pose))
        with dist-list = (cram-tf:3d-vector->list dist)
        with correction
        for distance in dist-list
        for axis in '(:x :y :z)
        do (when (> (abs distance) *perception-instability-threshold*)
             (push (/ distance (abs distance)) correction)
             (push axis correction))
        finally (return correction)))

(defun apply-bullet-object-pose (btr-world object-name object-new-pose)
  "Moves the object to the new pose and erases old velocity."
  ;; apply new pose
  (setf (btr:pose (btr:object btr-world object-name))
        object-new-pose)
  ;; erase object velocity from previous simulation
  (mapcar (lambda (body)
            (setf (cl-bullet:linear-velocity body)
                  (cl-transforms:make-identity-vector)
                  (cl-bullet:angular-velocity body)
                  (cl-transforms:make-identity-vector))
            (cl-bullet:clear-forces body))
          (btr:rigid-bodies (btr:object btr-world object-name))))

(defun stabilize-perceived-object-pose (btr-world object-name object-pose)
  (when *perception-instability-threshold*
    (apply-bullet-object-pose btr-world object-name object-pose)
    (btr:simulate btr-world *perception-noise-simulation-timeout*)
    (let* ((object-simulated-pose
             (btr:pose (btr:object btr-world object-name)))
           (object-pose-correction
             (calculate-perception-pose-correction object-pose object-simulated-pose)))
      (when object-pose-correction
        (loop for offset = *perception-noise-correction-offset*
                then (+ offset *perception-noise-correction-step*)
              for iteration from 1 to *perception-noise-correction-iterations*
              do (let* ((scaled-correction
                          (mapcar (lambda (x) (if (numberp x) (* x offset) x))
                                  object-pose-correction))
                        (object-corrected-pose
                          (apply #'cram-tf:translate-pose object-pose scaled-correction)))
                   (apply-bullet-object-pose btr-world object-name object-corrected-pose)
                   (btr:simulate btr-world 10)
                   (let ((object-second-simulated-pose
                           (btr:pose (btr:object btr-world object-name))))
                     (if (> (cl-transforms:v-dist
                             (cl-transforms:origin object-corrected-pose)
                             (cl-transforms:origin object-second-simulated-pose))
                            *perception-instability-threshold*)
                         ;; if the correction did not make the object stable,
                         ;; reset the pose back and continue with the loop
                         (apply-bullet-object-pose btr-world object-name object-pose)
                         ;; if the correction helped, exit the loop
                         (return)))))))))




