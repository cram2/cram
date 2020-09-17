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

(defparameter *perception-instability-threshold* 0.1
  "The distance threshold in meters,
in which the simulation is expected to safely correct the perceived object pose.
Anything beyond this distance will cause a correction method to be triggered
to try and correct the unstable object")

(defparameter *artificial-perception-noise-factor* 0.0
  "Gives the maximum distance in meters of simulated perception noise.
Eg. A value 0.1 means the perceived object can be moved around up to 0.1 meters
when simulated perception noise is applied. 0 means no artificial noise.")

(defparameter *perception-noise-correction-offset* 0.04
  "Distance in meters, with which the unstable perceived object will be
counter-moved when it falls down upon simulation.")

(defun add-artificial-perception-noise (object-name object-pose)
  "Randomly moves around the object to simulate the errors in perception.
The range at which the object is moved is determined by
*artificial-perception-noise-factor* and setting it to zero will disable noise."
  (when (> *simulated-perception-noise-factor* 0.0)
    (let* ((offset-direction (nth (random 3) '(:x :y :z)))
           (offset-distance (- (random (* 2 *simulated-perception-noise-factor*))
                               *simulated-perception-noise-factor*))
           (new-pose (cram-tf:translate-pose object-pose offset-direction offset-distance)))
      (setf (btr:pose (btr:object btr:*current-bullet-world* object-name))
            new-pose))))

(defun correct-perception-pose-noise (initial-pose final-pose)
  "Returns a corrected pose to counteract the movement of the object due
to perception noise.
The correction only happens if the distance between the poses is larger than
*perception-instability-threshold*.
The correction is only applied to the axis with the maximum
absolute deviation from the original pose and the offsets for the other axes are
set to zero. The correction distance is determined by
*perception-noise-correction-offset*.
Returns the new corrected pose."
  (let* ((dist (cl-transforms:v- (cl-transforms:origin final-pose)
                                 (cl-transforms:origin initial-pose)))
         (dist-list (cram-tf:3d-vector->list dist))
         (max-dist (apply #'max dist-list))
         (max-dist-norm (abs max-dist)))
    (when (> max-dist-norm *perception-instability-threshold*)
      (let* ((max-sign (/ max-dist max-dist-norm))
             (max-index (position max-dist dist-list))
             (correction-axis (nth max-index '(:x :y :z))))
        (cram-tf:translate-pose initial-pose correction-axis
                                (* max-sign *perception-noise-correction-offset*))))))


(defun correct-bullet-object-pose (btr-world object-name object-new-pose)
  "Moves the object to the new pose and erases old velocity."
  ;; apply new pose
  (setf (btr:pose (btr:object btr-world object-name))
        object-new-pose)
  ;; erase object velocity from previous simulation
  (mapcar (lambda (body)
            (setf (cl-bullet:linear-velocity body)
                  (cl-transforms:make-identity-vector)
                  (cl-bullet:angular-velocity body)
                  (cl-transforms:make-identity-vector)))
          (btr:rigid-bodies (btr:object btr-world object-name))))

(defun stabilize-perceived-object-pose (btr-world object-name object-pose)
  (btr:simulate btr-world 1)
  (let* ((object-simulated-pose
           (btr:pose (btr:object btr-world object-name)))
         (object-corrected-pose
           (correct-perception-pose-noise object-pose object-simulated-pose)))
    (when object-corrected-pose
      ;; Retry by spawning the object a corrected distance from the original pose
      (correct-bullet-object-pose btr-world object-name object-corrected-pose)
      (btr:simulate btr-world 10)
      (let ((object-second-simulated-pose
              (btr:pose (btr:object btr-world object-name))))
        (unless (< (cl-transforms:v-dist
                    (cl-transforms:origin object-pose)
                    (cl-transforms:origin object-second-simulated-pose))
                   *perception-instability-threshold*)
          ;; if the correction did not make the object stable,
          ;; reset the pose back and throw a warning
          (roslisp:ros-warn (btr-belief consistency)
                            "Perceived pose of ~a is unstable..." object-name)
          (correct-bullet-object-pose btr-world object-name object-pose))))))



