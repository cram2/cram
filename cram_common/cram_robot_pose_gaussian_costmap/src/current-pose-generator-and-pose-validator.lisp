;;;
;;; Copyright (c) 2015, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :gaussian-costmap)

(defun robot-current-pose-tf-generator (desig)
  (when (or (cram-robot-interfaces:reachability-designator-p desig)
            (cram-robot-interfaces:visibility-designator-p desig))
    (when cram-tf:*transformer*
      (handler-case
          (list (cram-tf:robot-current-pose))
        (cl-transforms-stamped:transform-stamped-error () nil)))))

;; TODO: disabling this for now, in the future use next-different-solution instead of next-solution
;; (desig:register-location-generator
;;  3 robot-current-pose-tf-generator
;;  "We should move the robot only if we really need to move. Try the
;;  current robot pose as a first solution.")


(defun robot-location-on-floor (designator pose)
  (cond ((not (or (cram-robot-interfaces:reachability-designator-p designator)
                  (cram-robot-interfaces:visibility-designator-p designator)))
         :unknown)
        ((< (cl-transforms:z (cl-transforms:origin pose)) 0.05)
         :accept)
        (t
         :reject)))

(desig:register-location-validation-function
 1 robot-location-on-floor
 "Verifies that the z coordinate of the robot is actually on the floor
 if searching for poses where the robot should stand")


(defun calculate-z-angle (pose)
  (multiple-value-bind (axis angle)
      (cl-transforms:quaternion->axis-angle (cl-transforms:orientation pose))
    (when (< (cl-transforms:z axis) 0)
      (setf angle (- angle)))
    (cl-transforms:normalize-angle angle)))

(defun reachable-location-validator (designator pose)
  (if (cram-robot-interfaces:reachability-designator-p designator)
      (cut:with-vars-bound (?to-reach-point ?min-distance ?max-distance
                                            ?orientation-samples ?orientation-sample-step)
          (cut:lazy-car
           (prolog:prolog
            `(and (bagof ?pose
                         (cram-robot-interfaces:designator-reach-pose ,designator ?pose ?_)
                         ?poses)
                  (lisp-fun costmap:2d-pose-covariance ?poses 0.5 (?to-reach-point ?_))
                  (costmap:costmap-reach-minimal-distance ?min-distance)
                  (costmap:costmap-in-reach-distance ?max-distance)
                  (costmap:orientation-samples ?orientation-samples)
                  (costmap:orientation-sample-step ?orientation-sample-step))))
        (if (or (cut:is-var ?to-reach-point)
                (cut:is-var ?min-distance)
                (cut:is-var ?max-distance))
            :unknown
            (let ((dist (cl-transforms:v-dist ?to-reach-point (cl-transforms:origin pose)))
                  (perfect-angle (costmap::angle-to-point-direction
                                  (cl-transforms:x (cl-transforms:origin pose))
                                  (cl-transforms:y (cl-transforms:origin pose))
                                  ?to-reach-point))
                  (allowed-range (* ?orientation-sample-step
                                    (ceiling (/ ?orientation-samples 2))))
                  (generated-angle (calculate-z-angle pose)))
              (if (and (< dist ?max-distance)
                       (> dist ?min-distance)
                       (<= (abs (- perfect-angle generated-angle)) allowed-range))
                  :accept
                  :reject))))
      :unknown))

(desig:register-location-validation-function
 5 reachable-location-validator
 "Verifies that reachable location is indeed in close distance to pose")


(defun visible-location-validator (designator pose)
  (if (cram-robot-interfaces:visibility-designator-p designator)
      (cut:with-vars-bound (?to-see-pose ?max-distance
                                         ?orientation-samples ?orientation-sample-step)
          (cut:lazy-car
           (prolog:prolog
            `(and (or (and (or (desig:desig-prop ,designator (:obj ?object))
                               (desig:desig-prop ,designator (:object ?object)))
                           (btr-belief:object-designator-name ?object ?object-name)
                           (btr:bullet-world ?world)
                           (btr:object-pose ?world ?object-name ?to-see-pose))
                      (and (desig:desig-prop ,designator (:location ?location))
                           (desig:desig-prop ?location (:pose ?to-see-pose))))
                  (costmap:visibility-costmap-size ?max-distance)
                  (costmap:orientation-samples ?orientation-samples)
                  (costmap:orientation-sample-step ?orientation-sample-step))))
        (if (or (cut:is-var ?to-see-pose) (cut:is-var ?max-distance))
            :unknown
            (let ((dist (cl-transforms:v-dist
                         (cl-transforms:origin ?to-see-pose)
                         (cl-transforms:origin pose)))
                  (perfect-angle (costmap::angle-to-point-direction
                                  (cl-transforms:x (cl-transforms:origin pose))
                                  (cl-transforms:y (cl-transforms:origin pose))
                                  ?to-see-pose))
                  (allowed-range (* ?orientation-sample-step
                                    (ceiling (/ ?orientation-samples 2))))
                  (generated-angle (calculate-z-angle pose)))
              (if (and (< dist ?max-distance)
                       (<= (abs (- perfect-angle generated-angle)) allowed-range))
                  :accept
                  :reject))))
      :unknown))

(desig:register-location-validation-function
 5 visible-location-validator
 "Verifies that visible location is indeed in close distance to pose")

;; (defun robot-current-pose-bullet-generator (desig)
;;   (when (or (cram-robot-interfaces:reachability-designator-p desig)
;;             (cram-robot-interfaces:visibility-designator-p desig))
;;     (handler-case
;;         (cut:var-value '?pose
;;                        (car (prolog `(and (btr:bullet-world ?w)
;;                                           (cram-robot-interfaces:robot ?robot-name)
;;                                           (btr:object-pose ?w ?robot-name ?pose)))))
;;       (error () nil))))
