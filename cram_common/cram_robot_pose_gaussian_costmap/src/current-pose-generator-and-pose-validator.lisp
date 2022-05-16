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
  (return-from robot-current-pose-tf-generator nil)
  (when (or (cram-robot-interfaces:reachability-designator-p desig)
            (cram-robot-interfaces:visibility-designator-p desig))
    (when cram-tf:*transformer*
      (handler-case
          (list (cram-tf:robot-current-pose))
        (cl-transforms-stamped:transform-stamped-error () nil)))))

;; (defun robot-current-pose-bullet-generator (desig)
;;   (when (or (cram-robot-interfaces:reachability-designator-p desig)
;;             (cram-robot-interfaces:visibility-designator-p desig))
;;     (handler-case
;;         (cut:var-value '?pose
;;                        (car (prolog `(and (btr:bullet-world ?w)
;;                                           (cram-robot-interfaces:robot ?robot-name)
;;                                           (btr:object-pose ?w ?robot-name ?pose)))))
;;       (error () nil))))

(desig:register-location-generator
 3 robot-current-pose-tf-generator
 "We should move the robot only if we really need to move. Try the
 current robot pose as a first solution.")


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
      (cut:with-vars-bound (?to-reach-pose
                            ?min-distance ?max-distance
                            ?orientation-samples ?orientation-sample-step
                            ?orientation-offset)
          (cut:lazy-car
           (prolog:prolog
            `(and
              ;; (bagof ?pose
              ;;        (cram-robot-interfaces:designator-reach-pose ,designator ?pose ?_)
              ;;        ?poses)
              ;; (lisp-fun costmap:2d-pose-covariance ?poses 0.5 (?to-reach-point ?_))
              (once (or (and (desig:desig-prop ,designator (:object ?some-object))
                             (desig:current-designator ?some-object ?object)
                             (lisp-fun man-int:get-object-pose-in-map ?object
                                       ?to-reach-pose)
                             (lisp-pred identity ?to-reach-pose)
                             (-> (desig:desig-prop ?object (:location ?loc))
                                 (not (man-int:location-always-reachable ?loc))
                                 (true)))
                        (and (desig:desig-prop ,designator
                                               (:location ?some-location))
                             (desig:current-designator ?some-location ?location)
                             (not (man-int:location-always-reachable ?location))
                             (desig:designator-groundings ?location ?to-reach-poses)
                             (member ?to-reach-pose ?to-reach-poses))))
              (rob-int:robot ?robot-name)
              (costmap:costmap-reach-minimal-distance ?robot-name ?min-distance)
              (costmap:costmap-in-reach-distance ?robot-name ?max-distance)
              (costmap:orientation-samples ?robot-name ?orientation-samples)
              (costmap:orientation-sample-step ?robot-name ?orientation-sample-step)
              (once (or (costmap:reachability-orientation-offset ?robot-name ?orientation-offset)
                        (equal ?orientation-offset 0.0))))))
        (if (or (cut:is-var ?to-reach-pose)
                (cut:is-var ?min-distance)
                (cut:is-var ?max-distance))
            :unknown
            (let (;; DIST ist the distance from the suggested robot pose to to-see-pose
                  (dist (cl-transforms:v-dist
                         ;; project ?TO-REACH-POSE onto the floor,
                         ;; to make distance calculations accurate
                         (cl-transforms:copy-3d-vector
                          (cl-transforms:origin ?to-reach-pose)
                          :z 0)
                         (cl-transforms:origin pose)))
                  ;; PERFECT-ANGLE is the angle around Z to orient the robot in fixed frame
                  ;; such that to-reach-pose is directly in front of the robot
                  (perfect-angle (costmap::angle-to-point-direction
                                  (cl-transforms:x (cl-transforms:origin pose))
                                  (cl-transforms:y (cl-transforms:origin pose))
                                  (cl-transforms:origin ?to-reach-pose)))
                  ;; ALLOWED-RANGE is the DELTA that the suggested robot pose angle
                  ;; is allowed to deviate from PERFECT-ANGLE to still be a valid
                  ;; robot base orientation to see to-reach-pose
                  (allowed-range (* ?orientation-sample-step
                                    (ceiling (/ ?orientation-samples 2))))
                  ;; GENERATED-ANGLE is the suggested robot base pose angle around Z
                  (generated-angle (calculate-z-angle pose)))
              (if (and (< dist ?max-distance)
                       (> dist ?min-distance)
                       (<= (abs (- (cl-transforms:normalize-angle
                                    (- generated-angle perfect-angle))
                                   (cl-transforms:normalize-angle
                                    ?orientation-offset)))
                           allowed-range))
                  :accept
                  :reject))))
      :unknown))

(desig:register-location-validation-function
 5 reachable-location-validator
 "Verifies that reachable location is indeed in close distance to pose")


(defun visible-location-validator (designator pose)
  (if (cram-robot-interfaces:visibility-designator-p designator)
      (cut:with-vars-bound (?to-see-pose
                            ?max-distance
                            ?orientation-samples ?orientation-sample-step
                            ?orientation-offset)
          (cut:lazy-car
           (prolog:prolog
            `(and (once
                   (or (and (desig:desig-prop ,designator (:object ?some-object))
                            (desig:current-designator ?some-object ?object)
                            (lisp-fun man-int:get-object-pose-in-map ?object
                                      ?to-see-pose)
                            (lisp-pred identity ?to-see-pose)
                            (-> (desig:desig-prop ?object (:location ?loc))
                                (not (man-int:location-always-reachable ?loc))
                                (true)))
                       (and (desig:desig-prop ,designator (:location ?some-location))
                            (desig:current-designator ?some-location ?location)
                            (not (man-int:location-always-reachable ?location))
                            (desig:designator-groundings ?location ?to-see-poses)
                            (member ?to-see-pose ?to-see-poses))))
                  (rob-int:robot ?robot-name)
                  (costmap:visibility-costmap-size ?robot-name ?max-distance)
                  (costmap:orientation-samples ?robot-name ?orientation-samples)
                  (costmap:orientation-sample-step ?robot-name
                                                   ?orientation-sample-step)
                  (once (or (costmap:visibility-orientation-offset
                             ?robot-name ?orientation-offset)
                            (equal ?orientation-offset 0.0))))))
        (if (or (cut:is-var ?to-see-pose) (cut:is-var ?max-distance))
            :unknown
            (let (;; DIST ist the distance from the suggested robot pose to to-see-pose
                  (dist (cl-transforms:v-dist
                         ;; project ?TO-SEE-POSE onto the floor,
                         ;; to make distance calculations accurate
                         (cl-transforms:copy-3d-vector
                          (cl-transforms:origin ?to-see-pose)
                          :z 0)
                         (cl-transforms:origin pose)))
                  ;; PERFECT-ANGLE is the angle around Z to orient the robot in fixed frame
                  ;; such that to-see-pose is directly in front of the robot
                  (perfect-angle (costmap::angle-to-point-direction
                                  (cl-transforms:x (cl-transforms:origin pose))
                                  (cl-transforms:y (cl-transforms:origin pose))
                                  ?to-see-pose))
                  ;; ALLOWED-RANGE is the DELTA that the suggested robot pose angle
                  ;; is allowed to deviate from PERFECT-ANGLE to still be a valid
                  ;; robot base orientation to see to-see-pose
                  (allowed-range (* ?orientation-sample-step
                                    (ceiling (/ ?orientation-samples 2))))
                  ;; GENERATED-ANGLE is the suggested robot base pose angle around Z
                  (generated-angle (calculate-z-angle pose)))
              (if (and (< dist ?max-distance)
                       (<= (abs (- (cl-transforms:normalize-angle
                                    (- generated-angle perfect-angle))
                                   (cl-transforms:normalize-angle
                                    ?orientation-offset)))
                           allowed-range))
                  :accept
                  :reject))))
      :unknown))

(desig:register-location-validation-function
 5 visible-location-validator
 "Verifies that visible location is indeed in close distance to pose")
