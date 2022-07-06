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

(in-package :pp-plans)

;; (an action
;;     (type pouring)
;;     (target-object (an object))
;;     (side front)
;;     (source-object (an object))
;;     (arm right)
;;     (tilt-angle ?pi/2)
;;     (vertical-offset 0.02)
;;     (wait-duration 5))

(defun pour-without-retries (&key
                               ((:arm ?arm))
                               side
                               grasp
                               ((:left-reach-poses ?left-reach-poses))
                               ((:right-reach-poses ?right-reach-poses))
                               ((:left-tilt-down-poses ?left-tilt-down-poses))
                               ((:right-tilt-down-poses ?right-tilt-down-poses))
                               ((:left-tilt-up-poses ?left-tilt-up-poses))
                               ((:right-tilt-up-poses ?right-tilt-up-poses))
                               ((:left-retract-poses ?left-retract-poses))
                               ((:right-retract-poses ?right-retract-poses))
                               ((:target-object ?target-object))
                               source-object
                               ((:wait-duration ?wait-duration))
                               ((:look-location ?look-location))
                               robot-arm-is-also-a-neck
                             &allow-other-keys)
  (declare (type (or null list)
                 ?left-reach-poses ?right-reach-poses
                 ?left-tilt-down-poses ?right-tilt-down-poses
                 ?left-tilt-up-poses ?right-tilt-up-poses
                 ?left-retract-poses ?right-retract-poses)
           (type desig:object-designator ?target-object source-object)
           (type desig:location-designator ?look-location)
           (type keyword ?arm side grasp)
           (type number ?wait-duration)
           (ignore side grasp source-object))
  (roslisp:ros-info (pick-place pour) "Reaching")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pour)
                           "Manipulation messed up: ~a~%Failing."
                           e)))
    (cpl:par
      (unless robot-arm-is-also-a-neck
        (let ( ;; (?goal `(cpoe:looking-at ,?look-location))
              )
          (exe:perform (desig:an action
                                 (type turning-towards)
                                 (target ?look-location)
                                 ;; (goal ?goal)
                                 ))))
      (let ((?goal `(cpoe:tool-frames-at ,?left-reach-poses ,?right-reach-poses)))
        (exe:perform
         (desig:an action
                   (type reaching)
                   (object ?target-object)
                   (left-poses ?left-reach-poses)
                   (right-poses ?right-reach-poses)
                   (goal ?goal))))))
  (roslisp:ros-info (pick-place pour) "Tilting down")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pour)
                           "Manipulation messed up: ~a~%Failing."
                           e)))
    (let ((?goal `(cpoe:tool-frames-at ,?left-tilt-down-poses
                                       ,?right-tilt-down-poses)))
      (exe:perform
       (desig:an action
                 (type tilting)
                 (left-poses ?left-tilt-down-poses)
                 (right-poses ?right-tilt-down-poses)
                 (goal ?goal)))))
  (roslisp:ros-info (pick-place pour) "Waiting")
  (exe:perform
   (desig:an action
             (type waiting)
             (duration ?wait-duration)))
  (roslisp:ros-info (pick-place pour) "Tilting up")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pour)
                           "Manipulation messed up: ~a~%Failing."
                           e)))
    (let ((?goal `(cpoe:tool-frames-at ,?left-tilt-up-poses
                                       ,?right-tilt-up-poses)))
      (exe:perform
       (desig:an action
                 (type tilting)
                 (left-poses ?left-tilt-up-poses)
                 (right-poses ?right-tilt-up-poses)
                 (goal ?goal)))))
  (roslisp:ros-info (pick-place pour) "Retracting")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pour)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         (return)))
    (let ((?goal `(cpoe:tool-frames-at ,?left-retract-poses
                                       ,?right-retract-poses)))
      (exe:perform
       (desig:an action
                 (type retracting)
                 (left-poses ?left-retract-poses)
                 (right-poses ?right-retract-poses)
                 (goal ?goal)))))
  (roslisp:ros-info (pick-place pour) "Parking")
  (exe:perform
   (desig:an action
             (type parking-arms)
             (arms (?arm)))))


(defun pour (&key
               ((:arm ?arm))
               sides
               ((:source-object ?source-object))
               ((:target-object ?target-object))
               ((:wait-duration ?wait-duration))
             &allow-other-keys)
  (declare (type desig:object-designator ?source-object ?target-object)
           (type keyword ?arm)
           (type number ?wait-duration)
           (type list sides))

  (let ((?side (cut:lazy-car sides)))
    ;; if pouring fails, try to pour from another side
    (cpl:with-retry-counters ((side-retries 3))
      (cpl:with-failure-handling
          (((or common-fail:manipulation-low-level-failure
                common-fail:object-unreachable
                desig:designator-error) (e)
             (common-fail:retry-with-list-solutions
                 sides
                 side-retries
                 (:error-object-or-string
                  (format NIL "Pouring failed: ~a.~%Next" e)
                  :warning-namespace (mpp-plans pour))
               (setf ?side (cut:lazy-car sides)))))

        (exe:perform
         (desig:an action
                   (type pouring-without-retries)
                   (side ?side)
                   (target-object ?target-object)
                   (desig:when ?arm
                    (arm ?arm))
                   (desig:when ?source-object
                     (source-object ?source-object))
                   (desig:when ?wait-duration
                     (wait-duration ?wait-duration))))))))



(defun add (?source-object ?target-object)
  (exe:perform
   (desig:an action
             (type searching)
             (object ?source-object)))
  (exe:perform
   (desig:an action
             (type fetching)
             (object ?source-object)))
  (exe:perform
   (desig:an action
             (type searching)
             (object ?target-object)))
  (exe:perform
   (desig:an action
             (type pouring)
             (source-object ?source-object)
             (target-object ?target-object))))
