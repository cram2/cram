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
                               ((:side ?side))
                               ;;grasp
                               ((:left-reach-poses ?left-reach-poses))
                               ((:left-tilt-poses ?left-tilt-poses))
                               
                               ((:right-reach-poses ?right-reach-poses))
                               ((:right-tilt-poses ?right-tilt-poses))

                               ((:on-object ?on-object))
                               ;;object
                               ((:wait-duration ?wait-duration))
                               ((:look-location ?look-location))
                               robot-arm-is-also-a-neck
                             &allow-other-keys)
  
  ;; (declare (type (or null list)
  ;;                ?left-reach-poses ?right-reach-poses
  ;;                ?left-tilt-down-poses ?right-tilt-down-poses
  ;;                ?left-tilt-up-poses ?right-tilt-up-poses
  ;;                ?left-retract-poses ?right-retract-poses)
  ;;          (type desig:object-designator ?on-object object)
  ;;          (type desig:location-designator ?look-location)
  ;;          ;;(type keyword ?arm side grasp)
  ;;          (type number ?wait-duration)
  ;;          (ignore side grasp object))
  (let* ((sleepy nil)
         (?movy nil)
         (?align-planes-left nil)
         (?align-planes-right nil)
         (?move-base-when-reaching t))

    (cpl:with-failure-handling
        (((or common-fail:manipulation-low-level-failure
              common-fail:manipulation-goal-not-reached) (e)
           (roslisp:ros-warn (pp-plans pour-reach)
                             "Manipulation messed up: ~a~%Ignoring."
                             e)
           (return)))
      (let ((?goal `(cpoe:tool-frames-at ,?left-reach-poses ,?right-reach-poses)))
        (exe:perform
         (desig:an action
                   (type reaching)
                   (object ?on-object)
                   (left-poses ?left-reach-poses)
                   (right-poses ?right-reach-poses)
                   (move-base ?move-base-when-reaching)
                   (goal ?goal)))))

    (setf ?move-base-when-reaching nil)
 
    (cpl:with-retry-counters ((giskardside-retries 3))
      (cpl:with-failure-handling
          (((or common-fail:manipulation-low-level-failure
                common-fail:manipulation-goal-not-reached) (e)
             (roslisp:ros-warn (pp-plans pour-reach)
                               "Manipulation messed up: ~a~%Failing."
                               e)
             
             (cpl:do-retry giskardside-retries
               (break)
               (cpl:retry))
             (return)))
        (let ((?goal `(cpoe:tool-frames-at ,?left-reach-poses ,?right-reach-poses)))
          (exe:perform
           (desig:an action
                     (type reaching)
                     (object ?on-object)
                     (left-poses ?left-reach-poses)
                     (right-poses ?right-reach-poses)
                     (move-base ?move-base-when-reaching)
                     (goal ?goal))))))

    
   
    
    (when sleepy
      (sleep 2))

    (cpl:with-retry-counters ((giskardside-retries 3))
      (cpl:with-failure-handling
          (((or common-fail:manipulation-low-level-failure
                common-fail:manipulation-goal-not-reached) (e)
             (roslisp:ros-warn (pp-plans pour-tilt-down-more)
                               "Manipulation messed up: ~a~%Failing."
                               e)
             
             (cpl:do-retry giskardside-retries
               (cpl:retry))
             (return)))

        
        (let ((?goal `(cpoe:tool-frames-at ,?left-tilt-poses ,?right-tilt-poses)))
          (exe:perform
           (desig:an action
                     (type tilting)
                     (object ?on-object)
                     (left-poses ?left-tilt-poses)
                     (right-poses ?right-tilt-poses)
                     (align-planes-left ?align-planes-left)
                     (align-planes-right ?align-planes-right)
                     (move-base ?movy)
                     ;;(collision-mode :allow-attached)
                     (goal ?goal))))
        ))
    
    
    (when sleepy
      (sleep 5))


    
    
    
    (cpl:with-retry-counters ((giskardside-retries 3))
      (cpl:with-failure-handling
          (((or common-fail:manipulation-low-level-failure
                common-fail:manipulation-goal-not-reached) (e)
             (roslisp:ros-warn (pp-plans pour-retract)
                               "Manipulation messed up: ~a~%Failing."
                               e)
             
             (cpl:do-retry giskardside-retries
               (cpl:retry))
             (return)))

        
        (let ((?goal `(cpoe:tool-frames-at ,?left-reach-poses ,?right-reach-poses)))
          (exe:perform
           (desig:an action
                     (type retracting)
                     (object ?on-object)
                     (left-poses ?left-reach-poses)
                     (right-poses ?right-reach-poses)
                     (application-context pouring)
                     (goal ?goal))))
        ))

    
    ) )



(defun pour (&key
               ((:arm ?arm))
               sides
               ;; ((:object ?object))
               ((:on-object ?on-object))
               ((:wait-duration ?wait-duration))
             &allow-other-keys)
  (declare (type desig:object-designator ;; ?object
                 ?on-object)
           (type keyword ?arm)
           ;(type (or nil number) ?wait-duration)
           (type list sides))
  (let ((?side (cut:lazy-car sides)))

    
    ;; if pouring fails, try to pour from another side
    ;; (cpl:with-retry-counters ((side-retries 3))
    ;;   (cpl:with-failure-handling
    ;;       (((or common-fail:manipulation-low-level-failure
    ;;             common-fail:object-unreachable
    ;;             desig:designator-error) (e)
    ;;          (common-fail:retry-with-list-solutions
    ;;              sides
    ;;              side-retries
    ;;              (:error-object-or-string
    ;;               (format NIL "Pouring failed: ~a.~%Next" e)
    ;;               :warning-namespace (mpp-plans pour))
    ;;            (setf ?side (cut:lazy-car sides)))))
        (print ?side)
        (exe:perform
         (desig:an action
                   (type pouring-without-retries)
                   (configuration ?side)
                   (on-object ?on-object)
                   (desig:when ?arm
                    (arm ?arm))
                   ;; (desig:when ?object
                   ;;   (object ?object))
                   (desig:when ?wait-duration
                     (wait-duration ?wait-duration))
                   ))
        ))



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


(defmethod man-int:get-object-type-robot-frame-tilt-approach-transform 
    ((object-type (eql :bowl))
     arm
     (grasp (eql :top-left)))
  '((-0.02 -0.17 0.06)(0 0 0 1)))

(defmethod man-int:get-object-type-robot-frame-tilt-approach-transform
    ((object-type (eql :bowl))
     arm
     (grasp (eql :top-right)))
  '((-0.02 0.17 0.06)(0 0 0 1)))

(defmethod man-int:get-object-type-robot-frame-tilt-approach-transform
    ((object-type (eql :bowl))
     arm
     (grasp (eql :top-front)))
  '((-0.1 0.0 0.019)(0 0 0 1)))

