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

(cpl:def-cram-function pick-up (?object-designator
                                ?arm ?gripper-opening ?grip-effort ?grasp
                                ?left-reach-poses ?right-reach-poses
                                ?left-grasping-poses ?right-grasping-poses
                                ?left-lift-poses ?right-lift-poses)
  (cram-tf:visualize-marker (obj-int:get-object-pose ?object-designator)
                            :r-g-b-list '(1 1 0) :id 300)

  (cpl:par
    (roslisp:ros-info (pick-place pick-up) "Opening gripper")
    (exe:perform
     (desig:an action
               (type setting-gripper)
               (gripper ?arm)
               (position ?gripper-opening)))
    (roslisp:ros-info (pick-place pick-up) "Reaching")
    (cpl:with-failure-handling
        ((common-fail:manipulation-low-level-failure (e)
           (roslisp:ros-warn (pp-plans pick-up)
                             "Manipulation messed up: ~a~%Ignoring."
                             e)
           ;; (return)
           ))
      (exe:perform
       (desig:an action
                 (type reaching)
                 (left-poses ?left-reach-poses)
                 (right-poses ?right-reach-poses)))))
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pick-up)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         (return)
         ))
    (exe:perform
     (desig:an action
               (type grasping)
               (left-poses ?left-grasping-poses)
               (right-poses ?right-grasping-poses))))
  (roslisp:ros-info (pick-place pick-up) "Gripping")
  (exe:perform
   (desig:an action
             (type gripping)
             (gripper ?arm)
             (effort ?grip-effort)
             (object ?object-designator)))
  (roslisp:ros-info (pick-place pick-up) "Assert grasp into knowledge base")
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-attached
     :object-name (desig:desig-prop-value ?object-designator :name)
     :arm ?arm))
  (roslisp:ros-info (pick-place pick-up) "Lifting")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pick-up)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         (return)))
    (exe:perform
     (desig:an action
               (type lifting)
               (left-poses ?left-lift-poses)
               (right-poses ?right-lift-poses)))))


(cpl:def-cram-function place (?object-designator
                              ?arm
                              ?left-reach-poses ?right-reach-poses
                              ?left-put-poses ?right-put-poses
                              ?left-retract-poses ?right-retract-poses
                              ?placing-location)
  (roslisp:ros-info (pick-place place) "Reaching")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pick-up)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         ;; (return)
         ))
    (exe:perform
     (desig:an action
               (type reaching)
               (left-poses ?left-reach-poses)
               (right-poses ?right-reach-poses))))
  (roslisp:ros-info (pick-place place) "Putting")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pick-up)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         (return)))
    (exe:perform
     (desig:an action
               (type putting)
               (left-poses ?left-put-poses)
               (right-poses ?right-put-poses))))
  (roslisp:ros-info (pick-place place) "Opening gripper")
  (exe:perform
   (desig:an action
             (type releasing)
             (gripper ?arm)))
  (roslisp:ros-info (pick-place place) "Retract grasp in knowledge base")
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-detached
     :arm ?arm
     :object-name (desig:desig-prop-value ?object-designator :name)))
  (roslisp:ros-info (pick-place place) "Retracting")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pick-up)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         (return)
         ))
    (exe:perform
     (desig:an action
               (type retracting)
               (left-poses ?left-retract-poses)
               (right-poses ?right-retract-poses)))))


;; (defun perform-phases-in-sequence (action-designator)
;;   (declare (type desig:action-designator action-designator))
;;   (let ((phases (desig:desig-prop-value action-designator :phases)))
;;     (mapc (lambda (phase)
;;             (format t "Executing phase: ~%~a~%~%" phase)
;;             (exe:perform phase))
;;           phases)))

;; (cpl:def-cram-function pick-up (action-designator object arm grasp)
;;   (perform-phases-in-sequence action-designator)
;;   (cram-occasions-events:on-event
;;    (make-instance 'cpoe:object-gripped :object object :arm arm :grasp grasp)))


;; (cpl:def-cram-function place (action-designator object arm)
;;   (perform-phases-in-sequence action-designator)
;;   (cram-occasions-events:on-event
;;    (make-instance 'cpoe:object-released :arm arm :object object)))

