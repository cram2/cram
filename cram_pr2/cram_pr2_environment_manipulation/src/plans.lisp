;;;
;;; Copyright (c) 2018, Christopher Pollok <cpollok@uni-bremen.de>
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

(in-package :pr2-em)

(defun open-container (?arm ?gripper-opening
                       ?left-reach-poses ?right-reach-poses
                       ?left-lift-pose ?right-lift-pose
                       ?left-2nd-lift-pose ?right-2nd-lift-pose
                       &optional
                         (joint-name nil)
                         (environment nil))
  (cpl:par
    (roslisp:ros-info (environment-manipulation open-container) "Opening gripper")
    (exe:perform
     (desig:an action
               (type setting-gripper)
               (gripper ?arm)
               (position ?gripper-opening)))
    (roslisp:ros-info (environment-manipulation open-container) "Reaching")
    (cpl:with-failure-handling
        ((common-fail:manipulation-low-level-failure (e)
           (roslisp:ros-warn (env-plans open)
                             "Manipulation messed up: ~a~%Ignoring."
                             e)
           (return)))
      (exe:perform
       (desig:an action
                 (type reaching)
                 (left-poses ?left-reach-poses)
                 (right-poses ?right-reach-poses)))))
  (roslisp:ros-info (environment-manipulation open-container) "Gripping")
  (exe:perform
   (desig:an action
             (type gripping)
             (gripper ?arm)))
  (roslisp:ros-info (environment-manipulation open-container) "Opening")

  (when (and joint-name environment)
    (cram-occasions-events:on-event
     (make-instance 'cpoe:container-handle-grasping-event
       :joint-name joint-name
       :side ?arm
       :environment environment)))

  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (env-plans open)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         (return)))
    (exe:perform
     (desig:an action
               (type lifting)
               (left-poses ?left-lift-pose)
               (right-poses ?right-lift-pose))))

  (when (and joint-name environment)
    (cram-occasions-events:on-event
     (make-instance 'cpoe:container-opening-event
       :joint-name joint-name
       :side ?arm
       :environment environment)))

  (exe:perform
   (desig:an action
             (type releasing)
             (gripper ?arm)))

  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (env-plans open)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         (return)))
    (exe:perform
     (desig:an action
               (type retracting)
               (left-poses ?left-2nd-lift-pose)
               (right-poses ?right-2nd-lift-pose)))))

(defun close-container (?arm ?gripper-opening
                        ?left-reach-poses ?right-reach-poses
                        ?left-lift-pose ?right-lift-pose
                        ?left-2nd-lift-pose ?right-2nd-lift-pose
                        &optional
                          (joint-name nil)
                          (environment nil))
  (roslisp:ros-info (environment-manipulation close-container) "Opening gripper")
  (exe:perform
   (desig:an action
             (type opening)
             (gripper ?arm)))
  (roslisp:ros-info (environment-manipulation close-container) "Reaching")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (env-plans open)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         (return)))
    (exe:perform
     (desig:an action
               (type reaching)
               (left-poses ?left-reach-poses)
               (right-poses ?right-reach-poses))))
  ;; (roslisp:ros-info (environment-manipulation close-container) "Gripping")
  ;; (exe:perform
  ;;  (desig:an action
  ;;            (type setting-gripper)
  ;;            (gripper ?arm)
  ;;            (position 0)))
  (roslisp:ros-info (environment-manipulation close-container) "Closing")
  (when (and joint-name environment)
    (cram-occasions-events:on-event
     (make-instance 'cpoe:container-handle-grasping-event
                    :joint-name joint-name
                    :side ?arm
                    :environment environment)))

  (exe:perform
   (desig:an action
             (type putting)
             (left-poses ?left-lift-pose)
             (right-poses ?right-lift-pose)))

  (when (and joint-name environment)
    (cram-occasions-events:on-event
     (make-instance 'cpoe:container-closing-event
       :joint-name joint-name
       :side ?arm
       :environment environment)))
  ;; (exe:perform
  ;;  (desig:an action
  ;;            (type setting-gripper)
  ;;            (gripper ?arm)
  ;;            (position 0.1)))
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (env-plans open)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         (return)))
    (exe:perform
     (desig:an action
               (type retracting)
               (left-poses ?left-2nd-lift-pose)
               (right-poses ?right-2nd-lift-pose)))))
