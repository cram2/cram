;;;
;;; Copyright (c) 2020, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :giskard)

(defun make-environment-manipulation-goal (open-or-close arm
                                           handle-link joint-state
                                           prefer-base
                                           &optional hinge-point-stamped)
  (declare (type keyword open-or-close arm)
           (type symbol handle-link)
           (type (or number null) joint-state)
           (type boolean prefer-base)
           (type (or null cl-transforms-stamped:point-stamped) hinge-point-stamped))
  (make-giskard-goal
   :constraints (list
                 (when prefer-base
                   (make-prefer-base-constraint :do-not-rotate T))
                 ;; (make-base-velocity-constraint
                 ;;  *base-max-velocity-slow-xy* *base-max-velocity-slow-theta*)
                 (make-open-or-close-constraint
                  open-or-close arm handle-link joint-state)
                 (when (eq (rob-int:get-robot-name) :tiago-dual)
                   (make-diffdrive-base-arch-constraint hinge-point-stamped))
                 (make-avoid-joint-limits-constraint
                  :joint-list (cut:var-value
                               '?joints
                               (car
                                (prolog:prolog
                                 `(and (rob-int:robot ?robot-name)
                                       (rob-int:arm-joints ?robot-name ,arm ?joints))))))
                 (make-head-pointing-at-hand-constraint arm))
   :collisions (make-constraints-vector
                (make-allow-hand-collision
                 (list arm) (rob-int:get-environment-name) handle-link))))

(defun make-environment-manipulation-diffdrive-pregoal (arm handle-link
                                                        &optional hinge-point-stamped)
  (declare (type keyword arm)
           (type symbol handle-link)
           (type (or null cl-transforms-stamped:point-stamped) hinge-point-stamped))
  (make-giskard-goal
   :constraints (list
                 (let ((wrist-link
                         (if (eq arm :left)
                             cram-tf:*robot-left-wrist-frame*
                             cram-tf:*robot-right-wrist-frame*)))
                   (make-cartesian-constraint
                    cram-tf:*odom-frame*
                    wrist-link
                    (cl-transforms-stamped:pose->pose-stamped
                     wrist-link 0.0
                     (cl-transforms:make-identity-pose))))
                 (when (eq (rob-int:get-robot-name) :tiago-dual)
                   (make-diffdrive-base-arch-constraint hinge-point-stamped :small-weight t))
                 (make-avoid-joint-limits-constraint
                  :joint-list (cut:var-value
                               '?joints
                               (car
                                (prolog:prolog
                                 `(and (rob-int:robot ?robot-name)
                                       (rob-int:arm-joints
                                        ?robot-name ,arm ?joints))))))
                 (make-head-pointing-at-hand-constraint arm))
   :collisions (make-constraints-vector
                (make-allow-hand-collision
                 (list arm) (rob-int:get-environment-name) handle-link))))

(defun call-environment-manipulation-action (&key
                                               action-timeout
                                               open-or-close arm
                                               handle-link joint-angle
                                               prefer-base
                                               joint-pose)
  (declare (type keyword open-or-close arm)
           (type symbol handle-link)
           (type (or number null) joint-angle action-timeout)
           (type boolean prefer-base)
           (type (or cl-transforms:pose null) joint-pose))

  (let ((joint-point-stamped
          (when joint-pose
            (cl-transforms-stamped:make-point-stamped
             cram-tf:*fixed-frame*
             0.0
             (cl-transforms:origin joint-pose)))))

    (when (eq (rob-int:get-robot-name) :tiago-dual)
      (call-action
       :action-goal (make-environment-manipulation-diffdrive-pregoal
                     arm handle-link joint-point-stamped)
       :action-timeout action-timeout))

    (call-action
     :action-goal (make-environment-manipulation-goal
                   open-or-close arm handle-link joint-angle prefer-base
                   joint-point-stamped)
     :action-timeout action-timeout
     :check-goal-function (lambda (result status)
                            (declare (ignore result))
                            (when (or (not status)
                                      (member status '(:preempted :aborted :timeout)))
                              (make-instance
                                  'common-fail:environment-manipulation-goal-not-reached
                                :description "Giskard action failed.")))))
  ;; (if (eq open-or-close :open)
  ;;     (dotimes (i 2)
  ;;       (call-action
  ;;        :action-goal (make-environment-manipulation-goal
  ;;                      open-or-close arm handle-link (/ joint-angle 2.0) prefer-base)
  ;;        ;; This needs a fix: we open the joint 2x for the same joint state
  ;;        ;; Instead do open half, update global env joint-state, open full.
  ;;        :action-timeout action-timeout
  ;;        :check-goal-function (lambda (result status)
  ;;                               (declare (ignore result))
  ;;                               (when (or (not status)
  ;;                                         (member status '(:preempted :aborted :timeout)))
  ;;                                 (make-instance
  ;;                                     'common-fail:environment-manipulation-goal-not-reached
  ;;                                   :description "Giskard action failed.")))))
  ;;     (call-action
  ;;      :action-goal (make-environment-manipulation-goal
  ;;                    open-or-close arm handle-link joint-angle prefer-base)
  ;;      :action-timeout action-timeout
  ;;      :check-goal-function (lambda (result status)
  ;;                             (declare (ignore result))
  ;;                             (when (or (not status)
  ;;                                       (member status '(:preempted :aborted :timeout)))
  ;;                               (make-instance
  ;;                                   'common-fail:environment-manipulation-goal-not-reached
  ;;                                 :description "Giskard action failed.")))))
  )
