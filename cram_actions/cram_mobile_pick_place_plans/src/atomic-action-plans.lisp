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

(defun go-to-target (&key
                       ((:pose ?pose-stamped))
                     &allow-other-keys)
  (declare (type cl-transforms-stamped:pose-stamped ?pose-stamped))
  "Go to `?pose-stamped', if a failure happens propagate it up, robot-state-changed event."

  (unwind-protect
       (cpl:with-retry-counters ((nav-retries 0))
         (cpl:with-failure-handling
             ((common-fail:navigation-low-level-failure (e)
                (roslisp:ros-warn (pick-and-place go)
                                  "Some low-level failure happened: ~a"
                                  e)
                (cpl:do-retry nav-retries
                  (roslisp:ros-warn (pick-and-place go) "Retrying...")
                  (cpl:retry))))
           (exe:perform
            (desig:a motion (type going) (pose ?pose-stamped)))))
    (cram-occasions-events:on-event
     (make-instance 'cram-plan-occasions-events:robot-state-changed))))


(defun go-with-torso (&key
                     ((:joint-angle ?joint-angle))
                     &allow-other-keys)
  (declare (type (or number keyword) ?joint-angle))
  "Go to `?joint-angle' with torso, if a failure happens propagate it up, robot-state-changed event."
  (unwind-protect
       (cpl:with-retry-counters ((torso-retries 1))
         (cpl:with-failure-handling
             ((common-fail:torso-low-level-failure (e)
                (roslisp:ros-warn (pick-and-place move-torso)
                                  "Some low-level failure happened: ~a"
                                  e)
                (cpl:do-retry torso-retries
                  (roslisp:ros-warn (pick-and-place move-torso) "Retrying...")
                  (cpl:retry))))
           (exe:perform
            (desig:a motion (type moving-torso) (joint-angle ?joint-angle)))))
    (cram-occasions-events:on-event
     (make-instance 'cram-plan-occasions-events:robot-state-changed))))

(defun perceive (&key
                   ((:object ?object-designator))
                   (object-chosing-function #'identity)
                 &allow-other-keys)
  (declare (type desig:object-designator ?object-designator))
  "Call detecting motion on `?object-designator', retry on failure, issue perceived event,
equate resulting designator to the original one."

  (let ((retries (if (find :cad-model (desig:properties ?object-designator) :key #'car)
                     1
                     4)))
    (cpl:with-retry-counters ((perceive-retries retries))
      (cpl:with-failure-handling
          ((common-fail:perception-low-level-failure (e)
             (cpl:do-retry perceive-retries
               (roslisp:ros-warn (pick-and-place perceive) "~a" e)
               (cpl:retry))))

        (let* ((resulting-designators
                 (exe:perform
                  (desig:a motion
                           (type detecting)
                           (object ?object-designator))))
               (resulting-designator
                 (funcall object-chosing-function resulting-designators)))
          (if (listp resulting-designators)
              (mapcar (lambda (desig)
                        (cram-occasions-events:on-event
                         (make-instance 'cram-plan-occasions-events:object-perceived-event
                           :object-designator desig
                           :perception-source :whatever)))
                      resulting-designators)
              (cram-occasions-events:on-event
               (make-instance 'cram-plan-occasions-events:object-perceived-event
                 :object-designator resulting-designators
                 :perception-source :whatever)))
          (desig:equate ?object-designator resulting-designator)
          resulting-designator)))))


(defun move-arms-in-sequence (&key
                                left-poses
                                right-poses
                                ((:collision-mode ?collision-mode))
                                ((:collision-object-b ?collision-object-b))
                                ((:collision-object-b-link ?collision-object-b-link))
                                ((:collision-object-a ?collision-object-a))
                                ((:move-the-ass ?move-the-ass))
                              &allow-other-keys)
  (declare (type (or list cl-transforms-stamped:pose-stamped) left-poses right-poses)
           (type (or null keyword) ?collision-mode)
           (type (or null symbol) ?collision-object-b ?collision-object-a)
           (type (or null string symbol) ?collision-object-b-link))
  "Move arms through all but last poses of `left-poses' and `right-poses',
while ignoring failures; and execute the last pose with propagating the failures."

  ;; Make `left-poses' and `right-poses' to lists if they are not already
  (unless (listp left-poses)
    (setf left-poses (list left-poses)))
  (unless (listp right-poses)
    (setf right-poses (list right-poses)))
  (multiple-value-bind (left-poses right-poses)
      (cut:equalize-two-list-lengths (butlast left-poses) (butlast right-poses))

    ;; Move arms through all but last poses of `?left-poses' and `?right-poses'
    ;; while ignoring failures: accuracy is not so important in intermediate poses.
    (mapc (lambda (?left-pose ?right-pose)

            (cpl:with-failure-handling
                ((common-fail:manipulation-low-level-failure (e) ; ignore failures
                   (roslisp:ros-warn (pick-place move-arms-in-sequence) "~a~%Ignoring." e)
                   (return)))

              (exe:perform
               (desig:a motion
                        (type moving-tcp)
                        (desig:when ?left-pose
                          (left-pose ?left-pose))
                        (desig:when ?right-pose
                          (right-pose ?right-pose))
                        (desig:when ?collision-mode
                          (collision-mode ?collision-mode))
                        (desig:when ?collision-object-b
                          (collision-object-b ?collision-object-b))
                        (desig:when ?collision-object-b-link
                          (collision-object-b-link ?collision-object-b-link))
                        (desig:when ?collision-object-a
                          (collision-object-a ?collision-object-a))
                        (desig:when ?move-the-ass
                          (move-the-ass ?move-the-ass))))

              (cram-occasions-events:on-event
               (make-instance 'cram-plan-occasions-events:robot-state-changed))))

          left-poses right-poses))

  ;; Move arm to the last pose of `?left-poses' and `?right-poses'.
  (let ((?left-pose (car (last left-poses)))
        (?right-pose (car (last right-poses))))

    (cpl:with-failure-handling
        ((common-fail:manipulation-low-level-failure (e)
           ;; propagate failures up
           (roslisp:ros-error (pick-place move-arms-in-sequence) "~a~%Failing." e)))

      (exe:perform
       (desig:a motion
                (type moving-tcp)
                (desig:when ?left-pose
                  (left-pose ?left-pose))
                (desig:when ?right-pose
                  (right-pose ?right-pose))
                (desig:when ?collision-mode
                  (collision-mode ?collision-mode))
                (desig:when ?collision-object-b
                  (collision-object-b ?collision-object-b))
                (desig:when ?collision-object-b-link
                  (collision-object-b-link ?collision-object-b-link))
                (desig:when ?collision-object-a
                  (collision-object-a ?collision-object-a))
                (desig:when ?move-the-ass
                  (move-the-ass ?move-the-ass))))

      (cram-occasions-events:on-event
       (make-instance 'cram-plan-occasions-events:robot-state-changed)))))


(defun move-arms-into-configuration (&key
                                       ((:left-joint-states ?left-joint-states))
                                       ((:right-joint-states ?right-joint-states))
                                     &allow-other-keys)
  (declare (type list ?left-joint-states ?right-joint-states))
  "Calls moving-arm-joints motion, while ignoring failures, and robot-state-changed event."

  (unwind-protect
       (cpl:with-failure-handling
           ((common-fail:manipulation-low-level-failure (e)
              (roslisp:ros-warn (mobile-pp-plans move-arms-into-configuration)
                                "A low-level manipulation failure happened: ~a~%Ignoring." e)
              (return)))

         (exe:perform
          (desig:a motion
                   (type moving-arm-joints)
                   (desig:when ?left-joint-states
                     (left-joint-states ?left-joint-states))
                   (desig:when ?right-joint-states
                     (right-joint-states ?right-joint-states))))
         ;; (cpl:seq
         ;;   (exe:perform
         ;;    (desig:a motion
         ;;             (type moving-arm-joints)
         ;;             (right-joint-states ?right-configuration)))
         ;;   (exe:perform
         ;;    (desig:a motion
         ;;             (type moving-arm-joints)
         ;;             (left-joint-states ?left-configuration))))
         )
    (cram-occasions-events:on-event
     (make-instance 'cram-plan-occasions-events:robot-state-changed))))


(defun release (&key
                  ((:gripper ?left-or-right))
                &allow-other-keys)
  (declare (type (or keyword list) ?left-or-right))
  "Call OPENING-GRIPPER motion while ignoring failures and issue robot-state-changed event"

  (unwind-protect
       (cpl:with-failure-handling
           ((common-fail:gripper-low-level-failure (e) ; ignore failures
              (roslisp:ros-warn (pick-and-place release) "~a" e)
              (return)))
         (exe:perform
          (desig:a motion
                   (type opening-gripper)
                   (gripper ?left-or-right))))
    (cram-occasions-events:on-event
     (make-instance 'cram-plan-occasions-events:robot-state-changed))))

(defun grip (&key
               ((:gripper ?left-or-right))
               ((:effort ?effort))
             &allow-other-keys)
  (declare (type (or keyword list) ?left-or-right)
           (type (or number null) ?effort))
  "Perform GRIPPING motion and retries once, otherwise propagate failure up.
In any case, issue ROBOT-STATE-CHANGED event."

  (unwind-protect
       (cpl:with-retry-counters ((grasping-retries 1))
         (cpl:with-failure-handling
             ((common-fail:gripper-low-level-failure (e) ; regrasp once then propagate up
                (cpl:do-retry grasping-retries
                  (roslisp:ros-warn (pick-and-place grip) "~a~%Retrying" e)
                  (cpl:retry))
                (roslisp:ros-warn (pick-and-place grip) "No retries left. Propagating up.")))
           (exe:perform
            (desig:a motion
                     (type gripping)
                     (gripper ?left-or-right)
                     (desig:when ?effort
                       (effort ?effort))))))
    (cram-occasions-events:on-event
     (make-instance 'cram-plan-occasions-events:robot-state-changed))))

(defun open-or-close-gripper (&key
                                ((:type ?action-type))
                                ((:gripper ?left-or-right))
                              &allow-other-keys)
  (declare (type keyword ?action-type)
           (type (or keyword list) ?left-or-right))
  "Call ACTION-TYPE motion while ignoring failures and issue robot-state-changed."

  (unwind-protect
       (cpl:with-failure-handling
           ((common-fail:gripper-low-level-failure (e) ; ignore failures
              (roslisp:ros-warn (pick-and-place close-gripper) "~a" e)
              (return)))
         (exe:perform
          (desig:a motion
                   (type ?action-type)
                   (gripper ?left-or-right))))
    (cram-occasions-events:on-event
     (make-instance 'cram-plan-occasions-events:robot-state-changed))))

(defun set-gripper-to-position (&key
                                  ((:gripper ?left-or-right))
                                  ((:position ?position))
                                &allow-other-keys)
  (declare (type (or keyword list) ?left-or-right)
           (type number ?position))
  "Call MOVING-GRIPPER-JOINT motion while ignoring failures and issue robot-state-changed."

  (unwind-protect
       (cpl:with-failure-handling
           ((common-fail:gripper-low-level-failure (e) ; ignore failures
              (roslisp:ros-warn (pick-and-place set-gripper-to-pos) "~a" e)
              (return)))
         (exe:perform
          (desig:a motion
                   (type moving-gripper-joint)
                   (gripper ?left-or-right)
                   (joint-angle ?position))))
    (cram-occasions-events:on-event
     (make-instance 'cram-plan-occasions-events:robot-state-changed))))


(defun look-at (&key
                  ((:pose ?pose))
                  ((:joint-states ?joint-states))
                &allow-other-keys)
  (declare (type (or cl-transforms-stamped:pose-stamped null) ?pose)
           (type list ?joint-states))
  "Perform LOOKING motion and retry once on failures, otherwise propagate failure up.
In any case, issue ROBOT-STATE-CHANGED event."

  (unwind-protect

       (cpl:with-retry-counters ((look-retries 1))
         (cpl:with-failure-handling
             ((common-fail:ptu-low-level-failure (e)
                (roslisp:ros-warn (pp-plans look-at) "Looking-at had a problem: ~a" e)
                (cpl:do-retry look-retries
                  (roslisp:ros-warn (pp-plans look-at) "Retrying.")
                  (cpl:retry)))))

         (exe:perform
          (desig:a motion
                   (type looking)
                   (desig:when ?pose
                     (pose ?pose))
                   (desig:when ?joint-states
                     (joint-states ?joint-states)))))

    (cram-occasions-events:on-event
     (make-instance 'cram-plan-occasions-events:robot-state-changed))))
