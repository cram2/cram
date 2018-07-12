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

(cpl:def-cram-function go-to-target (?location-designator)
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
            (desig:a motion (type going) (target ?location-designator)))))
    (cram-occasions-events:on-event
     (make-instance 'cram-plan-occasions-events:robot-state-changed))))


(cpl:def-cram-function perceive (?object-designator
                                 &key
                                 (object-chosing-function #'identity))
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


(cpl:def-cram-function move-arms-in-sequence (left-poses right-poses &optional ?collision-mode)
  "Make `?left-poses' and `?right-poses' to lists if they are not already"

  (flet ((fill-in-with-nils (some-list desired-length)
           (let ((current-length (length some-list)))
             (if (> desired-length current-length)
                 (append some-list (make-list (- desired-length current-length)))
                 some-list))))

    (unless (listp left-poses)
      (setf left-poses (list left-poses)))
    (unless (listp right-poses)
      (setf right-poses (list right-poses)))

    ;; Move arms through all but last poses of `?left-poses' and `?right-poses'
    ;; while ignoring failures: accuracy is not so important in intermediate poses.
    (let ((max-length (max (length left-poses) (length right-poses))))
      (mapc (lambda (?left-pose ?right-pose)

              (cpl:with-failure-handling
                  ((common-fail:manipulation-low-level-failure (e) ; ignore failures
                     (roslisp:ros-warn (pick-place move-arms-in-sequence) "~a~%Ignoring." e)
                     (return)))

                (exe:perform
                 (desig:a motion
                          (type moving-tcp)
                          (desig:when ?left-pose
                            (left-target (desig:a location (pose ?left-pose))))
                          (desig:when ?right-pose
                            (right-target (desig:a location (pose ?right-pose))))
                          (desig:when ?collision-mode
                            (collision-mode ?collision-mode))))

                (cram-occasions-events:on-event
                 (make-instance 'cram-plan-occasions-events:robot-state-changed))))

            (fill-in-with-nils (butlast left-poses) max-length)
            (fill-in-with-nils (butlast right-poses) max-length)))

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
                    (left-target (desig:a location (pose ?left-pose))))
                  (desig:when ?right-pose
                    (right-target (desig:a location (pose ?right-pose))))
                  (desig:when ?collision-mode
                    (collision-mode ?collision-mode))))

        (cram-occasions-events:on-event
         (make-instance 'cram-plan-occasions-events:robot-state-changed))))))


(cpl:def-cram-function park-arms (&key (arm '(:left :right)) (carry t))
  (let ((carry?
          (or (prolog:prolog `(cpoe:object-in-hand ?obj ?arm))
              carry)))
   (flet ((get-arm-parking-joint-states (arm)
            (let* ((bindings
                     (if carry?
                         (prolog:prolog
                          `(and (cram-robot-interfaces:robot ?robot)
                                (cram-robot-interfaces:robot-arms-carrying-joint-states
                                 ?robot ?joint-states ,arm)))
                         (prolog:prolog
                          `(and (cram-robot-interfaces:robot ?robot)
                                (cram-robot-interfaces:robot-arms-parking-joint-states
                                 ?robot ?joint-states ,arm)))))
                   (joint-states (cut:var-value '?joint-states (car bindings))))
              (unless joint-states
                (error "ROBOT-ARMS-PARKING-JOINT-STATES undefined! ~
                        Did you forget to load a robot description package?"))
              (mapcar #'second joint-states))))

     (unless (listp arm)
       (setf arm (list arm)))
     (let (?left-configuration ?right-configuration)
       (when (member :left arm)
         (setf ?left-configuration (get-arm-parking-joint-states :left)))
       (when (member :right arm)
         (setf ?right-configuration (get-arm-parking-joint-states :right)))

       (unwind-protect
            (cpl:with-failure-handling
                ((common-fail:manipulation-low-level-failure (e)
                   (roslisp:ros-warn (pick-and-place park-arms)
                                     "A low-level manipulation failure happened: ~a~%Ignoring." e)
                   (return)))

              (if carry?
                  (exe:perform
                     (desig:a motion
                              (type moving-arm-joints)
                              (left-configuration ?left-configuration)
                              (right-configuration ?right-configuration)))
                  ;; (cpl:seq
                  ;;   (exe:perform
                  ;;    (desig:a motion
                  ;;             (type moving-arm-joints)
                  ;;             (right-configuration ?right-configuration)))
                  ;;   (exe:perform
                  ;;    (desig:a motion
                  ;;             (type moving-arm-joints)
                  ;;             (left-configuration ?left-configuration))))
                  (cpl:seq
                    (exe:perform
                     (desig:a motion
                              (type moving-arm-joints)
                              (left-configuration ?left-configuration)))
                    (exe:perform
                     (desig:a motion
                              (type moving-arm-joints)
                              (right-configuration ?right-configuration))))))
         (cram-occasions-events:on-event
          (make-instance 'cram-plan-occasions-events:robot-state-changed)))))))


(cpl:def-cram-function release (?left-or-right)
  (unwind-protect
       (cpl:with-failure-handling
           ((common-fail:gripper-low-level-failure (e) ; ignore failures
              (roslisp:ros-warn (pick-and-place release) "~a" e)
              (return)))
         (exe:perform
          (desig:a motion
                   (type opening)
                   (gripper ?left-or-right))))
    (cram-occasions-events:on-event
     (make-instance 'cram-plan-occasions-events:robot-state-changed))))

(cpl:def-cram-function open-or-close-gripper (?left-or-right ?action-type)
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

(cpl:def-cram-function set-gripper-to-position (?left-or-right ?position)
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

(cpl:def-cram-function grip (?left-or-right ?effort)
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


(cpl:def-cram-function look-at (&key target frame direction object)
  (unwind-protect
       (cpl:with-retry-counters ((look-retries 1))
         (cpl:with-failure-handling
             ((common-fail:ptu-low-level-failure (e)
                (roslisp:ros-warn (pp-plans look-at) "Looking-at had a problem: ~a" e)
                (cpl:do-retry look-retries
                  (roslisp:ros-warn (pp-plans look-at) "Retrying.")
                  (cpl:retry)))))

         (cond (target
                (let ((?target target))
                  (exe:perform
                   (desig:a motion
                            (type looking)
                            (target ?target)))))
               (frame
                (let ((?frame frame))
                  (exe:perform
                   (desig:a motion
                            (type looking)
                            (frame ?frame)))))
               (direction
                (let ((?direction direction))
                  (exe:perform
                   (desig:a motion
                            (type looking)
                            (direction ?direction)))))
               (object
                (let ((?pose (cram-object-interfaces:get-object-pose object)))
                  (exe:perform
                   (desig:a motion
                            (type looking)
                            (target (desig:a location (pose ?pose)))))))))
    (cram-occasions-events:on-event
     (make-instance 'cram-plan-occasions-events:robot-state-changed))))
