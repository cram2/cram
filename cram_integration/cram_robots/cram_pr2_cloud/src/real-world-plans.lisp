;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :pr2-cloud)

(defmacro with-real-robot (&body body)
  `(cram-process-modules:with-process-modules-running
       (pr2-pms::pr2-perception-pm pr2-pms::pr2-base-pm pr2-pms::pr2-arms-pm
                                   pr2-pms::pr2-grippers-pm pr2-pms::pr2-ptu-pm)
     (cpl:top-level
       ,@body)))

(defun test-gripper ()
  (with-real-robot
    (exe:perform
     (desig:a motion (type opening) (gripper left)))))

(defun test-navigation ()
  (with-real-robot
    (exe:perform
     (let ((?pose (cl-transforms-stamped:make-pose-stamped
                   "map" 0.0
                   (cl-transforms:make-3d-vector 0.5 -0.8 0)
                   (cl-transforms:make-identity-rotation))))
       (desig:a motion (type going) (target (desig:a location (pose ?pose))))))))

(defun test-manipulation ()
  (with-real-robot
    (let ((?pose (local-robot-pose-in-map-from-handle)))
      (exe:perform
       (desig:an action (type going) (target (desig:a location (pose ?pose))))))
    (let ((?pose (strip-transform-stamped
                  (car (subseq (local-gripper-trajectory-in-base "MoveFridgeHandle") 23)))))
      (exe:perform
       (desig:a motion (type moving-tcp) (left-target (desig:a location (pose ?pose))))))))


(defun park-arms ()
  (let ((?left-pose (cl-transforms-stamped:make-pose-stamped
                     cram-tf:*robot-base-frame*
                     0.0
                     (cl-transforms:make-3d-vector 0.09611d0 0.68d0 0.35466d0)
                     (cl-transforms:make-quaternion -0.45742778331019085d0
                                                    0.3060123951483878d0
                                                    0.3788581151804847d0
                                                    0.744031427853262d0)))
        (?right-pose (cl-transforms-stamped:make-pose-stamped
                      cram-tf:*robot-base-frame*
                      0.0
                      (cl-transforms:make-3d-vector 0.0848d0 -0.712d0 0.35541d0)
                      (cl-transforms:make-quaternion -0.061062529688043946d0
                                                     -0.6133522138254498d0
                                                     0.197733462359113d0
                                                     -0.7622151317882601d0))))
    (exe:perform
     (desig:a motion
              (type moving-tcp)
              (left-target (desig:a location (pose ?left-pose)))
              (right-target (desig:a location (pose ?right-pose)))))     ))

(defun move-projected-pr2-away ()
  (btr-utils:move-object 'cram-pr2-description:pr2
                         (cl-transforms:make-pose
                          (cl-transforms:make-3d-vector 0 0 0)
                          (cl-transforms:make-identity-rotation))))

(defun environment-articulation-plan (&key (projected-or-original :original))
  (let* ((?handle-pose (strip-transform-stamped (local-handle-transform)))
         (?arm (kr-cloud::arm-used-in-action "OpenFridge"))
         (?robot 'cram-pr2-description:pr2)
         (?location-for-robot (desig:a location
                                       (reachable-for ?robot)
                                       (target ?handle-pose)
                                       (context "OpenFridge")))
         (?target (ecase ?arm (:left :left-target) (:right :right-target))))

    (btr-utils:kill-all-objects)
    (move-projected-pr2-away)
    (cram-bullet-reasoning::clear-costmap-vis-object)
    (park-arms)

    (cpl:with-failure-handling
        (((or common-fail:actionlib-action-timed-out
              common-fail:manipulation-low-level-failure) (e)
           (format t "Manipulation failed: ~a. Retrying.~%" e)
           (btr-utils:spawn-object 'red-dot :pancake-maker :color '(1 0 0 0.5)
                                                           :pose '((1.5 -1.05 1.6) (0 0 0 1)))
           (cpl:sleep 0.5)
           (park-arms)
           (move-projected-pr2-away)
           (setf ?location-for-robot (desig:next-solution ?location-for-robot))
           (cpl:sleep 0.5)
           (btr-utils:kill-object 'red-dot)
           (cpl:retry)))

      (cpl:par
        (exe:perform
         (desig:an action (type going) (target ?location-for-robot)))
        (exe:perform
         (desig:an action (type opening) (gripper ?arm))))

      (let ((?trajectory (filter-trajectory-in-base
                          (gripper-trajectory-in-map->in-base
                           (ecase projected-or-original
                             (:original
                              (local-gripper-trajectory-in-map
                               "MoveFridgeHandle"))
                             (:projected
                              (local-gripper-projected-trajectory-in-map
                               "MoveFridgeHandle")))))))

        (cpl:with-retry-counters ((reach-retry 2))
          (cpl:with-failure-handling
              ((common-fail:low-level-failure (e)
                 (roslisp:ros-warn (pr2-cloud open-fridge) "~a" e)
                 (cpl:do-retry reach-retry
                   (cpl:retry))))
            (let ((?pose (strip-transform-stamped (first ?trajectory))))
              (cram-pr2-low-level:visualize-marker ?pose)
              (exe:perform
               (desig:a motion
                        (type moving-tcp)
                        (?target (desig:a location (pose ?pose))))))))

        (exe:perform
         (desig:an action (type closing) (gripper ?arm)))

        (mapcar (lambda (transform)
                  (exe:perform
                   (let ((?pose (strip-transform-stamped transform)))
                     (desig:a motion
                              (type moving-tcp)
                              (?target (desig:a location (pose ?pose)))))))
                (cdr ?trajectory))

        (exe:perform
         (desig:an action (type opening) (gripper ?arm)))

        (btr-utils:spawn-object 'green-dot :pancake-maker :color '(0 1 0 0.5)
                                                          :pose '((1.5 -1.05 1.6) (0 0 0 1)))))))

;;; arms down

