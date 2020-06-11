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
       (pr2-pms::pr2-base-pm pr2-pms::pr2-arms-pm
                             pr2-pms::pr2-grippers-pm pr2-pms::pr2-ptu-pm)
     (cpl:top-level
       ,@body)))

(defun test-gripper ()
  (with-real-robot
    (exe:perform
     (desig:a motion (type opening-gripper) (gripper left)))))

(defun test-navigation ()
  (with-real-robot
    (exe:perform
     (let ((?pose (cl-transforms-stamped:make-pose-stamped
                   "map" 0.0
                   (cl-transforms:make-3d-vector 0.5 -0.8 0)
                   (cl-transforms:make-identity-rotation))))
       (desig:a motion (type going) (pose ?pose))))))

;; (defun test-manipulation ()
;;   (with-real-robot
;;     (let ((?pose (local-robot-pose-in-map-from-handle)))
;;       (exe:perform
;;        (desig:an action (type going) (target (desig:a location (pose ?pose))))))
;;     (let ((?pose (strip-transform-stamped
;;                   (car (subseq (local-gripper-trajectory-in-base "MoveFridgeHandle") 23)))))
;;       (exe:perform
;;        (desig:a motion (type moving-tcp) (left-pose ?pose))))))

(defun test-manipulation ()
  (with-real-robot
    (let ((?pose (strip-transform-stamped
                  (car (local-gripper-trajectory-in-base-from-radius)))))
      (exe:perform
       (desig:a motion (type moving-tcp) (left-pose ?pose))))))

(defun move-projected-pr2-away ()
  (btr-utils:move-object 'cram-pr2-description:pr2
                         (cl-transforms:make-pose
                          (cl-transforms:make-3d-vector 0 0 0)
                          (cl-transforms:make-identity-rotation))))

(defun environment-articulation-plan (&key (projected-or-original :original)
                                        (location-designator))
  (let* ((?handle-pose (strip-transform-stamped (local-handle-transform)))
         (?arm (kr-cloud::arm-used-in-action "OpenFridge"))
         (?robot 'cram-pr2-description:pr2)
         (?location-for-robot (or location-designator
                                  (desig:a location
                                           (my-reachable-for ?robot)
                                           (location (desig:a location (pose ?handle-pose)))
                                           (context "OpenFridge"))))
         (?target (ecase ?arm (:left :left-pose) (:right :right-pose))))

    (btr-utils:kill-all-objects)
    (move-projected-pr2-away)
    (cram-bullet-reasoning::clear-costmap-vis-object)
    (exe:perform (desig:an action
                           (type parking-arms)))
    ;; (exe:perform (desig:a motion (type moving-torso) (joint-angle 0.15)))

    (cpl:with-failure-handling
        (((or common-fail:actionlib-action-timed-out
              common-fail:manipulation-low-level-failure) (e)
           (roslisp:ros-warn (pr2-cloud open-fridge) "Plan failed: ~a. Retrying.~%" e)
           (btr-utils:spawn-object 'red-dot :pancake-maker :color '(1 0 0 0.5)
                                                           :pose '((1.5 -1.05 1.6) (0 0 0 1)))
           (cpl:sleep 0.5)
           (exe:perform (desig:an action (type opening-gripper) (gripper right)))
           (exe:perform (desig:an action
                                  (type parking-arms)))
           (move-projected-pr2-away)
           (unless location-designator
             (setf ?location-for-robot (desig:next-solution ?location-for-robot)))
           (cpl:sleep 0.5)
           (btr-utils:kill-object 'red-dot)
           (cpl:retry)))

      (cpl:par
        (exe:perform
         (desig:an action (type going) (target ?location-for-robot)))
        (exe:perform
         (desig:an action (type opening-gripper) (gripper ?arm))))

      (let ((?right-arm-init-config '(-0.6177226607749531d0 0.8595855682477204d0
                                      -0.22685404643554485d0 -2.1215879821638572d0
                                      -27.55566886200435d0 -1.9241091851082714d0
                                      9.343508274913418d0)))
        (exe:perform
         (desig:a motion
                  (type moving-arm-joints)
                  (right-joint-states ?right-arm-init-config))))

      (let ((?trajectory
              (gripper-trajectory-in-map->in-base
               (ecase projected-or-original
                 (:original
                  (local-gripper-trajectory-in-map
                   "MoveFridgeHandle"))
                 (:projected
                  (local-gripper-projected-trajectory-in-map
                   "MoveFridgeHandle"))))))

        (cpl:with-retry-counters ((reach-retry 2))
          (cpl:with-failure-handling
              ((common-fail:low-level-failure (e)
                 (roslisp:ros-warn (pr2-cloud open-fridge) "~a" e)
                 (cpl:do-retry reach-retry
                   (cpl:retry))
                 (return)))
            (let ((?pose (strip-transform-stamped (first ?trajectory))))
              (cram-tf:visualize-marker ?pose)
              (exe:perform
               (desig:a motion
                        (type moving-tcp)
                        (?target ?pose))))))
        (exe:perform
         (desig:an action (type gripping) (gripper ?arm) (effort 100)))

        (mapcar (lambda (transform)
                  (let ((?pose (strip-transform-stamped transform)))
                    (cpl:with-failure-handling
                        ((common-fail:manipulation-goal-not-reached (e)
                           (when location-designator
                             (roslisp:ros-warn (pr2-cloud execute-traj) "~a" e)
                             (roslisp:ros-warn (pr2-cloud execute-traj) "Ignoring.")
                             (return))))
                      (exe:perform
                       (desig:a motion
                                (type moving-tcp)
                                (?target ?pose))))))
                (cdr ?trajectory))

        (exe:perform
         (desig:an action (type opening-gripper) (gripper ?arm)))

        (btr-utils:spawn-object 'green-dot :pancake-maker :color '(0 1 0 0.5)
                                                          :pose '((1.5 -1.05 1.6) (0 0 0 1)))

        (let ((?right-arm-init-config '(-2.040980560942328d0 0.2712278780562381d0
                                        -0.6460215625664274d0 -2.028500414744249d0
                                        -26.026134035944583d0 -1.1912449349507037d0
                                        12.292977968464921d0)))
        (exe:perform
         (desig:a motion
                  (type moving-arm-joints)
                  (right-joint-states ?right-arm-init-config))))

        ?location-for-robot))))



(defun main (&optional (real? t))
  (pr2-cloud::init)
  (let ((location (with-simulated-robot
                    (pr2-cloud::environment-articulation-plan
                     :projected-or-original :projected))))
    (sleep 3.0)
    (when real?
     (with-real-robot
       (pr2-cloud::environment-articulation-plan 
        :projected-or-original :projected
        :location-designator location)))))

