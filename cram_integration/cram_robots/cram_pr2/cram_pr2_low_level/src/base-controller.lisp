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

(in-package :pr2-ll)

;; (defparameter *robot-base-frame* "base_footprint" "Robot's TF frame for navigation.")
;; (defparameter *fixed-frame* "odom_combined" "World's TF frame for localization.")


(defvar *robot-base-frame-pose* (cpl:make-fluent :name :robot-base-frame-pose)
  "Current pose of *robot-base-frame* compared to *fixed-frame*.")


(defparameter *navigation-linear-velocity-default* 0.2
  "In [meter / second].")

(defparameter *navigation-angular-velocity-max* (/ pi 4)
  "In [radiant / second].")

(defparameter *robot-pose-fluent-update-interval* 0.1
  "How often to update *robot-base-frame-pose*, in seconds.
If the robot is moving 0.2 [meter / second], then if we update the pose
every 0.1 [seconds], we will have maximum 0.1 * 0.2 [meter] = 2 [cm] error.")

(defparameter *navigation-goal-reached-accuracy* 0.1
  "With how much accuracy to perform the navigation to goal task, in meters.
If *robot-base-frame-pose* has maximum 2 [cm] error,
with goal accuracy of 0.1 [m] navigation will have 12 [cm] error max.")


;; (defparameter *tf-default-timeout* 2.0
;;   "Timeout to wait for transform in seconds. 0.0 means wait 0.0 seconds.")

;; (defvar *transformer* nil "TF transform listener")

;; (defun init-transformer ()
;;   "Initializes *transformer*"
;;   (setf *transformer* (make-instance 'cl-tf:transform-listener))
;;   (when (roslisp:get-param "use_sim_time" nil)
;;     (setf *tf-default-timeout* 4.0)))

;; (defun destroy-transformer ()
;;   (setf *transformer* nil))

;; (roslisp-utilities:register-ros-init-function init-transformer)
;; (roslisp-utilities:register-ros-cleanup-function destroy-transformer)


;; (defun lookup-robot-base-frame-pose (&key
;;                                        (timeout *tf-default-timeout*)
;;                                        (use-current-time?-otherwise-zero t))
;;   (if *transformer*
;;       (let ((time (if use-current-time?-otherwise-zero
;;                       (roslisp:ros-time)
;;                       0.0)))
;;         (handler-case
;;             (cl-transforms-stamped:transform-pose-stamped
;;              *transformer*
;;              :target-frame *fixed-frame*
;;              :pose (cl-transforms-stamped:make-pose-stamped
;;                     *robot-base-frame*
;;                     time
;;                     (cl-transforms:make-identity-vector)
;;                     (cl-transforms:make-identity-rotation))
;;              :timeout timeout)
;;           (cl-transforms-stamped:timeout-error (e)
;;             (error 'cl-transforms-stamped:timeout-error
;;                    :description (format nil "~a. Time used: ~a."
;;                                         (cl-transforms-stamped::description e)
;;                                         time)))))
;;       (error 'cl-transforms-stamped:transform-stamped-error :description
;;              "*transformer* is dead right now.")))

(defun update-robot-base-frame-pose (&optional (ignore-or-retry-or-error :error))
  (declare (type keyword ignore-or-retry-or-error))
  (cpl:with-failure-handling
        ((cl-transforms-stamped:transform-stamped-error (e)
           (roslisp:ros-warn (update-robot-pose) "TF error happened: ~a." e)
           (ecase ignore-or-retry-or-error
             (:ignore (roslisp:ros-warn (update-robot-pose) "Ignoring and returning.")
                      (return nil))
             (:retry (roslisp:ros-warn (update-robot-pose) "Retrying.")
                     (cpl:retry))
             (:error (error e)))))
      (let ((new-pose (robot-current-pose :use-current-time-p t)))
        (setf (cpl:value *robot-base-frame-pose*) new-pose))))

(defun loop-update-robot-base-frame-pose ()
  (roslisp:loop-at-most-every *robot-pose-fluent-update-interval*
    (update-robot-base-frame-pose :skip)))


(defvar *base-command-pub* nil
  "Velocity command publisher for base controller node")

(defun init-base-command-pub ()
  "Initializes *base-command-pub* ROS publisher"
  (setf *base-command-pub*
        (roslisp:advertise "base_controller/command"
                           "geometry_msgs/Twist")))

(defun get-base-command-pub ()
  (or *base-command-pub*
      (init-base-command-pub)))

(defun destroy-base-command-pub ()
  (setf *base-command-pub* nil))

(roslisp-utilities:register-ros-cleanup-function destroy-base-command-pub)


(defun send-velocity-command (linear-velocity angular-velocity)
  (roslisp:publish (get-base-command-pub)
                   (roslisp:make-message
                    "geometry_msgs/Twist"
                    (:x :linear) linear-velocity
                    (:z :angular) angular-velocity)))

(defun send-direction-command (direction)
  (send-velocity-command (if (eql direction :backward)
                             (- *navigation-linear-velocity-default*)
                             *navigation-linear-velocity-default*)
                         (case direction
                                    (:left *navigation-angular-velocity-max*)
                                    (:right (- *navigation-angular-velocity-max*))
                                    (t 0))))

(defun drive-keyboard ()
  (format t "Type a command and then press ENTER.~%")
  (format t "Use ASDW to move around and Q to exit.~%")

  (loop named input-loop
        for input-char = (read-char)
        for input-command = (if (eql input-char #\newline)
                                :n
                                (intern (string-upcase input-char) :keyword))
        do (case input-command
             (:q (return-from input-loop))
             ((:w :a :s :d) (dotimes (n 5)
                              (send-direction-command
                               (getf '(:w :forward
                                       :a :left
                                       :s :backward
                                       :d :right)
                                     input-command))
                              (roslisp:wait-duration 0.1)))
             (:n nil)
             (otherwise (format t "Use ASDW to move around and Q to exit.~%")))))







;; (defun send-velocity-command-in-loop (direction loop-rate goal-reached-pred)
;;   (loop
;;     until (funcall goal-reached-pred)
;;     do (send-direction-command direction)
;;        (roslisp:wait-duration loop-rate)))

;; (defun drive-forward-odom (distance &optional (direction :forward))
;;   (handler-case
;;       (let ((initial-transform
;;               (cl-transforms-stamped:lookup-transform
;;                *transformer*
;;                "odom_combined" "base_footprint"
;;                :timeout 2.0 :time 0.0)))
;;         (send-velocity-command-in-loop
;;          direction
;;          0.1
;;          (lambda ()
;;            (let* ((current-transform
;;                     (cl-transforms-stamped:lookup-transform
;;                      *transformer*
;;                      "odom_combined" "base_footprint"
;;                      :timeout 2.0 :time 0.0))
;;                   (relative-transform (cl-transforms:transform*
;;                                        (cl-transforms:transform-inv initial-transform)
;;                                        current-transform))
;;                   (distance-moved (cl-transforms:v-norm
;;                                    (cl-transforms:translation relative-transform))))
;;              (> distance-moved distance)))))
;;     (cl-transforms-stamped:transform-stamped-error (e)
;;       (roslisp:ros-error (odometry) "TF error happened: ~a~%" e))))

;; (defun turn-odom (angle &optional (direction :left))
;;   (assert (or (eql direction :left) (eql direction :right)) nil
;;           "In TURN-ODOM direction can only be :left or :right and not ~a~%" direction)
;;   (handler-case
;;       (let ((angle (cl-transforms:normalize-angle angle))
;;             (initial-transform (cl-transforms-stamped:lookup-transform
;;                                 *transformer*
;;                                 "odom_combined" "base_footprint"
;;                                 :timeout 2.0 :time 0.0)))
;;         (send-velocity-command-in-loop
;;          direction
;;          0.1
;;          (lambda ()
;;            (let* ((current-transform
;;                     (cl-transforms-stamped:lookup-transform
;;                      *transformer*
;;                      "odom_combined" "base_footprint"
;;                      :timeout 2.0 :time 0.0))
;;                   (relative-transform (cl-transforms:transform*
;;                                        (cl-transforms:transform-inv initial-transform)
;;                                        current-transform))
;;                   (angle-moved (cl-transforms:get-yaw
;;                                 (cl-transforms:rotation relative-transform))))
;;              (> angle-moved angle)))))
;;     (cl-transforms-stamped:transform-stamped-error (e)
;;       (roslisp:ros-error (odometry) "TF error happened: ~a~%" e))))



(defun drive-forward (distance &optional (direction :forward))
  (let ((initial-pose (update-robot-base-frame-pose :retry)))
    (flet ((calculate-distance-moved (new-pose)
             (cl-transforms:v-norm
              (cl-transforms:translation
               (cl-transforms:transform*
                (cl-transforms:transform-inv
                 (cl-transforms:pose->transform initial-pose))
                (cl-transforms:pose->transform new-pose))))))
      (let ((goal-reached-fl
              (cpl:>
               (cpl:fl-funcall #'calculate-distance-moved *robot-base-frame-pose*)
               distance)))
        (cpl:pursue
          (loop-update-robot-base-frame-pose)
          (roslisp:loop-at-most-every *robot-pose-fluent-update-interval*
            (send-direction-command direction))
          (cpl:wait-for goal-reached-fl))))))

(defun turn (angle &optional (direction :left))
  (assert (or (eql direction :left) (eql direction :right)) nil
          "In TURN-ODOM direction can only be :left or :right and not ~a~%" direction)
  (let ((angle (cl-transforms:normalize-angle
                (ecase direction
                  (:left angle)
                  (:right (- angle)))))
        (initial-transform (cl-transforms:pose->transform
                            (update-robot-base-frame-pose :retry))))
    (flet ((calculate-moved-angle (new-pose)
             (cl-transforms:get-yaw
              (cl-transforms:rotation
               (cl-transforms:transform*
                (cl-transforms:transform-inv initial-transform)
                (cl-transforms:pose->transform new-pose))))))
      (cpl:pursue
        (loop-update-robot-base-frame-pose)
        (roslisp:loop-at-most-every *robot-pose-fluent-update-interval*
          (send-direction-command direction))
        (cpl:wait-for
         (ecase direction
           (:left (cpl:>
             (cpl:fl-funcall #'calculate-moved-angle *robot-base-frame-pose*)
             angle))
           (:right (cpl:<
             (cpl:fl-funcall #'calculate-moved-angle *robot-base-frame-pose*)
             angle)))))
      (format t "DONE~%"))))


(defun relative-angle-to (goal-point original-pose)
  (declare (type cl-transforms:point goal-point)
           (type cl-transforms:pose original-pose))
  "Calculate the angle by which `original-pose' has to be turned to point at `goal-point'."
  (let ((diff-pose (cl-transforms:transform-point
                    (cl-transforms:transform-inv
                     (cl-transforms:pose->transform original-pose))
                    goal-point)))
    (atan
     (cl-transforms:y diff-pose)
     (cl-transforms:x diff-pose))))

(defun calculate-angular-cmd (goal-point)
  (declare (type cl-transforms:point goal-point))
  "Uses the current robot pose and calculates the angular velocity command
to turn towards the `goal-point'. Clips the signal with certain maximal value."
  (let* ((angle (cl-transforms:normalize-angle
                 (relative-angle-to goal-point (cpl:value *robot-base-frame-pose*))))
         (direction (if (> angle 0) 1 -1)))
    (if (> (abs angle) *navigation-angular-velocity-max*)
        (* direction *navigation-angular-velocity-max*)
        angle)))

(defun move-to (goal-point)
  (declare (type cl-transforms:point goal-point))
  "Sends velocity commands until `goal-point' is reached with `distance-threshold' accuracy."
  (flet ((check-if-goal-reached (current-pose)
           (< (cl-transforms:v-dist
               (cl-transforms:origin current-pose)
               (cl-transforms:copy-3d-vector
                goal-point
                :z 0))
              *navigation-goal-reached-accuracy*)))
    (update-robot-base-frame-pose :retry)
    (let ((reached-fl (cpl:fl-funcall #'check-if-goal-reached *robot-base-frame-pose*)))
      (unwind-protect
           (cpl:pursue
             (loop-update-robot-base-frame-pose)
             (cpl:wait-for reached-fl)
             (loop do
               (send-velocity-command
                *navigation-linear-velocity-default*
                (calculate-angular-cmd goal-point))
               (roslisp:wait-duration *robot-pose-fluent-update-interval*)))
        (send-velocity-command 0 0)))))
