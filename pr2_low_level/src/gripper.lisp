
(in-package :pr2)

(defvar *gripper-action-clients* '(:left nil :right nil)
  "A list to store PR2 gripper action clients for left and right gripper.")

(defun init-gripper-action-client (left-or-right)
  (let ((action-server-name (concatenate
                             'string
                             (ecase left-or-right
                               (:left "l")
                               (:right "r"))
                             "_gripper_controller/gripper_action")))
    (setf (getf *gripper-action-clients* left-or-right)
          (actionlib:make-action-client
           action-server-name
           "pr2_controllers_msgs/Pr2GripperCommandAction"))
    (loop until (actionlib:wait-for-server
                 (getf *gripper-action-clients* left-or-right)
                 5.0)
          do (roslisp:ros-info (gripper-action)
                               "Waiting for ~a gripper action server."
                               left-or-right))
    (roslisp:ros-info (gripper-action)
                      "~a gripper action client created."
                      left-or-right)))

(defun init-gripper-action-clients ()
  (mapc #'init-gripper-action-client '(:left :right)))

(defun destroy-gripper-action-clients ()
  (setf *gripper-action-clients* '(:left nil :right nil)))

(roslisp-utilities:register-ros-cleanup-function destroy-gripper-action-clients)


(defun get-gripper-action-client (left-or-right)
  (or (getf *gripper-action-clients* left-or-right)
      (init-gripper-action-client left-or-right)))

(defun make-gripper-action-goal (action-client position &optional max-effort)
  (actionlib:make-action-goal action-client
    (position command) (etypecase position
                         (number (cond
                                   ((< position 0.0)
                                    (roslisp:ros-warn
                                     (gripper-action)
                                     "POSITION (~a) cannot be smaller than 0.0. Clipping."
                                     position)
                                    0.0)
                                   ((> position 0.085)
                                    (roslisp:ros-warn
                                     (gripper-action)
                                     "POSITION (~a) shouldn't be bigger than 0.085. Clipping."
                                     position)
                                    0.085)
                                   (t
                                    position)))
                         (keyword (ecase position
                                    (:open 0.085)
                                    (:close 0.0))))
    (max_effort command) (or max-effort (etypecase position
                                          (number 50.0)
                                          (keyword (ecase position
                                                     (:open -1)
                                                     (:close 50.0)))))))

;; TODO: when something in the hand, gripper action result is :ABORTED
;; and STALLED is T. Use this for checking if grasp succeeded.
(defun call-gripper-action (left-or-right position &optional max-effort)
  (multiple-value-bind (result status)
      (cpl:with-failure-handling
          ((simple-error (e)
             (format t "Actionlib error occured!~%~a~%Reinitializing...~%~%" e)
             (init-gripper-action-clients)
             (cpl:retry)))
        (let ((actionlib:*action-server-timeout* 10.0)
              (action-client (get-gripper-action-client left-or-right)))
          (actionlib:call-goal
           action-client
           (make-gripper-action-goal action-client position max-effort)
           :timeout 10.0)))
    (roslisp:ros-info (gripper-action) "~a gripper action finished: ~a."
                      left-or-right status)
    (values result status)))
