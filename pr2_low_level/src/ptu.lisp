
(in-package :pr2)

(defvar *ptu-action-client* nil)

(defun init-ptu-action-client ()
  (setf *ptu-action-client* (actionlib:make-action-client
                             "head_traj_controller/point_head_action"
                             "pr2_controllers_msgs/PointHeadAction"))
  (loop until (actionlib:wait-for-server *ptu-action-client* 5.0)
        do (roslisp:ros-info (ptu-action-client) "Waiting for PTU action server..."))
  (roslisp:ros-info (ptu-action-client) "PTU action client created.")
  *ptu-action-client*)

(defun destroy-ptu-action-client ()
  (setf *ptu-action-client* nil))

(roslisp-utilities:register-ros-cleanup-function destroy-ptu-action-client)

(defun get-ptu-action-client ()
  (or *ptu-action-client*
      (init-ptu-action-client)))

(defun make-ptu-action-goal (frame point)
  (actionlib:make-action-goal
      (get-ptu-action-client)
    max_velocity 10
    min_duration 0.3
    pointing_frame "high_def_frame"
    (x pointing_axis) 1.0
    (y pointing_axis) 0.0
    (z pointing_axis) 0.0
    target (cl-transforms-stamped:to-msg
            (cl-transforms-stamped:make-point-stamped frame 0.0 point))))

(defun call-ptu-action (&key (frame "base_link") (point (cl-transforms:make-identity-vector)))
  (multiple-value-bind (result status)
      (cpl:with-failure-handling
          ((simple-error (e)
             (format t "Actionlib error occured!~%~a~%Reinitializing...~%~%" e)
             (init-ptu-action-client)
             (cpl:retry)))
        (let ((actionlib:*action-server-timeout* 10.0))
          (actionlib:call-goal
           (get-ptu-action-client)
           (make-ptu-action-goal frame point)
           :timeout 5.0)))
    (roslisp:ros-info (ptu-action-client) "PTU action finished.")
    (values result status)))

(defun shake-head (n-times)
  (dotimes (n n-times)
    (call-ptu-action :point (cl-transforms:make-3d-vector 5.0 1.0 1.2))
    (call-ptu-action :point (cl-transforms:make-3d-vector 5.0 -1.0 1.2))))

(defun look-at-gripper (left-or-right)
  (call-ptu-action :frame (ecase left-or-right
                            (:left "l_gripper_tool_frame")
                            (:right "r_gripper_tool_frame"))))
