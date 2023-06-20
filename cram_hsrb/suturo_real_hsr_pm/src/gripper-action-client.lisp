(in-package :su-real)

(defparameter *gripper-action-client* NIL)

(defun get-gripper-action-client ()
  "returns the currently used gripper-action client. If none yet exists,
   creates one." 
  (roslisp:ros-info (gripper-action-client)
                    "Getting gripper action client")
  (or *gripper-action-client*
      (init-gripper-action-client)))

(defun init-gripper-action-client ()
  "initializes the gripper-action-client and makes sure it is connected to the
action server."
  (roslisp:ros-info (gripper-action-client)
                    "Initialising gripper action client")
  (setf *gripper-action-client*
        (actionlib:make-action-client "/hsrb/gripper_controller/grasp"
                                      "tmc_control_msgs/GripperApplyEffortAction"))
  (loop until
        (actionlib:wait-for-server *gripper-action-client*))
  (roslisp:ros-info (gripper-action-client)
                    "gripper action client initialised"))

(roslisp-utilities:register-ros-init-function init-gripper-action-client)

(defun call-gripper-action (effort)

  ;; call the actionlib action
  (multiple-value-bind (result status)
      (actionlib:call-goal (get-gripper-action-client)
                           (make-gripper-action-goal
                            :effort effort))

    ;; print a debug statement if the status is unexpected
    (case status
      (:preempted
       (roslisp:ros-warn (giskard action-client)
                         "Gripper action preempted.~%Result: ~a" result))
      (:timeout
       (roslisp:ros-warn (giskard action-client)
                         "Gripper action timed out."))
      (:aborted
       (roslisp:ros-warn (giskard action-client)
                         ;; "Giskard action aborted.~%Result: ~a" result
                         "Gripper action aborted.")))

    ;; return the result and status
    (values result status)))

(defun make-gripper-action-goal (&key effort)
  (actionlib:make-action-goal
      (get-gripper-action-client)
    :effort effort))
