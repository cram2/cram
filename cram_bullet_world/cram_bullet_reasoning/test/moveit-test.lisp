
(in-package :bullet-reasoning-tests)

(defun compute-ik-client (position-ik-request)
  (roslisp:with-ros-node ("moveit-test-client")
    (if (not (roslisp:wait-for-service "compute_ik" 10))
        (roslisp:ros-warn (bullet-reasoning-tests)
                          "Timed out waiting for service moveit/compute_ik")
        (roslisp:call-service "compute_ik"
                              "moveit_msgs/GetPositionIK"
                              :ik_request position-ik-request))))

(define-test test-moveit-service-setup
  (let* ((request (roslisp:make-message
                   "moveit_msgs/PositionIKRequest"
                   :pose_stamped (cl-transforms-stamped:to-msg
                                  (cl-transforms-stamped:make-pose-stamped
                                   "torso_lift_link"
                                   (roslisp:ros-time)
                                   (cl-transforms:make-3d-vector 0.45 0.2 0.31)
                                   (cl-transforms:make-identity-rotation)))
                   :group_name "right_arm"
                   :timeout 1.0))
         (result (compute-ik-client request)))
    (assert-eql (roslisp-msg-protocol:symbol-code 'moveit_msgs-msg:moveiterrorcodes
                                                  :success)
                (moveit_msgs-msg:val (moveit_msgs-srv:error_code result)))))
