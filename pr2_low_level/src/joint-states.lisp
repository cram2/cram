
(in-package :pr2)

(defvar *joint-state-sub* nil
  "Subscriber for robot's joint state topic.")

(defvar *robot-joint-states-msg* (cpl:make-fluent :name :robot-joint-states)
  "ROS message containing robot's current joint states.")

(defun init-joint-state-sub ()
  "Initializes *joint-state-sub*"
  (flet ((joint-state-sub-cb (joint-state-msg)
           (setf (cpl:value *robot-joint-states-msg*) joint-state-msg)))
    (setf *joint-state-sub*
          (roslisp:subscribe "joint_states"
                             "sensor_msgs/JointState"
                             #'joint-state-sub-cb))))
(defun destroy-joint-state-sub ()
  (setf *joint-state-sub* nil))

(roslisp-utilities:register-ros-init-function init-joint-state-sub)
(roslisp-utilities:register-ros-cleanup-function destroy-joint-state-sub)

(defclass joint-state ()
  ((name :reader joint-state-name
         :initarg :name
         :type string)
   (position :reader joint-state-position
             :initarg :position
             :type float)
   (velocity :reader joint-state-velocity
             :initarg :velocity
             :type float)
   (effort :reader joint-state-effort
           :initarg :effort
           :type float)))

(defun joint-states (names)
  "Returns the joint states of type JOINT-STATE + the corresponding timestamp
as multiple values."
  (let ((last-joint-state-msg (cpl:value *robot-joint-states-msg*)))
    (values
     (mapcar (lambda (name)
               (let ((index (position
                             name
                             (roslisp:msg-slot-value last-joint-state-msg :name)
                             :test #'string-equal)))
                 (when index
                   (make-instance 'joint-state
                     :name name
                     :position (aref (roslisp:msg-slot-value last-joint-state-msg :position)
                                     index)
                     :velocity (aref (roslisp:msg-slot-value last-joint-state-msg :velocity)
                                     index)
                     :effort (aref (roslisp:msg-slot-value last-joint-state-msg :effort)
                                   index)))))
             names)
     (roslisp:msg-slot-value
      (roslisp:msg-slot-value last-joint-state-msg :header)
      :stamp))))

(defun joint-positions (names)
  "Returns the joint positions as a list + timestamp"
  (let ((last-joint-state-msg (cpl:value *robot-joint-states-msg*)))
    (values
     (mapcar (lambda (name)
               (let ((index (position
                             name
                             (roslisp:msg-slot-value last-joint-state-msg :name)
                             :test #'string-equal)))
                 (when index
                   (aref (roslisp:msg-slot-value last-joint-state-msg :position)
                         index))))
             names)
     (roslisp:msg-slot-value
      (roslisp:msg-slot-value last-joint-state-msg :header)
      :stamp))))


