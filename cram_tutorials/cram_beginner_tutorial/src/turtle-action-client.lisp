(in-package :tut)

(defvar *navp-client* nil)

(defun init-action-client ()
  (setf *navp-client* (actionlib:make-action-client
                       "turtle_shape"
                       "turtle_actionlib/ShapeAction"))
  (roslisp:ros-info (turtle-shape-action-client)
                    "Waiting for turtle shape action server...")
  ;; workaround for race condition in actionlib wait-for server
  (loop until
        (actionlib:wait-for-server *navp-client*))
  (roslisp:ros-info (turtle-shape-action-client)
                    "Turtle shape action client created."))

(defun get-action-client ()
  (when (null *navp-client*)
    (init-action-client))
  *navp-client*)

(defun make-shape-action-goal (in-edges in-radius)
  (actionlib:make-action-goal (get-action-client)
    edges in-edges
    radius in-radius))

(defun call-shape-action (&key edges radius)
  (multiple-value-bind (result status)
      (with-failure-handling
          ((simple-error (e)
             (format t "An error occured!~%~a~%Reinitializing...~%~%" e)
             (setf *navp-client* nil)
             (retry)))
        (let ((actionlib:*action-server-timeout* 10.0))
          (actionlib:call-goal
           (get-action-client)
           (make-shape-action-goal edges radius))))
    (roslisp:ros-info (turtle-shape-action-client) "Nav action finished.")
    (values result status)))
