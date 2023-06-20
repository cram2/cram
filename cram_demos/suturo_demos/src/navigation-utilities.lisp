(in-package :su-demos)

(defvar *nav-client* nil)

;;@author Torge Olliges, Phillip Klein
(defun try-movement-stamped-list (listStamped)
  "Receives a list of stamped poses `listStamped'. Tries out all poses in the bulletworld simulation and returns a possible pose."
  ;;  (car listStamped))
  (let ((?nav-pose listStamped))
    (cpl:with-retry-counters ((going-retry 3))
      (cpl:with-failure-handling
          (((or common-fail:low-level-failure 
                cl::simple-error
                cl::simple-type-error)
               (e)
             (setf ?nav-pose (cdr ?nav-pose))
             (setf listStamped (cdr ?nav-pose))
             (cpl:do-retry going-retry
               (roslisp:ros-warn (going-demo movement-fail)
                                 "~%Failed to move to given position~%")
               (cpl:retry))
             (roslisp:ros-warn  (going-demo movement-fail)
                                "~%No more retries~%")))
        (let ((?actual-nav-pose (car ?nav-pose))) 
          (exe:perform
           (desig:a motion
                    (type going)
                    (pose ?actual-nav-pose)))
          (setf listStamped (cdr ?nav-pose))
          ?actual-nav-pose)))))


(defun init-nav-client ()
  "Initialize the navigation client"
  (unless (eq roslisp::*node-status* :running)
    (roslisp:start-ros-node "nav-action-client"))
  (setf *nav-client* (actionlib:make-action-client
                      "/move_base/move"
                      "move_base_msgs/MoveBaseAction"))                      
  (roslisp:ros-info (nav-action-client)
                    "Waiting for Navigation Action server...")
  (loop until
        (actionlib:wait-for-server *nav-client*))
  (roslisp:ros-info (nav-action-client)
                    "Navigation action client created."))

(roslisp-utilities:register-ros-init-function init-nav-client)

(defun get-nav-action-client ()
  "Returns the navigation action client. If none exists yet, one will be created"
  (when (null *nav-client*)
    (init-nav-client))
  *nav-client*)

(defun make-nav-action-goal (pose-stamped-goal)
  "Receives stamped pose `pose-stamped-goal'. Creates a navigation action goal"
  ;; make sure a node is already up and running, if not, one is initialized here.
  (roslisp:ros-info (navigation-action-client)
                    "Make navigation action goal")
  (unless (eq roslisp::*node-status* :running)
    (roslisp:start-ros-node "navigation-action-lisp-client"))
  (actionlib:make-action-goal (get-nav-action-client)
    target_pose pose-stamped-goal))

(defun call-nav-action (x y euler-z &optional (frame-id "map"))
 "Receives coordinates `x', `y' within frame ID `frame-id' and rotation around z axis `euler-z'. Calls the navigation action."
  (print "Nav action processed")
  (unless (eq roslisp::*node-status* :running)
    (roslisp:start-ros-node "nav-action-lisp-client"))
  (multiple-value-bind (result status)
      (let* ((actionlib:*action-server-timeout* 10.0)
             (pose-stamped (cl-tf:make-pose-stamped
                            frame-id (roslisp::ros-time)
                            (cl-tf:make-3d-vector x y 0.0)
                            (cl-tf:euler->quaternion :ax 0.0 :ay 0.0 :az euler-z)))
             (the-goal (cl-tf:to-msg pose-stamped)))
        (publish-marker-pose pose-stamped :g 1.0)
        (actionlib:call-goal 
         (get-nav-action-client)
         (make-nav-action-goal the-goal)))
    (roslisp:ros-info (nav-action-client)
                      "Navigation action finished.")
    (values result status)))

(defun call-nav-action-ps (pose-stamped)
  "Receives stamped pose `pose-stamped'. Calls the navigation client and passes the given pose-stamped to it."  
  (setf pose-stamped (cl-tf:copy-pose-stamped pose-stamped :origin
                                              (cl-tf:copy-3d-vector
                                               (cl-tf:origin pose-stamped)
                                               :z 0.0)))
  (unless (eq roslisp::*node-status* :running)
    (roslisp:start-ros-node "nav-action-lisp-client"))
  (multiple-value-bind (result status)
      (let ((actionlib:*action-server-timeout* 20.0)
            (the-goal (cl-tf:to-msg
                       pose-stamped)))
        ;;publish the pose the robot will navigate to
        (publish-marker-pose pose-stamped :g 1.0)
        (actionlib:call-goal
         (get-nav-action-client)
         (make-nav-action-goal the-goal)))
    (roslisp:ros-info (nav-action-client)
                      "Navigation action finished.")
    ;; (case status
    ;;   (:succeeded (call-text-to-speech-action "Goal reached successfully!"))
    ;;   (otherwise (call-text-to-speech-action "Something went wrong!")))
    (format t "result : ~a" status)
    (values result status)))

;;====================================================================================================

(defparameter *perceived-data* nil)
(defparameter *perception-subscriber* nil)
(defparameter *marker-publisher* nil)


(defun init-marker-publisher()
  "Initializes marker-publisher."
  (setf *marker-publisher*
        (roslisp:advertise "~location_marker" "visualization_msgs/Marker")))

(defun get-marker-publisher ()
  "Returns the current marker-publisher. If none exists, one is created."
  (unless *marker-publisher*
    (init-marker-publisher))
  *marker-publisher*)

(defun publish-marker-pose (pose &key (parent "map") id (g 0.0))
  "Receives pose `pose'. Places a visualization marker at `pose'."
  (let ((point (cl-transforms:origin pose))
        (rot (cl-transforms:orientation pose))
        (current-index 0))
    (roslisp:publish (get-marker-publisher)
                     (roslisp:make-message "visualization_msgs/Marker"
                                           (std_msgs-msg:stamp header) 
                                           (roslisp:ros-time)
                                           (std_msgs-msg:frame_id header)
                                           (typecase pose
                                             (cl-tf:pose-stamped (cl-tf:frame-id pose))
                                             (t parent))
                                           ns "goal_locations"
                                           id (or id (incf current-index))
                                           type (roslisp:symbol-code
                                                 'visualization_msgs-msg:<marker> :arrow)
                                           action (roslisp:symbol-code
                                                   'visualization_msgs-msg:<marker> :add)
                                           (x position pose) (cl-transforms:x point)
                                           (y position pose) (cl-transforms:y point)
                                           (z position pose) (cl-transforms:z point)
                                           (x orientation pose) (cl-transforms:x rot)
                                           (y orientation pose) (cl-transforms:y rot)
                                           (z orientation pose) (cl-transforms:z rot)
                                           (w orientation pose) (cl-transforms:w rot)
                                           (x scale) 0.09
                                           (y scale) 0.09
                                           (z scale) 0.09
                                           (r color) 1.0
                                           (g color) g
                                           (b color) 0.0
                                           (a color) 1.0))))
