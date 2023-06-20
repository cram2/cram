(in-package :su-demos)

(defvar *text-to-speech-publisher* nil)
(defparameter *enable-speech* T)

(defvar *text-to-speech-action-client* nil)

(defun init-text-to-speech-action-client ()
  "Initializes the text to speech action client."
  (setf *text-to-speech-action-client* (actionlib:make-action-client
     "talk_request_action"
     "tmc_msgs/TalkRequestAction"))
  (loop until
    (actionlib:wait-for-server *text-to-speech-action-client*))
  (roslisp:ros-info (text-to-speech-action-client) 
                    "Text to speech action client created."))

(roslisp-utilities:register-ros-init-function init-text-to-speech-action-client)

(defun get-text-to-speech-action-client ()
  "Returns the current text to speech client. If none exists, one is created."
  (when (null *text-to-speech-action-client*)
    (init-text-to-speech-action-client))
  *text-to-speech-action-client*)

(defun make-text-action-goal (text)
  "Create a text-to-speech action goal with the given `text'"
  (actionlib:make-action-goal (get-text-to-speech-action-client)
    :data (roslisp:make-message "tmc_msgs/Voice"
      :interrupting nil
      :queueing nil
      :language 1
      :sentence text)))

(defun call-text-to-speech-action (text)
  "Calls the text to speech action to perform the given `text'"
  (when *enable-speech*
    (multiple-value-bind (result status)
        (let ((actionlib:*action-server-timeout* 10.0))
          (actionlib:call-goal
           (get-text-to-speech-action-client)
           (make-text-action-goal text)))
      (roslisp:ros-info (text-to-speech-action-client) "Text to speech action finished.")
      (values result status)
      result)))
