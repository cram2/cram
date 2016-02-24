
(in-package :pr2)

(defvar *torso-action-client* nil)

(defun init-torso-action-client ()
  (setf *torso-action-client* (actionlib:make-action-client
                               "torso_controller/position_joint_action"
                               "pr2_controllers_msgs/SingleJointPositionAction"))
  (loop until (actionlib:wait-for-server *torso-action-client* 5.0)
        do (roslisp:ros-info (torso-action-client)
                             "Waiting for torso controller actionlib server..."))
  (roslisp:ros-info (torso-action-client) "Torso shape action client created."))

(defun destroy-torso-action-client ()
  (setf *torso-action-client* nil))

(roslisp-utilities:register-ros-cleanup-function destroy-torso-action-client)

(defun get-torso-action-client ()
  (or *torso-action-client*
      (init-torso-action-client)))

(defun make-torso-action-goal (position)
  (actionlib:make-action-goal
      (get-torso-action-client)
    :position (etypecase position
                (number (cond
                          ((< position 0.02)
                           (roslisp:ros-warn
                            (torso-action)
                            "POSITION (~a) cannot be smaller than 0.02. Clipping."
                            position)
                           0.02)
                          ((> position 0.32)
                           (roslisp:ros-warn
                            (torso-action)
                            "POSITION (~a) shouldn't be bigger than 0.32. Clipping."
                            position)
                           0.32)
                          (t
                           position)))
                (keyword (ecase position
                           (:up 0.32)
                           (:down 0.02))))))

(defun call-torso-action (position)
  (multiple-value-bind (result status)
      (cpl:with-failure-handling
          ((simple-error (e)
             (format t "Actionlib error occured!~%~a~%Reinitializing...~%~%" e)
             (init-torso-action-client)
             (cpl:retry)))
        (let ((actionlib:*action-server-timeout* 10.0))
          (actionlib:call-goal
           (get-torso-action-client)
           (make-torso-action-goal position)
           :timeout 20.0)))
    (roslisp:ros-info (torso-action-client) "Torso action finished.")
    (values result status)))






;; (defconstant +max-torso-joint-position+ 10000.0)
;; (defconstant +min-torso-joint-position+ 0.0)

;; (defun make-torso-action-goal-cpl (position)
;;   (actionlib:make-action-goal *torso-action-client*
;;     position (etypecase position
;;                (number position)
;;                (keyword (ecase position
;;                           (:up +max-torso-joint-position+)
;;                           (:down +min-torso-joint-position+))))))

;; ;;; TODO: check if it's still stuck at the same spot after 5 seconds
;; (defparameter *position-delta* 0.001)
;; (defun abort-when-small-improvement (feedback-signal)
;;   (setf *last-torso-position* feedback-signal)
;;   (with-slots (actionlib:feedback) feedback-signal
;;     (roslisp:with-fields (position (stamp (stamp header)))
;;         actionlib:feedback
;;       (let ((position-some-time-ago stamp)) ;; it's f(stamp) not stamp
;;         (when (< (abs (- position position-some-time-ago)) *position-delta*)
;;                 (roslisp:ros-warn (torso-action)
;;                                   "Torso action not converging. Aborting.")
;;                 (invoke-restart 'actionlib:abort-goal))))))

;; (defun call-torso-action-cpl (position &key (timeout 60.0))
;;   "Per default this action should take no longer than one minute."
;;   (multiple-value-bind (result status)
;;       (cpl:with-failure-handling
;;           ((simple-error (e)
;;              (format t "Actionlib error occured!~%~a~%Reinitializing...~%~%" e)
;;              (init-torso-action-client)
;;              (cpl:retry)))
;;         (let ((actionlib:*action-server-timeout* 10.0))
;;           (handler-bind
;;               ((actionlib:feedback-signal #'abort-when-small-improvement))
;;             (actionlib:call-goal
;;              *torso-action-client*
;;              (make-torso-action-goal position)
;;              :timeout timeout))))
;;     (roslisp:ros-info (torso-action-client) "Torso action finished.")
;;     (values result status)))
