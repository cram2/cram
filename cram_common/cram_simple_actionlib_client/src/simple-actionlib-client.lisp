;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :actionlib-client)

(defvar *action-timeouts* (make-hash-table)
  "Hash table of symbol for name of action associated with integer of timeout duration in secs.")

(defvar *action-ros-names* (make-hash-table)
  "Hash table of symbol for name of action associated with its ROS name string.")

(defvar *action-ros-types* (make-hash-table)
  "Hash table of symbol for name of action associated with its ROS type string.")

(defvar *action-clients* (make-hash-table)
  "Hash table of symbol for name of action associated with action client instance.")


(defun init-simple-action-client (name)
  (let ((action-client
          (setf (gethash name *action-clients*)
                (actionlib:make-action-client
                 (gethash name *action-ros-names*)
                 (gethash name *action-ros-types*)))))
    (loop until (and action-client
                     (actionlib:wait-for-server action-client 5.0))
          do (roslisp:ros-info (simple-action-client init)
                               "Waiting for ~a action server..." name)
          unless action-client do (return))
    ;; (dotimes (seconds 5) ; give the client some time to settle down
    ;;     (roslisp:ros-info (robots-common action-client)
    ;;                       "Goal subscribers: ~a~%"
    ;;                       (mapcar (lambda (connection)
    ;;                                 (roslisp::subscriber-uri connection))
    ;;                               (roslisp::subscriber-connections
    ;;                                (actionlib::goal-pub ,action-var-name))))
    ;;     (cpl:sleep 1))
    (if action-client
        (roslisp:ros-info (simple-action-client init)
                          "~a action client created."  name)
        (roslisp:ros-info (simple-action-client init)
                          "Waiting for ~a action client cancelled." name))
    action-client))

(defun get-simple-action-client (name)
  (or (gethash name *action-clients*)
      (init-simple-action-client name)))

(defun get-action-timeout (name)
  (gethash name *action-timeouts*))


(defun make-simple-action-client (name ros-name ros-type timeout &key initialize-now)
  (declare (type symbol name)
           (type string ros-name ros-type)
           (type number timeout))
  "Creates syntactic sugar for managing actionlib clients and makes them
do their magic invisibly in the background.
`name' is a symbol the user uses to refer to the client, e.g. 'ptu,
`ros-name' is the name expected by actionlib:make-action-client, e.g. 'some_namespace/move_base',
`ros-type' is the type expected by make-action-client, e.g. 'move_base_msgs/MoveBaseAction',
`timeout' is the time to wait for the action to execute in seconds,
`initialize-now' defines if the topic communication should be setup at once."

  ;; store the timeout duration, ROS action name and ROS action type for the client
  (setf (gethash name *action-timeouts*) timeout)
  (setf (gethash name *action-ros-names*) ros-name)
  (setf (gethash name *action-ros-types*) ros-type)

  ;; initialize the action client if requested so
  (when initialize-now (init-simple-action-client name))

  name)


(defun call-simple-action-client (name &key action-goal action-timeout)
  (declare (type (or null roslisp-msg-protocol:ros-message) action-goal)
           (type (or null number) action-timeout))
  ;; (roslisp:ros-info (simple-action-client call)
  ;;                   "Calling ~a actionlib client with goal:~%~a" name action-goal)
  (roslisp:ros-info (simple-action-client call) "Calling ~a actionlib client." name)
  (unless action-timeout
    (setf action-timeout (gethash name *action-timeouts*)))
  (cpl:with-failure-handling
      ((simple-error (e)
                     (format t "Actionlib error occured!~%~a~%Reinitializing...~%~%" e)
                     (init-simple-action-client name)
                     (cpl:retry)))
    (let ((actionlib:*action-server-timeout* 10.0)
          (client (get-simple-action-client name)))
      (if client
          (multiple-value-bind (result status)
              (actionlib:call-goal client action-goal :timeout action-timeout)
            ;; (roslisp:ros-info (simple-action-client call)
            ;;                   "Done with calling ~a with status ~a and result:~%~a"
            ;;                   name status result)
            (roslisp:ros-info (simple-action-client call)
                              "Calling ~a returned with status ~a." name status)
            (values result status))
          (progn
            (roslisp:ros-info (simple-action-client call)
                              "Calling ~a aborted because the client was destroyed."
                              name)
            (values nil nil))))))


(defun destroy-simple-action-clients ()
  (setf *action-timeouts* (make-hash-table))
  (setf *action-ros-names* (make-hash-table))
  (setf *action-ros-types* (make-hash-table))
  (loop for client-name being the hash-keys in *action-clients*
        do (setf (gethash client-name *action-clients*) nil))
  (setf *action-clients* (make-hash-table)))

(roslisp-utilities:register-ros-cleanup-function destroy-simple-action-clients)
