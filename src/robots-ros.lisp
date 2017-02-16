;;;
;;; Copyright (c) 2016, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :commander)

(defparameter *reference-service-name* "/reference_designator")
(defparameter *perform-service-name* "/perform_designator")

(defparameter *reference-service-type* 'sherpa_msgs-srv:ReferenceDesignator)
(defparameter *perform-action-type* "sherpa_msgs/PerformDesignatorAction")

(defparameter *perform-action-timeout* 5
  "How many seconds to wait before returning from perform_designator action.")

(defvar *perform-action-clients* (make-hash-table :test #'equal)
  "Hash table where key is agent-namespace and value is its perform client")

(defvar *active-goals* (make-hash-table :test #'equal)
  "A hash table of hash tables of active goal handles,
key is the agent namespace, value is a hash table of goal-handle ID and its handle.")

(defun init-perform-action-client (agent-namespace)
  (let ((ros-name (concatenate 'string agent-namespace *perform-service-name*)))
    (setf (gethash agent-namespace *perform-action-clients*)
          (actionlib:make-action-client ros-name *perform-action-type*))
   (loop until (actionlib:wait-for-server (gethash agent-namespace *perform-action-clients*) 5.0)
         do (roslisp:ros-info (commander perform-client)
                              "Waiting for ~a action server..." ros-name))
   ;; (dotimes (seconds 5) ; give the client some time to settle down
   ;;     (roslisp:ros-info (robots-common action-client)
   ;;                       "Goal subscribers: ~a~%"
   ;;                       (mapcar (lambda (connection)
   ;;                                 (roslisp::subscriber-uri connection))
   ;;                               (roslisp::subscriber-connections
   ;;                                (actionlib::goal-pub ,action-var-name))))
   ;;     (cpl:sleep 1))
   (roslisp:ros-info (commander perform-client) "~a action client created." ros-name)
   (gethash agent-namespace *perform-action-clients*)))

(defun destroy-perform-action-clients ()
  (maphash (lambda (agent-namespace client)
             (setf client nil)
             (remhash agent-namespace *perform-action-clients*))
           *perform-action-clients*))

(roslisp-utilities:register-ros-cleanup-function destroy-perform-action-clients)
;; (roslisp-utilities:register-ros-init-function ,init-function-name)
;; The init function is necessary because there is a bug when
;; nodes don't subscribe to a publisher started from a terminal executable

(defun get-perform-action-client (agent-namespace)
  (or (gethash agent-namespace *perform-action-clients*)
      (init-perform-action-client agent-namespace)))

(defun action-result-callback (agent-namespace goal-unique-id action-designator start-time
                               status-msg result-msg)
  (format t "Goal ~a of ~a finished~%" goal-unique-id agent-namespace)
  (remhash goal-unique-id (gethash agent-namespace *active-goals*))
  (format t "STATUS: ~a~%RESULT: ~a~%" status-msg result-msg)
  ;; (format t "Now logging ~a~%" action-designator)
  ;; (robots-common::log-owl action-designator :start-time start-time
  ;;                                           :agent (robots-common:derosify agent-namespace))
  )

(defun action-feedback-callback (feedback-msg)
  (format t "FEEDBACK: ~a~%" feedback-msg))

;; (defun action-active-callback ()
;;   (format t "ACTIVE"))

(defun call-perform-action (agent-namespace goal-unique-id
                            &key action-goal action-timeout action-designator)
  (declare (type (or null sherpa_msgs-msg:PerformDesignatorGoal) action-goal)
           (type (or null number) action-timeout)
           (type string goal-unique-id))
  (roslisp:ros-info (commander perform-client)
                    "Calling CALL-PERFORM-ACTION on ~a with goal:~%~a"
                    agent-namespace action-goal)
  (unless action-timeout
    (setf action-timeout *perform-action-timeout*))
  (cpl:with-failure-handling
      ((simple-error (e)
         (format t "Actionlib error occured!~%~a~%Reinitializing...~%~%" e)
         (init-perform-action-client agent-namespace)
         (cpl:retry)))
    (let ((actionlib:*action-server-timeout* 10.0))
      (actionlib:send-goal
       (get-perform-action-client agent-namespace)
       action-goal
       (alexandria:curry #'action-result-callback
                         agent-namespace goal-unique-id action-designator (roslisp:ros-time))
       #'action-feedback-callback))))

(defun make-perform-action-goal (action-or-motion-designator)
  (declare (type desig:designator action-or-motion-designator))
  (robots-common:make-symbol-type-message
   'sherpa_msgs-msg:PerformDesignatorGoal
   :designator (action-designator->json action-or-motion-designator)))

(defun get-designator-type-keyword (designator)
  (declare (type (or null desig:designator) designator))
  (car (rassoc (type-of designator)
               (get 'desig:make-designator :desig-types))))

(defmethod yason:encode ((object symbol) &optional (stream *standard-output*))
  (write-char #\" stream)
  (princ object stream)
  (write-char #\" stream))

(defmethod yason:encode ((object desig:designator) &optional (stream *standard-output*))
  (format stream "[\"A\",\"~a\","
          (get-designator-type-keyword object))
  (yason:encode-alist (desig:properties object) stream)
  (write-char #\] stream))

(defmethod yason:encode ((pose cl-transforms:pose) &optional (stream *standard-output*))
  (let ((origin (cl-transforms:origin pose))
        (orientation (cl-transforms:orientation pose)))
    (with-slots ((x cl-transforms:x) (y cl-transforms:y) (z cl-transforms:z))
        origin
      (with-slots ((q1 cl-transforms:x) (q2 cl-transforms:y) (q3 cl-transforms:z)
                   (w cl-transforms:w))
          orientation
        (format stream "[[~,6f,~,6f,~,6f],[~,6f,~,6f,~,6f,~,6f]]"
                x y z q1 q2 q3 w)))))

(defun action-designator->json (action-designator)
  (let ((stream (make-string-output-stream)))
    (yason:encode action-designator stream)
    (get-output-stream-string stream)))

(defun call-reference (action-designator agent-namespace)
  "Calls the action designator reference ROS service"
  (roslisp:call-service
   (concatenate 'string agent-namespace *reference-service-name*)
   *reference-service-type*
   :designator (action-designator->json action-designator)))

(defun add-to-active-goals (agent-namespace goal-unique-id goal-handle)
  (declare (type string agent-namespace goal-unique-id)
           (type actionlib::client-goal-handle goal-handle))
  (unless (gethash agent-namespace *active-goals*)
    (setf (gethash agent-namespace *active-goals*)
          (make-hash-table :test #'equal)))
  (setf (gethash goal-unique-id (gethash agent-namespace *active-goals*))
        goal-handle))

(defun cancel-actions (agent-namespace)
  (declare (type string agent-namespace))
  (roslisp:ros-info (commander perform-client)
                    "STOPPING all goals of ~a" agent-namespace)
  (let ((current-goals-hash-tbl (gethash agent-namespace *active-goals*)))
    (when current-goals-hash-tbl
      (roslisp:ros-info (commander perform-client)
                        "Currently active ~a goals" (hash-table-count current-goals-hash-tbl))
      (maphash #'(lambda (goal-unique-id goal-handle)
                   (actionlib:cancel-goal goal-handle)
                   (remhash goal-unique-id current-goals-hash-tbl))
               current-goals-hash-tbl))))

(defun generate-unique-id (agent-namespace)
  "because actionlib goal-id is not accessible in callbacks"
  (format nil "~a_~10,5$" agent-namespace (roslisp:ros-time)))

(defun call-perform (action-designator agent-namespace)
  (declare (type desig:designator action-designator)
           (type (or null string) agent-namespace))
  "Calls the action performing ROS service.
It has to come back immediately for the HMI interface not to be blocked
If the action is of type STOPPING it will stop all the goals of the agent."
  (unless agent-namespace
    (setf agent-namespace (choose-agent action-designator)))
  (if (or (eq :stop (desig:desig-prop-value action-designator :to))
          (eq :stopping (desig:desig-prop-value action-designator :type)))
      (cancel-actions agent-namespace)
      (let ((goal-unique-id (generate-unique-id agent-namespace)))
        (add-to-active-goals
         agent-namespace goal-unique-id
         (call-perform-action
          agent-namespace
          goal-unique-id
          :action-goal (make-perform-action-goal action-designator)
          :action-designator action-designator)))))


