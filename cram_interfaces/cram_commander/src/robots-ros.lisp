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

(defparameter *reference-service-type* 'cram_commander-srv:ReferenceDesignator)
(defparameter *perform-service-type* 'cram_commander-srv:PerformDesignator)

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

(defun call-perform (action-designator agent-namespace)
  (declare (type desig:designator action-designator)
           (type (or null string) agent-namespace))
  "Calls the action performing ROS service.
It has to come back immediately for the HMI interface not to be blocked
If the action is of type STOPPING it will stop all the goals of the agent."


  (labels ((commander-perform (action-designator ?agent-namespace)
             (declare (ignore action-designator ?agent-namespace))
             ;; (if (or (eq :charge (desig:desig-prop-value action-designator :to))
             ;;         (eq :charging (desig:desig-prop-value action-designator :type)))
             ;;     (call-perform (desig:an action (to mount) (agent ?agent-namespace))
             ;;                     "donkey")
             ;;     nil)
             ))


    (unless agent-namespace
      (setf agent-namespace (choose-agent action-designator)))

    (unless (commander-perform action-designator agent-namespace)

      (roslisp:call-service
       (concatenate 'string agent-namespace *perform-service-name*)
       *perform-service-type*
       :designator (action-designator->json action-designator)))))
