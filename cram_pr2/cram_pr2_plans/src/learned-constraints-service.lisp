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

(in-package :pr2-plans)

(defvar *learned-constraints-service* nil
  "Service client for querying leaned constraints service.")

(defparameter *learned-constraints-service-name* "/task_database/query_motion")

(defun make-learned-constraints-request (source-name-keyword source-pose-stamped
                                         target-name-keyword target-pose-stamped
                                         pour-volume liquid-in-source)
  (declare (type keyword target-name-keyword source-name-keyword)
           (type cl-transforms-stamped:pose-stamped target-pose-stamped source-pose-stamped))

  (flet ((make-task-object-msg (name-keyword pose-stamped role
                                &optional (liquid-volume 0.0))
           (declare (type cl-transforms-stamped:pose-stamped pose-stamped)
                    (type keyword role))
           "role can either be :source or :target"
           (roslisp:make-message
            "giskard_msgs/TaskObject"
            :name (string-downcase (symbol-name name-keyword))
            :pose (cl-transforms-stamped:to-msg pose-stamped)
            :role (roslisp:symbol-code 'giskard_msgs-msg:taskobject
                                       (intern (format nil "~a_ROLE" role) :keyword))
            :liquid_volume liquid-volume)))

    (let* ((target (make-task-object-msg target-name-keyword target-pose-stamped :target))
           (source (make-task-object-msg source-name-keyword source-pose-stamped
                                         :source liquid-in-source)))
      (roslisp:make-request
       giskard_msgs-srv:querymotion
       :task (roslisp:make-message
              "giskard_msgs/Task"
              :type "pouring"
              :objects (vector target source)
              :pour_volume pour-volume)))))

(defun ensure-learned-constraints-input-parameters (source-name-keyword source-pose-stamped
                                                    target-name-keyword target-pose-stamped)
  (let ((source-name-keyword
          (intern (string-upcase source-name-keyword) :keyword))
        (target-name-keyword
          (intern (string-upcase target-name-keyword) :keyword))
        (source-pose-stamped
          (cram-tf:ensure-pose-in-frame
           source-pose-stamped
           cram-tf:*robot-base-frame*
           :use-zero-time t))
        (target-pose-stamped
          (cram-tf:ensure-pose-in-frame
           target-pose-stamped
           cram-tf:*robot-base-frame*
           :use-zero-time t)))
    (values source-name-keyword source-pose-stamped
            target-name-keyword target-pose-stamped)))

(defun ensure-learned-constraints-result (result)
  (unless result
    (error "learned constraints server didn't answer"))
  (unless (= (length result) 3)
    (error "expected 3 phases, got ~a" (length result)))
  (map 'list (lambda (phase-constraint)
               (roslisp:with-fields (name constraints)
                   phase-constraint
                 (let ((name (intern (substitute #\- #\_ (string-upcase name)) :keyword))
                       (constraints (map 'list (lambda (constraint)
                                                 (roslisp:with-fields (name lower upper)
                                                     constraint
                                                   (list name
                                                         (read-from-string
                                                          (format nil "~,4f" lower))
                                                         (read-from-string
                                                          (format nil "~,4f" upper)))))
                                         constraints)))
                   (list name constraints))))
       result))

(defun call-learned-constraints-service (source-name-keyword source-pose-stamped
                                         target-name-keyword target-pose-stamped
                                         &optional
                                           (pour-volume 0.0)
                                           (liquid-in-source 0.0))
  (loop until (roslisp:wait-for-service *learned-constraints-service-name* 5)
        do (roslisp:ros-info (learned-constr-service) "Waiting for learned constraints service."))
  (multiple-value-bind (source-name-keyword source-pose-stamped
                        target-name-keyword target-pose-stamped)
      (ensure-learned-constraints-input-parameters source-name-keyword source-pose-stamped
                                                   target-name-keyword target-pose-stamped)
    (roslisp:with-fields (phases)
        (roslisp:call-service
         "/task_database/query_motion"
         'giskard_msgs-srv:querymotion
         (make-learned-constraints-request source-name-keyword source-pose-stamped
                                           target-name-keyword target-pose-stamped
                                           pour-volume liquid-in-source))
      (ensure-learned-constraints-result phases))))
