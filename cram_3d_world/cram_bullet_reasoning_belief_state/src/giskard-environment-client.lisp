;;;
;;; Copyright (c) 2018, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :cram-bullet-reasoning-belief-state)

(defvar *giskard-environment-service* nil
  "Persistent service client for querying ...")

(defparameter *giskard-environment-service-name* "/giskard/update_world")

(defun init-giskard-environment-service ()
  "Initializes *robosherlock-service* ROS publisher"
  (loop until (roslisp:wait-for-service *giskard-environment-service-name* 5)
        do (roslisp:ros-info (gisk-env-service) "Waiting for giskard environment service."))
  (prog1
      (setf *giskard-environment-service*
            (make-instance 'roslisp:persistent-service
              :service-name *giskard-environment-service-name*
              :service-type 'giskard_msgs-srv:updateworld))
    (roslisp:ros-info (gisk-env-service) "Giskard environment service client created.")))

(defun get-giskard-environment-service ()
  (if (and *giskard-environment-service*
           (roslisp:persistent-service-ok *giskard-environment-service*))
      *giskard-environment-service*
      (init-giskard-environment-service)))

(defun destroy-giskard-environment-service ()
  (when *giskard-environment-service*
    (roslisp:close-persistent-service *giskard-environment-service*))
  (setf *giskard-environment-service* nil))

(roslisp-utilities:register-ros-cleanup-function destroy-giskard-environment-service)


(defun make-giskard-environment-request (add-or-kill-or-attached name
                                         &optional
                                           (pose (cl-transforms-stamped:make-pose-stamped
                                                  "map"
                                                  0.0
                                                  (cl-transforms:make-identity-vector)
                                                  (cl-transforms:make-identity-rotation)))
                                           (dimensions '(1.0 1.0 1.0)))
  (declare (type keyword add-or-kill-or-attached)
           (type string name))
  (ecase add-or-kill-or-attached
    (:add
     (roslisp:make-request
      'giskard_msgs-srv:updateworld
      :operation (roslisp:symbol-code
                  'giskard_msgs-srv:updateworld-request
                  :add)
      :rigidly_attached nil
      :body (roslisp:make-msg
             'giskard_msgs-msg:worldbody
             :type (roslisp:symbol-code
                    'giskard_msgs-msg:worldbody
                    :primitive_body)
             :name name
             :shape (roslisp:make-msg
                     'shape_msgs-msg:solidprimitive
                     :type (roslisp:symbol-code 'shape_msgs-msg:solidprimitive :box)
                     :dimensions (map 'vector #'identity dimensions)))
      :pose (cl-transforms-stamped:to-msg pose)))
    (:add-kitchen
     (roslisp:make-request
      'giskard_msgs-srv:updateworld
      :operation (roslisp:symbol-code
                  'giskard_msgs-srv:updateworld-request
                  :add)
      :rigidly_attached nil
      :body (roslisp:make-msg
             'giskard_msgs-msg:worldbody
             :type (roslisp:symbol-code
                    'giskard_msgs-msg:worldbody
                    :urdf_body)
             :name name
             :urdf (roslisp:get-param cram-bullet-reasoning-belief-state::*kitchen-parameter* nil)
             :joint_state_topic "kitchen/joint_states")
      :pose (cl-transforms-stamped:to-msg pose)))
    (:kill
     (roslisp:make-request
      'giskard_msgs-srv:updateworld
      :operation (roslisp:symbol-code
                  'giskard_msgs-srv:updateworld-request
                  :remove)
      :body (roslisp:make-msg 'giskard_msgs-msg:worldbody :name name)))
    (:kill-all
     (roslisp:make-request
      'giskard_msgs-srv:updateworld
      :operation (roslisp:symbol-code
                  'giskard_msgs-srv:updateworld-request
                  :remove_all)))
    (:attached
     (roslisp:make-request
      'giskard_msgs-srv:updateworld
      :operation (roslisp:symbol-code
                  'giskard_msgs-srv:updateworld-request
                  :add)
      :rigidly_attached t
      :body (roslisp:make-msg
             'giskard_msgs-msg:worldbody
             :type (roslisp:symbol-code
                    'giskard_msgs-msg:worldbody
                    :primitive_body)
             :name name
             :shape (roslisp:make-msg
                     'shape_msgs-msg:solidprimitive
                     :type (roslisp:symbol-code 'shape_msgs-msg:solidprimitive :box)
                     :dimensions (map 'vector #'identity dimensions)))
      :pose (cl-transforms-stamped:to-msg pose)))))

(defun ensure-giskard-environment-service-input-parameters ()
  )

(defun ensure-giskard-environment-service-result ()
  ;; (unless result
  ;;   (cpl:fail 'common-fail:perception-low-level-failure :description "robosherlock didn't answer"))
  ;; (let ((number-of-objects (length result)))
  ;;   (when (< number-of-objects 1)
  ;;     (cpl:fail 'common-fail:perception-object-not-found :description "couldn't find the object"))
  ;;   (etypecase quantifier
  ;;     (keyword (ecase quantifier
  ;;                ((:a :an) (parse-json-result (aref result 0))
  ;;                 ;; this case should return a lazy list but I don't like them so...
  ;;                 )
  ;;                (:the (if (= number-of-objects 1)
  ;;                          (parse-json-result (aref result 0))
  ;;                          (cpl:fail 'common-fail:perception-low-level-failure
  ;;                                    :description "There was more than one of THE object")))
  ;;                (:all (map 'list #'parse-json-result result))))
  ;;     (number (if (= number-of-objects quantifier)
  ;;                 (map 'list #'parse-json-result result)
  ;;                 (cpl:fail 'common-fail:perception-low-level-failure
  ;;                           :description (format nil "perception returned ~a objects ~
  ;;                                                     although there should've been ~a"
  ;;                                                number-of-objects quantifier))))))
  )


(defun call-giskard-environment-service (add-or-kill-or-attached name
                                         &optional pose dimensions)

  ;; (multiple-value-bind (key-value-pairs-list quantifier)
  ;;     (ensure-robosherlock-input-parameters keyword-key-value-pairs-list quantifier)
  (roslisp:with-fields (giskard_msgs-srv:error_msg)
        (cpl:with-failure-handling
            (((or simple-error roslisp:service-call-error) (e)
               (format t "Service call error occured!~%~a~%Reinitializing...~%~%" e)
               (destroy-giskard-environment-service)
               (init-giskard-environment-service)
               (let ((restart (find-restart 'roslisp:reconnect)))
                 (if restart
                     (progn (roslisp:wait-duration 5.0)
                            (invoke-restart 'roslisp:reconnect))
                     (progn (cpl:retry))))))
          (roslisp:call-persistent-service
           (get-giskard-environment-service)
           (make-giskard-environment-request add-or-kill-or-attached name pose dimensions)))
      giskard_msgs-srv:error_msg))
