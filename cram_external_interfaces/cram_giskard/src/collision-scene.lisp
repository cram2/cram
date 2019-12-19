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

(in-package :giskard)

(defvar *giskard-environment-service* nil
  "Persistent service client for querying ...")

(defparameter *giskard-environment-service-name* "/giskard/update_world")

(defun init-giskard-environment-service ()
  "Initializes *robosherlock-service* ROS publisher"
  (let (ready)
    (loop repeat 12                     ; for one minute then give up
          until (setf ready (roslisp:wait-for-service *giskard-environment-service-name* 5))
          do (roslisp:ros-info (gisk-env-service) "Waiting for giskard environment service."))
   (if ready
       (prog1
           (setf *giskard-environment-service*
                 (make-instance 'roslisp:persistent-service
                   :service-name *giskard-environment-service-name*
                   :service-type 'giskard_msgs-srv:updateworld))
         (roslisp:ros-info (gisk-env-service) "Giskard environment service client created."))
       (progn
         (roslisp:ros-error (gisk-env-service) "Giskard environment service doesn't reply. Ignoring.")
         nil))))

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


(defun make-giskard-environment-request (add-or-remove-or-attach-or-detach
                                         &key
                                           name
                                           pose
                                           dimensions
                                           joint-state-topic)
  (declare (type keyword add-or-remove-or-attach-or-detach)
           (type (or null string) name)
           (type (or null cl-transforms-stamped:pose-stamped) pose))
  (unless pose
    (setf pose (cl-transforms-stamped:make-pose-stamped
                "map"
                0.0
                (cl-transforms:make-identity-vector)
                (cl-transforms:make-identity-rotation))))
  (unless dimensions
    (setf dimensions '(1.0 1.0 1.0)))
  (ecase add-or-remove-or-attach-or-detach
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
    (:add-environment
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
             :name (roslisp-utilities:rosify-underscores-lisp-name
                    (btr:name (btr:get-environment-object)))
             :urdf (roslisp:get-param btr-belief:*kitchen-parameter* nil)
             :joint_state_topic joint-state-topic)
      :pose (cl-transforms-stamped:to-msg pose)))
    (:remove
     (roslisp:make-request
      'giskard_msgs-srv:updateworld
      :operation (roslisp:symbol-code
                  'giskard_msgs-srv:updateworld-request
                  :remove)
      :body (roslisp:make-msg 'giskard_msgs-msg:worldbody :name name)))
    (:remove-all
     (roslisp:make-request
      'giskard_msgs-srv:updateworld
      :operation (roslisp:symbol-code
                  'giskard_msgs-srv:updateworld-request
                  :remove_all)))
    (:attach
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
      :pose (cl-transforms-stamped:to-msg pose)))
    (:detach
     (roslisp:make-request
      'giskard_msgs-srv:updateworld
      :operation (roslisp:symbol-code
                  'giskard_msgs-srv:updateworld-request
                  :detach)
      :body (roslisp:make-msg 'giskard_msgs-msg:worldbody :name name)))))

(defun ensure-giskard-environment-service-input-parameters ()
  )

(defun ensure-giskard-environment-service-result ()
  )


(defun call-giskard-environment-service (add-or-remove-or-attach-or-detach
                                         &key name pose dimensions joint-state-topic)

  (ensure-giskard-environment-service-input-parameters)
  (cpl:with-failure-handling
      (((or simple-error roslisp:service-call-error) (e)
         (format t "Service call error occured!~%~a~%Reinitializing...~%~%" e)
         (destroy-giskard-environment-service)
         (init-giskard-environment-service)
         (let ((restart (find-restart 'roslisp:reconnect)))
           (if restart
               (progn (roslisp:wait-duration 5.0)
                      (invoke-restart 'roslisp:reconnect))
               (cpl:retry)))))
    (let ((service (get-giskard-environment-service)))
      (if service
          (roslisp:msg-slot-value
           (roslisp:call-persistent-service
           service
           (make-giskard-environment-request
            add-or-remove-or-attach-or-detach
            :name name
            :pose pose
            :dimensions dimensions
            :joint-state-topic joint-state-topic))
           'giskard_msgs-srv:error_msg)
          nil))))


;;;;;;;;;;;;;;;;;;;;;;;; EVENT HANDLERS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod coe:on-event giskard-attach-object ((event cpoe:object-attached-robot))
  (unless cram-projection:*projection-environment*
    (let* ((object-name
             (cpoe:event-object-name event))
           (object-name-string
             (roslisp-utilities:rosify-underscores-lisp-name object-name))
           (btr-object
             (btr:object btr:*current-bullet-world* object-name))
           (link (if (cpoe:event-arm event)
                     (cut:var-value
                      '?ee-link
                      (car (prolog:prolog
                            `(and (cram-robot-interfaces:robot ?robot)
                                  (cram-robot-interfaces:end-effector-link
                                   ?robot ,(cpoe:event-arm event)
                                   ?ee-link)))))
                     (cpoe:event-link event))))
      (when (cut:is-var link)
        (error "[GISKARD OBJECT-ATTACHED] Couldn't find robot's EE link."))
      (unless btr-object
        (error "[GISKARD OBJECT-ATTACHED] there was no corresponding btr object."))

      (let* ((map-to-ee-transform (cl-transforms-stamped:lookup-transform
                                   cram-tf:*transformer*
                                   cram-tf:*fixed-frame*
                                   link
                                   :timeout 2
                                   :time 0))
             (ee-to-map-transform (cram-tf:transform-stamped-inv map-to-ee-transform))
             (map-to-obj-transform (cram-tf:pose->transform-stamped
                                    cram-tf:*fixed-frame*
                                    object-name-string
                                    0.0
                                    (btr:pose btr-object)))
             (ee-to-object-transform (cram-tf:multiply-transform-stampeds
                                      link object-name-string
                                      ee-to-map-transform map-to-obj-transform))
             (ee-to-object-pose (cram-tf:strip-transform-stamped ee-to-object-transform)))
        ;; remove the object first, maybe it was already attached to something
        (call-giskard-environment-service
         :remove
         :name object-name-string)
        (call-giskard-environment-service
         :attach
         :name object-name-string
         :pose ee-to-object-pose
         :dimensions (with-slots (cl-transforms:x cl-transforms:y cl-transforms:z)
                         (btr:calculate-bb-dims btr-object)
                       (list cl-transforms:x cl-transforms:y cl-transforms:z)))))))

(defmethod coe:on-event giskard-detach-object ((event cpoe:object-detached-robot))
  (unless cram-projection:*projection-environment*
    (let* ((object-name (cpoe:event-object-name event))
           (object-name-string (roslisp-utilities:rosify-underscores-lisp-name object-name)))
      (call-giskard-environment-service
       :detach
       :name object-name-string))))

(defmethod coe:on-event giskard-perceived ((event cpoe:object-perceived-event))
  (unless cram-projection:*projection-environment*
    (let* ((object-name (desig:desig-prop-value (cpoe:event-object-designator event) :name))
           (object-name-string (roslisp-utilities:rosify-underscores-lisp-name object-name))
           (btr-object (btr:object btr:*current-bullet-world* object-name)))
      (call-giskard-environment-service
       :remove
       :name object-name-string)
      (call-giskard-environment-service
       :add
       :name object-name-string
       :pose (cl-transforms-stamped:pose->pose-stamped
              cram-tf:*fixed-frame* 0.0 (btr:pose btr-object))
       :dimensions (with-slots (cl-transforms:x cl-transforms:y cl-transforms:z)
                       (btr:calculate-bb-dims btr-object)
                     (list cl-transforms:x cl-transforms:y cl-transforms:z))))))

(defmethod coe:clear-belief giskard-clear ()
  (unless cram-projection:*projection-environment*
    (call-giskard-environment-service
     :remove-all)
    (when (btr:get-environment-object)
      (call-giskard-environment-service
       :add-environment
       :name (roslisp-utilities:rosify-underscores-lisp-name (btr:name (btr:get-environment-object)))
       :pose (cl-transforms-stamped:pose->pose-stamped
              cram-tf:*fixed-frame* 0.0 (btr:pose (btr:get-environment-object)))
       :joint-state-topic "kitchen/joint_states"))))
