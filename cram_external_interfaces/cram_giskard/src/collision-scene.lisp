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

(defparameter *giskard-environment-service-timeout* 10
  "in seconds. Service will wait that long until responding being busy.")

(defvar *giskard-environment-service* nil
  "Persistent service client for querying.")

(defparameter *giskard-environment-service-name* "/giskard/update_world")

(defvar *giskard-groups-service* nil
  "Persistent service client for groups.")

(defparameter *giskard-groups-service-name* "/giskard/get_group_names")

(defvar *giskard-register-groups-service* nil
  "Persistent service client for registering new groups.")

(defparameter *giskard-register-groups-service-name* "/giskard/register_groups")


(defun init-giskard-environment-service ()
  "Initializes *giskard-environment-service*"
  (let (ready)
    (loop repeat 12                     ; for one minute then give up
          until (setf ready
                      (roslisp:wait-for-service *giskard-environment-service-name* 5))
          do (roslisp:ros-info (gisk-env-service)
                               "Waiting for giskard environment service."))
   (if ready
       (prog1
           (setf *giskard-environment-service*
                 (make-instance 'roslisp:persistent-service
                   :service-name *giskard-environment-service-name*
                   :service-type 'giskard_msgs-srv:updateworld))
         (roslisp:ros-info (gisk-env-service)
                           "Giskard environment service client created."))
       (progn
         (roslisp:ros-error (gisk-env-service)
                            "Giskard environment service doesn't reply. Ignoring.")
         nil))))

(defun init-giskard-groups-service ()
  "Initializes *giskard-groups-service*"
  (let (ready)
    (loop repeat 12                     ; for one minute then give up
          until (setf ready
                      (roslisp:wait-for-service *giskard-groups-service-name* 5))
          do (roslisp:ros-info (gisk-env-service)
                               "Waiting for giskard groups service."))
   (if ready
       (prog1
           (setf *giskard-groups-service*
                 (make-instance 'roslisp:persistent-service
                   :service-name *giskard-groups-service-name*
                   :service-type 'giskard_msgs-srv:getgroupnames))
         (roslisp:ros-info (gisk-env-service)
                           "Giskard groups service client created."))
       (progn
         (roslisp:ros-error (gisk-env-service)
                            "Giskard groups service doesn't reply. Ignoring.")
         nil))))

(defun init-giskard-register-groups-service ()
  "Initializes *giskard-groups-service*"
  (let (ready)
    (loop repeat 12                     ; for one minute then give up
          until (setf ready
                      (roslisp:wait-for-service
                       *giskard-register-groups-service-name* 5))
          do (roslisp:ros-info (gisk-env-service)
                               "Waiting for giskard register-groups service."))
   (if ready
       (prog1
           (setf *giskard-register-groups-service*
                 (make-instance 'roslisp:persistent-service
                   :service-name *giskard-register-groups-service-name*
                   :service-type 'giskard_msgs-srv:registergroup))
         (roslisp:ros-info (gisk-env-service)
                           "Giskard register-groups service client created."))
       (progn
         (roslisp:ros-error (gisk-env-service)
                            "Giskard register-groups service doesn't reply. Ignoring.")
         nil))))

(defun get-giskard-environment-service ()
  (if (and *giskard-environment-service*
           (roslisp:persistent-service-ok *giskard-environment-service*))
      *giskard-environment-service*
      (init-giskard-environment-service)))

(defun get-giskard-groups-service ()
  (if (and *giskard-groups-service*
           (roslisp:persistent-service-ok *giskard-groups-service*))
      *giskard-groups-service*
      (init-giskard-groups-service)))

(defun get-giskard-register-groups-service ()
  (if (and *giskard-register-groups-service*
           (roslisp:persistent-service-ok *giskard-register-groups-service*))
      *giskard-register-groups-service*
      (init-giskard-register-groups-service)))

(defun destroy-giskard-environment-service ()
  (when *giskard-environment-service*
    (roslisp:close-persistent-service *giskard-environment-service*))
  (setf *giskard-environment-service* nil))

(defun destroy-giskard-groups-service ()
  (when *giskard-groups-service*
    (roslisp:close-persistent-service *giskard-groups-service*))
  (setf *giskard-groups-service* nil))

(defun destroy-giskard-register-groups-service ()
  (when *giskard-register-groups-service*
    (roslisp:close-persistent-service *giskard-register-groups-service*))
  (setf *giskard-register-groups-service* nil))

(roslisp-utilities:register-ros-cleanup-function destroy-giskard-environment-service)
(roslisp-utilities:register-ros-cleanup-function destroy-giskard-groups-service)
(roslisp-utilities:register-ros-cleanup-function destroy-giskard-register-groups-service)

(defun make-giskard-environment-request (add-or-remove-or-attach-or-detach
                                         &key
                                           name
                                           pose
                                           mesh-path
                                           dimensions
                                           joint-state-topic
                                           parent-link
                                           parent-link-group)
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
  (let ((body (if mesh-path
                  (roslisp:make-msg
                   'giskard_msgs-msg:worldbody
                   :type (roslisp:symbol-code
                          'giskard_msgs-msg:worldbody
                          :mesh_body)
                   :mesh mesh-path)
                  (roslisp:make-msg
                   'giskard_msgs-msg:worldbody
                   :type (roslisp:symbol-code
                          'giskard_msgs-msg:worldbody
                          :primitive_body)
                   :shape (roslisp:make-msg
                           'shape_msgs-msg:solidprimitive
                           :type (roslisp:symbol-code 'shape_msgs-msg:solidprimitive :box)
                           :dimensions (map 'vector #'identity dimensions))))))
    (ecase add-or-remove-or-attach-or-detach
      (:add
       (roslisp:make-request
        'giskard_msgs-srv:updateworld
        :operation (roslisp:symbol-code
                    'giskard_msgs-srv:updateworld-request
                    :add)
        :timeout *giskard-environment-service-timeout*
        :group_name name
        :body body
        :pose (cl-transforms-stamped:to-msg pose)
        :timeout *giskard-environment-service-timeout*))
      (:remove
       (roslisp:make-request
        'giskard_msgs-srv:updateworld
        :operation (roslisp:symbol-code
                    'giskard_msgs-srv:updateworld-request
                    :remove)
        :group_name name
        :timeout *giskard-environment-service-timeout*))
      (:alter
       (roslisp:make-request
        'giskard_msgs-srv:updateworld
        :operation (roslisp:symbol-code
                    'giskard_msgs-srv:updateworld-request
                    :update_pose)
        :body body
        :group_name name
        :pose (cl-transforms-stamped:to-msg pose)
        :timeout *giskard-environment-service-timeout*))
      (:remove-all
       (roslisp:make-request
        'giskard_msgs-srv:updateworld
        :operation (roslisp:symbol-code
                    'giskard_msgs-srv:updateworld-request
                    :remove_all)
        :timeout *giskard-environment-service-timeout*))
      (:add-environment
       (roslisp:make-request
        'giskard_msgs-srv:updateworld
        :operation (roslisp:symbol-code
                    'giskard_msgs-srv:updateworld-request
                    :add)
        :group_name name
        :body (roslisp:make-msg
               'giskard_msgs-msg:worldbody
               :type (roslisp:symbol-code
                      'giskard_msgs-msg:worldbody
                      :urdf_body)
               :urdf (roslisp:get-param rob-int:*environment-description-parameter* nil)
               :joint_state_topic joint-state-topic)
        :pose (cl-transforms-stamped:to-msg pose)
        :timeout *giskard-environment-service-timeout*))
      (:detach
       (roslisp:make-request
        'giskard_msgs-srv:updateworld
        :operation (roslisp:symbol-code
                    'giskard_msgs-srv:updateworld-request
                    :update_parent_link)
        :group_name name
        :timeout *giskard-environment-service-timeout*))
      (:attach
       (roslisp:make-request
        'giskard_msgs-srv:updateworld
        :operation (roslisp:symbol-code
                    'giskard_msgs-srv:updateworld-request
                    :update_parent_link)
        :group_name name
        :parent_link parent-link
        :parent_link_group
        (or parent-link-group
            (when (gethash parent-link (btr:links (btr:get-environment-object)))
              (roslisp-utilities:rosify-underscores-lisp-name (rob-int:get-environment-name)))
            (when (gethash parent-link (btr:links (btr:get-robot-object)))
              (roslisp-utilities:rosify-underscores-lisp-name (rob-int:get-robot-name)))
            "")
        :timeout *giskard-environment-service-timeout*
        :body body
        :pose (cl-transforms-stamped:to-msg pose))))))

(defun ensure-giskard-environment-service-input-parameters ()
  )

(defun ensure-giskard-environment-service-result ()
  )


(defun call-giskard-environment-service (add-or-remove-or-attach-or-detach
                                         &key name pose dimensions mesh-path
                                           joint-state-topic parent-link
                                           parent-link-group)
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
          (when (string-equal
                 (roslisp:msg-slot-value
                  (roslisp:call-persistent-service
                   service
                   (make-giskard-environment-request
                    add-or-remove-or-attach-or-detach
                    :name name
                    :pose pose
                    :mesh-path mesh-path
                    :dimensions dimensions
                    :joint-state-topic joint-state-topic
                    :parent-link parent-link
                    :parent-link-group parent-link-group))
                  'giskard_msgs-srv:error_msg)
                 "Cannot load URDF file.")
            (roslisp:msg-slot-value
             (roslisp:call-persistent-service
              service
              (make-giskard-environment-request
               add-or-remove-or-attach-or-detach
               :name name
               :pose pose
               :mesh-path nil
               :dimensions dimensions
               :joint-state-topic joint-state-topic
               :parent-link parent-link
               :parent-link-group parent-link-group))
             'giskard_msgs-srv:error_msg))
          nil))))

;;;;;;;;; GROUPS SERVICE CALLS;;;;;;;;;

(defun call-giskard-groups-service ()
  (roslisp:msg-slot-value
   (roslisp:call-persistent-service
    (get-giskard-groups-service)
    (roslisp:make-request giskard_msgs-srv:getgroupnames))
   'giskard_msgs-srv:group_names))

(defun get-groups ()
  (coerce (call-giskard-groups-service) 'list))

(defun call-giskard-register-groups-service (group-name
                                             parent-group-name
                                             root-link-name)
  (declare (type string group-name parent-group-name root-link-name))
  (roslisp:msg-slot-value
   (roslisp:call-persistent-service
    (get-giskard-register-groups-service)
    (roslisp:make-request
     giskard_msgs-srv:registergroup
     :group_name group-name
     :parent_group_name parent-group-name
     :root_link_name root-link-name))
   'giskard_msgs-srv:error_codes))

(defun register-group (&key group-name parent-group-name root-link)
  (declare (type string root-link parent-group-name))
  "Create a new group of links. They are used in collision entries e.g. for arms and grippers.
If `group-name' is NIL, take the `root-link' as group name.
The `parent-group-name' is usually the robot or environment."
  (unless (member (or group-name root-link) (get-groups) :test #'string=)
    (call-giskard-register-groups-service (or group-name root-link)
                                          parent-group-name
                                          root-link)))

;;;;;;;;;;;;;;;;;;;;;;;; UTILS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun reset-collision-scene ()
  (call-giskard-environment-service
   :remove-all)
  (when (btr:get-environment-object)
    (call-giskard-environment-service
     :add-environment
     :name (roslisp-utilities:rosify-underscores-lisp-name
            (rob-int:get-environment-name))
     :pose (cl-transforms-stamped:pose->pose-stamped
            cram-tf:*fixed-frame* 0.0 (btr:pose (btr:get-environment-object)))
     :joint-state-topic "kitchen/joint_states")))

(defun update-object-pose-in-collision-scene (object-name)
  (when object-name
    (let* ((object-name-string
             (roslisp-utilities:rosify-underscores-lisp-name object-name))
           (btr-object
             (btr:object btr:*current-bullet-world* object-name)))
      (call-giskard-environment-service
       :alter
       :name object-name-string
       :pose (cl-transforms-stamped:pose->pose-stamped
              cram-tf:*fixed-frame* 0.0 (btr:pose btr-object))
       :mesh-path (when btr-object
                    (second (assoc (car (btr:item-types btr-object))
                                   btr::*mesh-files*)))
       :dimensions (with-slots (cl-transforms:x cl-transforms:y cl-transforms:z)
                       (btr:calculate-bb-dims btr-object)
                     (list cl-transforms:x cl-transforms:y cl-transforms:z))))))

(defun add-object-to-collision-scene (object-name)
  (let* ((object-name-string
           (roslisp-utilities:rosify-underscores-lisp-name object-name))
         (btr-object
           (btr:object btr:*current-bullet-world* object-name))
         (robot-links-object-is-attached-to
           (or (btr:object-attached (btr:get-environment-object) btr-object)
               (btr:object-attached (btr:get-robot-object) btr-object))))
    ;; to update an object pose, first remove the old object together with the pose
    (when (member object-name-string (get-groups) :test #'string=)
      (call-giskard-environment-service
       :remove
       :name object-name-string))
    ;; add it at the new perceived pose
    (call-giskard-environment-service
     :add
     :name object-name-string
     :pose (cl-transforms-stamped:pose->pose-stamped
            cram-tf:*fixed-frame* 0.0 (btr:pose btr-object))
     :mesh-path (when btr-object
                  (second (assoc (car (btr:item-types btr-object))
                                 btr::*mesh-files*)))
     :dimensions (with-slots (cl-transforms:x cl-transforms:y cl-transforms:z)
                     (btr:calculate-bb-dims btr-object)
                   (list cl-transforms:x cl-transforms:y cl-transforms:z)))
    ;; reattach the object if it was attached somewhere
    (when robot-links-object-is-attached-to
      (let ((link (car robot-links-object-is-attached-to)))
        (call-giskard-environment-service
         :attach
         :name object-name-string
         :pose (cl-transforms-stamped:pose->pose-stamped
                cram-tf:*fixed-frame* 0.0 (btr:pose btr-object))
         :mesh-path (when btr-object
                      (second (assoc (car (btr:item-types btr-object))
                                     btr::*mesh-files*)))
         :dimensions (with-slots (cl-transforms:x cl-transforms:y cl-transforms:z)
                         (btr:calculate-bb-dims btr-object)
                       (list cl-transforms:x cl-transforms:y cl-transforms:z))
         :parent-link link)))))

(defun detach-object-in-collision-scene (object-name)
  (let* ((object-name-string
           (roslisp-utilities:rosify-underscores-lisp-name object-name))
         (btr-object
           (btr:object btr:*current-bullet-world* object-name)))
    (when btr-object
      (let ((attached-to-another-link-as-well?
              (> (length
                  (btr:object-name-attached-links
                   (btr:get-robot-object)
                   object-name))
                 1)))
        (unless attached-to-another-link-as-well?
          (call-giskard-environment-service
           :detach
           :name object-name-string))))))

(defun attach-object-to-arm-in-collision-scene (object-name arm link)
  (let* ((object-name-string
           (roslisp-utilities:rosify-underscores-lisp-name object-name))
         (btr-object
           (btr:object btr:*current-bullet-world* object-name))
         (link (if arm
                   (cut:var-value
                    '?ee-link
                    (car (prolog:prolog
                          `(and (rob-int:robot ?robot)
                                (rob-int:end-effector-link ?robot ,arm ?ee-link)))))
                   link)))
    (when (cut:is-var link)
      (error "[GISKARD OBJECT-ATTACHED] Couldn't find robot's EE link."))
    (when btr-object
      (let* ((map-to-ee-transform
               (cl-transforms-stamped:lookup-transform
                cram-tf:*transformer*
                cram-tf:*fixed-frame*
                link
                :timeout cram-tf:*tf-default-timeout*
                :time 0))
             (ee-to-map-transform
               (cram-tf:transform-stamped-inv map-to-ee-transform))
             (map-to-obj-transform
               (cram-tf:pose->transform-stamped
                cram-tf:*fixed-frame*
                object-name-string
                0.0
                (btr:pose btr-object)))
             (ee-to-object-transform
               (cram-tf:multiply-transform-stampeds
                link object-name-string
                ee-to-map-transform map-to-obj-transform))
             (ee-to-object-pose
               (cram-tf:strip-transform-stamped ee-to-object-transform)))
        ;; remove the object first, maybe it was already attached to something
        (call-giskard-environment-service
         :detach
         :name object-name-string)
        (call-giskard-environment-service
         :attach
         :name object-name-string
         :pose ee-to-object-pose
         :mesh-path (when btr-object
                      (second (assoc (car (btr:item-types btr-object))
                                     btr::*mesh-files*)))
         :dimensions (with-slots (cl-transforms:x cl-transforms:y cl-transforms:z)
                         (btr:calculate-bb-dims btr-object)
                       (list cl-transforms:x cl-transforms:y cl-transforms:z))
         :parent-link link
         :parent-link-group (roslisp-utilities:rosify-underscores-lisp-name
                             (rob-int:get-robot-name)))))))

(defun full-update-collision-scene ()
  (mapcar (lambda (object)
            (when (typep object 'btr:item)
              (let ((object-name (btr:name object)))
                (add-object-to-collision-scene object-name)
                (when (btr:object-attached
                       (btr:get-robot-object)
                       (btr:object btr:*current-bullet-world* object-name))
                  (attach-object-to-arm-in-collision-scene
                   object-name
                   nil
                   (car (btr:object-name-attached-links
                         (btr:get-robot-object)
                         object-name)))))))
          (btr:objects btr:*current-bullet-world*)))

;;;;;;;;;;;;;;;;;;;;;;;; EVENT HANDLERS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod coe:clear-belief giskard-clear ()
  (unless cram-projection:*projection-environment*
    (reset-collision-scene)))

(defmethod coe:on-event giskard-attach-object ((event cpoe:object-attached-robot))
  (unless cram-projection:*projection-environment*
    (attach-object-to-arm-in-collision-scene
     (cpoe:event-object-name event)
     (cpoe:event-arm event)
     (cpoe:event-link event))))

(defmethod coe:on-event giskard-detach-object 1 ((event cpoe:object-detached-robot))
  (unless cram-projection:*projection-environment*
    (let ((object-name (cpoe:event-object-name event)))
      (if object-name
          ;; if object-name is given, detach given object
          (detach-object-in-collision-scene object-name)
          ;; otherwise detach all objects from the given arm
          (let* ((arm
                   (cpoe:event-arm event))
                 (link
                   (if arm
                       (cut:var-value
                        '?link
                        (car
                         (prolog:prolog
                          `(and (rob-int:robot ?rob)
                                (rob-int:end-effector-link ?rob ,arm ?link)))))
                       (cpoe:event-link event))))
            (unless (cut:is-var link)
              (mapcar #'detach-object-in-collision-scene
                      (btr:link-attached-object-names
                       (btr:get-robot-object)
                       link))))))))

(defmethod coe:on-event giskard-detach-object-after 3 ((event cpoe:object-detached-robot))
  (unless cram-projection:*projection-environment*
    (update-object-pose-in-collision-scene (cpoe:event-object-name event))))

(defmethod coe:on-event giskard-perceived ((event cpoe:object-perceived-event))
  (unless cram-projection:*projection-environment*
    (add-object-to-collision-scene
     (desig:desig-prop-value (cpoe:event-object-designator event) :name))))
