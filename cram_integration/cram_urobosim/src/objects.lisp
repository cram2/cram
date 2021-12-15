;;;
;;; Copyright (c) 2021, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;                     Arthur Niedzwiecki <aniedz@cs.uni-bremen.de>
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

(in-package :unreal)

(defun spawn-object (object-type pose
                     &key
                       (object-name object-type)
                       (mass 1.0)
                       (gravity T)
                       (mobility 2))
  (declare (type (or symbol keyword string) object-name object-type)
           (type cl-transforms:pose pose))
  (setf pose (ensure-stripped-pose pose))
  (let ((service (get-service :spawn-object)))
    (roslisp:with-fields (id name success etype)
        (roslisp:call-persistent-service
         service
         (roslisp:make-request
          'world_control_msgs-srv:spawnmodel
          :id (string-upcase (string object-name))
          :name (string-upcase (string object-type))
          :pose (cl-transforms-stamped:to-msg pose)
          ;; mobility: 0 static, 1 stationary, 2 movable
          (:mobility :physics_properties) mobility 
          (:gravity :physics_properties) gravity
          (:mass :physics_properties) mass
          ;; makes the object movable by the robot
          (:generate_overlap_events :physics_properties) T
          ;; creates the tag CramObject;type,OBJECT-TYPE;
          :tags (vector (roslisp:make-message
                         'world_control_msgs-msg:Tag
                         :type "CramObject"
                         :key "type"
                         :value (string-upcase (string object-type))))
          ;; :path ""
          :actor_label (string-upcase (string object-name))
          ;; :material_names #("")
          ;; :material_paths #("")
          :parent_id ""
          ;; :spawn_collision_check NIL
          ))
      (unless success
        (if (string= etype "1")
            (progn
              (roslisp:ros-warn (unreal spawn-object)
                                "Object ID ~a already in use. Moving existing object."
                                object-name)
              (set-object-pose object-name pose))
            (roslisp:ros-error (unreal spawn-object)
                               "Spawning ~a failed. Check if asset exists in Unreal project files."
                               (string object-type))))
      id)))

(defun set-object-pose (object-name pose)
  (declare (type (or symbol keyword string) object-name)
           (type cl-transforms:pose pose))
  (setf pose (ensure-stripped-pose pose))
  (let ((service (get-service :set-object-pose)))
    (roslisp:with-fields (success)
        (roslisp:call-persistent-service
         service
         (roslisp:make-request
          'world_control_msgs-srv:setmodelpose
          :id (string object-name)
          :pose (cl-transforms-stamped:to-msg pose)))
      (unless success
       (roslisp:ros-error (unreal set-object-pose)
                          "~a~%~a~%Does the object with ID ~a exist?"
                          "Setting object pose failed."
                          "Probably the new pose would cause a collision with something else."
                          (string object-name))))))

(defun delete-object (object-name)
  (declare (type (or symbol keyword string) object-name))
  (let ((service (get-service :delete-object)))
    (roslisp:with-fields (success)
        (roslisp:call-persistent-service
         service
         (roslisp:make-request
          'world_control_msgs-srv:deletemodel
          :id (string object-name)))
      (unless success
        (roslisp:ros-error (unreal delete-object)
                           "There's no object with ID ~a to delete."
                           (string object-name)))
      success)))

(defun get-object-pose (object-name)
  "Returns the object's pose as Unreal pose (Y is inverted wrt TF)"
  (declare (type (or symbol keyword string) object-name))
  (let ((service (get-service :get-object-pose)))
    (roslisp:with-fields (success pose)
        (roslisp:call-persistent-service
         service
         (roslisp:make-request
          'world_control_msgs-srv:getmodelpose
          :id (string object-name)))
      (if success
          (cl-transforms-stamped:from-msg pose)
          (progn (roslisp:ros-error (unreal get-object-pose)
                                    "Pose unkown for the object with ID ~a."
                                    (string object-name))
                 NIL)))))


(defun attach-object () (error "Not implemented."))
(defun change-material () (error "Not implemented."))
(defun delete-all () (error "Not implemented."))
(defun get-socket-pose () (error "Not implemented."))
(defun highlight-object () (error "Not implemented."))
(defun object-to-object-state () (error "Not implemented."))
(defun set-object-physics () (error "Not implemented."))
(defun spawn-constraint () (error "Not implemented."))
(defun spawn-mesh () (error "Not implemented."))
(defun spawn-map () (error "Not implemented."))


;;;;;;;;;;;;;;; UTILS ;;;;;;;;;;;;;;;

(defun ensure-stripped-pose (pose)
  (declare (type cl-transforms:pose pose))
  "Strips pose of it's stamp and ensures it to be in the world's *fixed-frame*."
  (if (typep pose 'cl-transforms-stamped:pose-stamped)
      (cl-transforms-stamped:pose-stamped->pose
       (cram-tf:ensure-pose-in-frame pose cram-tf:*fixed-frame*))
      pose))
