;;;
;;; Copyright (c) 2021, Arthur Niedzwiecki <aniedz@cs.uni-bremen.de>
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

(defparameter *unreal-service-names*
  '(:attach-object "attach_model_to_parent"
    :change-material "change_material"
    :delete-all "delete_all"
    :delete-object "delete_model"
    :get-object-pose "get_model_pose"
    :get-socket-pose "get_model_socket_pose"
    :highlight-object "highlight_models"
    :object-to-object-state "object_to_object_state"
    :reset-world "reset_level"
    :set-object-pose "set_model_pose"
    :set-object-physics "set_physics_properties"
    :spawn-object "spawn_model"
    :spawn-constraint "spawn_physics_constraint"
    :spawn-mesh "spawn_pro_mesh"
    :spawn-map "spawn_semantic_map")
  "Maps each world_control_msgs service to a keyword.")

(defparameter *unreal-services* (make-hash-table)
  "Collection of clients for each world_control_msgs service.")

(defparameter *srv-ns* "UnrealSim"
  "Namespace of the service server.")

(defparameter *msg-ns* "world_control_msgs"
  "Namespace of the service messages.")

(defun srv-key->srv-type (srv-key)
  "Maps the `srv-key' to the service's message type."
  (concatenate 'string *msg-ns* "/"
               (remove #\_ (string-capitalize (getf *unreal-service-names* srv-key)))))

(defun srv-key->srv-full-name (srv-key)
  "Maps the `srv-key' to the service's full name as <service-namespace>/<service-name>."
  (concatenate 'string *srv-ns* "/"
               (or (getf *unreal-service-names* srv-key)
                   (error "Service for key ~a can't be resolved, try one of ~a"
                          srv-key
                          *unreal-service-names*))))

(defun get-service (service-key)
  (declare (type keyword service-key))
  "Returns the service for the given `service-key' stored in `*unreal-services*'.
   If the persistent service doesn't exist yet, it will be created."
  (unless (eq (roslisp:node-status) :RUNNING)
    (roslisp-utilities:startup-ros))
  (let ((service (gethash service-key *unreal-services*))
        (service-name (getf *unreal-service-names* service-key)))
    (unless service-name
      (error "Service for ~a is unknown, try one of these:~%~a"
             service-key *unreal-service-names*))
    (if (and service (roslisp:persistent-service-ok service))
        service
        (setf (gethash service-key *unreal-services*)
              (make-instance 'roslisp:persistent-service
                 :service-name (format NIL "~a/~a"
                                       *srv-ns*
                                       service-name)
                 :service-type (srv-key->srv-type service-key))))))

(defun close-services ()
  "Shuts down and cleans up all persistent `*unreal-services*'."
  (loop for service being the hash-values in *unreal-services*
        do (roslisp:close-persistent-service service))
  (setf *unreal-services* (make-hash-table)))

(roslisp-utilities:register-ros-cleanup-function close-services)

(defun reset-world ()
  "Calls service to reset the Unreal world."
  (roslisp:call-persistent-service (get-service :reset-world)
    (roslisp:make-request 'world_control_msgs-srv:resetlevel :id "0"))
  ;; Resetting takes a moment in unreal
  (sleep 1))

