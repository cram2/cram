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

(defun spawn-object (object-type pose object-name
                     &key (mass 1.0) (gravity T) (mobility 2))
  (declare (type (or symbol keyword string) object-name object-type)
           (type cl-transforms:pose pose))
  (when (typep pose 'cl-transforms-stamped:pose-stamped)
    (setf pose (cl-transforms-stamped:pose-stamped->pose
                (cram-tf:ensure-pose-in-frame pose cram-tf:*fixed-frame*))))
  (let ((service (get-service :spawn-object)))
    (roslisp:with-fields (success etype)
        (roslisp:call-persistent-service
         service
         (roslisp:make-request
          'world_control_msgs-srv:spawnmodel
          :id (string object-name)
          :name (string object-type)
          :pose (cl-transforms-stamped:to-msg pose)
          (:mobility :physics_properties) mobility ;; 0 static, 1 stationary, 2 movable
          (:gravity :physics_properties) gravity
          (:mass :physics_properties) mass))
      (unless success
        (if (string= etype "1")
            (progn
              (roslisp:ros-warn (unreal spawn-object)
                                "Object ID ~a already in use. Moving existing object."
                                object-name)
              (set-object-pose object-name pose))
            (roslisp:ros-error (unreal spawn-object)
    "Spawning object failed. Check if the asset ~a exists in the Unreal project files."
                               (string object-type)))))))

;; Attach to existing btr methods
;; (defmethod btr:add-object :after (...) )

(defun set-object-pose (object-name pose)

;;   rosservice call /UnrealSim/set_model_pose "id: ''
;; pose:
;;   position:
;;     x: 0.0
;;     y: 0.0
;;     z: 0.0
;;   orientation:
;;     x: 0.0
;;     y: 0.0
;;     z: 0.0
;;     w: 0.0" 

  )

(defun delete-object (object-name))

;; "name: 'Bowl'
;; pose:
;;   position: {x: 0.0, y: 1.0, z: 2.0}
;;   orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
;; id: ''
;; tags:
;; - {type: '', key: '', value: ''}
;; path: ''
;; actor_label: ''
;; physics_properties: {mobility: 2, gravity: true, generate_overlap_events: false,
;;   mass: 1.0}
;; material_names: ['']
;; material_paths: ['']
;; parent_id: ''
;; spawn_collision_check: false"



;; (roslisp:wait-for-service (concatenate 'string
;;                     (getf *ik-service-namespaces* left-or-right) "/get_ik_solver_info") 10.0)
;;               (roslisp:call-service
;;                (concatenate 'string (getf *ik-service-namespaces* left-or-right) "/get_ik")
;;                "moveit_msgs/GetPositionIK"
;;                (roslisp:make-request
;;                 "moveit_msgs/GetPositionIK"
;;                 (:ik_link_name :ik_request) ik-link
;;                 (:pose_stamped :ik_request) (cl-transforms-stamped:to-msg
;;                                              (cl-transforms-stamped:pose->pose-stamped
;;                                               ik-base-frame 0.0 cartesian-pose))
;;                 (:joint_state :robot_state :ik_request) (make-current-seed-state left-or-right)
;;                 (:timeout :ik_request) 1.0))

;; /UnrealSim/attach_model_to_parent
;; /UnrealSim/change_material
;; /UnrealSim/delete_all
;; /UnrealSim/delete_model
;; /UnrealSim/get_model_pose
;; /UnrealSim/get_model_socket_pose
;; /UnrealSim/highlight_models
;; /UnrealSim/object_to_object_state
;; /UnrealSim/reset_level
;; /UnrealSim/set_model_pose
;; /UnrealSim/set_physics_properties
;; /UnrealSim/spawn_model
;; /UnrealSim/spawn_physics_constraint
;; /UnrealSim/spawn_pro_mesh
;; /UnrealSim/spawn_semantic_map
