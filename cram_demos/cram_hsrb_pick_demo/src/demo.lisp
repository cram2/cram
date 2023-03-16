;;;
;;; Copyright (c) 2019, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;                      Vanessa Hassouna <hassouna@uni-bremen.de>
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
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors
;;;       may be used to endorse or promote products derived from this software
;;;       without specific prior written permission.
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

(in-package :demo)

(defun spawn-pickup-cylinder-table ()
  "Spawn primitive cylinder as :pringles item and try to pick up from table."
  (urdf-proj:with-simulated-robot
    (btr:add-object btr:*current-bullet-world* :cylinder-item 'cylinder-1
                    '((-0.7 -0.7 0.85) (0 0 1 1))
                    :mass 0.2
                    :size (cl-transforms:make-3d-vector 0.03 0.03 0.08)
                    :item-type :pringles)
    (pick-up-object 'cylinder-1 :pringles)))

(defun spawn-pickup-cylinder-air ()
  "Spawn primitive cylinder as :pringles item and try to pick up."
  (urdf-proj:with-simulated-robot
    (btr:add-object btr:*current-bullet-world* :cylinder-item 'cylinder-1
                    '((0.7 0.0777 0.65) (0 0 1 1))
                    :mass 0.2 :size (cl-transforms:make-3d-vector 0.03 0.03 0.08)
                    :item-type :pringles)
    (pick-up-object 'cylinder-1 :pringles)))

;;;;;;;;;;;;;
;; Test-Demos

;; Aus irgendeinem grund wird beim retry von going die collision nicht entfernt
;; wenn ich das failurehandling wegnehme und (car ?nav-poses) zu (second ?nav-poses) ändere
;; von welcher ich weiß dass es eine funktionierende Pose ist, funktioniert es. Mit
;; failurehandling allerdings wird in cram_urdf_projection -> low-level.lisp bei drive
;; der fehler ausgelöst, dass der hsr mit der welt kollidiert, selbst wenn ich den roboter
;; schweben lasse, so dass er definitiv mit nichts kollidieren kann.
;; Wenn ich bei try-looking-pose 1.0 eingebe, funktioniert es nicht, da der roboter dann in der
;; ersten pose mit der wand hinter dem tisch knapp kollidiert
;; wenn ich allerdings 1.1 eingebe funktioniert es auf einmal, da es nun keine kollision mit der
;; wand gibt
(defun fake-pickup-test ()
  "Spawn primitive cylinder as :pringles item and try to pick up."
  (urdf-proj:with-simulated-robot
    ;;Spawn object
    (spawn-pringles)
    
    (let*((?nav-pose (knowledge-get-nav-pose1)) ;; Dummy navigation pose query
          (?object-hc-pose (knowledge-get-looking-pose 'cylinder-1))) ;; Dummy looking pose query
      
      (hsr-carry-pose) ;; Carry pose
      (exe:perform
       (desig:an action
                 (type going)
                 (target (desig:a location (pose ?nav-pose)))))
            
      (exe:perform
       (desig:an action
                 (type looking)
                 (target (desig:a location (pose ?object-hc-pose)))))
      (let*((?object-desig
              (exe:perform (desig:a motion
                                    (type detecting)
                                    (object (desig:an object
                                                      (type :pringles)))))))
              
        (exe-perform-type :picking-up
                          :arm :left
                          :object ?object-desig)))))


(defun fake-place-test ()
  "Spawn primitive cylinder as :pringles item and try to pick up."
  (urdf-proj:with-simulated-robot
    ;;Spawn object
    
    (let*((?target-pose (knowledge-get-target-pose2)) ;; Dummy looking pose query
          (?nav-pose (knowledge-get-nav-pose2))) ;; Dummy navigation pose query
      
      (hsr-carry-pose) ;; Carry pose
      (exe:perform
       (desig:an action
                 (type going)
                 (target (desig:a location (pose ?nav-pose)))))
            
      (exe:perform
       (desig:an action
                 (type looking)
                 (target (desig:a location (pose ?target-pose)))))
            
              
      (exe-perform-type :placing
                        :arm :left
                        :pose ?target-pose))))

(defun fake-open-test ()
  "Spawn primitive cylinder as :pringles item and try to pick up."
  (urdf-proj:with-simulated-robot
  
    (let*((?nav-pose (knowledge-get-nav-pose3));; Dummy navigation pose query    
          (?shelf-left (knowledge-get-shelf-left))   
          (?shelf-right (knowledge-get-shelf-right))) ;; Dummy looking pose query
      
      (hsr-carry-pose) ;; Carry pose
      (exe:perform
       (desig:an action
                 (type going)
                 (target (desig:a location (pose ?nav-pose)))))
            
      (exe-perform-type :opening
                        :object ?shelf-left)
      
      (exe-perform-type :opening
                        :object ?shelf-right))))

(defun fake-close-test ()
  "Spawn primitive cylinder as :pringles item and try to pick up."
  (urdf-proj:with-simulated-robot
  
    (let*((?nav-pose (knowledge-get-nav-pose3));; Dummy navigation pose query    
          (?shelf-left (knowledge-get-shelf-left))   
          (?shelf-right (knowledge-get-shelf-right))) ;; Dummy looking pose query
      
      (hsr-carry-pose) ;; Carry pose
      (exe:perform
       (desig:an action
                 (type going)
                 (target (desig:a location (pose ?nav-pose)))))
            
      (exe-perform-type :opening
                        :object ?shelf-left)
      
      (exe-perform-type :opening
                        :object ?shelf-right))))

(defun serve-breakfast ()
  "Spawn primitive cylinder as :pringles item and try to pick up."
  (urdf-proj:with-simulated-robot
    (spawn-pringles-shelf)
    (door-closed)

    (let*((?nav-pose (knowledge-get-entrance-pose)))
      (hsr-carry-pose) ;; Carry pose
      (sleep 1)
      (exe:perform
       (desig:an action
                 (type going)
                 (target (desig:a location (pose ?nav-pose))))))


    (let ((?entrance (knowledge-get-entrance)))
      (sleep 1)
      (exe-perform-type :opening
                        :object ?entrance))


    (let ((?entrance (knowledge-get-entrance)))
      (sleep 1)
      (exe-perform-type :closing
                        :object ?entrance))
    (door-closed)

    (let*((?nav-pose (knowledge-get-shelf-pose)))
      (hsr-carry-pose) ;; Carry pose
      (sleep 1)
      (exe:perform
       (desig:an action
                 (type going)
                 (target (desig:a location (pose ?nav-pose))))))
      

    (let ((?shelf-left (knowledge-get-shelf-left)))
      (sleep 1)
      (exe-perform-type :opening
                        :object ?shelf-left))
      
    (let ((?shelf-right (knowledge-get-shelf-right)))
      (sleep 1)
      (exe-perform-type :opening
                        :object ?shelf-right))
    
    (let ((?object-pose (knowledge-get-looking-pose 'cylinder-1)))
      (sleep 1)
      (exe:perform
       (desig:an action
                 (type looking)
                 (target (desig:a location (pose ?object-pose)))))
      (let((?object-desig
             (exe:perform (desig:a motion
                                   (type detecting)
                                   (object (desig:an object
                                                     (type :pringles)))))))
        (sleep 1)
        (exe-perform-type :picking-up
                          :object ?object-desig)))
    
    (let* ((?target-nav-pose (knowledge-get-table-pose1))
           (?target-pose (knowledge-get-target-pose1)))
      (sleep 1)
      (exe:perform
       (desig:an action
                 (type going)
                 (target (desig:a location (pose ?target-nav-pose)))))
      
      (exe:perform
       (desig:an action
                 (type looking)
                 (target (desig:a location (pose ?target-pose)))))
            
      (sleep 1)
      (exe-perform-type :placing
                        :pose ?target-pose))
    
    (let*((?nav-pose (knowledge-get-shelf-pose)))
      (hsr-carry-pose) ;; Carry pose
      (sleep 1)
      (exe:perform
       (desig:an action
                 (type going)
                 (target (desig:a location (pose ?nav-pose))))))
      
    (let ((?shelf-left (knowledge-get-shelf-left)))
      (sleep 1)
      (exe-perform-type :closing
                        :object ?shelf-left))
      
    (let ((?shelf-right (knowledge-get-shelf-right)))
      (sleep 1)
      (exe-perform-type :closing
                        :object ?shelf-right))))



;;;;;;;;;;;;;;;;;;
;; Fake-Designator

(defgeneric exe-perform-type (action-type &key &allow-other-keys))

(defmethod exe-perform-type ((action-type (eql :picking-up)) &key (arm :left) object)
  (let*((gripper-pose
            (cl-tf::lookup-transform cram-tf:*transformer* "map" "/hand_gripper_tool_frame"))
          (obj-name (desig:desig-prop-value object :name)))
      (roslisp:with-fields (translation rotation)
          gripper-pose
        (btr-utils:move-object
         obj-name
         (cl-tf:make-pose-stamped
          "map" 0.0
          translation
          (cl-tf:make-quaternion 0 0 0 1))))
      (cram-occasions-events:on-event
       (make-instance 'cpoe:object-attached-robot
                      :arm arm
                      :object-name obj-name
                      :object-designator object
                      :grasp :front))))

(defmethod exe-perform-type ((action-type (eql :placing)) &key (arm :left) pose)
  (let*((obj-name (check-arm-for-object)))
    (btr-utils:move-object
     obj-name
     pose)
    (cram-occasions-events:on-event
     (make-instance 'cpoe:object-detached-robot
                    :arm (list arm)
                    :object-name obj-name))))

(defmethod exe-perform-type ((action-type (eql :opening)) &key object (distance 1.7))
    (cond
      ((search "right" object)  (let ((?object-pose (knowledge-get-right-handle-pose)))
                                  (exe:perform
                                   (desig:an action
                                             (type looking)
                                             (target (desig:a location (pose ?object-pose)))))
                                  (sleep 1)
                                  (let ((dist (* distance 1)))
                                    (btr:set-robot-state-from-joints
                                     `((,object ,dist))
                                     (btr:get-environment-object)))))
       
      ((search "left" object)  (let ((?object-pose (knowledge-get-left-handle-pose)))
                                 (exe:perform
                                  (desig:an action
                                            (type looking)
                                            (target (desig:a location (pose ?object-pose)))))
                                 (sleep 1)
                                 (let ((dist (* distance -1)))
                                   (btr:set-robot-state-from-joints
                                    `((,object ,dist))
                                    (btr:get-environment-object)))))
      
      ((search "origin" object)  (let ((?object-pose (knowledge-get-entrance-handle-pose)))
                                  (exe:perform
                                   (desig:an action
                                             (type looking)
                                             (target (desig:a location (pose ?object-pose)))))
                                   (sleep 1)
                                   (let ((dist (if (> distance 0)
                                                    0
                                                    distance)))
                                     (btr:set-robot-state-from-joints
                                      `((,object ,dist))
                                      (btr:get-environment-object)))))
      (t (error "Please enter a valid openable object"))))

(defmethod exe-perform-type ((action-type (eql :closing)) &key object)
  (cond
      ((search "right" object)  (let ((?object-pose (knowledge-get-right-handle-pose)))
                                  (exe:perform
                                   (desig:an action
                                             (type looking)
                                             (target (desig:a location (pose ?object-pose)))))
                                  (sleep 1)
                                  (btr:set-robot-state-from-joints
                                   `((,object 0))
                                   (btr:get-environment-object))))
       
      ((search "left" object)  (let ((?object-pose (knowledge-get-left-handle-pose)))
                                 (exe:perform
                                  (desig:an action
                                            (type looking)
                                            (target (desig:a location (pose ?object-pose)))))
                                 (sleep 1)
                                 (btr:set-robot-state-from-joints
                                  `((,object 0))
                                  (btr:get-environment-object))))
      
      ((search "origin" object)  (let ((?object-pose (knowledge-get-entrance-handle-pose)))
                                   (exe:perform
                                    (desig:an action
                                              (type looking)
                                              (target (desig:a location (pose ?object-pose)))))
                                   (sleep 1)
                                     (btr:set-robot-state-from-joints
                                      `((,object -1.4))
                                      (btr:get-environment-object))))
      (t (error "Please enter a valid openable object"))))
      

;;;;;;;;;;;;;;;
;; Pose-Utility

(defun ground-pose (pose)
  (roslisp:with-fields (x y z)
      (cl-transforms:origin pose)
    (cl-transforms-stamped:make-pose-stamped
     "map" 0
     (cl-transforms:make-3d-vector x y 0.0)
     (cl-transforms:orientation pose))))

(defun rotate-once-pose (pose angle axis)
  (cl-transforms-stamped:copy-pose-stamped
   pose
   :orientation (let ((pose-orientation (cl-transforms:orientation pose)))
                  (cl-tf:normalize
                   (cl-transforms:q*
                    (cl-transforms:axis-angle->quaternion
                     (case axis
                       (:x (cl-transforms:make-3d-vector 1 0 0))
                       (:y (cl-transforms:make-3d-vector 0 1 0))
                       (:z (cl-transforms:make-3d-vector 0 0 1))
                       (t (error "in ROTATE-ONCE-POSE forgot to specify axis properly: ~a" axis)))
                     angle)
                    pose-orientation)))))

(defun translate-once-pose (pose dist axis)
  (cl-transforms-stamped:copy-pose-stamped
   pose
   :origin (let ((pose-orientation (cl-transforms:origin pose)))
              (cl-transforms:v+
               (case axis
                 (:x (cl-transforms:make-3d-vector dist 0 0))
                 (:y (cl-transforms:make-3d-vector 0 dist 0))
                 (:z (cl-transforms:make-3d-vector 0 0 dist))
                 (t (error "in TRANSLATE-ONCE-POSE forgot to specify axis properly: ~a" axis)))
               pose-orientation))))

(defun try-looking-pose (obj-pose  &key axis (dist 0.8))
  (case axis
    (:x (list (ground-pose (rotate-once-pose (translate-once-pose obj-pose dist axis) 3.14159 :z))
              (ground-pose (rotate-once-pose (translate-once-pose obj-pose (* -1 dist) axis) 0.0 :z))))
    (:y (list (ground-pose (rotate-once-pose (translate-once-pose obj-pose dist axis) -1.5708 :z))
              (ground-pose (rotate-once-pose (translate-once-pose obj-pose (* -1 dist) axis) 1.5708 :z))))))

;;;;;;;;;;;;;;
;; BTR-Utility

(defun move-hsr-debug (?pose)
  (urdf-proj:with-simulated-robot
    (exe:perform
     (desig:an action
               (type going)
               (target (desig:a location (pose ?pose)))))))

(defun hsr-carry-pose ()
  (urdf-proj::move-joints '(-2.6d0
                           0.0d0
                           1.0d0
                            0.0d0)))

(defun spawn-pringles()
  (btr:add-object btr:*current-bullet-world* :cylinder-item 'cylinder-1
                    '((1.5 -1.5 0.78) (0 0 0 1))
                    :mass 0.2 :size (cl-transforms:make-3d-vector 0.03 0.03 0.08)
                    :item-type :pringles))

(defun door-closed()
  (btr:set-robot-state-from-joints
   `((,"iai_kitchen:living_room:door_origin_revolute_joint" -1.4))
   (btr:get-environment-object)))

(defun spawn-pringles-shelf()
  (btr:add-object btr:*current-bullet-world* :cylinder-item 'cylinder-1
                    '((0 1.75 0.655) (0 0 0 1))
                    :mass 0.2 :size (cl-transforms:make-3d-vector 0.03 0.03 0.08)
                    :item-type :pringles))

(defun move-obj (obj-name pose)
  (btr-utils:move-object
   obj-name
   pose))

(defun check-arm-for-object ()
  (let ((link
        (cut:var-value
          '?link
          (car
           (prolog:prolog
            `(and (rob-int:robot ?rob)
                  (rob-int:end-effector-link ?rob :left ?link)))))))
    (car
     (btr:link-attached-object-names
     (btr:get-robot-object)
     link))))

;;;;;;;;;;;;
;; Knowledge

(defgeneric knowledge-get-nav-pose(obj-or-pose))

(defmethod knowledge-get-nav-pose((obj-or-pose symbol))
  (let ((obj-pose (cl-tf:pose->pose-stamped
                   cram-tf:*fixed-frame*
                   0 
                   (btr:object-pose obj-or-pose))))
    (concatenate 'list
                 (try-looking-pose obj-pose :axis :x :dist 1.0)
                 (try-looking-pose obj-pose :axis :y))))

(defmethod knowledge-get-nav-pose((obj-or-pose cl-transforms-stamped:pose-stamped))
  (concatenate 'list
               (try-looking-pose obj-or-pose :axis :x)
               (try-looking-pose obj-or-pose :axis :y)))

(defun knowledge-get-target-pose1()
  (cl-transforms-stamped:make-pose-stamped
   "map" 0
   (cl-transforms:make-3d-vector 1.5 -1.5 0.78)
   (cl-transforms:make-quaternion 0 0 0 1)))

(defun knowledge-get-table-pose1()
  (cl-transforms-stamped:make-pose-stamped
   "map" 0
   (cl-transforms:make-3d-vector 0.7 -1.5 0.0)
   (cl-transforms:make-quaternion 0 0 0 1)))

(defun knowledge-get-target-pose2()
  (cl-tf:make-pose-stamped
    "map" 0.0
    (cl-tf:make-3d-vector -2.2 -1.5 0.78)
    (cl-tf:make-quaternion 0 0 0 1)))

(defun knowledge-get-nav-pose2()
  (cl-tf:make-pose-stamped
    "map" 0.0
    (cl-tf:make-3d-vector -1.4 -1.5 0.0)
    (cl-tf:make-quaternion 0 0 1 0)))

(defun knowledge-get-left-handle-pose()
  (cl-tf:pose->pose-stamped
   cram-tf:*fixed-frame*
   0 
   (btr:link-pose (btr:get-environment-object) "shelf:shelf:shelf_door_left:shelf_link_handle")))

(defun knowledge-get-right-handle-pose()
  (cl-tf:pose->pose-stamped
   cram-tf:*fixed-frame*
   0 
   (btr:link-pose (btr:get-environment-object) "shelf:shelf:shelf_door_right:shelf_link_handle")))

(defun knowledge-get-entrance-handle-pose()
  (cl-tf:pose->pose-stamped
   cram-tf:*fixed-frame*
   0 
   (btr:link-pose (btr:get-environment-object) "iai_kitchen:living_room:door_handle_inside")))

(defun knowledge-get-shelf-pose()
  (cl-tf:make-pose-stamped
    "map" 0.0
    (cl-tf:make-3d-vector 0 0.8 0.0)
    (cl-tf:make-quaternion 0 0 1 1)))

(defun knowledge-get-entrance-pose()
  (cl-tf:make-pose-stamped
    "map" 0.0
    (cl-tf:make-3d-vector 1 0 0.0)
    (cl-tf:make-quaternion 0 0 0.47942555198327147d0 0.8775825545813541d0)))

(defun knowledge-get-shelf-left()
  "shelf:shelf:shelf_door_left:shelf_joint")

(defun knowledge-get-shelf-right()
  "shelf:shelf:shelf_door_right:shelf_joint")

(defun knowledge-get-entrance()
  "iai_kitchen:living_room:door_origin_revolute_joint")

(defun knowledge-get-looking-pose(obj-name)
  (cl-tf:pose->pose-stamped
                   cram-tf:*fixed-frame*
                   0 
                   (btr:object-pose obj-name)))



  
