;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

;;; LISTS OF CONTAINERS (NAMES, LINKS, JOINTS, etc)

(defparameter *container*
  '(:fridge
    :fridge-drawer
    :island-left-drawer
    :island-middle-drawer
    :island-right-drawer
    :trash-drawer
    :dishwasher
    :kitchen-drawer
    :oven
    :oven-left-drawer
    :oven-right-drawer
    :oven-drawer
    ))

(defparameter *container-links*
  '((:fridge . "iai_fridge_main")
    (:fridge-drawer . "fridge_area_lower_drawer_main")
    (:island-left-drawer . "kitchen_island_left_upper_drawer_main")
    (:island-middle-drawer . "kitchen_island_middle_upper_drawer_main")
    (:island-right-drawer . "kitchen_island_right_upper_drawer_main")
    (:trash-drawer . "sink_area_trash_drawer_main")
    (:dishwasher . "sink_area_dish_washer_door_main")
    (:kitchen-drawer . "sink_area_left_upper_drawer_main")
    (:oven . "oven_area_oven_door_main")
    (:oven-left-drawer . "oven_area_area_left_drawer_main")
    (:oven-right-drawer . "oven_area_area_right_drawer_main")
    (:oven-drawer . "oven_area_area_middle_upper_drawer_main")
    ))

(defparameter *container-handle-links*
  '((:fridge . "iai_fridge_door_handle")
    (:fridge-drawer . "fridge_area_lower_drawer_drawer_handle")
    (:island-left-drawer . "kitchen_island_left_upper_drawer_handle")
    (:island-middle-drawer . "kitchen_island_middle_upper_drawer_handle")
    (:island-right-drawer . "kitchen_island_right_upper_drawer_handle")
    (:trash-drawer . "sink_area_trash_drawer_handle")
    (:dishwasher . "sink_area_dish_washer_door_handle")
    (:kitchen-drawer . "sink_area_left_upper_drawer_handle")
    (:oven . "oven_area_oven_door_handle")
    (:oven-left-drawer . "oven_area_area_left_drawer_handle")
    (:oven-right-drawer . "oven_area_area_right_drawer_handle")
    (:oven-drawer . "oven_area_area_middle_upper_drawer_handle")
    ))

(defparameter *container-joints*
  '((:fridge . "iai_fridge_door_joint")
    (:fridge-drawer . "fridge_area_lower_drawer_main_joint")
    (:island-left-drawer . "kitchen_island_left_upper_drawer_main_joint")
    (:island-middle-drawer . "kitchen_island_middle_upper_drawer_main_joint")
    (:island-right-drawer . "kitchen_island_right_upper_drawer_main_joint")
    (:trash-drawer . "sink_area_trash_drawer_main_joint")
    (:dishwasher . "sink_area_dish_washer_door_joint")
    (:kitchen-drawer . "sink_area_left_upper_drawer_main_joint")
    (:oven . "oven_area_oven_door_joint")
    (:oven-left-drawer . "oven_area_area_left_drawer_main_joint")
    (:oven-right-drawer . "oven_area_area_right_drawer_main_joint")
    (:oven-drawer . "oven_area_area_middle_upper_drawer_main_joint")
    ))

(defparameter *container-angles*
  '((:fridge . 1.6)
    (:fridge-drawer . 0.4)
    (:island-left-drawer . 0.4)
    (:island-middle-drawer . 0.4)
    (:island-right-drawer . 0.4)
    (:trash-drawer . 0.4)
    (:dishwasher . 1.6)
    (:kitchen-drawer . 0.4)
    (:oven . 1.6)
    (:oven-left-drawer . 0.4)
    (:oven-right-drawer . 0.4)
    (:oven-drawer . 0.4)
    ))


;;; OBJECT-INTERFACE METHODS

(defparameter *drawer-handle-grasp-y-offset* -0.05 "in meters")
(defparameter *drawer-handle-pregrasp-y-offset* 0.10 "in meters")
(defparameter *drawer-handle-lift-y-offset* -0.4 "in meters")

; Might be necessary to find out what kind of handle we are dealing with. But we could also just open wide and be done with it.
(defmethod obj-int:get-object-type-gripper-opening ((object-type (eql :container)))
  0.09)


;; Commented, because to use the interface the transform has to be from the object to the gripper.
;; The version below does this.
;; (defmethod obj-int:get-object-type-to-gripper-transform ((object-type (eql :container))
;;                                                  object-name
;;                                                  arm
;;                                                  (grasp (eql :front)))
;;   (let ((handle-name (cl-urdf:name (get-handle-link (roslisp-utilities:rosify-underscores-lisp-name object-name)))))
;;     (cl-transforms-stamped:make-transform-stamped
;;      ;;(roslisp-utilities:rosify-underscores-lisp-name handle-name)
;;      handle-name
;;      (ecase arm
;;        (:left cram-tf:*robot-left-tool-frame*)
;;        (:right cram-tf:*robot-right-tool-frame*))
;;      0.0
;;      (cl-transforms:make-3d-vector 0.0d0 *drawer-handle-grasp-y-offset* 0.0d0)
;;      (cl-transforms:matrix->quaternion
;;       #2A((0 1 0)
;;           (1 0 0)
;;           (0 0 -1))))))

; Find out where the handle is and calculate the transform from there.
(defmethod obj-int:get-object-type-to-gripper-transform ((object-type (eql :container))
                                                         object-name
                                                         arm
                                                         (grasp (eql :front)))
  (setf object-name (roslisp-utilities:rosify-underscores-lisp-name object-name))
  (let* ((handle-name (cl-urdf:name (get-handle-link object-name)))
         (handle-tf (cl-tf:transform->transform-stamped "map" handle-name 0
                                                        (cl-tf:pose->transform
                                                         (get-urdf-link-pose handle-name))))
         (container-tf (cl-tf:transform->transform-stamped "map" object-name 0
                                                           (cl-tf:pose->transform
                                                            (get-urdf-link-pose object-name))))
         (tool-frame (ecase arm
                       (:left cram-tf:*robot-left-tool-frame*)
                       (:right cram-tf:*robot-right-tool-frame*))))
    (cram-tf:multiply-transform-stampeds object-name
                                         tool-frame
                                         (cram-tf:multiply-transform-stampeds object-name handle-name
                                                                              (cram-tf:transform-stamped-inv container-tf)
                                                                              handle-tf)
                                         (cl-transforms-stamped:make-transform-stamped
                                          handle-name
                                          tool-frame
                                          0.0
                                          (cl-transforms:make-3d-vector 0.0d0 *drawer-handle-grasp-y-offset* 0.0d0)
                                          (cl-transforms:matrix->quaternion
                                           #2A((0 1 0)
                                               (1 0 0)
                                               (0 0 -1)))))))

; Should be fine without a joint-type.
(defmethod obj-int:get-object-type-pregrasp-pose ((object-type (eql :container))
                                          arm
                                          (grasp (eql :front))
                                          grasp-pose)
  (cram-tf:translate-pose grasp-pose :y-offset *drawer-handle-pregrasp-y-offset*))

; We need the joint-type, maybe add a optional parameter to the generic.
; But for now I can live with this.
(defmethod obj-int:get-object-type-lift-pose ((object-type (eql :container))
                                      arm
                                      (grasp (eql :front))
                                      grasp-pose)
  (let ((grasp-pose (cram-tf:ensure-pose-in-frame
                     grasp-pose
                     cram-tf:*robot-base-frame*
                     :use-zero-time t)))
    (cram-tf:translate-pose grasp-pose :y-offset *drawer-handle-lift-y-offset*)))


;;; PROLOG DESIG GROUNDING

(def-fact-group environment-manipulation (desig:action-grounding)
  (<- (desig:action-grounding ?action-designator (open-container ?container-name ?arm
                                                                 ?gripper-opening
                                                                 ?left-reach-poses ?right-reach-poses
                                                                 ?left-lift-poses ?right-lift-poses))
    (spec:property ?action-designator (:type :opening))
    (spec:property ?action-designator (:object ?container-designator))
    (spec:property ?container-designator (:type :container))
    (spec:property ?container-designator (:name ?container-name))
    (spec:property ?container-designator (:part-of ?environment))
    (-> (spec:property ?action-designator (:arm ?arm))
        (true)
        (and (cram-robot-interfaces:robot ?robot)
             (cram-robot-interfaces:arm ?robot ?arm)))
    ;; infer missing information like ?gripper-opening, opening trajectory
    (lisp-fun obj-int:get-object-type-gripper-opening ?container-type ?gripper-opening)
    (lisp-fun obj-int:get-object-transform ?container-designator ?container-transform)
    (lisp-fun obj-int:get-object-grasping-poses ?container-name :container :left :front ?container-transform ?left-poses)
    (lisp-fun obj-int:get-object-grasping-poses ?container-name :container :right :front ?container-transform ?right-poses)
    (lisp-fun cram-mobile-pick-place-plans::extract-pick-up-manipulation-poses ?arm ?left-poses ?right-poses
              (?left-reach-poses ?right-reach-poses ?left-lift-poses ?right-lift-poses))
    ))


;;; SIMULATION

(defun move-joint (name angle)
  (format T "Move joint ~a to angle ~a.~%" name angle)
  (btr:set-robot-state-from-joints
   `((,name ,angle))
   (btr:object btr:*current-bullet-world* :kitchen)))

(defun open-container (container-type)
  (format T "Open container ~a.~%" container-type)
  (move-joint (cdr (assoc container-type *container-joints*))
              (cdr (assoc container-type *container-angles*))))

(defun close-container (container-type)
  (format T "Close container ~a.~%" container-type)
  (move-joint (cdr (assoc container-type *container-joints*))
              0))

(defun open-all ()
  (dolist (c *container*) (open-container c)))

(defun close-all ()
  (dolist (c *container*) (close-container c)))


;; (defun get-container-pose-in-base (container-name)
;;   (cl-tf:transform-pose cram-tf:*transformer*
;;                         :pose (cl-tf:pose->pose-stamped "map" 0
;;                                                         (cl-tf:transform->pose
;;                                                          (gethash (cdr (assoc container-name *container-links*))
;;                                                                   (btr::link-offsets
;;                                                                    (btr:object btr:*current-bullet-world* :kitchen)))))
;;                         :target-frame "base_footprint"))

;; (defun get-container-transform-in-base (container-name)
;;   (cram-tf:multiply-transform-stampeds "base_footprint" (cdr (assoc container-name *container-links*))
;;                                        (cl-tf:lookup-transform cram-tf:*transformer* "base_footprint" "map")
;;                                        (cl-tf:transform->transform-stamped "map" (cdr (assoc container-name *container-links*)) 0
;;                                                                            (gethash (cdr (assoc container-name *container-links*))
;;                                                                                     (btr::link-offsets
;;                                                                                      (btr:object btr:*current-bullet-world* :kitchen))))))


(defun get-opening-desig (&optional (?name 'iai_fridge_main))
  (let* ((name-str (roslisp-utilities:rosify-underscores-lisp-name ?name))
         (urdf-pose (get-urdf-link-pose name-str)))
    (with-simulated-robot
      (let* ((?pose (cl-tf:transform-pose-stamped cram-tf:*transformer*
                                                  :target-frame "base_footprint"
                                                  :pose (cl-tf:pose->pose-stamped "map" 0 urdf-pose)))
             (?transform (cl-tf:make-transform-stamped "base_footprint" name-str
                                                       (cl-tf:stamp ?pose)
                                                       (cl-tf:origin ?pose)
                                                       (cl-tf:orientation ?pose))))
        (an action
            (type opening)
            (object (an object
                        (type container)
                        (name ?name)
                        (part-of kitchen)
                        (pose ((pose ?pose) (transform ?transform)))
                        )))))))


(defun get-urdf-link-pose (name)
  (btr:pose (btr:rigid-body (btr:object btr:*current-bullet-world* :kitchen) (btr::make-rigid-body-name "KITCHEN" name))))

(defun get-container-link (container-name)
  (gethash container-name (cl-urdf:links (btr:urdf (btr:object btr:*current-bullet-world* :kitchen)))))

(defun get-door-joint (container-name)
  (car (cl-urdf:to-joints (get-container-link container-name))))

(defun get-door-link (container-name)
  (cl-urdf:child (get-door-joint container-name)))

(defun get-handle-joint (container-name)
  (car (cl-urdf:to-joints (get-door-link container-name))))

(defun get-handle-link (container-name)
  (cl-urdf:child (get-handle-joint container-name)))
  
;; (gethash "iai_fridge_main" (cl-urdf:links  (btr:urdf (btr:object btr:*current-bullet-world* :kitchen))))


;; handles aus drawer-link erschließen aus urdf (beim berechnen der greifpose)
;; (an object (type container) (part-of kitchen) (name "iai_fridge_main") (pose ...))
;; btr:make-rigid-body-name
