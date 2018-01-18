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

(defparameter *drawer-handle-grasp-x-offset* -0.02 "in meters")
(defparameter *drawer-handle-pregrasp-x-offset* -0.10 "in meters")
(defparameter *drawer-handle-lift-x-offset* -0.4 "in meters")

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
    ;(cram-tf:translate-transform-stamped
     (cram-tf:multiply-transform-stampeds object-name
                                          tool-frame
                                          (cram-tf:multiply-transform-stampeds object-name handle-name
                                                                               (cram-tf:transform-stamped-inv container-tf)
                                                                               handle-tf)
                                          (cl-transforms-stamped:make-transform-stamped
                                           handle-name
                                           tool-frame
                                           0.0
                                           (cl-transforms:make-3d-vector *drawer-handle-grasp-x-offset* 0.0d0 0.0d0)
                                           (cl-transforms:matrix->quaternion
                                            #2A((0 0 -1)
                                                (0 1 0)
                                                (1 0 0)))))
    ; :x-offset *drawer-handle-grasp-x-offset*
    ))

; Should be fine without a joint-type.
(defmethod obj-int:get-object-type-pregrasp-pose ((object-type (eql :container))
                                          arm
                                          (grasp (eql :front))
                                          grasp-pose)
  (cram-tf:translate-pose grasp-pose :x-offset *drawer-handle-pregrasp-x-offset*))

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
    (cram-tf:translate-pose grasp-pose :x-offset *drawer-handle-lift-x-offset*)))


;;; PLANS

(defun open-container (;;container-name
                       ?arm ?gripper-opening
                       ?left-reach-poses ?right-reach-poses
                       ?left-lift-poses ?right-lift-poses)
  (cpl:par
    (roslisp:ros-info (environment-manipulation open-container) "Opening gripper")
    (exe:perform
     (desig:an action
               (type setting-gripper)
               (gripper ?arm)
               (position ?gripper-opening)))
    (roslisp:ros-info (environment-manipulation open-container) "Reaching")
    (exe:perform
     (desig:an action
               (type reaching)
               (left-poses ?left-reach-poses)
               (right-poses ?right-reach-poses))))
  (roslisp:ros-info (environment-manipulation open-container) "Gripping")
  (exe:perform
   (desig:an action
             (type setting-gripper)
             (gripper ?arm)
             (position 0)))
  (roslisp:ros-info (environment-manipulation open-container) "Opening")
  (exe:perform
   (desig:an action
             (type lifting)
             (left-poses ?left-lift-poses)
             (right-poses ?right-lift-poses))))

(defun drive-to-and-open-container (?container-desig)
  (let* ((handle-link (get-handle-link (car (alexandria:assoc-value (desig:description ?container-desig) :name))))
         (?handle-pose (get-urdf-link-pose (cl-urdf:name handle-link)))
         (?manipulated-handle-pose (get-manipulated-pose (cl-urdf:name handle-link) 1 :relative T)))
    ;; Drive to it
    (exe:perform (a motion
                    (type going)
                    (target
                     (a location
                        (reachable-for pr2)
                        (poses (?handle-pose
                                ?manipulated-handle-pose)))))))
  ;; Open it
  (exe:perform (an action
                   (type opening)
                   (object ?container-desig)))
  )


;;; PROLOG DESIG GROUNDING

(def-fact-group environment-manipulation (desig:action-grounding
                                          location-costmap:desig-costmap)
  
  (<- (desig:action-grounding ?action-designator (open-container ;; ?container-name
                                                                 ?arm
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
    )

  (<- (desig:action-grounding ?action-designator (drive-to-and-open-container ?container-designator))
    (spec:property ?action-designator (:type :driving-and-opening))
    (spec:property ?action-designator (:object ?container-designator))
    (spec:property ?container-designator (:type :container))
    ;;(spec:property ?container-designator (:name ?container-name))
    ;;(spec:property ?container-designator (:part-of ?environment))
    )
  )


;;; DRAWER COSTMAP

(defun make-poses-cost-function (poses)
  "`poses' are the poses according to which the relation is resolved."
  (let ((meancovs (location-costmap:2d-pose-covariance poses 0.05)))
    (location-costmap:make-gauss-cost-function (first meancovs)
                                               (second meancovs))))

(defmethod location-costmap:costmap-generator-name->score ((name (eql 'poses-cost-function))) 10)

(def-fact-group environment-manipulation-costmap (location-costmap:desig-costmap)
  (<- (location-costmap:desig-costmap ?designator ?costmap)
    ;;(or (cram-robot-interfaces:visibility-designator ?desig)
    ;;    (cram-robot-interfaces:reachability-designator ?desig))
    (desig:desig-prop ?designator (:poses ?poses))
    (location-costmap:costmap ?costmap)
    (location-costmap:costmap-add-function
     poses-cost-function
     (make-poses-cost-function ?poses)
     ?costmap)
    )
  )


;;; SIMULATION MANIPULATION

(defun move-joint (name angle)
  (format T "Move joint ~a to angle ~a.~%" name angle)
  (btr:set-robot-state-from-joints
   `((,name ,angle))
   (btr:object btr:*current-bullet-world* :kitchen)))

(defun open-container-joint (container-type)
  (format T "Open container ~a.~%" container-type)
  (move-joint (cdr (assoc container-type *container-joints*))
              (cdr (assoc container-type *container-angles*))))

(defun close-container-joint (container-type)
  (format T "Close container ~a.~%" container-type)
  (move-joint (cdr (assoc container-type *container-joints*))
              0))

(defun open-all ()
  (dolist (c *container*) (open-container-joint c)))

(defun close-all ()
  (dolist (c *container*) (close-container-joint c)))


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


;;; GET/FIND FUNCTIONS

(defun get-urdf-link-pose (name)
  (btr:pose (btr:rigid-body (btr:object btr:*current-bullet-world* :kitchen) (btr::make-rigid-body-name "KITCHEN" name :demo))))

(defun get-container-link (container-name)
  (gethash container-name (cl-urdf:links (btr:urdf (btr:object btr:*current-bullet-world* :kitchen)))))

(defun get-container-joint-type (container-name)
  (find-container-joint-type-under-joint (cl-urdf:from-joint (get-container-link container-name))))

(defun find-container-joint-type-under-joint (joint)
  "Return the first joint type different from :FIXED under the given JOINT."
  (if (eq :FIXED (cl-urdf:joint-type joint))
      (find-container-joint-type-under-joint (car (cl-urdf:to-joints
                                                   (cl-urdf:child joint))))
      (cl-urdf:joint-type joint)))

(defun get-handle-link (container-name)
  (when (symbolp container-name)
    (setf container-name (string-downcase container-name)))
  (find-handle-under-link
   (get-container-link container-name)))

(defun find-handle-under-link (link)
  (if (search "handle" (cl-urdf:name link))
      link
      (find-handle-under-link (cl-urdf:child
                     (car (cl-urdf:to-joints link))))))

(defun get-joint-position (joint)
  (gethash (cl-urdf:name joint)
           (btr:joint-states (btr:object btr:*current-bullet-world* :kitchen))))

(defun get-connecting-joint (part)
  "Returns the connecting (moveable) joint of `part', which can be either
  a link or a joint of the kitchen URDF."
  (when part
    (if (typep part 'cl-urdf:joint)
        (or
         (when (not (eql (cl-urdf:joint-type part) :FIXED))
           part)
         (get-connecting-joint (cl-urdf:parent part)))
        (when (typep part 'cl-urdf:link)
          (get-connecting-joint (cl-urdf:from-joint part))))))

(defun get-manipulated-pose (link-name joint-position &key relative)
  "Returns the pose of a link based on its connection joint position
  `joint-position'. If `relative' is T, the actual value is calculated
  by `joint-position' * <joint maximal value>. this method returns two
  values, the new pose of the object and the joint that was changed."
  (let ((link (get-container-link link-name)))
    (when (typep link 'cl-urdf:link)
      (let ((joint (get-connecting-joint link)))
        (when joint
          (values
           (case (cl-urdf:joint-type joint)
             (:PRISMATIC
              (cl-tf:transform->pose
               (cl-tf:transform*
                (cl-tf:pose->transform (get-urdf-link-pose link-name))
                (cl-tf:make-transform
                 (cl-tf:v*
                  (cl-urdf:axis joint)
                  (-
                   (if relative
                       (* joint-position
                          (cl-urdf:upper (cl-urdf:limits joint)))
                       joint-position)
                   (get-joint-position joint)))
                 (cl-tf:make-identity-rotation)))))
             (:REVOLUTE (error 'simple-error :format-control "Manipulation of revolute joints not implemented.")))
           joint))))))


;;; TEST FUNCTIONS

(defun get-opening-desig (&optional (?name 'sink_area_left_upper_drawer_main))
  (let ((?object (get-container-desig ?name)))
    (an action
        (type opening)
        (object ?object))))

(defun get-container-desig (&optional (?name 'sink_area_left_upper_drawer_main))
  (let* ((name-str (roslisp-utilities:rosify-underscores-lisp-name ?name))
         (urdf-pose (get-urdf-link-pose name-str)))
    (let* ((?pose (cl-tf:transform-pose-stamped cram-tf:*transformer*
                                                :target-frame "base_footprint"
                                                :pose (cl-tf:pose->pose-stamped "map" 0 urdf-pose)))
           (?transform (cl-tf:make-transform-stamped "base_footprint" name-str
                                                     (cl-tf:stamp ?pose)
                                                     (cl-tf:origin ?pose)
                                                     (cl-tf:orientation ?pose))))
      (an object
          (type container)
          (name ?name)
          (part-of kitchen)
          (pose ((pose ?pose) (transform ?transform)))
          ))))

(defun test-open ()
  (with-simulated-robot
    (let ((?desig (get-opening-desig 'sink_area_left_upper_drawer_main)))
      (exe:perform ?desig))))

(defun test ()
  (with-simulated-robot
    (let ((?object (get-container-desig 'sink_area_left_upper_drawer_main)))
          (exe:perform (an action (type driving-and-opening) (object ?object))))))

(defun move-pr2 (x y)
  (with-simulated-robot
        (let ((?goal (cl-transforms-stamped:make-pose-stamped
                    "map"
                    0.0
                    (cl-tf:make-3d-vector x y 0.0)
                    (cl-tf:make-identity-rotation))))
        (exe:perform (a motion (type going) (target (a location (pose ?goal))))))))

;; (gethash "iai_fridge_main" (cl-urdf:links  (btr:urdf (btr:object btr:*current-bullet-world* :kitchen))))


;; handles aus drawer-link erschließen aus urdf (beim berechnen der greifpose)
;; (an object (type container) (part-of kitchen) (name "iai_fridge_main") (pose ...))
;; btr:make-rigid-body-name
