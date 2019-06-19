;;;
;;; Copyright (c) 2018, Christopher Pollok <cpollok@uni-bremen.de>
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

(in-package :env-man)

(defun get-container-pose-and-transform (name btr-environment)
  "Return a list of the pose-stamped and transform-stamped of the object named
NAME in the environment BTR-ENVIRONMENT. In robot base frame."
  (let* ((name-rosified (roslisp-utilities:rosify-underscores-lisp-name name))
         (urdf-pose (get-urdf-link-pose name-rosified btr-environment))
         (pose (cram-tf:ensure-pose-in-frame
                (cl-transforms-stamped:pose->pose-stamped
                 cram-tf:*fixed-frame*
                 0.0
                 urdf-pose)
                cram-tf:*robot-base-frame*
                :use-zero-time t))
         (transform (cram-tf:pose-stamped->transform-stamped pose name-rosified)))
    (list pose transform)))

(def-fact-group environment-manipulation (desig:action-grounding)

  (<- (desig:action-grounding ?action-designator (open-container ?referenced-action-designator))
    (spec:property ?action-designator (:type :opening))
    (spec:property ?action-designator (:object ?container-designator))
    (spec:property ?container-designator (:type ?container-type))
    (man-int:object-type-subtype :container ?container-type)
    (spec:property ?container-designator (:urdf-name ?container-name))
    (spec:property ?container-designator (:part-of ?btr-environment))
    (-> (spec:property ?container-designator (:handle-axis ?handle-axis))
        (true)
        (lisp-fun get-handle-axis ?container-designator ?handle-axis))
    (-> (spec:property ?action-designator (:arm ?arm))
        (true)
        (man-int:robot-free-hand ?_ ?arm))
    (-> (spec:property ?action-designator (:distance ?distance))
        (true)
        (lisp-fun man-int:get-container-opening-distance
                  ?container-type ?distance))
    (lisp-fun get-relative-distance ?container-name ?btr-environment ?distance :opening
              ?rel-distance)
    (lisp-fun clip-distance ?container-name ?btr-environment ?rel-distance :opening
              ?clipped-distance)
    ;; infer joint information
    ;; joint-name
    (lisp-fun get-container-link ?container-name ?btr-environment ?container-link)
    (lisp-fun get-handle-link ?container-name ?btr-environment ?handle-link-object)
    (lisp-fun cl-urdf:name ?handle-link-object ?handle-link-string)
    (lisp-fun roslisp-utilities:lispify-ros-underscore-name
              ?handle-link-string :keyword ?handle-link)
    (lisp-fun get-connecting-joint ?container-link ?connecting-joint)
    (lisp-fun cl-urdf:name ?connecting-joint ?joint-name)
    ;; environment
    (btr:bullet-world ?world)
    (lisp-fun btr:object ?world ?btr-environment ?environment-obj)
    (lisp-fun btr:name ?environment-obj ?environment-name)
    ;; infer missing information like ?gripper-opening, opening trajectory
    (lisp-fun man-int:get-action-gripper-opening ?container-type ?gripper-opening)
    ;; TODO: this is here so far only for logging, in the future we should
    ;; implement grasps such as front grasp, top grasp etc.
    ;; and incorporate this into GET-ACTION-TRAJECTORY
    (-> (spec:property ?action-designator (:grasp ?grasp))
        (true)
        (or (and
             (lisp-fun get-container-pose-and-transform ?container-name ?btr-environment
                       (?_ ?object-transform))
             (lisp-fun man-int:get-action-grasps ?container-type ?arm ?object-transform
                       ?grasps)
             (member ?grasp ?grasps))
            (true)))
    ;; calculate trajectory
    (equal ?objects (?container-designator))
    (-> (equal ?arm :left)
        (and (lisp-fun man-int:get-action-trajectory
                       :opening ?arm :open ?objects
                       :opening-distance ?clipped-distance
                       :handle-axis ?handle-axis
                       ?left-trajectory)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :reaching
                       ?left-reach-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :grasping
                       ?left-grasp-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :opening
                       ?left-open-pose)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :retracting
                       ?left-retract-pose))
        (and (equal ?left-reach-poses NIL)
             (equal ?left-grasp-poses NIL)
             (equal ?left-open-pose NIL)
             (equal ?left-retract-pose NIL)))
    (-> (equal ?arm :right)
        (and (lisp-fun man-int:get-action-trajectory
                       :opening ?arm :open ?objects
                       :opening-distance ?clipped-distance
                       :handle-axis ?handle-axis
                       ?right-trajectory)
             (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :reaching
                       ?right-reach-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :grasping
                       ?right-grasp-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :opening
                       ?right-open-pose)
             (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :retracting
                       ?right-retract-pose))
        (and (equal ?right-reach-poses NIL)
             (equal ?right-grasp-poses NIL)
             (equal ?right-open-pose NIL)
             (equal ?right-retract-pose NIL)))
    (or (lisp-pred identity ?left-trajectory)
        (lisp-pred identity ?right-trajectory))
    ;; make new action designator
    (desig:designator :action ((:type :opening)
                               (:arm ?arm)
                               (:gripper-opening ?gripper-opening)
                               (:distance ?clipped-distance)
                               (:left-reach-poses ?left-reach-poses)
                               (:right-reach-poses ?right-reach-poses)
                               (:left-grasp-poses ?left-grasp-poses)
                               (:right-grasp-poses ?right-grasp-poses)
                               (:left-open-poses ?left-open-pose)
                               (:right-open-poses ?right-open-pose)
                               (:left-retract-poses ?left-retract-pose)
                               (:right-retract-poses ?right-retract-pose)
                               (:joint-name ?joint-name)
                               (:link-name ?handle-link)
                               (:environment ?environment-obj)
                               (:environment-name ?environment-name))
                      ?referenced-action-designator))


  (<- (desig:action-grounding ?action-designator (close-container ?referenced-action-designator))
    (spec:property ?action-designator (:type :closing))
    (spec:property ?action-designator (:object ?container-designator))
    (spec:property ?container-designator (:type ?container-type))
    (man-int:object-type-subtype :container ?container-type)
    (spec:property ?container-designator (:urdf-name ?container-name))
    (spec:property ?container-designator (:part-of ?btr-environment))
    (-> (spec:property ?container-designator (:handle-axis ?handle-axis))
        (true)
        (lisp-fun get-handle-axis ?container-designator ?handle-axis))
    (-> (spec:property ?action-designator (:arm ?arm))
        (true)
        (man-int:robot-free-hand ?_ ?arm))
    (-> (spec:property ?action-designator (:distance ?distance))
        (true)
        (lisp-fun man-int:get-container-opening-distance
                  ?container-type ?distance))
    (lisp-fun get-relative-distance ?container-name ?btr-environment ?distance :closing
              ?rel-distance)
    (lisp-fun clip-distance ?container-name ?btr-environment ?rel-distance :closing
              ?clipped-distance)
    ;; infer joint information
    ;; joint-name
    (lisp-fun get-container-link ?container-name ?btr-environment ?container-link)
    (lisp-fun get-handle-link ?container-name ?btr-environment ?handle-link-object)
    (lisp-fun cl-urdf:name ?handle-link-object ?handle-link-string)
    (lisp-fun roslisp-utilities:lispify-ros-underscore-name
              ?handle-link-string :keyword ?handle-link)
    (lisp-fun get-connecting-joint ?container-link ?connecting-joint)
    (lisp-fun cl-urdf:name ?connecting-joint ?joint-name)
    ;; environment
    (btr:bullet-world ?world)
    (lisp-fun btr:object ?world ?btr-environment ?environment-obj)
    (lisp-fun btr:name ?environment-obj ?environment-name)
    ;; infer missing information like ?gripper-opening, closing trajectory
    (lisp-fun man-int:get-action-gripper-opening ?container-type ?gripper-opening)
    ;; TODO: this is here so far only for logging, in the future we should
    ;; implement grasps such as front grasp, top grasp etc.
    ;; and incorporate this into GET-ACTION-TRAJECTORY
    (-> (spec:property ?action-designator (:grasp ?grasp))
        (true)
        (or (and
             (lisp-fun get-container-pose-and-transform ?container-name ?btr-environment
                       (?_ ?object-transform))
             (lisp-fun man-int:get-action-grasps ?container-type ?arm ?object-transform
                       ?grasps)
             (member ?grasp ?grasps))
            (true)))
    ;; calculate trajectory
    (equal ?objects (?container-designator))
    (-> (equal ?arm :left)
        (and (lisp-fun man-int:get-action-trajectory
                       :closing ?arm :close ?objects
                       :opening-distance ?clipped-distance
                       :handle-axis ?handle-axis
                       ?left-trajectory)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :reaching
                       ?left-reach-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :grasping
                       ?left-grasp-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :closing
                       ?left-close-pose)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory :retracting
                       ?left-retract-pose))
        (and (equal ?left-reach-poses NIL)
             (equal ?left-grasp-poses NIL)
             (equal ?left-close-pose NIL)
             (equal ?left-retract-pose NIL)))
    (-> (equal ?arm :right)
        (and (lisp-fun man-int:get-action-trajectory
                       :closing ?arm :close ?objects
                       :opening-distance ?clipped-distance
                       :handle-axis ?handle-axis
                       ?right-trajectory)
             (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :reaching
                       ?right-reach-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :grasping
                       ?right-grasp-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :closing
                       ?right-close-pose)
             (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory :retracting
                       ?right-retract-pose))
        (and (equal ?right-reach-poses NIL)
             (equal ?right-grasp-poses NIL)
             (equal ?right-close-pose NIL)
             (equal ?right-retract-pose NIL)))
    (or (lisp-pred identity ?left-trajectory)
        (lisp-pred identity ?right-trajectory))
    ;; make new action designator
    (desig:designator :action ((:type :closing)
                               (:arm ?arm)
                               (:gripper-opening ?gripper-opening)
                               (:distance ?clipped-distance)
                               (:left-reach-poses ?left-reach-poses)
                               (:right-reach-poses ?right-reach-poses)
                               (:left-grasp-poses ?left-grasp-poses)
                               (:right-grasp-poses ?right-grasp-poses)
                               (:left-close-poses ?left-close-pose)
                               (:right-close-poses ?right-close-pose)
                               (:left-retract-poses ?left-retract-pose)
                               (:right-retract-poses ?right-retract-pose)
                               (:joint-name ?joint-name)
                               (:link-name ?handle-link)
                               (:environment ?environment-obj)
                               (:environment-name ?environment-name))
                      ?referenced-action-designator)))
