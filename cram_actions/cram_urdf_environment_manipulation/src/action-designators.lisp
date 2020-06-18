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

(def-fact-group environment-manipulation (desig:action-grounding)
  (<- (desig:action-grounding ?action-designator (manipulate-container
                                                  ?referenced-action-designator))
    (or (spec:property ?action-designator (:type :opening))
        (spec:property ?action-designator (:type :closing)))
    (spec:property ?action-designator (:type ?action-type))
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
    (lisp-fun get-container-link ?container-name ?btr-environment ?container-link)
    (lisp-fun get-connecting-joint ?container-link ?connecting-joint)

    (-> (spec:property ?action-designator (:distance ?distance))
        (true)
        (-> (equal ?action-type :opening)
            (-> (lisp-pred man-int:get-container-opening-distance
                           ?container-name)
                (lisp-fun man-int:get-container-opening-distance
                          ?container-name ?distance)
                (and (lisp-fun cl-urdf:limits ?connecting-joint ?limits)
                     (lisp-fun cl-urdf:upper ?limits ?distance)))
            (-> (lisp-pred man-int:get-container-closing-distance
                           ?container-name)
                (lisp-fun man-int:get-container-closing-distance
                          ?container-name ?distance)
                (and (lisp-fun cl-urdf:limits ?connecting-joint ?limits)
                     (lisp-fun cl-urdf:lower ?limits ?distance)))))

    (lisp-fun get-relative-distance ?container-name ?btr-environment ?distance
              ?action-type
              ?rel-distance)
    (lisp-fun clip-distance ?container-name ?btr-environment ?rel-distance
              ?action-type
              ?clipped-distance)

    ;; infer joint information
    ;; joint-name
    (lisp-fun get-handle-link ?container-name ?btr-environment
              ?handle-link-object)
    (lisp-fun cl-urdf:name ?handle-link-object ?handle-link-string)
    (lisp-fun roslisp-utilities:lispify-ros-underscore-name
              ?handle-link-string :keyword ?handle-link)
    (lisp-fun cl-urdf:name ?connecting-joint ?joint-name)

    ;; environment
    (btr:bullet-world ?world)
    (lisp-fun btr:object ?world ?btr-environment ?environment-object)
    (lisp-fun btr:name ?environment-object ?environment-name)

    ;; infer missing information like ?gripper-opening, opening trajectory
    (lisp-fun man-int:get-action-gripper-opening ?container-type
              ?gripper-opening)

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
    (equal (?container-designator) ?objects)
    (-> (equal ?arm :left)
        (and (lisp-fun man-int:get-action-trajectory ?action-type
                       ?arm NIL NIL ?objects
                       :opening-distance ?clipped-distance
                       :handle-axis ?handle-axis
                       ?left-trajectory)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory
                       :reaching
                       ?left-reach-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory
                       :grasping
                       ?left-grasp-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory
                       ?action-type
                       ?left-manipulate-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?left-trajectory
                       :retracting
                       ?left-retract-poses))
        (and (equal ?left-reach-poses NIL)
             (equal ?left-grasp-poses NIL)
             (equal ?left-manipulate-poses NIL)
             (equal ?left-retract-poses NIL)))
    (-> (equal ?arm :right)
        (and (lisp-fun man-int:get-action-trajectory ?action-type
                       ?arm NIL NIL ?objects
                       :opening-distance ?clipped-distance
                       :handle-axis ?handle-axis
                       ?right-trajectory)
             (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory
                       :reaching
                       ?right-reach-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory
                       :grasping
                       ?right-grasp-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory
                       ?action-type
                       ?right-manipulate-poses)
             (lisp-fun man-int:get-traj-poses-by-label ?right-trajectory
                       :retracting
                       ?right-retract-poses))
        (and (equal ?right-reach-poses NIL)
             (equal ?right-grasp-poses NIL)
             (equal ?right-manipulate-poses NIL)
             (equal ?right-retract-poses NIL)))
    (or (lisp-pred identity ?left-trajectory)
        (lisp-pred identity ?right-trajectory))

    ;; make new action designator
    (desig:designator :action ((:type ?action-type)
                               (:arm ?arm)
                               (:gripper-opening ?gripper-opening)
                               (:distance ?clipped-distance)
                               (:left-reach-poses ?left-reach-poses)
                               (:right-reach-poses ?right-reach-poses)
                               (:left-grasp-poses ?left-grasp-poses)
                               (:right-grasp-poses ?right-grasp-poses)
                               (:left-manipulate-poses ?left-manipulate-poses)
                               (:right-manipulate-poses ?right-manipulate-poses)
                               (:left-retract-poses ?left-retract-poses)
                               (:right-retract-poses ?right-retract-poses)
                               (:joint-name ?joint-name)
                               (:link-name ?handle-link)
                               (:environment-name ?environment-name)
                               (:environment-object ?environment-object)
                               (:container-object ?container-designator))
                      ?referenced-action-designator)))
