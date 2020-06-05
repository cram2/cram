;;;
;;; Copyright (c) 2019, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :cram-manipulation-interfaces)

(defgeneric get-action-gripping-effort (object-type)
  (:method-combination cut:first-in-order-and-around)
  (:documentation "Returns effort in Nm, e.g. 50."))

(defgeneric get-action-gripper-opening (object-type)
  (:method-combination cut:first-in-order-and-around)
  (:documentation "How wide to open the gripper before grasping, in m or radiants."))

(defgeneric get-action-grasps (object-type arm object-transform-in-base)
  (:method-combination cut:first-in-order-and-around)
  (:documentation "Returns a (lazy) list of keywords that represent the possible
grasp orientations for `object-type' given `arm' and `object-transform-in-base'."))

(defgeneric get-object-type-carry-config (object-type grasp)
  (:method-combination cut:first-in-order-and-around)
  (:documentation "When carrying an object, which arm configuration to use.
The return value is a symbol (keyword), which associates with a robot-specific
joint state, specified in the robot description."))

(defgeneric get-action-trajectory (action-type arm grasp objects-acted-on
                                   &key &allow-other-keys)
  (:method-combination cut:first-in-order-and-around)
  (:documentation "Returns a list of TRAJ-SEGMENTs.
`engine' is the keyword describing the reasoning engine that calculates trajectories,
`action-type' describes for which type of action the trajectory will be,
`arm' a single keyword eg. :left,
`grasp' describes grasp orientation to use, e.g., :top, :left-side,
`objects-acted-on' are designators describing the objects used by the action."))

(defgeneric get-location-poses (location-designator)
  (:method-combination cut:first-in-order-and-around)
  (:documentation "Returns a (lazy) list of cl-transforms-pose-stamped that,
according to the reasoning engine, correspond to the given `location-designator'.")
  (:method :heuristics 20 (location-designator)
    (desig:resolve-location-designator-through-generators-and-validators
     location-designator)))

(defmethod desig:resolve-designator :around ((desig desig:location-designator) role)
  "We have to hijack DESIG:RESOLVE-DESIGNATOR because otherwise we would have to
make CRAM_DESIGNATORS package depend on CRAM_MANIPULATION_INTERFACES,
and man-int is way too high level to make it a dependency of CRAM_CORE.
This hijacking is kind of an ugly hack that Gaya feels bad about :(."
  (get-location-poses desig))

(defgeneric get-object-likely-location (object-type environment-name human-name context)
  (:method-combination cut:first-in-order-and-around)
  (:documentation "Returns a location designator representing the
location, where an object with given type `object-type' can typically be found at.
The likely location can depend on the `environment-name',
the human preferences represented as `human-name'
and `context' representing the name of the action context, e.g. :table-setting."))

(defgeneric get-object-destination (object-type environment-name human-name context)
  (:method-combination cut:first-in-order-and-around)
  (:documentation "Returns a location designator representing the
location, where an object with given type `object-type' usually should be brought to
in the given action context `context', e.g., :table-setting..
The likely destination can additionally depend on the `environment-name' and
the human preferences represented as `human-name'."))

(defgeneric get-container-opening-distance (container-name)
  (:method-combination cut:first-in-order-and-around)
  (:documentation "Returns a value describing the distance a container can
be openend.
`container-designator' is a designator describing the container.")
  (:method :heuristics 20 (container-name)
    nil))

(defgeneric get-container-closing-distance (container-name)
  (:method-combination cut:first-in-order-and-around)
  (:documentation "Returns a value describing the distance for which a container
is considered closed.
`container-designator' is a designator describing the container.")
  (:method :heuristics 20 (container-name)
    nil))
