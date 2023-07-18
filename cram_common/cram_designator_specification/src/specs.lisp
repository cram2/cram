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

(in-package :cram-designator-specification)

;; (Defgeneric get-designator-property (designator key)
;;   (:method (desig key)
;;     (declare (ignore key))
;;     (unless (typep desig 'desig:designator)
;;       (error 'simple-error
;;              :format-control "[designator-property]: ~a has to be of type DESIGNATOR"
;;              :format-arguments (list desig))))

;;   (:method ((desig desig:designator) key)
;;     (desig:description desig)))


(def-fact-group all-designator-specs (property %property)

  (<- (property-member (?key ?value) ?designator)
    (assert-type ?designator desig:designator "PROPERTY-MEMBER")
    (lisp-fun desig:description ?designator ?props)
    (member (?key ?value) ?props))

  (<- (property ?designator (?key ?value))
    (bound ?key)
    (bound ?value)
    (property-member (?key ?value) ?designator))

  (<- (property ?designator (?key ?value))
    (bound ?key)
    (not (bound ?value))
    (%property ?designator (?key ?value)))

  (<- (%property ?designator (?key ?value))
    (fail)))

(def-fact-group motion-designator-specs (%property)

  (<- (%property ?designator (?location-key ?location))
    (lisp-pred typep ?designator desig:motion-designator)
    (member ?location-key (:pose :left-pose :right-pose))
    (property-member (?location-key ?location) ?designator)
    (assert-type ?location cl-transforms-stamped:pose-stamped "MOTION SPEC:PROPERTY"))

  (<- (%property ?designator (?object-key ?object))
    (lisp-pred typep ?designator desig:motion-designator)
    (member ?object-key (:object :objects))
    (property-member (?object-key ?object) ?designator)
    (assert-type ?object desig:object-designator "MOTION SPEC:PROPERTY"))

  (<- (%property ?designator (?number-key ?value))
    (lisp-pred typep ?designator desig:motion-designator)
    (member ?number-key (:effort
                         :joint-angle :joint-angle-threshold :speed
                         :duration))
    (property-member (?number-key ?value) ?designator)
    (assert-type ?value (or keyword number) "MOTION SPEC:PROPERTY"))

  (<- (%property ?designator (?key ?value))
    (lisp-pred typep ?designator desig:motion-designator)
    (member ?key (:function))
    (property-member (?key ?value) ?designator)
    (assert-type ?value function "MOTION SPEC:PROPERTY"))

  (<- (%property ?designator (?keyword-key ?value))
    (lisp-pred typep ?designator desig:motion-designator)
    (member ?keyword-key (:gripper :direction :arm))
    (property-member (?keyword-key ?value) ?designator)
    (assert-type ?value (or keyword list) "MOTION SPEC:PROPERTY"))

  (<- (%property ?designator (?string-key ?value))
    (lisp-pred typep ?designator desig:motion-designator)
    (member ?string-key (:joint-name))
    (property-member (?string-key ?value) ?designator)
    (assert-type ?value string "MOTION SPEC:PROPERTY"))

  (<- (%property ?designator (?list-key ?value))
    (lisp-pred typep ?designator desig:motion-designator)
    (member ?list-key (:poses :joint-states :left-joint-states :right-joint-states))
    (property-member (?list-key ?value) ?designator)
    (assert-type ?value list "MOTION SPEC:PROPERTY"))

  (<- (%property ?designator (?key ?value))
    (lisp-pred typep ?designator desig:motion-designator)
    (member ?key (:avoid-collisions-not-much
                  :align-planes-left
                  :align-planes-right))
    (property-member (?key ?value) ?designator)
    (assert-type ?value boolean "MOTION SPEC:PROPERTY")))


(def-fact-group action-designator-specs (%property)

  (<- (%property ?designator (?location-key ?location))
    (lisp-pred typep ?designator desig:action-designator)
    (member ?location-key (:target :location :robot-location))
    (property-member (?location-key ?location) ?designator)
    (assert-type ?location desig:location-designator "ACTION SPEC:PROPERTY"))

  (<- (%property ?designator (?list-key ?value))
    (lisp-pred typep ?designator desig:action-designator)
    (member ?list-key (:poses :left-poses :right-poses :arms :grasps))
    (property-member (?list-key ?value) ?designator)
    (assert-type ?value list "ACTION SPEC:PROPERTY"))

  (<- (%property ?designator (?keyword-or-list-key ?value))
    (lisp-pred typep ?designator desig:action-designator)
    (member ?keyword-or-list-key (:gripper :arm :direction :grasp
                                  :left-grasp :right-grasp :camera :type
                                  :context :link :configuration :park-arms
                                  :left-configuration :right-configuration :collision-mode))

    (property-member (?keyword-or-list-key ?value) ?designator)
    (assert-type ?value (or keyword list) "ACTION SPEC:PROPERTY"))

  (<- (%property ?designator (?object-key ?object))
    (lisp-pred typep ?designator desig:action-designator)
    (member ?object-key (:object :on-object :with-object :supporting-object
                         :container-object :source))
    (property-member (?object-key ?object) ?designator)
    (assert-type ?object desig:object-designator "ACTION SPEC:PROPERTY"))

  (<- (%property ?designator (?number-key ?value))
    (lisp-pred typep ?designator desig:action-designator)
    (member ?number-key (:position :effort :distance :duration :reso :rounds))
    (property-member (?number-key ?value) ?designator)
    (assert-type ?value number "ACTION SPEC:PROPERTY"))

  (<- (%property ?designator (?key ?value))
    (lisp-pred typep ?designator desig:action-designator)
    (member ?key (:joint-angle-threshold))
    (property-member (?key ?value) ?designator)
    (assert-type ?value (or number null) "ACTION SPEC:PROPERTY"))

  (<- (%property ?designator (?string-key ?value))
    (lisp-pred typep ?designator desig:action-designator)
    (member ?string-key (:frame :joint-name))
    (property-member (?string-key ?value) ?designator)
    (assert-type ?value string "ACTION SPEC:PROPERTY"))

  (<- (%property ?designator (?key ?value))
    (lisp-pred typep ?designator desig:action-designator)
    (member ?key (:joint-angle :speed))
    (property-member (?key ?value) ?designator)
    (assert-type ?value (or keyword number) "ACTION SPEC:PROPERTY"))

  (<- (%property ?designator (:for ?for-value))
    (lisp-pred typep ?designator desig:action-designator)
    (property-member (:for ?for-value) ?designator)
    (assert-type ?for-value (or keyword desig:object-designator) "ACTION SPEC:PROPERTY"))

  (<- (%property ?designator (?key ?value))
    (lisp-pred typep ?designator desig:action-designator)
    (member ?key (:function))
    (property-member (?key ?value) ?designator)
    (assert-type ?value (or function null) "ACTION SPEC:PROPERTY")))


(def-fact-group location-designator-specs (%property)

  (<- (%property ?designator (:pose ?pose-stamped))
    (lisp-pred typep ?designator desig:location-designator)
    (property-member (:pose ?pose-stamped) ?designator)
    (assert-type ?pose-stamped cl-transforms-stamped:pose-stamped "LOCATION SPEC:PROPERTY"))

  (<- (%property ?designator (?list-key ?list-value))
    (lisp-pred typep ?designator desig:location-designator)
    (member ?list-key (:poses :attachments))
    (property-member (?list-key ?list-value) ?designator)
    (assert-type ?list-value list "LOCATION SPEC:PROPERTY"))

  (<- (%property ?designator (?object-desig-key ?value))
    (lisp-pred typep ?designator desig:location-designator)
    (member ?object-desig-key (:object
                               :in :on :above
                               :left-of :right-of :in-front-of :behind
                               :far-from :near
                               :of))
    (property-member (?object-desig-key ?value) ?designator)
    (assert-type ?value desig:object-designator "LOCATION SPEC:PROPERTY"))

  (<- (%property ?designator (?location-desig-key ?value))
    (lisp-pred typep ?designator desig:location-designator)
    (member ?location-desig-key (:location))
    (property-member (?location-desig-key ?value) ?designator)
    (assert-type ?value desig:location-designator "LOCATION SPEC:PROPERTY"))

  (<- (%property ?designator (?keyword-key ?value))
    (lisp-pred typep ?designator desig:location-designator)
    (member ?keyword-key (:arm :attachment))
    (property-member (?keyword-key ?value) ?designator)
    (assert-type ?value keyword "LOCATION SPEC:PROPERTY"))

  (<- (%property ?designator (?number-key ?value))
    (lisp-pred typep ?designator desig:location-designator)
    (member ?number-key (:z-offset))
    (property-member (?number-key ?value) ?designator)
    (assert-type ?value number "LOCATION SPEC:PROPERTY")))


(def-fact-group object-designator-specs (%property)

 (<- (%property ?designator (?keyword-key ?type))
  ;(<- (%property ?designator (:type ?type))
   (lisp-pred typep ?designator desig:object-designator)
    (member ?keyword-key (:type :tool))
    ;(property-member (:type ?type) ?designator)
    (property-member (?keyword-key ?type) ?designator)
    (assert-type ?type keyword "OBJECT SPEC:PROPERTY"))

  (<- (%property ?designator (?keyword-key ?name))
    (lisp-pred typep ?designator desig:object-designator)
    (member ?keyword-key (:name))
    (property-member (?keyword-key ?name) ?designator)
    (assert-type ?name symbol "OBJECT SPEC:PROPERTY"))

  (<- (%property ?designator (?keyword-key ?name))
    (lisp-pred typep ?designator desig:object-designator)
    (member ?keyword-key (:urdf-name))
    (property-member (?keyword-key ?name) ?designator)
    (assert-type ?name (or symbol string) "OBJECT SPEC:PROPERTY"))

  (<- (%property ?designator (:part-of ?environment-or-robot))
    (lisp-pred typep ?designator desig:object-designator)
    (property-member (:part-of ?environment-or-robot) ?designator)
    (assert-type ?environment-or-robot symbol "OBJECT SPEC:PROPERTY"))

  (<- (%property ?designator (:handle-axis ?axis))
    (lisp-pred typep ?designator desig:object-designator)
    (property-member (:handle-axis ?axis) ?designator)
    (assert-type ?axis cl-transforms:3d-vector "OBJECT SPEC:PROPERTY"))

  (<- (%property ?designator (:pose ?pose))
    (lisp-pred typep ?designator desig:object-designator)
    (property-member (:pose ?pose) ?designator)
    (assert-type ?pose list "OBEJCT SPEC:PROPERTY"))

  (<- (%property ?designator (:location ?location))
    (lisp-pred typep ?designator desig:object-designator)
    (property-member (:location ?location) ?designator)
    (assert-type ?location desig:location-designator "OBJECT SPEC:PROPERTY")))
