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

;; (defgeneric get-designator-property (designator key)
;;   (:method (desig key)
;;     (declare (ignore key))
;;     (unless (typep desig 'desig:designator)
;;       (error 'simple-error
;;              :format-control "[designator-property]: ~a has to be of type DESIGNATOR"
;;              :format-arguments (list desig))))

;;   (:method ((desig desig:designator) key)
;;     (desig:description desig)))


(def-fact-group all-designator-specs (property)

  (<- (property-member (?key ?value) ?designator)
    (assert-type ?designator desig:designator "PROPERTY-MEMBER")
    (lisp-fun desig:description ?designator ?props)
    (member (?key ?value) ?props))

  (<- (property ?designator (?key ?value))
    (bound ?key)
    ;; (bound ?value) ; set default behaviour to not throw errors if no spec is defined
    (bound ?value)
    (property-member (?key ?value) ?designator)))

(def-fact-group motion-designator-specs (property)

  (<- (property ?designator (?location-key ?location))
    (lisp-pred typep ?designator desig:motion-designator)
    (member ?location-key (:target :left-target :right-target))
    (property-member (?location-key ?location) ?designator)
    (assert-type ?location desig:location-designator "MOTION SPEC:PROPERTY"))

  (<- (property ?designator (?object-key ?object))
    (lisp-pred typep ?designator desig:motion-designator)
    (member ?object-key (:object :objects))
    (property-member (?object-key ?object) ?designator)
    (assert-type ?object desig:object-designator "MOTION SPEC:PROPERTY"))

  (<- (property ?designator (?string-key ?value))
    (lisp-pred typep ?designator desig:motion-designator)
    (member ?string-key (:frame))
    (property-member (?string-key ?value) ?designator)
    (assert-type ?value string "MOTION SPEC:PROPERTY"))

  (<- (property ?designator (?number-key ?value))
    (lisp-pred typep ?designator desig:motion-designator)
    (member ?number-key (:effort :joint-angle))
    (property-member (?number-key ?value) ?designator)
    (assert-type ?value number "MOTION SPEC:PROPERTY"))

  (<- (property ?designator (?keyword-key ?value))
    (lisp-pred typep ?designator desig:motion-designator)
    (member ?keyword-key (:gripper :direction :arm))
    (property-member (?keyword-key ?value) ?designator)
    (assert-type ?value (or keyword list) "MOTION SPEC:PROPERTY"))

  (<- (property ?designator (?list-key ?value))
    (lisp-pred typep ?designator desig:motion-designator)
    (member ?list-key (:left-configuration :right-configuration))
    (property-member (?list-key ?value) ?designator)
    (assert-type ?value list "MOTION SPEC:PROPERTY")))


(def-fact-group action-designator-specs (property)

  (<- (property ?designator (?location-key ?location))
    (lisp-pred typep ?designator desig:action-designator)
    (member ?location-key (:target :location))
    (property-member (?location-key ?location) ?designator)
    (assert-type ?location desig:location-designator "ACTION SPEC:PROPERTY"))

  (<- (property ?designator (?list-key ?value))
    (lisp-pred typep ?designator desig:action-designator)
    (member ?list-key (:left-poses :right-poses))
    (property-member (?list-key ?value) ?designator)
    (assert-type ?value list "ACTION SPEC:PROPERTY"))

  (<- (property ?designator (?keyword-or-list-key ?value))
    (lisp-pred typep ?designator desig:action-designator)
    (member ?keyword-or-list-key (:gripper :arm :direction :grasp :camera :type))
    (property-member (?keyword-or-list-key ?value) ?designator)
    (assert-type ?value (or keyword list) "ACTION SPEC:PROPERTY"))

  (<- (property ?designator (?object-key ?object))
    (lisp-pred typep ?designator desig:action-designator)
    (member ?object-key (:object :on-object :with-object))
    (property-member (?object-key ?object) ?designator)
    (assert-type ?object desig:object-designator "ACTION SPEC:PROPERTY"))

  (<- (property ?designator (?number-key ?value))
    (lisp-pred typep ?designator desig:action-designator)
    (member ?number-key (:position :effort :distance))
    (property-member (?number-key ?value) ?designator)
    (assert-type ?value number "ACTION SPEC:PROPERTY"))

  (<- (property ?designator (:for ?for-value))
    (lisp-pred typep ?designator desig:action-designator)
    (property-member (:for ?for-value) ?designator)
    (assert-type ?for-value (or keyword desig:object-designator) "ACTION SPEC:PROPERTY")))


(def-fact-group location-designator-specs (property)

  (<- (property ?designator (:pose ?pose-stamped))
    (lisp-pred typep ?designator desig:location-designator)
    (property-member (:pose ?pose-stamped) ?designator)
    (assert-type ?pose-stamped cl-transforms-stamped:pose-stamped "LOCATION SPEC:PROPERTY"))

  (<- (property ?designator (:object ?value))
    (lisp-pred typep ?designator desig:location-designator)
    (property-member (:object ?value) ?designator)
    (assert-type ?value desig:object-designator "LOCATION SPEC:PROPERTY"))

  (<- (property ?designator (?keyword-key ?value))
    (lisp-pred typep ?designator desig:location-designator)
    (member ?keyword-key (:arm))
    (property-member (?keyword-key ?value) ?designator)
    (assert-type ?value keyword "LOCATION SPEC:PROPERTY")))


(def-fact-group object-designator-specs (property)

  (<- (property ?designator (:type ?type))
    (lisp-pred typep ?designator desig:object-designator)
    (property-member (:type ?type) ?designator)
    (assert-type ?type keyword "OBJECT SPEC:PROPERTY"))

  (<- (property ?designator (?keyword-key ?name))
    (lisp-pred typep ?designator desig:object-designator)
    (member ?keyword-key (:name :urdf-name))
    (property-member (?keyword-key ?name) ?designator)
    (assert-type ?name symbol "OBJECT SPEC:PROPERTY"))

  (<- (property ?designator (:part-of ?environment))
    (lisp-pred typep ?designator desig:object-designator)
    (property-member (:part-of ?environment) ?designator)
    (assert-type ?environment keyword "OBJECT SPEC:PROPERTY")))
