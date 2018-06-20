;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :cram-bullet-reasoning-belief-state)

(def-fact-group occasions (cpoe:object-in-hand cpoe:object-picked cpoe:object-placed-at cpoe:loc)
  (<- (cpoe:object-in-hand ?object)
    (setof ?object (cpoe:object-in-hand ?object ?_) ?objects)
    (member ?object ?objects))

  (<- (cpoe:object-in-hand ?object ?side)
    (btr:bullet-world ?world)
    (cram-robot-interfaces:robot ?robot)
    (btr:attached ?world ?robot ?link ?object-name)
    (once
     (and (object-designator-name ?object ?object-name)
          (desig:obj-desig? ?object)))
    (cram-robot-interfaces:end-effector-link ?robot ?side ?link))

  (<- (cpoe:object-picked ?object)
    (cpoe:object-in-hand ?object))

  (<- (cpoe:object-placed-at ?object ?location)
    (cpoe:loc ?object ?location))

  (<- (cpoe:loc ?robot ?location)
    (cram-robot-interfaces:robot ?robot)
    (object-at-location ?_ ?robot ?location))

  (<- (cpoe:loc ?object ?location)
    (desig:obj-desig? object)
    (object-designator-name ?object ?object-name)
    (object-at-location ?_ ?object-name ?location)))



(def-fact-group occasion-utilities (object-designator-name desig:desig-location-prop)
  (<- (object-designator-name ?name ?name)
    (lisp-type ?name symbol))

  (<- (object-designator-name ?object-designator ?object-name)
    (or (and (bound ?object-designator)
             (desig:obj-desig? ?object-designator))
        (and (not (bound ?object-designator))
             (lisp-fun unique-object-designators ?object-designators)
             (member ?one-object-designator-from-chain ?object-designators)
             (desig:current-designator ?one-object-designator-from-chain ?object-designator)))
    (lisp-fun get-designator-object-name ?object-designator ?belief-name)
    (-> (lisp-pred identity ?belief-name)
        (equal ?object-name ?belief-name)
        (and (desig:desig-prop ?object-designator (:type ?object-type))
             (btr:bullet-world ?w)
             (btr:item-type ?world ?object-name ?object-type))))

  (<- (desig:desig-location-prop ?designator ?location)
    (desig:obj-desig? ?designator)
    (desig:desig-prop ?designator (:type ?type))
    (not (desig:desig-prop ?designator (:name ?name)))
    (not (desig:desig-prop ?designator (:pose ?pose)))
    (btr:bullet-world ?world)
    (btr:item-type ?world ?name ?type)
    (btr:pose ?world ?name ?location))

  (<- (desig:desig-location-prop ?desig ?loc)
    (desig:loc-desig? ?desig)
    (or (desig:desig-prop ?desig (:obj ?o))
        (desig:desig-prop ?desig (:object ?o)))
    (btr:object ?_ ?o)
    (btr:pose ?_ ?o ?loc))

  (<- (desig:desig-location-prop ?o ?loc)
    (btr:object ?_ ?o)
    (btr:pose ?_ ?o ?loc))

  (<- (object-at-location ?world ?object-name ?location-designator)
    (lisp-type ?location-designator desig:location-designator)
    (btr:bullet-world ?world)
    (lisp-fun desig:current-desig ?location-designator ?current-location)
    (lisp-pred identity ?current-location)
    (desig:designator-groundings ?current-location ?_)
    (or (btr:object-pose ?world ?object-name ?object-pose)
        (btr:object-bottom-pose ?world ?object-name ?object-pose))
    (lisp-pred desig:validate-location-designator-solution ?current-location ?object-pose))

  (<- (object-at-location ?world ?object-name ?location-designator)
    (not (bound ?location))
    (btr:bullet-world ?world)
    (btr:object-pose ?world ?object-name ?object-pose)
    (symbol-value cram-tf:*fixed-frame* ?fixed-frame)
    (lisp-fun cl-transforms-stamped:pose->pose-stamped ?fixed-frame 0.0 ?object-pose
              ?object-pose-stamped)
    (desig:designator :location ((:pose ?object-pose-stamped))
                      ?location-designator)))

(defun unique-object-designators ()
  "Returns all designators. For equated designators, only one instance
is returned."
  (remove-duplicates
   (remove-if-not (lambda (designator)
                    (and
                     (typep designator 'desig:object-designator)))
                  (reverse (desig:get-all-designators)))
   :test #'desig:desig-equal))
