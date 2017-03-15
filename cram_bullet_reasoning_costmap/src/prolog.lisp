;;; Copyright (c) 2012, Gayane Kazhoyan <kazhoyan@in.tum.de>
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

(in-package :btr-costmap)

(defmethod costmap-generator-name->score ((name (eql 'supporting-object))) 3)
(defmethod costmap-generator-name->score ((name (eql 'slot-generator))) 5)
(defmethod costmap-generator-name->score ((name (eql 'collision))) 10)

(defclass range-generator () ())
(defmethod costmap-generator-name->score ((name range-generator)) 2)

(defclass gaussian-generator () ())
(defmethod costmap-generator-name->score ((name gaussian-generator)) 6)

(defclass field-generator () ())
(defmethod costmap-generator-name->score ((name field-generator)) 7)


(def-fact-group spatial-relations-costmap (desig-costmap)
  ;; height generator for locations ((on something) (name something) (for some-obj))
  (<- (desig-costmap ?designator ?costmap)
    (desig-prop ?designator (:on ?_))
    (desig-prop ?designator (:name ?_))
    (desig-prop ?designator (:for ?for-object-name))
    (costmap ?costmap)
    (semantic-map-costmap::semantic-map-desig-objects ?designator ?sem-map-objects)
    (member ?sem-map-object ?sem-map-objects)
    (bullet-world ?world)
    (%object ?world ?for-object-name ?for-object)
    (costmap-add-height-generator
     (make-object-on-object-bb-height-generator ?sem-map-object ?for-object)
     ?costmap))

  (<- (near-costmap ?designator ?ref-obj-pose ?ref-obj-size ?ref-padding
                    ?for-obj-size ?for-padding ?costmap)
    (costmap ?costmap)
    ;;
    (near-costmap-gauss-std ?std)
    (instance-of gaussian-generator ?gaussian-generator-id)
    (costmap-add-function
     ?gaussian-generator-id
     (make-location-cost-function ?ref-obj-pose ?std)
     ?costmap)
    ;;
    (lisp-fun calculate-near-costmap-min-radius ?ref-obj-size ?for-obj-size
              ?ref-padding ?for-padding ?min-radius)
    (costmap-width-in-obj-size-percentage-near ?cm-width-perc)
    (lisp-fun calculate-costmap-width ?ref-obj-size ?for-obj-size ?cm-width-perc
              ?cm-width)
    ;;
    (lisp-fun + ?min-radius ?cm-width ?max-radius)
    (instance-of range-generator ?range-generator-id-1)
    (costmap-add-function
     ?range-generator-id-1
     (make-range-cost-function ?ref-obj-pose ?max-radius)
     ?costmap)
    ;;
    (instance-of range-generator ?range-generator-id-2)
    (costmap-add-function
     ?range-generator-id-2
     (make-range-cost-function ?ref-obj-pose ?min-radius :invert t)
     ?costmap))

  (<- (far-from-costmap ?designator ?ref-obj-pose ?ref-obj-size ?ref-padding
                        ?for-obj-size ?for-padding ?costmap)
    (costmap ?costmap)
    ;;
    (lisp-fun calculate-far-costmap-min-radius ?ref-obj-size ?for-obj-size
              ?ref-padding ?for-padding ?min-radius)
    (costmap-width-in-obj-size-percentage-far ?cm-width-perc)
    (lisp-fun calculate-costmap-width ?ref-obj-size ?for-obj-size ?cm-width-perc
              ?cm-width)
    ;;
    (lisp-fun + ?min-radius ?cm-width ?max-radius)
    (instance-of range-generator ?range-generator-id-1)
    (costmap-add-function
     ?range-generator-id-1
     (make-range-cost-function ?ref-obj-pose ?max-radius)
     ?costmap)
    ;;
    (instance-of range-generator ?range-generator-id-2)
    (costmap-add-function
     ?range-generator-id-2
     (make-range-cost-function ?ref-obj-pose ?min-radius :invert t)
     ?costmap))

  ;; near and far-from for bullet objects
  (<- (desig-costmap ?designator ?costmap)
    (or
     (desig-prop ?designator (:near ?ref-obj))
     (desig-prop ?designator (:far-from ?ref-obj)))
    (object-instance-name ?ref-obj ?ref-obj-name)
    (btr:bullet-world ?world)
    (btr:object ?world ?ref-obj-name)
    ;;
    (desig-location-prop ?ref-obj-name ?ref-obj-pose)
    (object-size-without-handles ?world ?ref-obj-name ?ref-obj-size)
    (padding-size ?world ?ref-obj-name ?ref-padding)
    ;;
    (desig-prop ?designator (:for ?for-obj))
    (object-instance-name ?for-obj ?for-obj-name)
    (object ?world ?for-obj-name)
    (object-size-without-handles ?world ?for-obj-name ?for-obj-size)
    (padding-size ?world ?for-obj-name ?for-padding)
    ;;
    (-> (desig-prop ?designator (:near ?ref-obj))
        (near-costmap ?designator ?ref-obj-pose ?ref-obj-size ?ref-padding
                      ?for-obj-size ?for-padding ?costmap)
        (far-from-costmap ?designator ?ref-obj-pose ?ref-obj-size ?ref-padding
                      ?for-obj-size ?for-padding ?costmap)))

  ;; uses make-potential-field-cost-function to resolve the designator
  ;; TODO height generator is based only on semantic map (no stacking up allowed)
  (<- (potential-field-costmap ?designator ?object ?relation ?costmap)
    ;; TODO GET RID OF THIS!
    ;; Todo new mechanism for switching between sampling functions...
    ;; (lisp-fun set location-costmap::*use-priority-sampling* t ?_)
    ;;
    (desig-location-prop ?object ?reference-pose)
    (lisp-fun get-y-of-pose ?reference-pose ?y-of-pose)
    (lisp-fun get-x-of-pose ?reference-pose ?x-of-pose)
    (costmap ?costmap)
    ;;
    (btr:bullet-world ?world)
    (supporting-rigid-body ?world ?object ?rigid-body)
    ;; costmap to exclude everything which is outside of supp. object boundaries
    (lisp-fun list ?rigid-body ?rigid-bodies)
    (costmap-add-function
     supporting-object
     (make-aabbs-costmap-generator ?rigid-bodies)
     ?costmap)
    ;; the actual potential field costmap
    ;; The axis of the potential field depends on to which side of supporting object
    ;; the reference object is the closest
    (lisp-fun cl-bullet:pose ?rigid-body ?supp-obj-pose)
    (lisp-fun btr::set-object-pose ?rigid-body ((0 0 0) (0 0 0 1)) ?_)
    (lisp-fun aabb ?rigid-body ?aabb)
    (lisp-fun cl-bullet:bounding-box-dimensions ?aabb ?supp-obj-dims)
    (lisp-fun btr::set-object-pose ?rigid-body ?supp-obj-pose ?_)
    (lisp-fun get-closest-edge ?reference-pose ?supp-obj-pose ?supp-obj-dims ?edge)
    (relation-axis-and-pred ?edge ?relation ?axis ?pred)
    (instance-of field-generator ?field-generator-id)
    (costmap-add-function
     ?field-generator-id
     (make-potential-field-cost-function ?axis ?x-of-pose ?y-of-pose
                                         ?supp-obj-pose ?pred)
     ?costmap))

  ;; left-of for bullet objects using potential field cost-function
  (<- (desig-costmap ?designator ?costmap)
    (desig-prop ?designator (:left-of ?object))
    (object-instance-name ?object ?obj-name)
    (btr:bullet-world ?world)
    (btr:object ?world ?obj-name)
    (potential-field-costmap ?designator ?obj-name :left-of ?costmap))

  ;; right-of for bullet objects using potential field cost-function
  (<- (desig-costmap ?designator ?costmap)
    (desig-prop ?designator (:right-of ?object))
    (object-instance-name ?object ?obj-name)
    (btr:bullet-world ?world)
    (btr:object ?world ?obj-name)
    (potential-field-costmap ?designator ?obj-name :right-of ?costmap))

  ;; in-front-of for bullet objects using potential field cost-function
  (<- (desig-costmap ?designator ?costmap)
    (desig-prop ?designator (:in-front-of ?object))
    (object-instance-name ?object ?obj-name)
    (btr:bullet-world ?world)
    (btr:object ?world ?obj-name)
    (potential-field-costmap ?designator ?obj-name :in-front-of ?costmap))

  ;; behind for bullet objects using potential field cost-function
  (<- (desig-costmap ?designator ?costmap)
    (desig-prop ?designator (:behind ?object))
    (object-instance-name ?object ?obj-name)
    (btr:bullet-world ?world)
    (btr:object ?world ?obj-name)
    (potential-field-costmap ?designator ?obj-name :behind ?costmap))

  ;; height generators
  (<- (desig-costmap ?designator ?costmap)
    (fail)
    (or
     (desig-prop ?designator (:left-of ?ref-obj))
     (desig-prop ?designator (:right-of ?ref-obj))
     (desig-prop ?designator (:in-front-of ?ref-obj))
     (desig-prop ?designator (:behind ?ref-obj))
     (desig-prop ?designator (:far-from ?ref-obj))
     (desig-prop ?designator (:near ?ref-obj)))
    (object-instance-name ?ref-obj ?ref-obj-name)
    (bullet-world ?world)
    (object ?world ?ref-obj-name)
    (bagof ?z (and (supporting-rigid-body ?world ?ref-obj-name ?rigid-body)
                   (lisp-fun get-rigid-body-aabb-top-z ?rigid-body ?z)) ?z-bag)
    (max ?z-bag ?highest-z)
    (costmap ?costmap)
    (-> (desig-prop ?designator (:for ?for-obj))
        (and (object-instance-name ?for-obj ?for-obj-name)
             (object ?world ?for-obj-name)
             (%object ?world ?for-obj-name ?for-object-instance)
             (lisp-fun aabb ?for-object-instance ?aabb)
             (lisp-fun cl-bullet:bounding-box-dimensions ?aabb ?dimensions)
             (lisp-fun cl-transforms:z ?dimensions ?height)
             (lisp-fun / ?height 2 ?offset)
             (lisp-fun + ?highest-z ?offset ?resulting-z))
        (equal ?highest-z ?resulting-z))
    (costmap-add-height-generator
     (make-constant-height-function ?resulting-z)
     ?costmap))

  ;; collision avoidance costmap for the spatial relations desigs
  ;; uses make-slot-cost-function
  (<- (slot-costmap ?designator ?supp-object ?context ?object-type ?object-count
                    ?costmap)
    (lisp-fun sem-map-utils:name ?supp-object ?supp-object-name)
    (paddings-list ?supp-object-name ?context ?paddings-list)
    (preferred-supporting-object-side ?supp-object-name ?context ?preferred-side)
    (max-slot-size ?object-type ?context ?max-slot-size)
    (min-slot-size ?object-type ?context ?min-slot-size)
    (position-deviation-threshold ?object-type ?context ?pos-dev-threshold)
    ;;
    (costmap ?costmap)
    (costmap-add-function
     slot-generator
     (make-slot-cost-function ?supp-object ?paddings-list ?preferred-side
                              ?object-count ?max-slot-size ?min-slot-size
                              ?pos-dev-threshold)
     ?costmap))

  ;; Disabled.
  (<- (desig-costmap ?desig ?cm)
    (fail)
    (or
     (desig-prop ?desig (:left-of ?_))
     (desig-prop ?desig (:right-of ?_))
     (desig-prop ?desig (:in-front-of ?_))
     (desig-prop ?desig (:behind ?_))
     (desig-prop ?desig (:far-from ?_))
     (desig-prop ?desig (:near ?_)))
    (collision-costmap-padding-in-meters ?padding)
    (-> (desig-prop ?desig (:for ?object))
        (and
         (object-instance-name ?object ?object-name) 
         (object ?world ?object-name)
         (object-size-without-handles ?world ?object-name ?obj-size) 
         (lisp-fun / ?obj-size 2 ?obj-size/2)
         (lisp-fun + ?obj-size/2 ?padding ?overall-padding)
         (collision-invert-costmap ?desig ?overall-padding ?cm))
        (collision-invert-costmap ?desig ?padding ?cm)))

  ;; for plates on table
  ;; '((on counter-top) (name kitchen-island)
  ;;   (context :table-setting) (for plate-1) (object-count 4))
    ;; uses make-slot-cost-function
  (<- (slot-costmap ?designator ?supp-object ?context ?object-type ?object-count
                    ?costmap)
    (lisp-fun sem-map-utils:name ?supp-object ?supp-object-name)
    (paddings-list ?supp-object-name ?context ?paddings-list)
    (preferred-supporting-object-side ?supp-object-name ?context ?preferred-side)
    (max-slot-size ?object-type ?context ?max-slot-size)
    (min-slot-size ?object-type ?context ?min-slot-size)
    (position-deviation-threshold ?object-type ?context ?pos-dev-threshold)
    ;;
    (costmap ?costmap)
    (costmap-add-function
     slot-generator
     (make-slot-cost-function ?supp-object ?paddings-list ?preferred-side
                              ?object-count ?max-slot-size ?min-slot-size
                              ?pos-dev-threshold)
     ?costmap))

  (<- (desig-costmap ?designator ?costmap)
    (desig-prop ?designator (:on ?_))
    (desig-prop ?designator (:name ?supp-obj-name))
    (desig-prop ?designator (:context :table-setting))
    (desig-prop ?designator (:for ?for-object))
    (desig-prop ?designator (:object-count ?object-count))
    (bullet-world ?world)
    (object-instance-name ?for-object ?object-name)
    (item-type ?world ?object-name ?object-type)
    (lisp-fun sem-map-desig:designator->semantic-map-objects
              ?designator ?supp-objects)
    (member ?supp-object ?supp-objects)
    (slot-costmap ?designator ?supp-object :table-setting ?object-type ?object-count
                  ?costmap))

  ;; for validators
  (<- (desig-solution-not-in-collision ?desig ?object-to-check ?pose)
    (bullet-world ?world)
    (with-copied-world ?world

      (object-instance-name ?object-to-check ?object-name)
      (object ?world ?object-name)
      (%object ?world ?object-name ?object-instance)

      (lisp-fun aabb ?object-instance ?aabb)
      (lisp-fun cl-bullet:bounding-box-dimensions ?aabb ?dimensions)
      (lisp-fun cl-transforms:z ?dimensions ?height)
      (lisp-fun / ?height 2 ?offset)

      (lisp-fun add-z-offset-to-pose ?pose ?offset ?new-pose)

      (assert (object-pose ?world ?object-name ?new-pose))
      (forall (contact ?world ?object-name ?other-object-name)
              (not (object-type ?world ?other-object-name btr::item))))))


(def-fact-group relations-lookup-table ()
  (<- (relation-axis-and-pred :front :left-of :Y >))
  (<- (relation-axis-and-pred :front :right-of :Y <))
  (<- (relation-axis-and-pred :front :behind :X >))
  (<- (relation-axis-and-pred :front :in-front-of :X <))
  (<- (relation-axis-and-pred :back :left-of :Y <))
  (<- (relation-axis-and-pred :back :right-of :Y >))
  (<- (relation-axis-and-pred :back :behind :X <))
  (<- (relation-axis-and-pred :back :in-front-of :X >))
  (<- (relation-axis-and-pred :right :left-of :X <))
  (<- (relation-axis-and-pred :right :right-of :X >))
  (<- (relation-axis-and-pred :right :behind :Y >))
  (<- (relation-axis-and-pred :right :in-front-of :Y <))
  (<- (relation-axis-and-pred :left :left-of :X >))
  (<- (relation-axis-and-pred :left :right-of :X <))
  (<- (relation-axis-and-pred :left :behind :Y <))
  (<- (relation-axis-and-pred :left :in-front-of :Y >)))


(def-fact-group location-desig-utils ()
  (<- (object-instance-name ?name ?name)
    (lisp-type ?name symbol))
  ;;
  (<- (object-instance-name ?designator ?name)
    (obj-desig? ?designator)
    (lisp-fun cram-bullet-reasoning-belief-state:get-designator-object-name
              ?designator ?name))

  ;; returns diameter or something similar in meters
  (<- (object-size-without-handles ?world ?obj-name ?size)
    (object-shape ?world ?obj-name ?shape)
    (%object ?world ?obj-name ?obj)
    (%object-size-without-handles ?world ?obj ?shape ?size))
  ;;
  (<- (%object-size-without-handles ?world ?obj ?shape ?size)
    (== ?shape :circle)
    (lisp-fun get-aabb-circle-diameter ?obj ?size))
  ;;
  (<- (%object-size-without-handles ?world ?obj ?shape ?size)
    (== ?shape :rectangle)
    (lisp-fun get-aabb-min-length ?obj ?size))
  ;;
  (<- (%object-size-without-handles ?world ?obj ?shape ?size)
    (== ?shape :oval)
    (lisp-fun get-aabb-oval-diameter ?obj ?size))
  ;;
  (<- (%object-size-without-handles ?world ?obj ?shape ?size)
    (== ?shape :complex)
    (lisp-fun get-aabb-circle-diameter ?obj ?obj-size)
    (%object ?world ?obj-name ?obj)
    (object-handle-size ?world ?obj-name ?handle-size)
    (lisp-fun - ?obj-size ?handle-size ?size))

  (<- (supporting-rigid-body ?world ?ref-obj-name ?rigid-body)
    (bullet-world ?world)
    (object ?world ?ref-obj-name)
    (supported-by ?world ?ref-obj-name ?supporting-object-name
                  ?supporting-object-link-name)
    (lisp-fun get-link-rigid-body ?supporting-object-name
              ?supporting-object-link-name ?rigid-body))

  (<- (supported-by-link-obj ?world ?obj-name ?link-obj)
    (supported-by ?world ?obj-name ?supp-obj-name ?supp-obj-link-name)
    (%object ?world ?supp-obj-name ?supp-obj)
    (lisp-fun get-sem-map-part ?supp-obj ?supp-obj-link-name ?link-obj)
    (lisp-pred identity ?link-obj)))
