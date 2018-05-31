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

(defmethod costmap:costmap-generator-name->score ((name (eql 'supporting-object))) 9)
(defmethod costmap:costmap-generator-name->score ((name (eql 'slot-generator))) 6)
(defmethod costmap:costmap-generator-name->score ((name (eql 'collision))) 10)
(defmethod costmap:costmap-generator-name->score ((name (eql 'side-generator))) 5)

(defclass range-generator () ())
(defmethod costmap:costmap-generator-name->score ((name range-generator)) 2)

(defclass gaussian-generator () ())
(defmethod costmap:costmap-generator-name->score ((name gaussian-generator)) 4)

(defclass field-generator () ())
(defmethod costmap:costmap-generator-name->score ((name field-generator)) 7)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(def-fact-group location-desig-utils (desig:desig-location-prop)
  ;; to get names of object designators
  (<- (object-designator-name ?name ?name)
    (lisp-type ?name symbol))
  ;;
  (<- (object-designator-name ?designator ?name)
    (desig:obj-desig? ?designator)
    (lisp-fun btr-belief:get-designator-object-name ?designator ?belief-name)
    (-> (lisp-pred identity ?belief-name)
        (equal ?name ?belief-name)
        (and (desig:desig-prop ?designator (:type ?object-type))
             (btr:bullet-world ?w)
             (btr:item-type ?world ?name ?object-type))))
  ;;
  (<- (desig:desig-location-prop ?designator ?location)
    (desig:obj-desig? ?designator)
    (desig:desig-prop ?designator (:type ?type))
    (not (desig:desig-prop ?designator (:name ?name)))
    (not (desig:desig-prop ?designator (:pose ?pose)))
    (btr:bullet-world ?world)
    (btr:item-type ?world ?name ?type)
    (btr:pose ?world ?name ?location))

  ;; returns diameter or something similar in meters
  (<- (object-size-without-handles ?world ?obj-name ?size)
    (object-shape ?world ?obj-name ?shape)
    (btr:%object ?world ?obj-name ?obj)
    (%object-size-without-handles ?world ?obj ?shape ?size))
  ;;
  (<- (%object-size-without-handles ?world ?obj :circle ?size)
    (lisp-fun get-aabb-circle-diameter ?obj ?size))
  ;;
  (<- (%object-size-without-handles ?world ?obj :rectangle ?size)
    (lisp-fun get-aabb-min-length ?obj ?size))
  ;;
  (<- (%object-size-without-handles ?world ?obj :oval ?size)
    (lisp-fun get-aabb-oval-diameter ?obj ?size))
  ;;
  (<- (%object-size-without-handles ?world ?obj :complex ?size)
    (lisp-fun get-aabb-circle-diameter ?obj ?obj-size)
    (btr:%object ?world ?obj-name ?obj)
    (object-handle-size ?world ?obj-name ?handle-size)
    (lisp-fun - ?obj-size ?handle-size ?size))

  ;; getting the supporting object of an object
  (<- (supporting-rigid-body ?world ?obj-name ?highest-supporting-rigid-body)
    (btr:bullet-world ?world)
    (btr:object ?world ?obj-name)
    (btr:%object ?world ?obj-name ?object)
    (btr:object-pose ?world ?obj-name ?object-pose)
    (bagof ?supp-rigid-body
           (and (btr:above ?world ?obj-name ?supp-object-name ?supp-object-link-name)
                (btr:%object ?world ?supp-object-name ?supp-object)
                (lisp-fun get-link-rigid-body ?supp-object ?supp-object-link-name ?supp-rigid-body)
                (lisp-pred pose-within-aabb ?object-pose ?supp-rigid-body))
           ?supporting-rigid-bodies-bag)
    (lisp-fun cut:force-ll ?supporting-rigid-bodies-bag ?supporting-rigid-bodies)
    (lisp-fun get-highest-rigid-body-below-object
              ?supporting-rigid-bodies ?object
              ?highest-supporting-rigid-body))
  ;;
  ;; (<- (supporting-semantic-map-link ?world ?obj-name ?supp-link-object)
  ;;   (btr:bullet-world ?world)
  ;;   (btr:object ?world ?obj-name)
  ;;   (btr:supported-by ?world ?obj-name ?supp-object-name ?supp-object-link-name)
  ;;   (btr:%object ?world ?supp-object-name ?supp-object)
  ;;   (lisp-fun get-sem-map-part ?supp-object ?supp-object-link-name ?supp-link-object))

  ;; for validators
  (<- (desig-solution-not-in-collision ?desig ?object-to-check ?pose)
    (btr:bullet-world ?world)
    (btr:with-copied-world ?world
      (object-designator-name ?object-to-check ?object-name)
      (btr:object ?world ?object-name)
      (btr:assert (btr:object-pose ?world ?object-name ?pose))
      (forall (btr:contact ?world ?object-name ?other-object-name)
              (not (btr:item-type ?world ?other-object-name ?_))))))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

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


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(def-fact-group spatial-relations-costmap (costmap:desig-costmap)
;;;;;;;;;; NEAR and FAR-FROM for bullet objects or locations ;;;;;;;;;;;
  (<- (near-or-far-costmap ?ref-obj-pose ?min-radius ?max-radius ?costmap)
    (instance-of range-generator ?range-generator-id-1)
    (costmap:costmap-add-function
     ?range-generator-id-1
     (costmap:make-range-cost-function ?ref-obj-pose ?min-radius :invert t)
     ?costmap)
    (instance-of range-generator ?range-generator-id-2)
    (costmap:costmap-add-function
     ?range-generator-id-2
     (costmap:make-range-cost-function ?ref-obj-pose ?max-radius)
     ?costmap))
  ;;
  (<- (near-costmap ?designator ?ref-obj-pose ?ref-obj-size ?ref-padding
                    ?for-obj-size ?for-padding ?costmap)
    (lisp-fun calculate-near-costmap-min-radius ?ref-obj-size ?for-obj-size
              ?ref-padding ?for-padding ?min-radius)
    (costmap-width-in-obj-size-percentage-near ?cm-width-perc)
    (lisp-fun calculate-costmap-width ?ref-obj-size ?for-obj-size ?cm-width-perc
              ?cm-width)
    (lisp-fun + ?min-radius ?cm-width ?max-radius)
    (near-or-far-costmap ?ref-obj-pose ?min-radius ?max-radius ?costmap)
    ;;
    (near-costmap-gauss-std ?std)
    (instance-of gaussian-generator ?gaussian-generator-id)
    (costmap:costmap-add-function
     ?gaussian-generator-id
     (costmap:make-location-cost-function ?ref-obj-pose ?std)
     ?costmap))
  ;;
  (<- (far-from-costmap ?designator ?ref-obj-pose ?ref-obj-size ?ref-padding
                        ?for-obj-size ?for-padding ?costmap)
    (lisp-fun calculate-far-costmap-min-radius ?ref-obj-size ?for-obj-size
              ?ref-padding ?for-padding ?min-radius)
    (costmap-width-in-obj-size-percentage-far ?cm-width-perc)
    (lisp-fun calculate-costmap-width ?ref-obj-size ?for-obj-size ?cm-width-perc
              ?cm-width)
    (lisp-fun + ?min-radius ?cm-width ?max-radius)
    (near-or-far-costmap ?ref-obj-pose ?min-radius ?max-radius ?costmap))
  ;;
  (<- (costmap:desig-costmap ?designator ?costmap)
    (or
     (desig:desig-prop ?designator (:near ?ref-obj))
     (desig:desig-prop ?designator (:far-from ?ref-obj)))
    (costmap:costmap ?costmap)
    (desig:desig-location-prop ?ref-obj ?ref-obj-pose)
    ;;
    (-> (desig:loc-desig? ?ref-obj)
        (and (equal ?ref-obj-size 0.1)
             (equal ?ref-padding 0.1))
        (and (object-designator-name ?ref-obj ?ref-obj-name)
             (btr:bullet-world ?world)
             (btr:object ?world ?ref-obj-name)
             (object-size-without-handles ?world ?ref-obj-name ?ref-obj-size)
             (padding-size ?world ?ref-obj-name ?ref-padding)))
    ;;
    (-> (desig:desig-prop ?designator (:for ?for-obj))
        (and (object-designator-name ?for-obj ?for-obj-name)
             (btr:object ?world ?for-obj-name)
             (object-size-without-handles ?world ?for-obj-name ?for-obj-size)
             (padding-size ?world ?for-obj-name ?for-padding))
        (and (equal ?for-obj-size 0.1)
             (equal ?for-padding 0.1)))
    ;;
    (-> (desig:desig-prop ?designator (:near ?ref-obj))
        (near-costmap ?designator ?ref-obj-pose ?ref-obj-size ?ref-padding
                      ?for-obj-size ?for-padding ?costmap)
        (far-from-costmap ?designator ?ref-obj-pose ?ref-obj-size ?ref-padding
                          ?for-obj-size ?for-padding ?costmap)))


;;;;;;;;;;;;; LEFT-OF etc. for bullet objects or locations ;;;;;;;;;;;;;;;;;;
  ;; uses make-potential-field-cost-function to resolve the designator
  (<- (potential-field-costmap ?edge ?relation ?reference-pose ?supp-obj-pose ?costmap)
    (relation-axis-and-pred ?edge ?relation ?axis ?pred)
    (instance-of field-generator ?field-generator-id)
    (costmap:costmap-add-function
     ?field-generator-id
     (make-potential-field-cost-function ?axis ?reference-pose ?supp-obj-pose ?pred)
     ?costmap))
  ;;
  (<- (costmap:desig-costmap ?designator ?costmap)
    (or (desig:desig-prop ?designator (:left-of ?ref-designator))
        (desig:desig-prop ?designator (:right-of ?ref-designator))
        (desig:desig-prop ?designator (:in-front-of ?ref-designator))
        (desig:desig-prop ?designator (:behind ?ref-designator)))
    (desig:desig-prop ?designator (?relation ?ref-designator))
    (costmap:costmap ?costmap)
    (desig:desig-location-prop ?ref-designator ?reference-pose)
    (-> (desig:loc-desig? ?ref-designator)
        (and (equal ?edge :front)
             (lisp-fun cl-transforms:make-identity-pose ?supp-obj-pose))
        (and (object-designator-name ?ref-designator ?obj-name)
             (btr:bullet-world ?world)
             (btr:object ?world ?obj-name)
             (-> (supporting-rigid-body ?world ?obj-name ?supporting-rigid-body)
                 (and
                  ;; costmap to exclude everything which is outside of supp. object boundaries
                  (lisp-fun list ?supporting-rigid-body ?rigid-bodies)
                  (costmap:costmap-add-function
                   supporting-object
                   (make-aabbs-costmap-generator ?rigid-bodies)
                   ?costmap)
                  ;; The axis of the potential field depends on to which side of supporting object
                  ;; the reference object is the closest
                  ;; (btr:with-copied-world ?world
                  ;; (lisp-fun cl-bullet:name ?supporting-rigid-body ?supporting-object-name)
                  ;; (btr:assert (btr:object-pose ?world ?supporting-object-name
                  ;;                              ((0 0 0) (0 0 0 1))))
                  (lisp-fun cl-bullet:pose ?supporting-rigid-body ?supp-obj-pose)
                  (lisp-fun calculate-bb-dims ?supporting-rigid-body ?supp-obj-dims)
                  (lisp-fun get-closest-edge ?reference-pose ?supp-obj-pose ?supp-obj-dims ?edge))
                 (and (equal ?edge :front)
                      (lisp-fun cl-transforms:make-identity-pose ?supp-obj-pose)))))
    (potential-field-costmap ?edge ?relation ?reference-pose ?supp-obj-pose ?costmap))
  ;;
  ;; Disabled.
  ;; collision avoidance costmap for the spatial relations desigs
  ;; uses make-objects-bounding-box-costmap-generator
  (<- (collision-invert-costmap ?desig ?padding ?cm)
    (btr:bullet-world ?world)
    (findall ?obj (and (btr:item-type ?world ?name ?_)
                       (btr:%object ?world ?name ?obj)) ?objs)
    (costmap:costmap ?cm)
    (costmap:costmap-add-function
     collision
     (make-aabbs-costmap-generator ?objs :invert t :padding ?padding)
     ?cm))
  ;;
  ;; should be using make-aabb-costmap-generator I guess
  (<- (costmap:desig-costmap ?desig ?cm)
    (fail)
    (or
     (desig:desig-prop ?desig (:left-of ?_))
     (desig:desig-prop ?desig (:right-of ?_))
     (desig:desig-prop ?desig (:in-front-of ?_))
     (desig:desig-prop ?desig (:behind ?_))
     (desig:desig-prop ?desig (:far-from ?_))
     (desig:desig-prop ?desig (:near ?_)))
    (collision-costmap-padding-in-meters ?padding)
    (-> (desig:desig-prop ?desig (:for ?object))
        (and
         (object-designator-name ?object ?object-name)
         (btr:object ?world ?object-name)
         (object-size-without-handles ?world ?object-name ?obj-size)
         (lisp-fun / ?obj-size 2 ?obj-size/2)
         (lisp-fun + ?obj-size/2 ?padding ?overall-padding)
         (collision-invert-costmap ?desig ?overall-padding ?cm))
        (collision-invert-costmap ?desig ?padding ?cm)))

;;;;;;;;;;;;;; for TABLE-SETTING context ON (SLOTS) ;;;;;;;;;;;;;;;;;;;;;;;;;
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
    (costmap:costmap ?costmap)
    (costmap:costmap-add-function
     slot-generator
     (make-slot-cost-function ?supp-object ?paddings-list ?preferred-side
                              ?object-count ?max-slot-size ?min-slot-size
                              ?pos-dev-threshold)
     ?costmap))
  ;;
  (<- (costmap:desig-costmap ?designator ?costmap)
    (desig:desig-prop ?designator (:on ?_))
    (desig:desig-prop ?designator (:name ?supp-obj-name))
    (desig:desig-prop ?designator (:context :table-setting))
    (desig:desig-prop ?designator (:for ?for-object))
    (desig:desig-prop ?designator (:object-count ?object-count))
    (btr:bullet-world ?world)
    (object-designator-name ?for-object ?object-name)
    (btr:item-type ?world ?object-name ?object-type)
    (lisp-fun sem-map-desig:designator->semantic-map-objects
              ?designator ?supp-objects)
    (member ?supp-object ?supp-objects)
    (slot-costmap ?designator ?supp-object :table-setting ?object-type ?object-count
                  ?costmap))
  ;;
  (<- (costmap:desig-costmap ?designator ?costmap)
    (desig:desig-prop ?designator (:on ?_))
    (desig:desig-prop ?designator (:name ?supp-obj-name))
    (desig:desig-prop ?designator (:side ?relation))
    (member ?relation (:left :right :front :back))
    (costmap:costmap ?costmap)
    (lisp-fun sem-map-desig:designator->semantic-map-objects ?designator ?supp-objects)
    (member ?supp-object ?supp-objects)
    (relation-axis-and-pred ?relation :in-front-of ?axis ?sign)
    (costmap:costmap-add-function
     side-generator
     (make-side-costmap-generator ?supp-object ?axis ?sign)
     ?costmap))

;;;;;;;;;;;;;;;;;;;;;; HEIGHT GENERATORS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; height generator for locations ((on something) (name something) (for some-obj))
  (<- (costmap:desig-costmap ?designator ?costmap)
    (desig:desig-prop ?designator (:on ?_))
    (desig:desig-prop ?designator (:name ?_))
    (desig:desig-prop ?designator (:for ?for-object))
    (costmap:costmap ?costmap)
    (semantic-map-costmap::semantic-map-desig-objects ?designator ?sem-map-objects)
    (btr:bullet-world ?world)
    (object-designator-name ?for-object ?for-object-name)
    (btr:%object ?world ?for-object-name ?for-object-instance)
    (costmap:costmap-add-height-generator
     (make-object-on-object-bb-height-generator ?sem-map-objects ?for-object-instance)
     ?costmap))

  ;; TODO height generator is based only on semantic map (no stacking up allowed)
  (<- (costmap:desig-costmap ?designator ?costmap)
    (or
     (desig:desig-prop ?designator (:left-of ?ref-obj))
     (desig:desig-prop ?designator (:right-of ?ref-obj))
     (desig:desig-prop ?designator (:in-front-of ?ref-obj))
     (desig:desig-prop ?designator (:behind ?ref-obj))
     (desig:desig-prop ?designator (:far-from ?ref-obj))
     (desig:desig-prop ?designator (:near ?ref-obj)))
    (costmap:costmap ?costmap)
    (object-designator-name ?ref-obj ?ref-obj-name)
    (btr:bullet-world ?world)
    (btr:object ?world ?ref-obj-name)
    (-> (bagof ?z (and (supporting-rigid-body ?world ?ref-obj-name ?rigid-body)
                       (lisp-fun get-rigid-body-aabb-top-z ?rigid-body ?z))
               ?z-bag)
        (and (max ?z-bag ?highest-z)
             (-> (desig:desig-prop ?designator (:for ?for-obj))
                 (and (object-designator-name ?for-obj ?for-obj-name)
                      (btr:object ?world ?for-obj-name)
                      (btr:%object ?world ?for-obj-name ?for-object-instance)
                      (lisp-fun calculate-bb-dims ?for-object-instance ?dimensions)
                      (lisp-fun cl-transforms:z ?dimensions ?height)
                      (lisp-fun / ?height 2 ?offset)
                      (lisp-fun + ?highest-z ?offset ?resulting-z))
                 (equal ?resulting-z ?highest-z)))
        (and (desig:desig-location-prop ?ref-obj ?pose)
             (lisp-fun cl-transforms:origin ?pose ?pose-origin)
             (lisp-fun cl-transforms:z ?pose-origin ?resulting-z)))
    (costmap:costmap-add-height-generator
     (costmap:make-constant-height-function ?resulting-z)
     ?costmap)))
