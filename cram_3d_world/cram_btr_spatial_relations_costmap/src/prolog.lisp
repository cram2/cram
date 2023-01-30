
;;;
;;; Copyright (c) 2012, Gayane Kazhoyan <kazhoyan@in.tum.de>
;;;                     Amar Fayaz <amar@uni-bremen.de>
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

(in-package :btr-spatial-cm)

(defmethod costmap:costmap-generator-name->score ((name (eql 'environment-free-space))) 4)
(defmethod costmap:costmap-generator-name->score ((name (eql 'supporting-object))) 5)
(defmethod costmap:costmap-generator-name->score ((name (eql 'slot-generator))) 6)
(defmethod costmap:costmap-generator-name->score ((name (eql 'collision))) 20)
(defmethod costmap:costmap-generator-name->score ((name (eql 'on-bounding-box))) 5)

(defclass side-generator () ())
(defmethod costmap:costmap-generator-name->score ((name side-generator)) 3)

(defclass range-generator () ())
(defmethod costmap:costmap-generator-name->score ((name range-generator)) 2)

(defclass gaussian-generator () ())
(defmethod costmap:costmap-generator-name->score ((name gaussian-generator)) 7)

(defclass field-generator () ())
(defmethod costmap:costmap-generator-name->score ((name field-generator)) 15)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(def-fact-group location-desig-utils ()
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
      (object-designator-from-name-or-type ?object-to-check ?object-name)
      (btr:object ?world ?object-name)
      (btr:assert (btr:object-pose ?world ?object-name ?pose))
      (forall (btr:contact ?world ?object-name ?other-object-name)
              (not (btr:item-type ?world ?other-object-name ?_)))))

  (<- (object-designator-from-name-or-type ?object-designator ?object-name)
    (or (and (bound ?object-designator)
             (desig:obj-desig? ?object-designator))
        (and (not (bound ?object-designator))
             (lisp-fun btr-belief::unique-object-designators ?object-designators)
             (member ?one-desig-from-chain ?object-designators)
             (desig:current-designator ?one-desig-from-chain ?object-designator)))
    (or (desig:desig-prop ?object-designator (:name ?object-name))
        (and (desig:desig-prop ?object-designator (:type ?object-type))
             (btr:bullet-world ?w)
             (btr:item-type ?world ?object-name ?object-type)))))



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
  ;;;;;;;;;; REACHABILITY AND VISIBILITY LOCATION padded from environment ;;;;;;
  (<- (costmap:desig-costmap ?designator ?costmap)
    (or (rob-int:visibility-designator ?designator)
        (rob-int:reachability-designator ?designator))
    ;; make sure that the location is not on the robot itself
    ;; if it is, don't generate a costmap
    (-> (desig:desig-prop ?designator (:location ?some-location))
        (and (desig:current-designator ?some-location ?location)
             (not (man-int:location-always-reachable ?location)))
        (-> (desig:desig-prop ?designator (:object ?some-object))
            (and (desig:current-designator ?some-object ?object)
                 (-> (desig:desig-prop ?object (:location ?some-loc))
                     (not (man-int:location-always-reachable ?some-loc))
                     (true)))
            (true)))
    (costmap:costmap ?costmap)
    (rob-int:robot ?robot-name)
    (btr:bullet-world ?world)
    (lisp-fun btr:get-environment-object ?kitchen-object)
    (lisp-fun btr:rigid-bodies ?kitchen-object ?rigid-bodies)
    (costmap:costmap-padding ?robot-name ?padding)
    (costmap:costmap-add-function
     environment-free-space
     (make-object-bounding-box-costmap-generator
      ?rigid-bodies :invert t :padding ?padding)
     ?costmap)
    ;; Locations to see and to reach are on the floor, so we can use a
    ;; constant height of 0
    (costmap:costmap-add-cached-height-generator
     (costmap:make-constant-height-function 0.0)
     ?costmap))


  ;;;;;;;;; NEAR and FAR-FROM for bullet objects or locations ;;;;;;;;;;;
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
        (and (object-designator-from-name-or-type ?ref-obj ?ref-obj-name)
             (btr:bullet-world ?world)
             (btr:object ?world ?ref-obj-name)
             (object-size-without-handles ?world ?ref-obj-name ?ref-obj-size)
             (padding-size ?world ?ref-obj-name ?ref-padding)))
    ;;
    (-> (desig:desig-prop ?designator (:for ?for-obj))
        (and (object-designator-from-name-or-type ?for-obj ?for-obj-name)
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

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Orienation Generator ;;;;;;;;;;;;;;;;;;;;;;;;;;

  (<- (generate-orientations ?tag ?supporting-rigid-body ?reference-pose ?costmap)
    (-> (member ?tag (:support-aligned :random :axis-aligned :upside-down))
        (-> (equal ?tag :support-aligned)
            (and (lisp-fun btr:calculate-bb-dims ?supporting-rigid-body ?supp-obj-dims)
                 (lisp-fun get-rigid-body-aabb-top-z ?supporting-rigid-body ?supp-obj-z)
                 (lisp-fun cl-bullet:pose ?supporting-rigid-body ?supp-obj-pose)
                 (costmap:costmap-add-orientation-generator
                  (make-supporting-obj-aligned-orientations-generator
                   ?supp-obj-dims ?supp-obj-pose ?supp-obj-z ?reference-pose)
                  ?costmap))
            (and (costmap:costmap-add-orientation-generator
                  (make-z-orientations-generator ?tag)
                  ?costmap)))
        (format "WARNING: ORIENTATION tag in a location can only be:~%~
                 SUPPORT-ALIGNED, RANDOM, AXIS-ALIGNED or UPSIDE-DOWN.~%~
                 Ignoring ORIENTATION tag.")))

  ;;;;;;;;;;;;; LEFT-OF etc. for bullet objects or locations ;;;;;;;;;;;;;;;;;;
  ;; uses make-potential-field-cost-function to resolve the designator
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
        (and (object-designator-from-name-or-type ?ref-designator ?obj-name)
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
                  (lisp-fun btr:aabb ?supporting-rigid-body ?supp-obj-bb)
                  (lisp-fun cl-bullet:bounding-box-dimensions ?supp-obj-bb
                            ?supp-obj-dims)
                  (lisp-fun get-closest-edge ?reference-pose ?supp-obj-pose ?supp-obj-dims ?edge))
                 (and (equal ?edge :front)
                      (lisp-fun cl-transforms:make-identity-pose ?supp-obj-pose)))))
    ;; position
    (once (or (desig:desig-prop ?designator (:threshold ?threshold))
              (symbol-value *potential-field-threshold* ?threshold)))
    (relation-axis-and-pred ?edge ?relation ?axis ?pred)
    (instance-of field-generator ?field-generator-id)
    (costmap:costmap-add-function
     ?field-generator-id
     (make-potential-field-cost-function ?axis ?reference-pose ?supp-obj-pose ?pred ?threshold)
     ?costmap)
    ;; orientation
    (once (or (desig:desig-prop ?designator (:orientation ?orientation-type))
              (equal ?orientation-type :random)))
    (generate-orientations ?orientation-type ?supporting-rigid-body ?reference-pose ?costmap))

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
    (or (desig:desig-prop ?desig (:left-of ?_))
        (desig:desig-prop ?desig (:right-of ?_))
        (desig:desig-prop ?desig (:in-front-of ?_))
        (desig:desig-prop ?desig (:behind ?_))
        (desig:desig-prop ?desig (:far-from ?_))
        (desig:desig-prop ?desig (:near ?_)))
    (collision-costmap-padding-in-meters ?padding)
    (-> (desig:desig-prop ?desig (:for ?object))
        (and (object-designator-from-name-or-type ?object ?object-name)
             (btr:object ?world ?object-name)
             (object-size-without-handles ?world ?object-name ?obj-size)
             (lisp-fun / ?obj-size 2 ?obj-size/2)
             (lisp-fun + ?obj-size/2 ?padding ?overall-padding)
             (collision-invert-costmap ?desig ?overall-padding ?cm))
        (collision-invert-costmap ?desig ?padding ?cm)))
  ;;
  ;;;;;;;;;;;;;;;;;;;;;; height generator for spatial relations ;;;;;;;;;;;;;;;
  ;; TODO height generator is based only on environment object (no stacking up allowed)
  (<- (costmap:desig-costmap ?designator ?costmap)
    (or
     (desig:desig-prop ?designator (:left-of ?ref-obj))
     (desig:desig-prop ?designator (:right-of ?ref-obj))
     (desig:desig-prop ?designator (:in-front-of ?ref-obj))
     (desig:desig-prop ?designator (:behind ?ref-obj))
     (desig:desig-prop ?designator (:far-from ?ref-obj))
     (desig:desig-prop ?designator (:near ?ref-obj)))
    (costmap:costmap ?costmap)
    (object-designator-from-name-or-type ?ref-obj ?ref-obj-name)
    (btr:bullet-world ?world)
    (btr:object ?world ?ref-obj-name)
    (-> (bagof ?z (and (supporting-rigid-body ?world ?ref-obj-name ?rigid-body)
                       (lisp-fun get-rigid-body-aabb-top-z ?rigid-body ?z))
               ?z-bag)
        (and (max ?z-bag ?highest-z)
             (-> (desig:desig-prop ?designator (:for ?for-obj))
                 (and (object-designator-from-name-or-type ?for-obj ?for-obj-name)
                      (btr:object ?world ?for-obj-name)
                      (btr:%object ?world ?for-obj-name ?for-object-instance)
                      (lisp-fun btr:calculate-bb-dims ?for-object-instance
                                :initial-pose t
                                ?dimensions)
                      (lisp-fun cl-transforms:z ?dimensions ?height)
                      (lisp-fun / ?height 2 ?offset)
                      (lisp-fun + ?highest-z ?offset ?resulting-z))
                 (equal ?resulting-z ?highest-z)))
        (and (desig:desig-location-prop ?ref-obj ?pose)
             (lisp-fun cl-transforms:origin ?pose ?pose-origin)
             (lisp-fun cl-transforms:z ?pose-origin ?resulting-z)))
    (costmap:costmap-add-height-generator
     (costmap:make-constant-height-function ?resulting-z)
     ?costmap))

  ;;;;;;;;;;;;;;; spatial relation ON for item objects ;;;;;;;;;;;;;;;;;;;;;;
  (<- (costmap:desig-costmap ?designator ?costmap)
    (desig:desig-prop ?designator (:on ?object))
    (not (desig:desig-prop ?designator (:attachment ?_)))
    (not (desig:desig-prop ?designator (:attachments ?_)))
    (object-designator-from-name-or-type ?object ?object-instance-name)
    (btr:bullet-world ?world)
    (btr:item-type ?world ?object-instance-name ?_)
    (btr:%object ?world ?object-instance-name ?object-instance)
    (costmap:costmap ?costmap)
    (costmap:costmap-add-function
     on-bounding-box
     (make-object-bounding-box-costmap-generator ?object-instance)
     ?costmap)
    (costmap:costmap-add-cached-height-generator
     (make-object-bounding-box-height-generator ?object-instance :on)
     ?costmap))
  ;;
  ;;;;;;;;;;;;;;; spatial relation ON for environment objects ;;;;;;;;;;;;;;;;;;;;;;
  (<- (costmap:desig-costmap ?designator ?costmap)
    (desig:desig-prop ?designator (:on ?object))
    (btr-belief:object-designator-name ?object ?object-instance-name)
    (spec:property ?object (:urdf-name ?urdf-name))
    (spec:property ?object (:part-of ?environment-name))
    (btr:bullet-world ?world)
    (btr:%object ?world ?environment-name ?environment-object)
    (lisp-fun get-link-rigid-body ?environment-object ?urdf-name ?environment-link)
    (lisp-pred identity ?environment-link)
    (costmap:costmap ?costmap)
    ;; costmap
    (costmap:costmap-add-function
     on-bounding-box
     (make-object-bounding-box-costmap-generator ?environment-link)
     ?costmap)
    ;; height generator
    (once (or (and (desig:desig-prop ?designator (:for ?for-object))
                   (object-designator-from-name-or-type ?for-object ?for-object-name)
                   (btr:%object ?world ?for-object-name ?for-object-instance)
                   (costmap:costmap-add-height-generator
                    (make-object-on/in-object-bb-height-generator
                     ?environment-link ?for-object-instance :on)
                    ?costmap))
              (costmap:costmap-add-cached-height-generator
               (make-object-bounding-box-height-generator ?environment-link :on)
               ?costmap)))
    ;; orientation generator
    (once (or (desig:desig-prop ?designator (:orientation ?orientation-type))
              (equal ?orientation-type :random)))
    (generate-orientations ?orientation-type ?environment-link nil ?costmap))


  ;;;;;;;;;;;;;;; spatial relation ABOVE for environment objects ;;;;;;;;;;;;
  ;; ABOVE works similar to ON. Major differences are
  ;; 1. prismatic containers are opened before manipulation
  ;; 2. it requires a z-offset to be provided to calculate height
  (<- (costmap:desig-costmap ?designator ?costmap)
    (and (desig:desig-prop ?designator (:above ?object))
         (not (desig:desig-prop ?designator (:attachment ?_)))
         (not (desig:desig-prop ?designator (:attachments ?_))))
    (desig:desig-prop ?designator (?original-tag ?object))
    (spec:property ?object (:urdf-name ?urdf-name))
    (spec:property ?object (:part-of ?environment-name))
    (once (or (spec:property ?designator (:z-offset ?z-offset))
              (equal 0.0 ?z-offset)))
    (btr:bullet-world ?world)
    (btr:%object ?world ?environment-name ?environment-object)
    (height-calculation-body-or-tag ?environment-object ?object
                                    ?environment-link ?original-tag
                                    ?height-calculation-tag)
    ;; Checking whether the object is a container to determine where
    ;; z-offset is calculated from. At present, always calculates
    ;; from the top of the object (so a location inside the container
    ;; needs a negative z-offset). Uncomment the following if-else
    ;; if the calculation needs to be contextual.
    ;; (-> (man-int:object-type-subtype :container ?object-type)
    ;; (equal :in ?height-tag)
    (equal :on ?height-tag)

    (lisp-pred identity ?environment-link)
    (costmap:costmap ?costmap)
    (once (or (and (desig:desig-prop ?designator (:for ?for-object))
                   (object-designator-from-name-or-type ?for-object ?for-object-name)
                   (btr:%object ?world ?for-object-name ?for-object-instance)
                   (costmap:costmap-add-function
                    on-bounding-box
                    (make-object-in-object-bounding-box-costmap-generator
                     ?environment-link ?for-object-instance)
                    ?costmap)
                   (costmap:costmap-add-height-generator
                    (make-object-on/in-object-bb-height-generator
                     ?environment-link ?for-object-instance ?height-tag
                     ?z-offset)
                    ?costmap))
              (and (costmap:costmap-add-function
                    on-bounding-box
                    (make-object-bounding-box-costmap-generator
                     ?environment-link)
                    ?costmap)
                   (costmap:costmap-add-cached-height-generator
                    (make-object-bounding-box-height-generator
                     ?environment-link ?height-tag ?z-offset)
                    ?costmap))))
    (once (or (desig:desig-prop ?designator (:orientation ?orientation-type))
              (equal ?orientation-type :random)))
    (generate-orientations ?orientation-type ?environment-link nil ?costmap))

  ;;
  ;;;;;;;;;;;;;;; spatial relation ON/IN for environment objects ;;;;;;;;;;;;;;;;;;;
  ;; LEVEL relationship for container type locations
  (<- (level-rigid-body ?environment-object ?urdf-name ?level-relation ?invert
                        ?level-rigid-body)
    (or (member ?level-relation (:topmost :bottommost :middle))
        (lisp-pred typep ?level-relation integer))
    (lisp-fun get-level-links-in-container ?environment-object ?urdf-name
              ?child-levels)
    (lisp-fun choose-level ?environment-object ?child-levels ?level-relation
              :invert ?invert
              ?level-rigid-body))
  ;;
  (<- (height-calculation-body-or-tag ?environment-object ?in-object ?link-rigid-body
                                      ?original-tag ?height-calculation-tag)
    (desig:desig-prop ?in-object (:urdf-name ?urdf-name))
    ;; if level keyword is found find sublevels in bottom up order
    (-> (desig:desig-prop ?in-object (:level ?relation))
        (and (level-rigid-body
              ?environment-object ?urdf-name ?relation nil ?link-rigid-body)
             (lisp-pred identity ?link-rigid-body)
             (equal ?height-calculation-tag :on))
        ;; if level-invert keyword is found find sublevels in top down order
        (-> (desig:desig-prop ?in-object (:level-invert ?invert-relation))
            (and (level-rigid-body
                  ?environment-object ?urdf-name ?invert-relation t ?link-rigid-body)
                 (lisp-pred identity ?link-rigid-body)
                 (equal ?height-calculation-tag :on))
            ;; else send the rigid body of the original object itself
            (and (lisp-fun get-link-rigid-body ?environment-object ?urdf-name
                           ?link-rigid-body)
                 (lisp-pred identity ?link-rigid-body)
                 (equal ?height-calculation-tag ?original-tag)))))
  ;;
  ;; the costmap
  (<- (costmap:desig-costmap ?designator ?costmap)
    (or (and (desig:desig-prop ?designator (:in ?object))
             (spec:property ?object (:type ?object-type))
             (man-int:object-type-subtype :container ?object-type))
        (and (desig:desig-prop ?designator (:on ?object))
             (not (desig:desig-prop ?designator (:attachment ?_)))
             (not (desig:desig-prop ?designator (:attachments ?_)))))
    (desig:desig-prop ?designator (?original-tag ?object))
    (spec:property ?object (:urdf-name ?urdf-name))
    (spec:property ?object (:part-of ?environment-name))
    (btr:bullet-world ?world)
    (btr:%object ?world ?environment-name ?environment-object)
    (height-calculation-body-or-tag ?environment-object ?object
                                    ?environment-link ?original-tag
                                    ?height-calculation-tag)
    (lisp-pred identity ?environment-link)
    (costmap:costmap ?costmap)
    ;; differentiate between locations FOR an object and without the FOR
    (once (or (and (desig:desig-prop ?designator (:for ?for-object))
                   (object-designator-from-name-or-type ?for-object ?for-object-name)
                   (btr:%object ?world ?for-object-name ?for-object-instance)
                   (costmap:costmap-add-function
                    on-bounding-box
                    (make-object-in-object-bounding-box-costmap-generator
                     ?environment-link ?for-object-instance)
                    ?costmap)
                   (costmap:costmap-add-height-generator
                    (make-object-on/in-object-bb-height-generator
                     ?environment-link ?for-object-instance ?height-calculation-tag)
                    ?costmap))
              (and (costmap:costmap-add-function
                    on-bounding-box
                    (make-object-bounding-box-costmap-generator
                     ?environment-link)
                    ?costmap)
                   (costmap:costmap-add-cached-height-generator
                    (make-object-bounding-box-height-generator
                     ?environment-link ?height-calculation-tag)
                    ?costmap))))
    (once (or (desig:desig-prop ?designator (:orientation ?orientation-type))
              (equal ?orientation-type :random)))
    (generate-orientations ?orientation-type ?environment-link nil ?costmap))
  ;;
  ;;;;;;;;;;;;;; for TABLE-SETTING context ON (SLOTS) ;;;;;;;;;;;;;;;;;;;;;;;;;
  ;; '((on counter-top) (name kitchen-island)
  ;;   (context :table-setting) (for plate-1) (object-count 4))
  ;; uses make-slot-cost-function
  (<- (slot-costmap ?designator ?supp-object ?supp-object-name ?context ?object-type
                    ?object-count
                    ?costmap)
    (paddings-list ?supp-object-name ?context ?paddings-list)
    (preferred-supporting-object-side ?supp-object-name ?context ?preferred-side)
    (preferred-supporting-object-axis ?supp-object-name ?context ?preferred-axis)
    (max-slot-size ?object-type ?context ?max-slot-size)
    (min-slot-size ?object-type ?context ?min-slot-size)
    (position-deviation-threshold ?object-type ?context ?pos-dev-threshold)
    ;;
    (costmap:costmap ?costmap)
    (costmap:costmap-add-function
     slot-generator
     (make-slot-cost-function ?supp-object ?paddings-list
                              ?preferred-side ?preferred-axis
                              ?object-count ?max-slot-size ?min-slot-size
                              ?pos-dev-threshold)
     ?costmap))
  ;;
  (<- (costmap:desig-costmap ?designator ?costmap)
    (desig:desig-prop ?designator (:context :table-setting))
    (desig:desig-prop ?designator (:on ?on-object))
    (desig:desig-prop ?designator (:for ?for-object))
    (desig:desig-prop ?designator (:object-count ?object-count))
    (desig:desig-prop ?on-object (:urdf-name ?urdf-name))
    (desig:desig-prop ?on-object (:part-of ?environment-name))
    (btr:bullet-world ?world)
    (btr:%object ?world ?environment-name ?environment-object)
    (lisp-fun get-link-rigid-body ?environment-object ?urdf-name ?environment-link)
    (lisp-pred identity ?environment-link)
    (object-designator-from-name-or-type ?for-object ?object-name)
    (btr:item-type ?world ?object-name ?object-type)
    (slot-costmap ?designator ?environment-link ?urdf-name :table-setting ?object-type
                  ?object-count
                  ?costmap))
  ;;
  ;;;;;;;;;;;;;;;;;;;;; SIDE and RANGE relations for ON, IN or ABOVE ;;;;;;;;;
  (<- (rigid-body-of-location-designator-reference-object ?designator ?rigid-body)
    (once
     (or
      (desig:desig-prop ?designator (:on ?object-designator))
      (desig:desig-prop ?designator (:in ?object-designator))
      (desig:desig-prop ?designator (:above ?object-designator))))
    (btr:bullet-world ?world)
    (once
     (or
      (and (desig:desig-prop ?object-designator (:level ?level))
           (desig:desig-prop ?object-designator (:urdf-name ?urdf-name))
           (desig:desig-prop ?object-designator (:part-of ?environment-name))
           (btr:%object ?world ?environment-name ?environment-object)
           (level-rigid-body ?environment-object ?urdf-name ?level nil ?rigid-body)
           (lisp-pred identity ?rigid-body))
      (and (desig:desig-prop ?object-designator (:level-invert ?level))
           (desig:desig-prop ?object-designator (:urdf-name ?urdf-name))
           (desig:desig-prop ?object-designator (:part-of ?environment-name))
           (btr:%object ?world ?environment-name ?environment-object)
           (level-rigid-body ?environment-object ?urdf-name ?level t ?rigid-body)
           (lisp-pred identity ?rigid-body))
      (and (desig:desig-prop ?object-designator (:urdf-name ?urdf-name))
           (desig:desig-prop ?object-designator (:part-of ?environment-name))
           (btr:%object ?world ?environment-name ?environment-object)
           (lisp-fun get-link-rigid-body ?environment-object ?urdf-name ?rigid-body)
           (lisp-pred identity ?rigid-body))
      (and (object-designator-from-name-or-type ?object-designator ?object-instance-name)
           (btr:item-type ?world ?object-instance-name ?_)
           (btr:%object ?world ?object-instance-name ?rigid-body)
           (lisp-pred identity ?rigid-body)))))
  ;;
  ;; SIDE relations
  (<- (costmap:desig-costmap ?designator ?costmap)
    (desig:desig-prop ?designator (:side ?relations))
    (-> (lisp-type ?relations list)
        (equal ?relations (?relation1 ?relation2))
        (equal (?relations nil) (?relation1 ?relation2)))
    (member ?relation1 (:left :right :front :back))
    (member ?relation2 (:left :right :front :back nil))
    (not (desig:desig-prop ?designator (:attachment ?_)))
    (not (desig:desig-prop ?designator (:attachments ?_)))
    (rigid-body-of-location-designator-reference-object ?designator ?rigid-body)
    (lisp-fun btr:pose ?rigid-body ?object-pose)
    (costmap:costmap ?costmap)
    (relation-axis-and-pred ?relation1 :in-front-of ?axis ?sign)
    (instance-of side-generator ?side-generator-id)
    (costmap:costmap-add-function
     ?side-generator-id
     (make-side-costmap-generator ?rigid-body ?axis ?sign)
     ?costmap)
    (-> (lisp-pred identity ?relation2)
        (and (relation-axis-and-pred ?relation2 :in-front-of ?axis2 ?sign2)
             (instance-of side-generator ?side-generator-id2)
             (costmap:costmap-add-function
              ?side-generator-id2
              (make-side-costmap-generator ?rigid-body ?axis2 ?sign2)
              ?costmap))
        (true)))
  ;; RANGE relation
  (<- (costmap:desig-costmap ?designator ?costmap)
    (desig:desig-prop ?designator (:range ?range))
    (not (desig:desig-prop ?designator (:attachment ?_)))
    (not (desig:desig-prop ?designator (:attachments ?_)))
    (rigid-body-of-location-designator-reference-object ?designator ?rigid-body)
    (lisp-fun btr:pose ?rigid-body ?object-pose)
    (costmap:costmap ?costmap)
    (instance-of range-generator ?range-generator-id)
    (costmap:costmap-add-function
     ?range-generator-id
     (costmap:make-range-cost-function ?object-pose ?range :invert nil)
     ?costmap))
  ;; RANGE-INVERT relation
  (<- (costmap:desig-costmap ?designator ?costmap)
    (desig:desig-prop ?designator (:range-invert ?range-invert))
    (not (desig:desig-prop ?designator (:attachment ?_)))
    (not (desig:desig-prop ?designator (:attachments ?_)))
    (rigid-body-of-location-designator-reference-object ?designator ?rigid-body)
    (lisp-fun btr:pose ?rigid-body ?object-pose)
    (costmap:costmap ?costmap)
    (instance-of range-generator ?range-invert-generator-id)
    (costmap:costmap-add-function
     ?range-invert-generator-id
     (costmap:make-range-cost-function ?object-pose ?range-invert :invert t)
     ?costmap)))


