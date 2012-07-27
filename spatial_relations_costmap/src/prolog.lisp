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

(in-package :spatial-relations-costmap)

(defmethod costmap-generator-name->score ((name (eql 'supporting-object))) 2)
(defmethod costmap-generator-name->score ((name (eql 'left-of-axis))) 8)
(defmethod costmap-generator-name->score ((name (eql 'collision))) 6)

(defclass padding-generator () ())
(defmethod costmap-generator-name->score ((name padding-generator)) 3)

(defclass gaussian-generator () ())
(defmethod costmap-generator-name->score ((name gaussian-generator)) 6)

(defclass field-generator () ())
(defmethod costmap-generator-name->score ((name field-generator)) 7)


(def-fact-group spatial-relations-costmap (desig-costmap)
  (<- (near-costmap ?designator ?ref-obj-pose ?ref-obj-size ?ref-padding
                    ?for-obj-size ?for-padding ?costmap)
    (costmap ?costmap)
    ;;
    (gauss-std ?std)
    (instance-of gaussian-generator ?gaussian-generator-id)
    (costmap-add-function
     ?gaussian-generator-id
     (make-location-cost-function ?ref-obj-pose ?std)
     ?costmap)
    ;;
    ;; (format "~a~%~a~%~a~%~a~%" ?ref-obj-size ?for-obj-size ?ref-padding ?for-padding)
    (lisp-fun calculate-near-costmap-radius ?ref-obj-size ?for-obj-size
              ?ref-padding ?for-padding ?cm-radius)
    ;; (format "radius = ~a~%" ?cm-radius)
    (costmap-width-in-obj-size-percentage ?cm-width-perc)
    (lisp-fun calculate-costmap-width/2 ?ref-obj-size ?for-obj-size ?cm-width-perc
              ?cm-width/2)
    ;;
    (lisp-fun + ?cm-radius ?cm-width/2 ?max)
    (instance-of padding-generator ?padding-generator-id-1)
    (costmap-add-function
     ?padding-generator-id-1
     (make-range-cost-function ?ref-obj-pose ?max)
     ?costmap)
    ;;
    (lisp-fun - ?cm-radius ?cm-width/2 ?min)
    (instance-of padding-generator ?padding-generator-id-2)
    (costmap-add-function
     ?padding-generator-id-2
     (make-range-cost-function ?ref-obj-pose ?min :invert t)
     ?costmap))

  (<- (far-from-costmap ?designator ?ref-obj-pose ?ref-obj-size ?ref-padding
                        ?for-obj-size ?for-padding ?costmap)
    (costmap ?costmap)
    ;;
    (lisp-fun calculate-far-costmap-radius ?ref-obj-size ?for-obj-size
              ?ref-padding ?for-padding ?cm-radius)
    (costmap-width-in-obj-size-percentage ?cm-width-perc)
    (lisp-fun calculate-costmap-width/2 ?ref-obj-size ?for-obj-size ?cm-width-perc
              ?cm-width/2)
    ;;
    (lisp-fun + ?cm-radius ?cm-width/2 ?max)
    (instance-of padding-generator ?padding-generator-id-1)
    (costmap-add-function
     ?padding-generator-id-1
     (make-range-cost-function ?ref-obj-pose ?max)
     ?costmap)
    ;;
    (lisp-fun - ?cm-radius ?cm-width/2 ?min)
    (instance-of padding-generator ?padding-generator-id-2)
    (costmap-add-function
     ?padding-generator-id-2
     (make-range-cost-function ?ref-obj-pose ?min :invert t)
     ?costmap))

  ;; TODO pred is too long!
  ;; uses make-potential-field-cost-function to resolve the designator
  ;; TODO fix such that would work without FOR!!!
  (<- (potential-field-costmap ?designator ?object ?relation ?costmap)
    (desig-location-prop ?object ?reference-pose)
    (lisp-fun get-y-of-pose ?reference-pose ?y-of-pose)
    (lisp-fun get-x-of-pose ?reference-pose ?x-of-pose)
    (costmap ?costmap)
    ;;
    (btr:bullet-world ?world)
    (supported-by-link-obj ?world ?object ?link-obj)
    (lisp-fun sem-map-utils:pose ?link-obj ?supp-obj-pose)
    (lisp-fun sem-map-utils:dimensions ?link-obj ?supp-obj-dims)
    ;; costmap to exclude everything which is outside of supp. object boundaries
    (lisp-fun list ?link-obj ?link-obj-list)
    ;; (costmap-add-function
    ;;  supporting-object
    ;;  (semantic-map-costmap::make-semantic-map-costmap ?link-obj-list)
    ;;  ?costmap)
    ;; the actual potential field costmap
    ;; The axis of the potential field depends on to which side of supporting object
    ;; the reference object is the closest
    (lisp-fun get-closest-edge ?reference-pose ?supp-obj-pose ?supp-obj-dims ?edge)
    (relation-axis-and-pred ?edge ?relation ?axis ?pred)
    (instance-of field-generator ?field-generator-id)
    (costmap-add-function
     ?field-generator-id
     (make-potential-field-cost-function ?axis ?x-of-pose ?y-of-pose
                                         ?supp-obj-pose ?pred)
     ?costmap)
    ;; collision costmap
    ;; (findall ?obj (and (household-object-type ?world ?name ?_)
    ;;                    (%object ?world ?name ?obj)) ?objs)
    ;; (costmap-padding-in-meters ?padding)
    ;; (desig-prop ?designator (for ?object))
    ;; (object-instance-name ?object ?obj-name)
    ;; (object ?world ?obj-name)
    ;; (object-size-without-handles ?world ?obj-name ?obj-size)
    ;; (lisp-fun / ?obj-size 2 ?obj-size/2)
    ;; (lisp-fun + ?obj-size/2 ?padding ?overall-padding)
    ;; (format "hop~%")
    ;; (costmap-add-function
    ;;  collision
    ;;  (make-objects-bounding-box-costmap-generator ?objs :invert t :padding ?padding)
    ;;  ?costmap)
    ;; the height generator
    ;;(semantic-map-costmap::semantic-map-objects ?objects)
    (costmap-add-height-generator
     (semantic-map-costmap::make-semantic-map-height-function ?link-obj-list :on)
                                        ;;?objects :on)
     ?costmap)
    ;; orientation generator
    (orientation-costmap ?designator ?object ?costmap))
  
  ;; uses make-objects-bounding-box-costmap-generator
  ;; (<- (collision-invert-costmap ?desig ?objs ?padding ?cm) 
  ;;   (costmap ?cm)    
  ;;   (costmap-add-function
  ;;    collision
  ;;    (make-objects-bounding-box-costmap-generator ?objs :invert t :padding ?padding)
  ;;    ?cm))

  ;; uses make-orientation-generator with supporting-obj-alligned-direction
  (<- (orientation-costmap ?designator ?ref-obj-name ?costmap)
    (desig-prop ?designator (for ?object))
    (object-instance-name ?object ?obj-name)
    (bullet-world ?world)
    (object ?world ?obj-name)
    (costmap ?costmap)
    (-> (orientation-matters ?obj-name)
        (and
         (format "~a: orientation matters!~%" ?obj-name)
         (supported-by-link-obj ?world ?ref-obj-name ?link-obj)
         (lisp-fun sem-map-utils:pose ?link-obj ?supp-obj-pose)
         (lisp-fun sem-map-utils:dimensions ?link-obj ?supp-obj-dims)
         (desig-location-prop ?ref-obj-name ?ref-obj-pose)
         (lisp-fun alexandria:rcurry supporting-obj-alligned-direction
                   ?supp-obj-pose ?supp-obj-dims
                   :ref-obj-dependent t
                   :ref-obj-pose ?ref-obj-pose
                   ?orientation-function)
         (orientation-samples ?sample-num)
         (orientation-samples-step ?samples-step) 
         (costmap-add-orientation-generator
          (make-orientation-generator ?orientation-function
                                      :samples ?sample-num
                                      :sample-step ?samples-step)
          ?costmap))
        (true)))

  
  ;; left-of for bullet objects using potential field cost-function
  (<- (desig-costmap ?designator ?costmap)
    (desig-prop ?designator (left-of ?object))
    (object-instance-name ?object ?obj-name)
    (btr:bullet-world ?world)
    (btr:object ?world ?obj-name)
    (potential-field-costmap ?designator ?obj-name left-of ?costmap))

  ;; right-of for bullet objects using potential field cost-function
  (<- (desig-costmap ?designator ?costmap)
    (desig-prop ?designator (right-of ?object))
    (object-instance-name ?object ?obj-name)
    (btr:bullet-world ?world)
    (btr:object ?world ?obj-name)
    (potential-field-costmap ?designator ?obj-name right-of ?costmap))

  ;; in-front-of for bullet objects using potential field cost-function
  (<- (desig-costmap ?designator ?costmap)
    (desig-prop ?designator (in-front-of ?object))
    (object-instance-name ?object ?obj-name)
    (btr:bullet-world ?world)
    (btr:object ?world ?obj-name)
    (potential-field-costmap ?designator ?obj-name in-front-of ?costmap))

  ;; behind for bullet objects using potential field cost-function
  (<- (desig-costmap ?designator ?costmap)
    (desig-prop ?designator (behind ?object))
    (object-instance-name ?object ?obj-name)
    (btr:bullet-world ?world)
    (btr:object ?world ?obj-name)
    (potential-field-costmap ?designator ?obj-name behind ?costmap))

  ;; near and far-from for bullet objects
  (<- (desig-costmap ?designator ?costmap)
    (or
     (desig-prop ?designator (near ?ref-obj))
     (desig-prop ?designator (far-from ?ref-obj)))
    (object-instance-name ?ref-obj ?ref-obj-name)
    (btr:bullet-world ?world)
    (btr:object ?world ?ref-obj-name)
    ;;
    (desig-location-prop ?ref-obj-name ?ref-obj-pose)
    (object-size-without-handles ?world ?ref-obj-name ?ref-obj-size)
    (padding-size ?world ?ref-obj-name ?ref-padding)
    ;;
    (desig-prop ?designator (for ?for-obj))
    (object-instance-name ?for-obj ?for-obj-name)
    (object ?world ?for-obj-name)
    (object-size-without-handles ?world ?for-obj-name ?for-obj-size)
    (padding-size ?world ?for-obj-name ?for-padding)
    ;;
    (-> (desig-prop ?designator (near ?ref-obj))
        (near-costmap ?designator ?ref-obj-pose ?ref-obj-size ?ref-padding
                      ?for-obj-size ?for-padding ?costmap)
        (far-from-costmap ?designator ?ref-obj-pose ?ref-obj-size ?ref-padding
                      ?for-obj-size ?for-padding ?costmap)))
  
  ;; ;; collision avoidance costmap for the spatial relations desigs
  ;; (<- (desig-costmap ?desig ?cm)
  ;;   (or
  ;;    (desig-prop ?desig (left-of ?_))
  ;;    (desig-prop ?desig (right-of ?_))
  ;;    (desig-prop ?desig (in-front-of ?_))
  ;;    (desig-prop ?desig (behind ?_))
  ;;    (desig-prop ?desig (far-from ?_))
  ;;    (desig-prop ?desig (near ?_)))
  ;;   (bullet-world ?world)
  ;;   (findall ?obj (and (household-object-type ?world ?name ?_)
  ;;                      (%object ?world ?name ?obj)) ?objs)
  ;;   ;;
  ;;   (costmap-padding-in-meters ?padding)
  ;;   (desig-prop ?designator (for ?obj))
  ;;   (object-instance-name ?obj ?obj-name)
  ;;   (object ?world ?obj-name)
  ;;   (object-size-without-handles ?world ?obj-name ?obj-size)
  ;;   (lisp-fun / ?obj-size 2 ?obj-size/2)
  ;;   (lisp-fun + ?obj-size/2 ?padding ?overall-padding)
  ;;   (collision-invert-costmap ?desig ?objs ?overall-padding ?cm)))
)

(def-fact-group relations-lookup-table ()
  (<- (relation-axis-and-pred :front left-of :Y >))
  (<- (relation-axis-and-pred :front right-of :Y <))
  (<- (relation-axis-and-pred :front behind :X >))
  (<- (relation-axis-and-pred :front in-front-of :X <))
  (<- (relation-axis-and-pred :back left-of :Y <))
  (<- (relation-axis-and-pred :back right-of :Y >))
  (<- (relation-axis-and-pred :back behind :X <))
  (<- (relation-axis-and-pred :back in-front-of :X >))
  (<- (relation-axis-and-pred :right left-of :X <))
  (<- (relation-axis-and-pred :right right-of :X >))
  (<- (relation-axis-and-pred :right behind :Y >))
  (<- (relation-axis-and-pred :right in-front-of :Y <))
  (<- (relation-axis-and-pred :left left-of :X >))
  (<- (relation-axis-and-pred :left right-of :X <))
  (<- (relation-axis-and-pred :left behind :Y <))
  (<- (relation-axis-and-pred :left in-front-of :Y >)))


(def-fact-group location-desig-utils ()
  (<- (object-instance-name ?name ?name)
    (lisp-type ?name symbol))
  ;;
  (<- (object-instance-name ?designator ?name)
    (obj-desig? ?designator)
    (lisp-fun cram-environment-representation::get-designator-object-name
              ?designator ?name))

  
  ;; returns diameter or something similar in meters
  (<- (object-size-without-handles ?world ?obj-name ?size)
    (object-shape ?world ?obj-name ?shape)
    (%object ?world ?obj-name ?obj)
    (lisp-fun aabb ?obj ?aabb)
    (lisp-fun bt:bounding-box-dimensions ?aabb ?dims)
    ;; (format "dimensions of ~a = ~a~%" ?obj-name ?dims)
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


  (<- (supported-by-link-obj ?world ?obj-name ?link-obj)
    ;; (format "support for ~a?~%" ?obj-name)
    (supported-by ?world ?obj-name ?supp-obj-name ?supp-obj-link-name)
    (format "~a has support: ~a~%" ?obj-name ?supp-obj-link-name)
    (%object ?world ?supp-obj-name ?supp-obj)
    (lisp-fun get-sem-map-part ?supp-obj ?supp-obj-link-name ?link-obj)
    (lisp-pred identity ?link-obj)))


