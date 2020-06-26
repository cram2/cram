;;;
;;; Copyright (c) 2018, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(def-fact-group object-designators (desig:desig-location-prop desig:location-grounding)

  (<- (desig:desig-location-prop ?desig ?loc)
    (desig:obj-desig? ?desig)
    (lisp-fun get-object-pose-in-map ?desig ?loc)
    (lisp-pred identity ?loc))

  (<- (desig:location-grounding ?designator ?pose-stamped)
    (desig:loc-desig? ?designator)
    (desig:desig-prop ?designator (:of ?object-designator))
    (lisp-type ?object-designator desig:object-designator)
    (desig:current-designator ?object-designator ?current-object-designator)
    (desig:desig-location-prop ?current-object-designator ?pose-stamped)))

(def-fact-group object-type-hierarchy (object-type-direct-subtype)
  (<- (object-type-direct-subtype ?type ?direct-subtype)
    (fail))

  (<- (object-type-subtype ?type ?type))

  (<- (object-type-subtype ?type ?subtype)
    (object-type-direct-subtype ?type ?type-s-child)
    (object-type-subtype ?type-s-child ?subtype)))


(def-fact-group object-knowledge (object-rotationally-symmetric
                                  orientation-matters
                                  unidirectional-attachment)

  ;; TODO: specify rotational symmetry axis
  (<- (object-rotationally-symmetric ?object-type)
    (fail))

  ;; The predicate ORIENTATION-MATTERS holds for all objects where the
  ;; orientation really matters when putting down the object. E.g. for
  ;; knives, forks, etc, the orientation is important while for plates
  ;; the orientation doesn't matter at all.
  (<- (orientation-matters ?object-type-symbol)
      (fail))

  ;; The predicate UNIDIRECTIONAL-ATTACHMENTS holds attachments which
  ;; are only used for unidirectional/loose attachments.
  ;; For example, if an object is standing on a tray,
  ;; the object is attached to the tray but the tray is not attached
  ;; to the object, such that when you move the object the tray would not follow.
  (<- (unidirectional-attachment ?attachment-type)
    (fail)))


(def-fact-group manipulation-knowledge (robot-free-hand)

  (<- (robot-free-hand ?robot ?arm)
    (rob-int:robot ?robot)
    (rob-int:arm ?robot ?arm)
    (not (cpoe:object-in-hand ?_ ?arm)))

  (<- (joint-state-for-arm-config ?robot ?config ?arm ?joint-state)
    (once
     (or (-> (and (equal ?config :park)
                  (cpoe:object-in-hand ?object-designator ?arm ?grasp))
             (and (desig:current-designator ?object-designator ?current-object-desig)
                  (spec:property ?current-object-desig (:type ?object-type))
                  (lisp-fun get-object-type-carry-config ?object-type ?grasp
                            ?carry-config)
                  (-> (lisp-pred identity ?carry-config)
                      (rob-int:robot-joint-states ?robot :arm ?arm ?carry-config
                                                  ?joint-state)
                      (rob-int:robot-joint-states ?robot :arm ?arm :carry
                                                  ?joint-state)))
             (rob-int:robot-joint-states ?robot :arm ?arm ?config ?joint-state))
         (equal ?joint-state NIL)))))



(defun symbol-to-prolog-rule (the-symbol &rest parameters)
  (let ((interned-symbol (find-symbol (string-upcase the-symbol))))
    (if interned-symbol
        (cram-utilities:var-value
         '?result
         (car (prolog `(,interned-symbol ,@parameters ?result))))
        the-symbol)))

;; todo(@gaya): ugliest piece of code ever...
;; spent 2 years cleaning up cram, now spend another 2 messing it up again...
(def-fact-group robot-parts-location (desig:desig-location-prop)
  (<- (desig:desig-location-prop ?object-designator ?pose-stamped)
    (desig:obj-desig? ?object-designator)
    (desig:desig-prop ?object-designator (:part-of ?robot))
    (rob-int:robot ?robot)
    (desig:desig-prop ?object-designator (:link ?link))
    (-> (desig:desig-prop ?object-designator (:which-link ?params))
        (lisp-fun symbol-to-prolog-rule ?link ?robot-name ?params ?link-name)
        (lisp-fun symbol-to-prolog-rule ?link ?robot-name ?link-name))
    (lisp-fun cram-tf:frame-to-pose-in-fixed-frame ?link-name ?pose-stamped)))


;; TODO: move to pick and place heuristics package, when it is created
(def-fact-group location-designator-stuff (desig:location-grounding)

  ;; Resolving (a location
  ;;              (for ?object)
  ;;              (on ?other-object)
  ;;              (attachment object-to-other-object))
  (<- (desig:location-grounding ?location-designator ?pose-stamped)
    (desig:current-designator ?location-designator ?current-loc-desig)
    (desig:desig-prop ?current-loc-desig (:for ?object-designator))
    (desig:desig-prop ?current-loc-desig (:on ?other-object-designator))
    (-> (desig:desig-prop ?current-loc-desig (:attachments ?attachments))
        (member ?attachment-type ?attachments)
        (desig:desig-prop  ?current-loc-desig (:attachment ?attachment-type)))
    (desig:current-designator ?object-designator ?current-object-designator)
    (spec:property ?current-object-designator (:type ?object-type))
    (spec:property ?current-object-designator (:name ?object-name))
    (desig:current-designator ?other-object-designator ?current-other-obj-desig)
    (spec:property ?current-other-obj-desig (:type ?other-object-type))
    ;;
    (-> (spec:property ?current-other-obj-desig (:urdf-name ?other-object-name))
        (and (lisp-fun roslisp-utilities:rosify-underscores-lisp-name
                       ?other-object-name ?link-name)
             (symbol-value cram-tf:*fixed-frame* ?parent-frame)
             (lisp-fun cram-tf:frame-to-transform-in-fixed-frame
                       ?link-name ?parent-frame
                       ?other-object-transform))
        (and (spec:property ?current-other-obj-desig (:name ?other-object-name))
             (-> (cpoe:object-in-hand ?current-other-obj-desig ?hand ?grasp ?link)
                 (and (rob-int:robot ?robot)
                      (-> (rob-int:end-effector-link ?robot ?arm ?link)
                          (and (rob-int:robot-tool-frame ?robot ?arm ?tool-frame)
                               (format "REFERENCING THIS GUY~%~%~%~%~%")
                               (symbol-value cram-tf:*fixed-frame* ?parent-frame)
                               (lisp-fun cram-tf:frame-to-transform-in-fixed-frame
                                         ?tool-frame ?parent-frame
                                         ?map-t-gripper)
                               (lisp-fun get-object-type-to-gripper-transform
                                         ?other-object-type ?other-object-name
                                         ?arm ?grasp
                                         ?object-t-std-gripper)
                               (lisp-fun cram-tf:transform-stamped-inv
                                         ?object-t-std-gripper
                                         ?std-gripper-t-object)
                               (rob-int:standard-to-particular-gripper-transform
                                ?robot
                                ?std-gripper-t-gripper-not-stamped)
                               (lisp-fun
                                cl-transforms-stamped:transform->transform-stamped
                                ?tool-frame ?tool-frame 0.0
                                ?std-gripper-t-gripper-not-stamped
                                ?std-gripper-t-gripper)
                               (lisp-fun cram-tf:transform-stamped-inv
                                         ?std-gripper-t-gripper
                                         ?gripper-t-std-gripper)
                               (lisp-fun cram-tf:apply-transform
                                         ?gripper-t-std-gripper ?std-gripper-t-object
                                         ?gripper-t-object)
                               (lisp-fun cram-tf:apply-transform
                                         ?map-t-gripper ?gripper-t-object
                                         ?other-object-transform))
                          (lisp-fun get-object-transform-in-map
                                    ?current-other-obj-desig
                                    ?other-object-transform)))
                 (lisp-fun get-object-transform-in-map ?current-other-obj-desig
                           ?other-object-transform))))
    ;;
    (lisp-fun get-object-placement-transform
              ?object-name ?object-type
              ?other-object-name ?other-object-type ?other-object-transform
              ?attachment-type
              ?attachment-transform-in-map)
    (lisp-fun cram-tf:strip-transform-stamped ?attachment-transform-in-map
              ?pose-stamped))

  ;; Resolving (a location
  ;;              (reachable-for pr2)
  ;;              (location (on/in (an object
  ;;                                   (type robot
  ;; First, a helper predicate to discern such a location
  ;; A location on/in the robot is always reachable
  (<- (location-always-reachable ?location-designator)
    (desig:loc-desig? ?location-designator)
    (desig:current-designator ?location-designator ?current-location-designator)
    (or (desig:desig-prop ?current-location-designator (:on ?object-designator))
        (desig:desig-prop ?current-location-designator (:in ?object-designator)))
    (desig:current-designator ?object-designator ?current-object-designator)
    (desig:desig-prop ?current-object-designator (:type :robot)))
  ;; Also, a location on an item that is held by the robot is also always reachable
  (<- (location-always-reachable ?location-designator)
    (desig:loc-desig? ?location-designator)
    (desig:current-designator ?location-designator ?current-location-designator)
    (desig:desig-prop ?current-location-designator (:on ?object-designator))
    (desig:current-designator ?object-designator ?current-object-designator)
    (cpoe:object-in-hand ?current-object-designator))
  ;; Also, a location of an object at a location that is always reachable
  ;; is also always reachable
  (<- (location-always-reachable ?location-designator)
    (desig:loc-desig? ?location-designator)
    (desig:current-designator ?location-designator ?current-location-designator)
    (spec:property ?current-location-designator (:of ?object-designator))
    (desig:current-designator ?object-designator ?current-object-designator)
    (spec:property ?current-object-designator (:location ?object-location))
    (man-int:location-always-reachable ?object-location))
  ;; TODO: a location attached to an object in hand is also always reachable

  (<- (object-is-a-robot ?some-object-designator)
    (desig:current-designator ?some-object-designator ?object-designator)
    (or (desig:desig-prop ?object-designator (:type :robot))
        (desig:desig-prop ?object-designator (:type :environment))
        (and (rob-int:robot ?robot)
             (desig:desig-prop ?object-designator (:part-of ?robot)))
        (desig:desig-prop ?object-designator (:part-of :environment))))

  ;; Now the actual location grounding for reachability and visibility
  (<- (desig:location-grounding ?location-designator ?pose-stamped)
    (desig:current-designator ?location-designator ?current-location-designator)
    (or (rob-int:reachability-designator ?current-location-designator)
        (rob-int:visibility-designator ?current-location-designator))
    (or (and (desig:desig-prop ?current-location-designator (:object ?some-object))
             (desig:current-designator ?some-object ?object)
             (lisp-fun man-int:get-object-pose-in-map ?object ?to-reach-pose)
             (lisp-pred identity ?to-reach-pose)
             (desig:desig-prop ?object (:location ?some-location)))
        (desig:desig-prop ?current-location-designator (:location ?some-location)))
    (desig:current-designator ?some-location ?location)
    ;; if the location is on the robot itself, use the current robot pose
    (location-always-reachable ?location)
    (lisp-fun cram-tf:robot-current-pose ?pose-stamped))

  ;; Helper to reason if a location is accessible
  (<- (location-accessible ?location-designator)
    (desig:loc-desig? ?location-designator)
    (desig:current-designator ?location-designator ?current-location-designator)
    (-> (spec:property ?current-location-designator (:in ?container-object))
        (and (desig:current-designator ?container-object ?object-designator)
             (or (desig:desig-prop ?object-designator (:type :robot))
                 (and (rob-int:robot ?robot)
                      (desig:desig-prop ?object-designator (:part-of ?robot)))))
        (true)))

  ;; most symbolic locations have a reference object
  ;; this predicate finds the reference object of the given location desig
  (<- (location-reference-object ?location-designator ?current-object-designator)
    (desig:loc-desig? ?location-designator)
    (desig:current-designator ?location-designator ?current-location-designator)
    (or (spec:property ?current-location-designator (:in ?object-designator))
        (spec:property ?current-location-designator (:on ?object-designator))
        (spec:property ?current-location-designator (:left-of ?object-designator))
        (spec:property ?current-location-designator (:right-of ?object-designator))
        (spec:property ?current-location-designator (:in-front-of ?object-designator))
        (spec:property ?current-location-designator (:behind ?object-designator))
        (spec:property ?current-location-designator (:near ?object-designator))
        (spec:property ?current-location-designator (:far-from ?object-designator))
        (spec:property ?current-location-designator (:of ?object-designator)))
    (desig:current-designator ?object-designator ?current-object-designator))

  (<- (location-certain ?some-location-designator)
    (desig:loc-desig? ?some-location-designator)
    (desig:current-designator ?some-location-designator ?location-designator)
    (or (and (location-reference-object ?location-designator ?reference-object)
             (or (object-is-a-robot ?reference-object)
                 (cpoe:object-in-hand ?reference-object)))
        (spec:property ?location-designator (:pose ?_))
        (spec:property ?location-designator (:poses ?_))))

  (<- (location-always-stable ?some-location-designator)
    (desig:loc-desig? ?some-location-designator)
    (desig:current-designator ?some-location-designator ?location-designator)
    (or (spec:property ?location-designator (:attachment ?_))
        (spec:property ?location-designator (:attachments ?_)))))
