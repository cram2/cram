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
    (not (cpoe:object-in-hand ?_ ?arm))))






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



(def-fact-group location-designator-with-attachment (desig:location-grounding)
  (<- (desig:location-grounding ?location-designator ?pose-stamped)
    (desig:current-designator ?location-designator ?current-location-designator)
    (desig:desig-prop ?current-location-designator (:for ?object-designator))
    (desig:desig-prop ?current-location-designator (:on ?other-object-designator))
    (desig:desig-prop ?current-location-designator (:attachment ?attachment-type))
    (desig:current-designator ?object-designator ?current-object-designator)
    (spec:property ?current-object-designator (:type ?object-type))
    (spec:property ?current-object-designator (:name ?object-name))
    (desig:current-designator ?other-object-designator ?current-other-object-designator)
    (spec:property ?current-other-object-designator (:type ?other-object-type))
    (spec:property ?current-other-object-designator (:name ?other-object-name))

    (lisp-fun get-object-transform ?current-other-object-designator
              ?other-object-transform)

    (lisp-fun get-object-placement-transform
              ?object-name ?object-type
              ?other-object-name ?other-object-type ?other-object-transform
              ?attachment-type
              ?attachment-transform)
    (lisp-fun cram-tf:strip-transform-stamped ?attachment-transform ?pose-stamped)))
