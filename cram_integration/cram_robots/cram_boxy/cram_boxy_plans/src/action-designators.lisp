;;;
;;; Copyright (c) 2016, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :boxy-plans)

;; (defun cram-manipulation-interfaces:get-object-transform (object-designator)
;;   (let* ((object-type (desig:desig-prop-value object-designator :type))
;;          (object-frame (concatenate 'string
;;                                     (remove #\- (string-capitalize (symbol-name object-type)))
;;                                     "1")))
;;     (cl-transforms-stamped:lookup-transform
;;      cram-tf:*transformer*
;;      cram-tf:*robot-base-frame*
;;      object-frame
;;      :time 0.0
;;      :timeout cram-tf:*tf-default-timeout*)))



;; TODO: spatial-relations-cm defines pose validators for FOR-ON relations.
;; These validators reject solutions from the result below.
;; SOLUTION: either create pose validators specific for costmaps,
;; OR, which is better, make sure costmap validators know they are validating results
;; from the costmap. But how to do this??
(def-fact-group location-designators (desig:location-grounding)
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
    (lisp-fun man-int:get-object-transform ?current-other-object-designator
              ?other-object-transform)
    (lisp-fun man-int:get-object-placement-transform
              ?object-name ?object-type
              ?other-object-name ?other-object-type ?other-object-transform
              ?attachment-type
              ?attachment-transform)
    (lisp-fun cram-tf:strip-transform-stamped ?attachment-transform ?pose-stamped)))



(def-fact-group boxy-designators (desig:action-grounding)

  ;; (<- (desig:action-grounding ?action-designator (wiggle ?left-poses ?right-poses))
  ;;   (property ?action-designator (:type :wiggling))
  ;;   (once (or (property ?action-designator (:left-poses ?left-poses))
  ;;             (equal ?left-poses nil)))
  ;;   (once (or (property ?action-designator (:right-poses ?right-poses))
  ;;             (equal ?right-poses nil))))

  (<- (desig:action-grounding ?action-designator (cram-inspect ?augmented-designator))
    (property ?action-designator (:type :inspecting))
    (property ?action-designator (:object ?object-designator))
    (property ?action-designator (:for ?for-value))
    (-> (lisp-type ?for-value desig:object-designator)
        (equal ?description-to-add (:for (:object)))
        (equal ?description-to-add (:for (?for-value))))
    (desig:desig-description ?object-designator ?properties)
    (equal ?augmented-description (?description-to-add . ?properties))
    (desig:designator :object ?augmented-description ?augmented-designator)
    (-> (lisp-type ?for-value desig:object-designator)
        (and (desig:current-designator ?for-value ?for-object-designator)
             ;; (property ?for-object-designator (:type ?for-object-type))
             (prolog:slot-value ?for-object-designator desig:quantifier ?for-quantifier)
             (prolog:slot-value ?augmented-designator desig:quantifier ?for-quantifier))
        (true)))

  ;; (<- (desig:action-grounding ?action-designator (perceive :inspecting ?object-designator))
  ;;   (property ?action-designator (:type :inspecting))
  ;;   (property ?action-designator (:object ?object-designator)))

  (<- (desig:action-grounding ?action-designator (look ?left-goal-pose ?right-goal-pose))
    (property ?action-designator (:type :looking))
    (property ?action-designator (:camera :wrist))
    (property ?action-designator (:object ?object-designator))
    (desig:current-designator ?object-designator ?current-object-designator)
    (lisp-fun cram-manipulation-interfaces:get-object-transform
              ?current-object-designator ?object-transform)
    ;; infer missing information
    (lisp-fun kr-assembly::get-object-look-pose :left ?object-transform ?left-goal-pose)
    ;; the only wrist camera is on left arm
    (equal ?right-goal-pose NIL)))
