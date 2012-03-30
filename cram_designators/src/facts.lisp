;;;
;;; Copyright (c) 2009, Lorenz Moesenlechner <moesenle@cs.tum.edu>,
;;;                     Nikolaus Demmel <demmeln@cs.tum.edu>
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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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
;;;

(in-package :desig)

(defun obj-desig-location (obj-desig)
  (when (and (typep obj-desig 'object-designator)
             (desig-prop-value obj-desig 'at)
             (typep (desig-prop-value obj-desig 'at) 'designator))
    (reference (current-desig (desig-prop-value obj-desig 'at)))))

(defun loc-desig-location (loc-desig)
  (when (and loc-desig (typep loc-desig 'location-designator))
    (reference loc-desig)))

;; This fact group contains general rules for location designators
;;
;; Process modules and other reasoning modules can extend location
;; designator resolution, as an example cram_highlevel/location_costmap
;; provides extentions, when loaded
;; Here, we only define trivial resolutions
(def-fact-group location-designators (desig-loc desig-location-prop)

  ;; checks designator starts with (location ...)
  (<- (loc-desig? ?desig)
    (lisp-pred typep ?desig location-designator))

  (<- (obj-desig? ?desig)
    (lisp-pred typep ?desig object-designator))

  (<- (desig-location-prop ?desig ?loc)
    (obj-desig? ?desig)
    (lisp-fun obj-desig-location ?desig ?loc)
    (lisp-pred identity ?loc))
  
  ;; (location ... (obj ?obj)...), e.g. location to see obj
  (<- (desig-location-prop ?desig ?loc)
    (or (desig-prop ?desig (obj ?obj))
        (desig-prop ?desig (object ?obj)))
    (lisp-type ?obj designator)
    (lisp-fun current-desig ?obj ?curr-obj)
    (lisp-fun obj-desig-location ?curr-obj ?loc)
    (lisp-pred identity ?loc))

    ;; (location ... (location ?loc)...), e.g. location to see location
  (<- (desig-location-prop ?desig ?loc)
    (desig-prop ?desig (location ?loc-desig))
    (lisp-fun current-desig ?loc-desig ?curr-loc-desig)
    (lisp-fun loc-desig-location ?loc-desig ?loc)))

(def-fact-group manipulation-designator (action-desig)
  (<- (trajectory-desig? ?desig)
    (lisp-pred typep ?desig action-designator)
    (desig-prop ?desig (type trajectory))))

(macrolet ((def-desig-accessor (slot &optional (predicate-name nil))
             (let* ((predicate-name (or predicate-name slot))
                    (name (format-symbol t "DESIG-~a" predicate-name))
                    (var (format-symbol t "?~a" predicate-name)))
               `(<- (,name ?desig ,var)
                  (bound ?desig)
                  (get-slot-value ?desig ,slot ,var)))))
  (def-fact-group designator-accessors (desig-value)
    ;; DESIG-TIMESTAMP
    (def-desig-accessor timestamp)
    ;; DESIG-DESCRIPTION
    (def-desig-accessor description)
    ;; DESIG-VALID
    (def-desig-accessor valid)
    ;; DESIG-VALUE
    (def-desig-accessor data value)

    (<- (desig-reference ?desig ?reference)
      (desig-valid ?desig ?valid)
      (lisp-pred identity ?valid)
      (lisp-fun reference ?desig ?reference))

    (<- (desig-solutions ?desig ?solutions)
      (ground ?desig)
      (lisp-fun designator-solutions ?desig ?solutions))

    (<- (equated-desigs ?desig ?equated)
      (lisp-fun get-equal-designators ?desig ?equated))))

(defun get-desig-class (desig)
  (car (rassoc (class-of desig) (get 'make-designator :desig-types)
               :key #'find-class)))

(def-fact-group designators (desig-value)

  ;; parses description for pairs matching (?prop-name ?prop)
  (<- (desig-prop ?desig (?prop-name ?prop))
    (bound ?desig)
    (lisp-fun check-desig-prop-package ?prop-name ?_)
    (lisp-fun description ?desig ?props)
    (member (?prop-name ?prop) ?props))

  ;; parses description and timestamp
  (<- (desig-prop ?desig (?prop-name ?prop) ?t)
    (bound ?desig)
    (lisp-fun check-desig-prop-package ?prop-name ?_)
    (desig-timestamp ?desig ?t)
    (desig-prop ?desig (?prop-name ?prop)))

  ;; parses timstamp of all equated designators
  (<- (desig-prop ?desig (?prop-name ?prop) ?t)
    (bound ?desig)
    (lisp-fun check-desig-prop-package ?prop-name ?_)
    (desig-equal ?desig ?d-2)
    (desig-timestamp ?d-2 ?t)
    (desig-prop ?d-2 (?prop-name ?prop)))

  (<- (desig-equal ?d1 ?d2)
    (desig ?d1)
    (desig ?d2)
    (lisp-pred desig-equal ?d1 ?d2))

  (<- (desig-class ?d ?type)
    (desig ?d)
    (lisp-fun get-desig-class ?d ?type)
    (lisp-pred identity ?type))

  (<- (desig-value ?d ?val ?t)
    (desig ?d)
    (desig-timestamp ?d ?t)
    (desig-value ?d ?val))

  (<- (desig-value ?d ?val ?t)
    (bound ?d)
    (desig ?other-d)
    (desig-equal ?d ?other-d)
    (desig-timestamp ?other-d ?t)
    (desig-value ?other-d ?val))

  ;; Constructor for designators
  (<- (designator ?class ?description ?desig)
    (ground (?class ?description))
    (not (bound ?desig))
    (lisp-fun make-designator ?class ?description ?desig))

  (<- (designator ?class ?description ?desig)
    (bound ?desig)
    (desig-class ?desig ?class)
    (desig-description ?desig ?description)))

(def-prolog-handler desig (bdgs ?desig)
  (let* ((?tmp-desig (gen-var)))
    (lazy-mapcan (lambda (bdg)
                   (multiple-value-bind (new-bdgs ok?)
                       (unify ?desig (var-value ?tmp-desig bdg) bdgs)
                     (when ok?
                       (list new-bdgs))))
                 (rete-holds `(desig-bound ,?tmp-desig ?_)))))
