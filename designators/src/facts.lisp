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
             (desig-prop-value obj-desig 'at))
    (reference (current-desig (desig-prop-value obj-desig 'at)))))

(defun loc-desig-location (loc-desig)
  (when (and (typep loc-desig 'location-designator)
             loc-desig)
    (reference loc-desig)))

(def-fact-group location-designators (desig-loc)
  (<- (loc-desig? ?desig)
    (lisp-pred typep ?desig location-designator))

  (<- (desig-location-prop ?desig ?loc)
    (desig-prop ?desig (obj ?obj))
    (lisp-fun obj-desig-location ?obj ?loc))

  (<- (desig-location-prop ?desig ?loc)
    (desig-prop ?desig (location ?loc-desig))
    (lisp-fun loc-desig-location ?loc-desig ?loc))
  
  (<- (desig-loc ?desig (pose ?p))
    (loc-desig? ?desig)
    (desig-prop ?desig (pose ?p)))

  (<- (desig-loc ?desig (pose ?p))
    (loc-desig? ?desig)
    (desig-prop ?desig (of ?obj))
    (lisp-fun obj-desig-location ?obj ?p)))

(def-fact-group manipulation-designator (action-desig)
  (<- (manip-desig? ?desig)
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
    (def-desig-accessor data value)))

(defun get-desig-class (desig)
  (car (rassoc (class-of desig) (get 'make-designator :desig-types)
               :key #'find-class)))

(def-fact-group designators (desig-value)
  (<- (desig-prop ?desig (?prop-name ?prop))
    (bound ?desig)
    (bound ?prop-name)
    (lisp-fun desig-prop-value ?desig ?prop-name ?prop)
    (lisp-pred identity ?prop))

  (<- (desig-prop ?desig (?prop-name ?prop))
    (bound ?desig)
    (not (bound ?prop-name))
    (lisp-fun description ?desig ?props)
    (member (?prop-name ?prop) ?props))

  (<- (desig-prop ?desig (?prop-name ?prop) ?t)
    (bound ?desig)
    (desig-timestamp ?desig ?t)
    (desig-prop ?desig (?prop-name ?prop)))

  (<- (desig-prop ?desig (?prop-name ?prop) ?t)
    (bound ?desig)
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
    (desig-value ?other-d ?val)))

(def-prolog-handler desig (bdgs ?desig)
  (let* ((?tmp-desig (gen-var)))
    (lazy-mapcan (lambda (bdg)
                   (multiple-value-bind (new-bdgs ok?)
                       (unify ?desig (var-value ?tmp-desig bdg) bdgs)
                     (when ok?
                       (list new-bdgs))))
                 (rete-holds `(desig-bound ,?tmp-desig ?_)))))
