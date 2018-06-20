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

(in-package :cram-object-interfaces)

(defmacro getassoc (key alist)
  `(cdr (assoc ,key ,alist :test #'equal)))

(defun get-object-transform (object-designator)
  (car (getassoc :transform (desig:desig-prop-value object-designator :pose))))

(defun get-object-pose (object-designator)
  (car (getassoc :pose (desig:desig-prop-value object-designator :pose))))

(def-fact-group object-designators (desig:desig-location-prop desig:location-grounding)
  (<- (desig:desig-location-prop ?desig ?loc)
    (desig:obj-desig? ?desig)
    (lisp-fun get-object-pose ?desig ?loc)
    (lisp-pred identity ?loc))

  (<- (desig:location-grounding ?designator ?pose-stamped)
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
