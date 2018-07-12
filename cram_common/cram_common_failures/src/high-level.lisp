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

(in-package :common-fail)

(define-condition navigation-high-level-failure (high-level-failure) ())

(define-condition navigation-goal-in-collision (navigation-high-level-failure)
  ((pose-stamped :initarg :pose-stamped :initform nil :reader navigation-failure-pose-stamped)))

(define-condition looking-high-level-failure (high-level-failure) ())

(define-condition object-unreachable (high-level-failure)
  ((object :initarg :object
           :initform NIL
           :reader object-unreachable-object))
  (:documentation "Thrown when no IK found for particular base pose."))

(define-condition manipulation-goal-in-collision (high-level-failure) ()
  (:documentation "Thrown when executing action results in a collision."))



(define-condition object-unfetchable (high-level-failure)
  ((object :initarg :object
           :initform NIL
           :reader object-unfetchable-object))
  (:documentation "Thrown when no base positioning can assure a reachable pose."))

(define-condition object-undeliverable (high-level-failure)
  ((object :initarg :object
           :initform NIL
           :reader object-undeliverable-object))
  (:documentation "Thrown when no base positioning can assure a reachable pose."))

(define-condition object-nowhere-to-be-found (high-level-failure)
  ((object :initarg :object
           :initform NIL
           :reader object-nowhere-to-be-found-object))
  (:documentation "Thrown when no base positioning can assure a reachable pose."))

(define-condition environment-unreachable (high-level-failure) ()
  (:documentation "Thrown when environment manipulation in collision or unreachable."))

(define-condition environment-manipulation-impossible (high-level-failure) ()
  (:documentation "Thrown when environment manipulation cannot be achieved."))
