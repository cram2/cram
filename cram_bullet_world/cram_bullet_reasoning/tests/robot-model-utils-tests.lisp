;;;
;;; Copyright (c) 2019, Thomas Lipps <tlipps@uni-bremen.de>
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

(in-package :btr-tests)

(defun clean-environment (from)
  (mapcar (lambda (name)
            (btr:remove-object btr:*current-bullet-world* name))
          from))

(defun spawn-objects-with-same-pose (names)
  (mapcar (lambda (name)
            (btr-utils:spawn-object name :mug :pose '((0 0 0) (0 0 0 1))))
          names))

(define-test robot-attached-objects-in-collision-negative
  ;; Tests if the function returns nil since not one object is attached
  ;; to a robot link.
  (setup-world)
  (spawn-objects-with-same-pose '(o oo))
  (assert-false (btr:robot-attached-objects-in-collision)) ;; -> no collision
  (clean-environment '(o oo)))

(define-test robot-attached-objects-in-collision-positive
  ;; Tests if the function registers a collision since one of the objects
  ;; is attached to the robot and the other is in collision to the
  ;; attached object.
  (setup-world)
  (spawn-objects-with-same-pose '(o oo))
  (btr:attach-object (btr:get-robot-object)
                     (btr:object btr:*current-bullet-world* 'o)
                     :link "base_footprint")
  (assert-true (btr:robot-attached-objects-in-collision)) ;; -> collision
  (clean-environment '(o oo)))

(define-test robot-attached-objects-in-collision-negative-because-of-attachment
  ;; Since the robot can attach things made of items attached to each other,
  ;; there should not be a collision if the robot attaches one of
  ;; these things. Even if the items in the "thing" are in collision
  ;; to each other.
  (setup-world)
  (spawn-objects-with-same-pose '(o oo))
  ;; "thing" 'o: 'oo is attached to 'o with the same pose, therefore
  ;; they are in collision.
  (btr:attach-object 'o 'oo)
  ;; attach "thing" 'o to a robot link
  (btr:attach-object (btr:get-robot-object)
                     (btr:object btr:*current-bullet-world* 'o)
                     :link "base_footprint")
  (assert-false (btr:robot-attached-objects-in-collision)) ;; -> no collision
  (clean-environment '(o oo)))
