;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :kr-belief)

(defun generate-object-id (object-type)
  (cut:var-value
   '?object_id
   (car (json-prolog:prolog `("get_new_object_id" ,object-type ?object_id)))))

(defun assert-object-at-location (object-type object-id transform)
  (declare (type cl-transforms-stamped:transform-stamped transform))
  (let ((kr-location (cram->knowrob transform))
        (kr-object-type (cram->knowrob object-type))
        (kr-object-id (cram->knowrob object-id)))
    (json-prolog:prolog
     `("assert_object_at_location" ,kr-object-type ,kr-object-id ,kr-location))))

(defun assert-object-grasped (gripper-id object-id robot-id grasp-spec)
  (cut:var-value
   '?grasp_id
   (car (json-prolog:prolog `("assert_grasp_on_object"
                              ,gripper-id ,object-id ,robot-id ,grasp-spec ?grasp_id)))))

(defun retract-object-grasped (grasp-id)
  (json-prolog:prolog `("assert_ungrasp" ,grasp-id)))

(defun assert-assemblage (assemblage-type connection-type
                          reference-object-id primary-object secondary-object)
  "`primary-object' and `secondary-object' are individuals of type AtomicPart or Assemblage"
  (cut:var-value
   '?assemblage_id
   (car (json-prolog:prolog `("assert_assemblage_created"
                              ,assemblage-type ,connection-type
                              ,reference-object-id ,primary-object ,secondary-object
                              ?assemblage_id)))))

(defun retract-assemblage (assemblage-id)
  (json-prolog:prolog `("assert_assemblage_destroyed" ,assemblage-id)))
