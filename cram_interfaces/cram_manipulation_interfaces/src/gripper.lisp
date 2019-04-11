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

(defgeneric get-object-type-gripping-effort (object-type)
  (:documentation "Returns effort in Nm, e.g. 50.")
  (:method (object-type)
    (let ((specific-type
            (find-most-specific-object-type-for-generic
             #'get-object-type-gripping-effort object-type)))
      (if specific-type
          (get-object-type-gripping-effort specific-type)
          (error "There is no applicable method for the generic function ~%~a~%~
                  with object-type ~a.~%To fix this either: ~
                  ~%- Add a method with (object-type (eql ~a)) as the first specializer or ~
                  ~%- Add ~a into the type hierarchy in the cram_object_knowledge package."
                 #'get-object-type-gripper-opening
                 object-type object-type object-type)))))

(defgeneric get-object-type-gripper-opening (object-type)
  (:documentation "How wide to open the gripper before grasping, in m.")
  (:method (object-type)
    (let ((specific-type
            (find-most-specific-object-type-for-generic
             #'get-object-type-gripper-opening object-type)))
      (if specific-type
          (get-object-type-gripper-opening specific-type)
          (error "There is no applicable method for the generic function ~%~a~%~
                  with object-type ~a.~%To fix this either: ~
                  ~%- Add a method with (object-type (eql ~a)) as the first specializer or ~
                  ~%- Add ~a into the type hierarchy in the cram_object_knowledge package."
                 #'get-object-type-gripper-opening
                 object-type object-type object-type)))))
