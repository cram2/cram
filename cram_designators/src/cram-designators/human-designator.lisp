;;;
;;; Copyright (c) 2015, Georg Bartels <georg.bartels@cs.uni-bremen.de>
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
;;;     * Neither the name of Universität Bremen, Germany, nor the names of its
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

;;;
;;; CRAM resolves human designators in the same way it resolves object designators:
;;;                          Through perception routines.
;;;
;;; As a result, there are no generator functions to produce hypothetical solutions
;;; for human designators in this package. Instead, perception routines have to set
;;; the `data' slot of a human designator which they were asked to perceive. In line
;;; with this thinking, the default implementation of `resolve-designator' for human
;;; designators throws an error.
;;;
;;; When calling `reference' on an unresolved human designator will throw an error
;;; because we consider it an error to reference the low-level data slot associated
;;; with an unperceived human.
;;;

(defclass human-designator (designator designator-id-mixin equate-notification-mixin)
  ())

(register-designator-class :human human-designator)

(defmethod reference ((desig human-designator) &optional (role *default-role*))
  "References the human-designators `desig'. If its data-slot is nil, i.e. `desig'
 is an unresolved designator, `reference' will call `resolve-designator' on `desig'."
  (with-slots (data) desig
    (or data (setf data (resolve-designator desig role)))))

(defmethod resolve-designator ((desig human-designator) (role t))
  "Tries to resolve the human designator `desig'. As CRAM resolves human designators
 through external perception routines, this behavior is considered faulty. Hence, an
 error is thrown."
  (error 'designator-error
         :format-control "Designator `~a' does not reference an human."
         :format-arguments (list desig)
         :designator desig))
