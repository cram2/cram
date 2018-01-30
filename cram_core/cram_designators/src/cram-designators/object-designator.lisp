;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(defclass object-designator-data ()
  ((pose :reader object-pose :initarg :pose)
   (object-identifier :reader object-identifier :initarg :object-identifier
                      :initform (gensym "OBJECT-"))
   (color :reader object-color :initarg :color))
  (:documentation "Base class for all objects that are bound to an
  object designator's data slot. The minimum data that needs to be
  provided is the pose of the object and an identifier for the
  object."))

(defclass object-designator (designator designator-id-mixin equate-notification-mixin)
  ())

(register-designator-class :object object-designator)

(defmethod reference ((desig object-designator) &optional (role *default-role*))
  (with-slots (data) desig
    (or data (setf data (resolve-designator desig role)))))

(defmethod resolve-designator ((desig object-designator) (role t))
  ;; Object designators are somehow special because the REFERENCE
  ;; method cannot generate a new reference. The reason is that
  ;; the robot needs to interact with the environment to reference
  ;; objects. Therefore, the data slot has to be set by externally.
  (error 'designator-error
         :format-control "Designator `~a' does not reference an object."
         :format-arguments (list desig)
         :designator desig))
