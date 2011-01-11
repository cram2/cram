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
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
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

(in-package :bt)

(defgeneric activation-state (body))
(defgeneric (setf activation-state) (new-value body))
(defgeneric collision-flags (body))
(defgeneric (setf collision-flags) (new-value body))
(defgeneric get-total-force (body))

(defclass rigid-body (foreign-class)
  ((mass :reader mass :initarg :mass :initform 0.0d0)
   (motion-state :reader motion-state :initarg :motion-state
                 :initform (make-instance 'motion-state))
   (collision-shape :reader collision-shape :initarg :collision-shape
                    :initform (error
                               'simple-error
                               :format-control "collision-shape argument required"))))

(defmethod pose ((body rigid-body))
  (pose (motion-state body)))

(defmethod (setf pose) (new-value (body rigid-body))
  (setf (pose (motion-state body)) new-value)
  (set-motion-state (foreign-obj body) (foreign-obj (motion-state body))))

(defmethod foreign-class-alloc ((body rigid-body) &key
                                pose activation-state collision-flags
                                &allow-other-keys)
  (when pose
    (setf (pose (motion-state body)) pose))
  (let ((foreign-body (new-rigid-body
                       (coerce (mass body) 'double-float)
                       (foreign-obj (motion-state body))
                       (foreign-obj (collision-shape body)))))
    (when activation-state
      (setf (activation-state body) activation-state))
    (when collision-flags
      (setf (collision-flags body) collision-flags))
    foreign-body))

(defmethod foreign-class-free-fun ((body rigid-body))
  #'delete-rigid-body)

(defmethod get-total-force ((body rigid-body))
  (cffi-get-total-force (foreign-obj body)))

(defmethod activation-state ((body rigid-body))
  (get-activation-state (foreign-obj body)))

(defmethod (setf activation-state) (new-value (body rigid-body))
  (set-activation-state (foreign-obj body) new-value))

(defmethod collision-flags ((body rigid-body))
  (get-collision-flags (foreign-obj body)))

(defmethod (setf collision-flags) (new-value (body rigid-body))
  (set-collision-flags (foreign-obj body) new-value))
