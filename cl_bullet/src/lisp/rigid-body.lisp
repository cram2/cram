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

(defclass rigid-body (foreign-class)
  ((mass :reader mass :initarg :mass :initform 0.0d0)
   (motion-state :reader motion-state :initarg :motion-state
                 :initform (make-instance 'motion-state))
   (collision-shape :reader collision-shape :initarg :collision-shape
                    :initform (error
                               'simple-error
                               :format-control "collision-shape argument required"))
   (activation-state :reader activation-state
                     :initarg :activation-state
                     :initform :active-tag)
   (collision-flags :reader collision-flags
                    :initarg :collision-flags
                    :initform :cf-kinematic-object)))

(defgeneric pose (rigid-body)
  (:method ((body rigid-body))))

(defgeneric (setf pose) (new-value rigid-body)
  (:method ((new-value cl-transforms:transform)
            (body rigid-body))
    nil)
  (:method ((new-value cl-transforms:pose)
            (body rigid-body))
    nil))

(defmethod foreign-class-alloc ((body rigid-body) &key &allow-other-keys)
  (let ((foreign-body (new-rigid-body
                       (mass body)
                       (foreign-obj (motion-state body))
                       (foreign-obj (collision-shape body)))))
    ;; Is this good? Shouldn't we better rely on foreign reader
    ;; methods and not set anything here?
    (set-activation-state foreign-body (activation-state body))
    (set-collision-flags foreign-body (collision-flags body))
    foreign-body))

(defmethod foreign-class-free-fun ((body rigid-body))
  #'delete-rigid-body)

(defmethod get-total-force ((body rigid-body))
  (cffi-get-total-force (foreign-obj body)))

(defmethod (setf activation-state) (new-value (body rigid-body))
  (setf (slot-value body 'activation-state) new-value)
  (set-activation-state (foreign-obj body) new-value))

(defmethod (setf collision-flags) (new-value (body rigid-body))
  (setf (slot-value body 'collision-flags) new-value)
  (set-collision-flags (foreign-obj body) new-value))
