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

(in-package :bullet)

(defstruct bounding-box
  (center (cl-transforms:make-3d-vector 0 0 0))
  dimensions)

(defgeneric activation-state (body))
(defgeneric (setf activation-state) (new-value body))
(defgeneric collision-flags (body))
(defgeneric (setf collision-flags) (new-value body))
(defgeneric total-force (body))
(defgeneric total-torque (body))
(defgeneric apply-force (body force rel-pos))
(defgeneric apply-central-force (body force))
(defgeneric apply-torque (body torque))
(defgeneric clear-forces (body))
(defgeneric linear-velocity (body))
(defgeneric (setf linear-velocity) (new-value body))
(defgeneric angular-velocity (body))
(defgeneric (setf angular-velocity) (new-value body))
(defgeneric aabb (body))
(defgeneric (setf collision-group) (new-value body))
(defgeneric (setf collision-mask) (new-value body))
(defgeneric (setf mass) (new-value body))

(defclass rigid-body (foreign-class)
  ((name :reader name :initarg :name :initform (gensym "RIGID-BODY-"))
   (mass :reader mass :initarg :mass :initform 0.0d0)
   (motion-state :reader motion-state :initarg :motion-state
                 :initform (make-instance 'motion-state))
   (collision-shape :reader collision-shape :initarg :collision-shape
                    :initform (error
                               'simple-error
                               :format-control "collision-shape argument required"))
   (collision-group :initarg :group :initform :default-filter
                    :reader collision-group)
   (collision-mask :initarg :mask :initform :all-filter
                   :reader collision-mask)))

(defgeneric activate (body)
  (:method ((body rigid-body))
    (setf (activation-state body) :active-tag)))

(defmethod pose ((body rigid-body))
  (pose (motion-state body)))

(defmethod (setf pose) (new-value (body rigid-body))
  (setf (pose (motion-state body)) new-value)
  (set-motion-state (foreign-obj body) (foreign-obj (motion-state body)))
  ;; In case the object was deactivated, i.e. it was not moving
  ;; anymore, we need to re-activate it.
  (activate body))

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
      (set-activation-state foreign-body activation-state))
    (when collision-flags
      (set-collision-flags foreign-body collision-flags))
    foreign-body))

(defmethod foreign-class-free-fun ((body rigid-body))
  #'delete-rigid-body)

(defmethod total-force ((body rigid-body))
  (cffi-get-total-force (foreign-obj body)))

(defmethod total-torque ((body rigid-body))
  (cffi-get-total-torque (foreign-obj body)))

(defmethod apply-force ((body rigid-body) force rel-pos)
  (cffi-apply-force (foreign-obj body) force rel-pos)
  (activate body))

(defmethod apply-central-force ((body rigid-body) force)
  (cffi-apply-central-force (foreign-obj body) force)
  (activate body))

(defmethod apply-torque ((body rigid-body) torque)
  (cffi-apply-torque (foreign-obj body) torque)
  (activate body))

(defmethod clear-forces ((body rigid-body))
  (cffi-clear-forces (foreign-obj body)))

(defmethod linear-velocity ((body rigid-body))
  (get-linear-velocity (foreign-obj body)))

(defmethod (setf linear-velocity) (new-value (body rigid-body))
  (set-linear-velocity (foreign-obj body) new-value)
  (activate body))

(defmethod angular-velocity ((body rigid-body))
  (get-angular-velocity (foreign-obj body)))

(defmethod (setf angular-velocity) (new-value (body rigid-body))
  (set-angular-velocity (foreign-obj body) new-value)
  (activate body))

(defmethod activation-state ((body rigid-body))
  (get-activation-state (foreign-obj body)))

(defmethod (setf activation-state) (new-value (body rigid-body))
  (set-activation-state (foreign-obj body) new-value))

(defmethod collision-flags ((body rigid-body))
  (get-collision-flags (foreign-obj body)))

(defmethod (setf collision-flags) (new-value (body rigid-body))
  (set-collision-flags (foreign-obj body) new-value)
  (activate body))

(defmethod aabb ((body rigid-body))
  (destructuring-bind (min max)
      (get-aabb (foreign-obj body))
    (make-bounding-box
     :center (cl-transforms:v* (cl-transforms:v+ min max) 0.5)
     :dimensions (cl-transforms:v- max min))))

(defmethod (setf collision-group) (new-value (body rigid-body))
  (with-slots (collision-group collision-mask) body
    (setf collision-group new-value)
    (set-collision-filter (foreign-obj body) new-value collision-mask))
  (activate body))

(defmethod (setf collision-mask) (new-value (body rigid-body))
  (with-slots (collision-group collision-mask) body
    (setf collision-mask new-value)
    (set-collision-filter (foreign-obj body) collision-group new-value))
  (activate body))

(defmethod (setf mass) (new-value (body rigid-body))
  (with-slots (mass) body
    (setf mass new-value)
    (set-mass-props (foreign-obj body) (float new-value 0.0d0)))
  (activate body))
