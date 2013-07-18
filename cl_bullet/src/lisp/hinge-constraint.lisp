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

(defclass hinge-constraint (constraint)
  ((frame-in-1 :initform (cl-transforms:make-transform
                          (cl-transforms:make-3d-vector 0 0 0)
                          (cl-transforms:make-quaternion 0 0 0 1))
               :initarg :frame-in-1 :reader frame-in-1)
   (frame-in-2 :initform (cl-transforms:make-transform
                          (cl-transforms:make-3d-vector 0 0 0)
                          (cl-transforms:make-quaternion 0 0 0 1))
               :initarg :frame-in-2 :reader frame-in-2)))

(defmethod foreign-class-alloc ((hinge hinge-constraint) &key)
  (with-slots (body-1 body-2 frame-in-1 frame-in-2) hinge
    (assert (and body-1 body-2) () "Hinge doesn't contain valid bodies")
    (new-hinge-constraint
     (foreign-obj body-1)
     (foreign-obj body-2)
     frame-in-1 frame-in-2)))

(defmethod limit ((hinge hinge-constraint) type)
  (ecase type
    (:lower (get-lower-limit (foreign-obj hinge)))
    (:upper (get-upper-limit (foreign-obj hinge)))))

(defmethod (setf limit) (new-value (hinge hinge-constraint) type)
  (ecase type
    (:lower (set-limit
             (foreign-obj hinge)
             (coerce new-value 'double-float)
             (get-upper-limit (foreign-obj hinge))))
    (:upper (set-limit
             (foreign-obj hinge)
             (get-lower-limit (foreign-obj hinge))
             (coerce new-value 'double-float)))))

(defmethod enabled ((hinge hinge-constraint))
  (get-enable-motor (foreign-obj hinge)))

(defmethod (setf enabled) (new-value (hinge hinge-constraint))
  (enable-motor (foreign-obj hinge) new-value))

(defmethod max-impulse ((hinge hinge-constraint))
  (get-max-motor-impulse (foreign-obj hinge)))

(defmethod (setf max-impulse) (new-value (hinge hinge-constraint))
  (set-max-motor-impulse (foreign-obj hinge) new-value))

(defmethod target-velocity ((hinge hinge-constraint))
  (get-motor-target-velocity (foreign-obj hinge)))

(defmethod (setf target-velocity) (new-value (hinge hinge-constraint))
  (let ((foreign-obj (foreign-obj hinge)))
    (enable-angular-motor
     foreign-obj
     (get-enable-motor foreign-obj)
     new-value
     (get-max-motor-impulse foreign-obj))))

(defmethod set-target ((hinge hinge-constraint) target dt)
  (set-motor-target (foreign-obj target) target (coerce dt 'double-float)))

(defmethod motor-position ((hinge hinge-constraint))
  (get-hinge-angle (foreign-obj hinge)))
