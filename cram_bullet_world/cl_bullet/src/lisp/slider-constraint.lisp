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

(defclass slider-constraint (constraint)
  ((frame-in-1 :initform (cl-transforms:make-transform
                          (cl-transforms:make-3d-vector 0 0 0)
                          (cl-transforms:make-quaternion 0 0 0 1))
               :initarg :frame-in-1 :reader frame-in-1)
   (frame-in-2 :initform (cl-transforms:make-transform
                          (cl-transforms:make-3d-vector 0 0 0)
                          (cl-transforms:make-quaternion 0 0 0 1))
               :initarg :frame-in-2 :reader frame-in-2))
  (:documentation "Wrapper for slider joints. To keep it as simple as
  possible, we do not wrap the rotational part of the joint but only
  the linear. We fix the angular limits so that it doesn't
  rotate. This behavior shoudl fit best to joints in urdf files."))

(defmethod foreign-class-alloc ((slider slider-constraint) &key)
  (with-slots (body-1 body-2 frame-in-1 frame-in-2) slider
    (new-slider-constraint
     (foreign-obj body-1) (foreign-obj body-2)
     frame-in-1 frame-in-2)))

(defmethod limit ((slider slider-constraint) type)
  (ecase type
    ((:lower :lower-lin) (get-lower-lin-limit (foreign-obj slider)))
    ((:upper :upper-lin) (get-upper-lin-limit (foreign-obj slider)))
    (:lower-ang (get-lower-ang-limit (foreign-obj slider)))
    (:upper-ang (get-upper-ang-limit (foreign-obj slider)))))

(defmethod (setf limit) (new-value (slider slider-constraint) type)
  (ecase type
    ((:lower :lower-lin) (set-lower-lin-limit (foreign-obj slider)
                                              (coerce new-value 'double-float)))
    ((:upper :upper-lin) (set-upper-lin-limit (foreign-obj slider)
                                              (coerce new-value 'double-float)))
    (:lower-ang (set-lower-ang-limit (foreign-obj slider)
                                     (coerce new-value 'double-float)))
    (:upper-ang (set-upper-ang-limit (foreign-obj slider)
                                     (coerce new-value 'double-float)))))

(defmethod enabled ((slider slider-constraint))
  (get-powered-lin-motor (foreign-obj slider)))

(defmethod (setf enabled) (new-value (slider slider-constraint))
  (set-powered-lin-motor (foreign-obj slider) new-value))

(defmethod max-impulse ((slider slider-constraint))
  (get-max-lin-motor-force (foreign-obj slider)))

(defmethod (setf max-impulse) (new-value (slider slider-constraint))
  (set-max-lin-motor-force (foreign-obj slider) new-value))

(defmethod target-velocity ((slider slider-constraint))
  (get-target-lin-motor-velocity (foreign-obj slider)))

(defmethod (setf target-velocity) (new-value (slider slider-constraint))
  (set-target-lin-motor-velocity (foreign-obj slider) new-value))

(defmethod set-target ((slider slider-constraint) target dt)
  (let ((foreign-obj (foreign-obj slider)))
    (set-target-lin-motor-velocity
     foreign-obj
     (/ (- target (get-linear-pos foreign-obj))
        dt))))

(defmethod motor-position ((slider slider-constraint))
  (get-linear-pos (foreign-obj slider)))
