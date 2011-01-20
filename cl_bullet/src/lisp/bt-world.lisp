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

(defgeneric gravity-vector (world))
(defgeneric (setf gravity-vector) (new-value world))
(defgeneric step-simulation (world time-step))
(defgeneric add-rigid-body (world body &optional group mask))
(defgeneric remove-rigid-body (world body))
(defgeneric bodies (world))
(defgeneric add-constraint (world constraint &optional disable-collision))
(defgeneric remove-constraint (world constraint))
(defgeneric constraints (world))
(defgeneric set-debug-drawer (world drawer))
(defgeneric get-debug-drawer (world))
(defgeneric debug-draw-world (world))
(defgeneric perform-collision-detection (world))
(defgeneric contact-manifolds (world))

(defclass bt-world (foreign-class)
  ((bodies :reader bodies :initform nil)
   (constraints :reader constraints :initform nil)
   (gravity-vector :reader gravity-vector
                   :initarg :gravity-vector
                   :initform (cl-transforms:make-3d-vector 0 0 -9.81))
   (debug-drawer :reader get-debug-drawer :initform nil)
   (foreign-alloc-fun :reader foreign-alloc-fun
                      :initform #'new-discrete-dynamics-world)
   (foreign-free-fun :reader foreign-class-free-fun
                     :initform #'delete-discrete-dynamics-world)
   (lock :initform (sb-thread:make-mutex))))

(defmacro with-world-locked (world &body body)
  `(sb-thread:with-mutex ((slot-value ,world 'lock))
     ,@body))

(defmethod foreign-class-alloc ((world bt-world) &key
                                &allow-other-keys)
  (funcall (foreign-alloc-fun world) (gravity-vector world)))

(defmethod (setf gravity-vector) (new-value (world bt-world))
  (set-gravity (foreign-obj world) new-value))

(defmethod step-simulation ((world bt-world) time-step)
  (with-world-locked world
    (cffi-step-simulation (foreign-obj world) (coerce time-step 'double-float))))

(defmethod add-rigid-body ((world bt-world) (body rigid-body) &optional group mask)
  (with-world-locked world
    (with-slots (bodies foreign-obj) world
      (push body bodies)
      (if mask
          (cffi-add-rigid-body-with-mask
           foreign-obj (foreign-obj body) group mask)
          (cffi-add-rigid-body
           foreign-obj (foreign-obj body))))))

(defmethod remove-rigid-body ((world bt-world) body)
  (with-world-locked world
    (cffi-remove-rigid-body (foreign-obj world) (foreign-obj body))
    (setf (slot-value world 'bodies) (remove body (bodies world)))))

(defmethod add-constraint ((world bt-world) constraint &optional disable-collision)
  (with-world-locked world
    (cffi-add-constraint (foreign-obj world) (foreign-obj constraint) disable-collision)))

(defmethod remove-constraint ((world bt-world) constraint)
  (with-world-locked world
    (cffi-remove-constraint (foreign-obj world) (foreign-obj constraint))
    (setf (slot-value world 'constraints) (remove constraint (constraints world)))))

(defmethod set-debug-drawer ((world bt-world) drawer)
  (with-world-locked world
    (cffi-set-debug-drawer (foreign-obj world) (foreign-obj drawer))))

(defmethod debug-draw-world ((world bt-world))
  (with-world-locked world
    (cffi-debug-draw-world (foreign-obj world))))

(defmethod perform-collision-detection ((world bt-world))
  (with-world-locked world
    (cffi-perform-collision-detection (foreign-obj world))))

(defmethod contact-manifolds ((world bt-world))
  (with-world-locked world
    (flet ((get-contact-points (manifold)
             (loop for i from 0 below (manifold-get-num-contact-points manifold)
                   collecting (make-instance 'contact-point
                                   :point-in-1 (manifold-get-contact-point-0 manifold i)
                                   :point-in-2 (manifold-get-contact-point-1 manifold i)))))
      (loop for i from 0 below (get-num-manifolds (foreign-obj world))
            for manifold = (get-manifold-by-index (foreign-obj world) i)
            for body-ptr-1 = (manifold-get-body-0 manifold)
            for body-ptr-2 = (manifold-get-body-1 manifold)
            when (> (manifold-get-num-contact-points manifold) 0)
              collecting (make-instance 'contact-manifold
                              :body-1 (find (manifold-get-body-0 manifold) (bodies world)
                                            :key #'foreign-obj :test (lambda (a b)
                                                                       (eql (pointer-address a)
                                                                            (pointer-address b))))
                              :body-2 (find (manifold-get-body-1 manifold) (bodies world)
                                            :key #'foreign-obj :test (lambda (a b)
                                                                       (eql (pointer-address a)
                                                                            (pointer-address b))))
                              :contact-points (get-contact-points manifold))))))
