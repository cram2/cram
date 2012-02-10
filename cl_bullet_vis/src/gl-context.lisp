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

(in-package :bt-vis)

(defvar *global-rendering-lock* (sb-thread:make-mutex)
  "A global mutex that needs to be acquired when a thread wants to
  perform OpenGL operations.")

(defgeneric get-texture-handle (gl-context name))

(defgeneric camera-transform (gl-context)
  (:documentation "Returns the transform of the current camera as a
  CL-TRANSFORMS:TRANSFORM."))

(defgeneric light-position (gl-context)
  (:documentation "Returns the position of the light as a
  CL-TRANSFORMS:3D-VECTOR."))

(defgeneric register-display-list (gl-context id obj)
  (:documentation "Registers a display list for garbage collection if
  obj is garbage collected"))

(defgeneric remove-display-list (gl-context id)
  (:documentation "Frees a display list with a specific ID"))

(defgeneric get-display-list-id (gl-context instance)
  (:documentation "Gets the display list the instance has
  registered."))

(defgeneric display-list-valid (gl-context id)
  (:documentation "Returns if a display list is still valid, i.e. if
  it is known to the gl-context and has not been garbage collected
  yet"))

(defgeneric gc-gl-context (gl-context)
  (:documentation "Garbage-collects the gl context."))

(defgeneric gl-object-transparent (obj)
  (:documentation "Returns T if an object is to be drawn transparent")
  (:method ((obj t))
    nil))

(defgeneric run-in-gl-context (gl-context function)
  (:documentation "Runs `function' in the rendering context of
  `gl-context', i.e. all GL calls in `function' are executed in `gl-context'"))

(defclass gl-context ()
  ((textures :initform (make-hash-table))
   (display-lists :initform nil)
   (camera-transform :initform (cl-transforms:make-transform
                                (cl-transforms:make-3d-vector 0 0 0)
                                (cl-transforms:make-quaternion 0 0 0 1))
                     :initarg :camera-transform
                     :accessor camera-transform)
   (light-position :initform (cl-transforms:make-3d-vector 0 0 5)
                   :initarg :light-position
                   :reader light-position)
   (gl-objects :initform nil :initarg :objects :accessor gl-objects)))

(defmethod get-texture-handle ((context gl-context) (name symbol))
  (or (gethash name (slot-value context 'textures))
      (values
        (setf (gethash name (slot-value context 'textures))
              (car (gl:gen-textures 1)))
        t)))

(defmethod register-display-list ((gl-context gl-context) id obj)
  (with-slots (display-lists) gl-context
    (pushnew (cons id (tg:make-weak-pointer obj)) display-lists)))

(defmethod remove-display-list ((gl-context gl-context) id)
  (when (display-list-valid gl-context id)
    (gl:delete-lists id 1)
    (with-slots (display-lists) gl-context
      (setf display-lists
            (remove id display-lists
                    :key #'car)))))

(defmethod get-display-list-id ((gl-context gl-context) instance)
  (with-slots (display-lists) gl-context
    (car (find instance display-lists :key
               (alexandria:compose #'tg:weak-pointer-value #'cdr)))))

(defmethod display-list-valid ((gl-context gl-context) id)
  (with-slots (display-lists) gl-context
    (let ((list-ptr (cdr (assoc id display-lists))))
      (and list-ptr (tg:weak-pointer-value list-ptr) t))))

(defmethod gc-gl-context ((gl-context gl-context))
  (flet ((gc-display-list (id)
           (gl:delete-lists id 1)))
    (with-slots (display-lists) gl-context
      (setf display-lists (reduce (lambda (l i)
                                    (if (tg:weak-pointer-value (cdr i))
                                        (cons i l)
                                        (prog1 l
                                          (gc-display-list (car i)))))
                                  display-lists
                                  :initial-value nil)))))

(defmacro with-rendering-lock (&body body)
  `(sb-thread:with-recursive-lock (*global-rendering-lock*)
     ,@body))

(defmacro with-gl-context (context &body body)
  "Executes `body' in the OpenGL rendering context of `context'."
  `(with-rendering-lock
     (run-in-gl-context ,context (lambda () ,@body))))
