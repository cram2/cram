;;; Copyright (c) 2011, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :cl-glx)

(defclass pixmap-rendering-context ()
  ((lock :reader rendering-context-lock
         :initform (sb-thread:make-mutex))
   (display :reader display)
   (visual :reader visual)
   (context :reader context)
   (pixmap :reader pixmap)
   (glx-pixmap :reader glx-pixmap)
   (width :reader width :initarg :width)
   (height :reader height :initarg :height)
   (depth :reader depth :initarg :depth)))

(defmethod initialize-instance :after ((rendering-context pixmap-rendering-context)
                                       &key (attributes `(,glx-rgba
                                                          (,glx-red-size . 1)
                                                          (,glx-green-size . 1)
                                                          (,glx-blue-size . 1))))
  (with-slots (visual width height depth) rendering-context
    (let ((display nil)
          (context nil)
          (pixmap nil)
          (glx-pixmap nil))
      (tg:finalize rendering-context
                   (lambda ()
                     (when (and display glx-pixmap)
                       (glx-destroy-glx-pixmap display glx-pixmap))
                     (when (and display pixmap)
                       (x-free-pixmap display pixmap))
                     (when (and display context)
                       (glx-destroy-context display context))
                     (when display
                       (x-close-display display))))
      (setf display (x-open-display ""))
      (assert (not (null-pointer-p display)) () "Unable to open display.")
      (setf visual (choose-visual display (alexandria:flatten attributes)))
      (assert (not (null-pointer-p visual)) () "Unable to get visual.")
      (setf context (glx-create-context display visual (null-pointer) 1))
      (assert (not (null-pointer-p visual)) () "Unable to create rendering context.")
      (setf pixmap (x-create-pixmap display (x-default-root-window display)
                                    width height depth))
      (setf glx-pixmap (glx-create-glx-pixmap display visual pixmap))
      (set-context-slots
       rendering-context
       :display display
       :context context
       :pixmap  pixmap
       :glx-pixmap glx-pixmap))))

(defun set-context-slots (rendering-context &key display context pixmap glx-pixmap)
  (setf (slot-value rendering-context 'display) display)
  (setf (slot-value rendering-context 'context) context)
  (setf (slot-value rendering-context 'pixmap) pixmap)
  (setf (slot-value rendering-context 'glx-pixmap) glx-pixmap))

(defun set-foreign-array (array values type)
  (loop for i from 0
        for value in values
        do (setf (mem-aref array type i) value)
        finally (return array)))

(defun choose-visual (display attributes)
  (with-foreign-object (foreign-attributes :int (length attributes))
    (set-foreign-array foreign-attributes attributes :int)
    (glx-choose-visual display (x-default-screen display) foreign-attributes)))

(defmacro with-rendering-context (rendering-context &body body)
  (alexandria:once-only (rendering-context)
    `(sb-thread:with-mutex ((rendering-context-lock ,rendering-context))
       (glx-make-current
        (display ,rendering-context) (glx-pixmap ,rendering-context)
        (context ,rendering-context))
       ,@body)))
