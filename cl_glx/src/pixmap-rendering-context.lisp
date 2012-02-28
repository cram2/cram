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
   (display :reader display :initform nil)
   (visual :reader visual :initform nil)
   (context :reader context :initform nil)
   (pixmap :reader pixmap :initform nil)
   (glx-pixmap :reader glx-pixmap :initform nil)
   (width :reader width :initarg :width)
   (height :reader height :initarg :height)
   (depth :reader depth :initarg :depth)))

(defgeneric rendering-context-destroy-function (rendering-context)
  (:method ((rendering-context pixmap-rendering-context))
    (let ((display (slot-value rendering-context 'display))
          (visual (slot-value rendering-context 'visual))
          (context (slot-value rendering-context 'context))
          (pixmap (slot-value rendering-context 'pixmap))
          (glx-pixmap (slot-value rendering-context 'glx-pixmap)))
      (lambda ()
        (when (and display glx-pixmap)
          (glx-destroy-glx-pixmap display glx-pixmap))
        (when (and display pixmap)
          (x-free-pixmap display pixmap))
        (when (and display context)
          (glx-destroy-context display context))
        (when visual
          (x-free visual))
        (when display
          (x-close-display display))))))

(defmethod initialize-instance :after ((rendering-context pixmap-rendering-context)
                                       &key (attributes `(,glx-rgba
                                                          (,glx-depth-size . 24)
                                                          (,glx-red-size . 8)
                                                          (,glx-green-size . 8)
                                                          (,glx-blue-size . 8))))
  (with-slots (display context pixmap glx-pixmap visual width height depth)
      rendering-context
    (unwind-protect
         (progn
           (setf display (x-open-display (null-pointer)))
           (assert (not (null-pointer-p display)) () "Unable to open display.")
           (setf visual (choose-visual display attributes))
           (assert (not (null-pointer-p visual)) () "Unable to get visual.")
           (setf context (glx-create-context display visual (null-pointer) 1))
           (assert (not (null-pointer-p visual)) () "Unable to create rendering context.")
           (setf pixmap (x-create-pixmap display (x-default-root-window display)
                                         width height depth))
           (setf glx-pixmap (glx-create-glx-pixmap display visual pixmap)))
      (tg:finalize
       rendering-context (rendering-context-destroy-function rendering-context)))))

(defun set-foreign-array (array values type)
  (loop for i from 0
        for value in values
        do (setf (mem-aref array type i) value)
        finally (return array)))

(defun ensure-valid-attributes-list (attributes)
  "Returns a new list that is a flattened copy of `attributes' and
  that is terminated by None."
  (append (alexandria:flatten attributes) (list none)))

(defun choose-visual (display attributes)
  (let ((attributes (ensure-valid-attributes-list attributes)))
    (with-foreign-object (foreign-attributes :int (length attributes))
      (set-foreign-array foreign-attributes attributes :int)
      (glx-choose-visual display (x-default-screen display) foreign-attributes))))

(defmacro with-rendering-context (rendering-context &body body)
  (alexandria:with-gensyms (current-display current-drawable current-context)
    (alexandria:once-only (rendering-context)
      `(let ((,current-display (glx-get-current-display))
             (,current-drawable (glx-get-current-drawable))
             (,current-context (glx-get-current-context)))
         (sb-thread:with-mutex ((rendering-context-lock ,rendering-context))
           (unwind-protect
                (progn
                  (assert (eql (glx-make-current
                                (display ,rendering-context) (glx-pixmap ,rendering-context)
                                (context ,rendering-context))
                               1)
                          () "Unable to activate rendering context.")
                  ,@body)
             (if (or (null-pointer-p ,current-display)
                     (eql ,current-drawable 0)
                     (null-pointer-p ,current-context))
                 (glx-make-current (if (null-pointer-p ,current-display)
                                       (display ,rendering-context)
                                       ,current-display)
                                   none (null-pointer))
                 (glx-make-current ,current-display ,current-drawable ,current-context))))))))
