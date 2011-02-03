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

(in-package :btr)

(defstruct object-visibility
  percentage
  occluding-objects)

(defclass flat-color-object-proxy ()
  ((object :initarg :object :reader proxied-object)
   (color :initarg :color)))

(defstruct drawable-list
  drawables)

(defmethod draw ((context gl-context) (obj flat-color-object-proxy))
  (with-slots (object color) obj
    (let ((*collision-shape-color-overwrite* color)
          (*disable-texture-rendering* t))
      (draw context object))))

(defmethod draw ((context gl-context) (drawables drawable-list))
  (dolist (obj (drawable-list-drawables drawables))
    (draw context obj)))

(defun calculate-object-visibility (gl-window reasoning-world camera-pose object)
  "Calculates how much of `object' is visible from `camera-pose' in
  `world'. Returns an instance of OBJECT-VISIBILITY."
  (with-bullet-window-context gl-window
    (let ((*collision-shape-color-overwrite* '(1.0 1.0 1.0 1.0))
          (*disable-texture-rendering* t))
      (gl:with-pushed-attrib (:enable-bit)
        (gl:disable :lighting)
        (let* ((camera-centered (cl-transforms:make-pose
                                 (cl-transforms:origin camera-pose)
                                 (look-at-object-rotation camera-pose
                                                          (pose object))))
               (object-total-buffer (car (render-to-framebuffer
                                          object
                                          (make-instance
                                           'camera
                                           :pose camera-centered))))
               (camera (make-instance 'camera :pose camera-pose))
               (object-buffer (car (render-to-framebuffer object camera)))
               (obj-ref-color nil)
               (object-proxies (loop with objects = (objects reasoning-world)
                                     with step = (if objects
                                                     (coerce (/ (length objects)) 'single-float)
                                                     1)
                                     for obj in objects
                                     for i from step by step
                                     for obj-proxy = (make-instance 'flat-color-object-proxy
                                                          :object obj :color `(,i ,i ,i 1))
                                     when (eq object obj) do (setf obj-ref-color i)
                                     collecting obj-proxy))
               (scene-buffer (car (render-to-framebuffer (make-drawable-list :drawables object-proxies)
                                                         camera))))
          (loop for i below (* (width camera) (height camera) 3) by 3
                with object-total-pixels = 0
                with object-visible-pixels = 0
                with occluding-object-colors = nil
                when (> (aref object-total-buffer i) 0.0)
                  do (incf object-total-pixels)
                if (eql (aref scene-buffer i) obj-ref-color)
                  do (incf object-visible-pixels)
                else do (when (and (> (aref scene-buffer i) 0)
                                   (> (aref object-buffer i) 0))
                          (pushnew (aref scene-buffer i) occluding-object-colors))
                finally (return (make-object-visibility
                                 :percentage (cond ((> object-visible-pixels object-total-pixels) 1.0)
                                                   ((> object-total-pixels 0) (coerce (/ object-visible-pixels
                                                                                         object-total-pixels)
                                                                                      'single-float))
                                                   (t 0.0))
                                 :occluding-objects (mapcar #'proxied-object
                                                            (remove-if-not (lambda (o)
                                                                             (member (car (slot-value o 'color))
                                                                                     occluding-object-colors))
                                                                           object-proxies))))))))))
