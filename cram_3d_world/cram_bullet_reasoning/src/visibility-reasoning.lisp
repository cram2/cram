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

(defvar *rendering-context* nil)
(defvar *visibility-threshold* 0.9)

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
      (gl:color (first color) (second color) (third color))
      (draw context object))))

(defmethod draw ((context gl-context) (drawables drawable-list))
  (dolist (obj (drawable-list-drawables drawables))
    (draw context obj)))

(defun get-rendering-context (width height)
  (unless (and *rendering-context*
               (eql width (cl-glx:width *rendering-context*))
               (eql height (cl-glx:height *rendering-context*)))
    (setf *rendering-context* (make-instance 'pixmap-gl-context
                                :width width
                                :height height
                                :depth 24)))
  *rendering-context*)

(defun calculate-object-visibility (reasoning-world camera-pose object
                                    &optional (width 320) (height 200))
  "Calculates how much of `object' is visible from `camera-pose' in
  `world'. Returns an instance of OBJECT-VISIBILITY."

  ;; This method works by creating three rendered images. First it
  ;; renderes the object centered in the camera. Then it renders the
  ;; object with the correct camera. And finally it renders the
  ;; complete scene with each object in a different color. Everything
  ;; is rendered without lighting. To calculate the visibility ratio,
  ;; the ratio of visible pixels of `object' in the complete scene to
  ;; pixels that belong to the object (by using the first image) is
  ;; calculated. To calculate which objects are occluding `object',
  ;; image 2 is used. All pixels that are set there and don't belong
  ;; to `object' in image 3 are occluding pixels and hence the
  ;; corresponding object is occluding `object.
  (let ((rendering-context (get-rendering-context width height)))
    (with-gl-context rendering-context
      (let ((*collision-shape-color-overwrite* '(1.0 1.0 1.0 1.0))
            (*disable-texture-rendering* t))
        (gl:with-pushed-attrib (:enable-bit :color-buffer-bit :depth-buffer-bit)
          (gl:disable :lighting)
          (gl:depth-func :lequal)
          (gl:enable :depth-test)
          (%gl:clear-color 0 0 0 0)
          (with-rendering-to-framebuffer (width height)
            (let* ((camera-centered (cl-transforms:make-pose
                                     (cl-transforms:origin camera-pose)
                                     (look-at-object-rotation camera-pose
                                                              (pose object))))
                   (object-total-buffer (car (render-to-framebuffer
                                              rendering-context object
                                              (make-instance 'camera
                                                :pose camera-centered
                                                :width width :height height))))
                   (camera (make-instance 'camera
                                          :pose camera-pose
                                          :width width :height height))
                   (object-buffer (car (render-to-framebuffer
                                        rendering-context object camera)))
                   (obj-ref-color nil)
                   (object-proxies (loop with objects = (objects reasoning-world)
                                         with step = (if objects
                                                         (coerce
                                                          (/ (length objects))
                                                          'single-float)
                                                         1)
                                         for obj in objects
                                         for i from step by step
                                         for obj-proxy = (make-instance
                                                          'flat-color-object-proxy
                                                          :object obj
                                                          :color `(,i ,i ,i 1))
                                         when (eq object obj)
                                           do (setf obj-ref-color i)
                                         collecting obj-proxy))
                   (scene-buffer (car (render-to-framebuffer
                                       rendering-context
                                       (make-drawable-list
                                        :drawables object-proxies)
                                       camera))))
              (declare (type (simple-array single-float *)
                             object-total-buffer
                             object-buffer scene-buffer))
              ;; (png::encode-file (to-png-image width height object-total-buffer) "/tmp/obj-total.png")
              ;; (png::encode-file (to-png-image width height object-buffer) "/tmp/obj-only.png")
              ;; (png::encode-file (to-png-image width height scene-buffer) "/tmp/scene.png")
              (loop for i below (* (width camera) (height camera) 3) by 3
                    with object-total-pixels = 0
                    with object-visible-pixels = 0
                    with occluding-object-colors = nil
                    ;; threshold since colors are float values and not
                    ;; always exactly what they should be
                    with object-color-threshold = (/ (* 2
                                                        (length
                                                         (objects
                                                          reasoning-world))))
                    when (> (aref object-total-buffer i) 0.0)
                      do (incf object-total-pixels)
                    if (< (abs (- (aref scene-buffer i) obj-ref-color))
                          object-color-threshold)
                      do (incf object-visible-pixels)
                    else do (when (and (> (aref scene-buffer i) 0)
                                       (> (aref object-buffer i) 0))
                              (pushnew (aref scene-buffer i)
                                       occluding-object-colors))
                    finally (return (make-object-visibility
                                     :percentage (cond ((> object-visible-pixels
                                                           object-total-pixels)
                                                        1.0)
                                                       ((> object-total-pixels
                                                           0)
                                                        (coerce
                                                         (/ object-visible-pixels
                                                            object-total-pixels)
                                                         'single-float))
                                                       (t 0.0))
                                     :occluding-objects
                                     (mapcar
                                      #'proxied-object
                                      (remove-if-not
                                       (lambda (o)
                                         (block block-check
                                         (loop for col-occ in occluding-object-colors
                                               with col-pr = (car (slot-value
                                                                   o 'color))
                                               when (< (abs (- col-occ
                                                               col-pr))
                                                       object-color-threshold)
                                                 do (return-from block-check
                                                      T))))
                                       object-proxies))))))))))))

(defun object-visible-p (world camera-pose object &optional(threshold *visibility-threshold*))
  "Returns T if at least `threshold' of the object is visible"
  (let ((visibility (calculate-object-visibility world camera-pose object)))
    (>= (object-visibility-percentage visibility)
        threshold)))

(defun occluding-objects (world camera-pose object &optional (threshold *visibility-threshold*))
  "Returns the list of occluding objects if less than `threshold' of `object' is visible"
  (let ((visibility (calculate-object-visibility world camera-pose object)))
    (when (< (object-visibility-percentage visibility) threshold)
      (object-visibility-occluding-objects visibility))))
