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

(defclass camera ()
  ((width :initarg :width :reader width :initform 320)
   (height :initarg :height :reader height :initform 200)
   (fov-y :initarg :fov-y :reader fov-y :initform 50
          :documentation "Field of view in y direction")
   (z-near :initarg :z-near :reader z-near :initform 0.1)
   (z-far :initarg :z-far :reader z-far :initform 10.0)
   (camera-axis :initarg :camera-axis :reader camera-axis
                :initform (cl-transforms:make-3d-vector 1 0 0))
   (pose :initarg :pose :reader pose
         :initform (cl-transforms:make-pose
                    (cl-transforms:make-3d-vector 0 0 0)
                    (cl-transforms:make-quaternion 0 0 0 1)))))

(defgeneric gl-execute-with-camera (camera function)
  (:documentation "Executes `function' with `camera' set.")
  (:method ((camera t) (function function))
    (funcall function))
  (:method :before ((camera camera) function)
    (declare (ignore function))
    (with-slots (width height fov-y z-near z-far) camera
      (gl:viewport 0 0 width height)
      (gl:matrix-mode :projection)
      (gl:load-identity)
      (glu:perspective fov-y (/ width height) z-near z-far)
      (gl:matrix-mode :modelview)
      (gl:load-identity)
      (gl:rotate 180 0 1 0)
      (set-camera (camera-transform camera)))))

(defmethod initialize-instance :after ((camera camera) &key transform)
  (when transform
    (setf (slot-value camera 'pose)
          (cl-transforms:transform->pose transform))))

(defmethod camera-transform ((camera camera))
  (cl-transforms:reference-transform (pose camera)))

(defun look-at-object-rotation (camera-pose object-pose)
  "Returns the transform that needs to be applied to the camera's pose
  in order to directly look at `object-pose'. It returns the transform
  to allign the z axis of the camera's pose with the vector from
  `camera' to `object-pose'"
  (let* ((obj-point-in-camera (cl-transforms:v-
                               (cl-transforms:origin object-pose)
                               (cl-transforms:origin camera-pose)))
         (z-axis (cl-transforms:make-3d-vector 0 0 1))
         (angle (acos (/ (cl-transforms:dot-product
                          obj-point-in-camera z-axis)
                         (cl-transforms:v-norm obj-point-in-camera))))
         (rot-axis (cl-transforms:cross-product
                    z-axis obj-point-in-camera)))
    (cl-transforms:q*
     (cl-transforms:axis-angle->quaternion rot-axis angle)
     (cl-transforms:axis-angle->quaternion z-axis (/ pi 2)))))

(defvar *framebuffer-enabled* nil)

(defmacro with-rendering-to-framebuffer ((width height) &body body)
  (let ((framebuffer (gensym "framebuffer"))
        (pixelbuffer (gensym "pixelbuffer"))
        (depthbuffer (gensym "depthbuffer")))
    `(let ((,framebuffer (car (cl-opengl:gen-framebuffers-ext 1)))
           (,pixelbuffer (car (cl-opengl:gen-renderbuffers-ext 1)))
           (,depthbuffer (car (cl-opengl:gen-renderbuffers-ext 1))))
       ;; Create the framebuffer
       (gl:bind-framebuffer-ext :framebuffer-ext ,framebuffer)
       ;; Set up pixel buffer
       (gl:bind-renderbuffer-ext :renderbuffer-ext ,pixelbuffer)
       (gl:renderbuffer-storage-ext :renderbuffer-ext :rgb ,width ,height)
       (gl:framebuffer-renderbuffer-ext :framebuffer-ext :color-attachment0-ext
                                        :renderbuffer-ext ,pixelbuffer)
       ;; Set up depth buffer
       (gl:bind-renderbuffer-ext :renderbuffer-ext ,depthbuffer)
       (gl:renderbuffer-storage-ext :renderbuffer-ext :depth-component
                                    ,width ,height)
       (gl:framebuffer-renderbuffer-ext :framebuffer-ext :depth-attachment-ext
                                        :renderbuffer-ext ,depthbuffer)
       (unwind-protect
            (let ((*framebuffer-enabled* t))
              (let ((framebuffer-status (gl:check-framebuffer-status-ext :framebuffer-ext)))
                (unless (gl::enum= framebuffer-status :framebuffer-complete-ext)
                  (error "Framebuffer not complete: ~A." framebuffer-status)))
              ,@body)
         (gl:bind-framebuffer-ext :framebuffer-ext 0)
         (gl:delete-framebuffers-ext (list ,framebuffer))
         (gl:delete-renderbuffers-ext (list ,pixelbuffer ,depthbuffer))))))

(defun render-to-framebuffer (gl-context drawable camera
                              &key (get-pixelbuffer t)
                                (get-depthbuffer nil)
                                (mirror nil))
  "Renders the object `drawable' into a framebuffer. It returns a list
  of the form (pix-buffer depth-buffer) with values set to NIL if the
  corresponding flag parameter `get-pixelbuffer' and `get-depthbuffer'
  respectively is set to NIL "
  (setf (camera-transform gl-context) (camera-transform camera))
  (flet ((do-rendering ()
           (let ((viewport (map 'list #'identity (gl:get* :viewport))))
             (unwind-protect
                  (progn
                    (gl:clear :color-buffer :depth-buffer)
                    (gl-execute-with-camera
                     camera (lambda ()
                              (draw gl-context drawable)
                              (gl:flush)))
                    (list (when get-pixelbuffer (read-pixelbuffer camera mirror))
                          (when get-depthbuffer (read-depthbuffer camera mirror))))
               (apply #'gl:viewport viewport)))))
    (if *framebuffer-enabled*
        (do-rendering)
        (with-rendering-to-framebuffer ((width camera) (height camera))
          (do-rendering)))))

(defun mirror-buffer (width height gl-buffer channels)
  (let ((result (make-array (array-dimension gl-buffer 0)
                            :element-type (array-element-type gl-buffer))))
    (declare (type simple-array gl-buffer result))
    (dotimes (y height result)
      (dotimes (x width)
        (dotimes (i channels)
          (setf (aref result (+ (* channels (+ (* y width) x)) i))
                (aref gl-buffer (+ (* channels
                                      (+ (* y width) (- width x 1))) i))))))))

(defun read-pixelbuffer (camera &optional mirror)
  (let* ((width (width camera))
         (height (height camera))
         (gl-buffer (bt-vis:read-pixels-float 0 0 width height :rgb)))
    ;; Note: gl's result is mirrored on the y axis, so let's mirror it back
    (if mirror
        (mirror-buffer width height gl-buffer 3)
        gl-buffer)))

(defun read-depthbuffer (camera &optional mirror)
  (let ((gl-buffer (bt-vis:read-pixels-float
                    0 0 (width camera) (height camera)
                    :depth-component)))
    (if mirror
        (mirror-buffer (width camera) (height camera) gl-buffer 1)
        gl-buffer)))

(defun to-png-image (width height buffer &optional (color-mode :rgb))
  (let ((channels (ecase color-mode
                    (:rgb 3)
                    (:gray 1))))
    (let* ((result (make-array `(,height ,width ,channels)
                               :element-type '(unsigned-byte 8)
                               :displaced-to (make-array (* height width channels)
                                                         :element-type (list 'unsigned-byte 8)))))
      (dotimes (i (array-total-size buffer) result)
        (setf (row-major-aref result i)
              (truncate (* (row-major-aref buffer i) #xff)))))))

