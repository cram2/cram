;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
;;;               2022, Gayane Kazhoyan <kazhoyan@uni-bremen.de>
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
;;;       Technische Universitaet Muenchen, nor the name of the Institute for
;;;       Artificial Intelligence / University of Bremen, nor the names of their
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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

(defun to-png-image (width height buffer &optional (color-mode :rgb))
  (let ((channels (ecase color-mode
                    (:rgb 3)
                    (:gray 1))))
    (let* ((result (png:make-image height width channels)))
      ;; `result' is an image array of size height x width x channels
      ;; `buffer' is a vector of length height*width*channels
      (dotimes (row height result)
        (dotimes (col width)
          (dotimes (rgb channels)
            (setf (aref result row (- width col 1) rgb)
                  (truncate (* (aref buffer (+ (* row width channels)
                                               (* col channels)
                                               (* rgb)))
                               #xff)))))))))

(defun png-from-camera-view (&key
                               (png-path "/tmp/scene.png")
                               (png-width 1000)
                               (png-height 1000)
                               (robot-camera-view t)
                               (camera-pose (cl-transforms:make-pose
                                             (cl-transforms:make-3d-vector -1 0.7 9)
                                             (cl-transforms:matrix->quaternion
                                              (make-array '(3 3)
                                                          :initial-contents
                                                          '(( 0 -1  0)
                                                            (-1  0  0)
                                                            ( 0  0 -1)))))))
  (let ((rendering-context
          (btr:get-rendering-context png-width png-height)))
    (when robot-camera-view
      (setf camera-pose
            (cut:var-value
             '?camera-pose
             (car (prolog:prolog
                   `(and (rob-int:robot ?robot)
                         (rob-int:camera-frame ?robot ?camera-frame)
                         (btr:link-pose ?robot ?camera-frame ?camera-pose)))))))
    (cl-bullet-vis:with-gl-context rendering-context
      (gl:with-pushed-attrib (:enable-bit :color-buffer-bit :depth-buffer-bit :texture-bit)
        (gl:matrix-mode :modelview)
        (gl:enable :light0 :lighting :cull-face :depth-test :color-material :blend :rescale-normal)
        (apply #'gl:clear-color cl-bullet-vis:*background-color*)
        (gl:clear :color-buffer :depth-buffer)
        (gl:light-model :light-model-ambient #(0.5 0.5 0.5 1.0))
        (gl:light :light0 :diffuse #(0.8 0.8 0.8 1))
        (gl:light :light0 :specular #(0.8 0.8 0.8 1))
        (gl:load-identity)
        (gl:rotate 90 1 0 0)
        (gl:rotate -90 0 0 1)
        (gl:rotate 180 1 0 0)
        (gl:depth-func :lequal)
        (gl:shade-model :smooth)
        (gl:blend-func :src-alpha :one-minus-src-alpha)
        (gl:hint :perspective-correction-hint :nicest)
        (btr:with-rendering-to-framebuffer (png-width png-height)
          (png::encode-file
           (btr:to-png-image
            png-width png-height
            (car (btr:render-to-framebuffer
                  rendering-context
                  (btr:make-drawable-list
                   :drawables
                   (bt-vis:gl-objects btr:*debug-window*))
                  (make-instance 'btr:camera
                    :pose camera-pose
                    :width png-width :height png-height))))
           png-path)))))
  png-path)
