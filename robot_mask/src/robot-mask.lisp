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

(in-package :robot-mask)

(defun get-mask (robot camera)
  (bt-vis:with-bullet-window-context *debug-window*
    (gl:with-pushed-attrib (:enable-bit)
      (gl:disable :lighting)
      (let* ((robot-proxy (make-instance
                           'flat-color-object-proxy
                           :object robot :color '(1.0 1.0 1.0 1.0)))
             (scene (car (render-to-framebuffer robot-proxy camera)))
             (result (make-array (* (width camera) (height camera)) :element-type 'fixnum :initial-element 0)))
        (dotimes (i (* (width camera) (height camera)) result)
          (setf (aref result i) (truncate (aref scene (* i 3)))))))))

(defun init-robot (robot msg &optional (fixed-frame "/base_footprint"))
  (let ((tf (make-instance 'tf:transformer)))
    (map 'nil (lambda (trans) (tf:set-transform tf trans))
         (tf:tf-message->transforms msg))
    (set-robot-state-from-tf tf robot fixed-frame)))

(defun main ()
  (with-ros-node ("robot_mask" :spin t)
    (let* ((world (make-instance 'bt-reasoning-world))
           (robot (add-object world 'urdf 'robot '((0 0 0) (0 0 0 1))
                              :urdf (cl-urdf:parse-urdf
                                     (get-param "robot_description")))))
      (add-debug-window world)
      (register-service-fn "~get_mask"
                           (lambda (request)
                             (with-fields (width height fovy tf_state camera_frame)
                                 request
                               (init-robot robot tf_state
                                           (get-param "~fixed_frame" "/base_footprint"))
                               (make-instance
                                'robot_mask-srv:getmask-response
                                :mask (get-mask
                                       robot
                                       (make-instance
                                        'camera
                                        :width width
                                        :height height
                                        :fov-y fovy
                                        :pose (link-pose robot camera_frame))))))
                           'robot_mask-srv:getmask))))
