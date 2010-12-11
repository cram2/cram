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

(in-package :cl-bullet)

(defvar *debug-arg-pool* (cut:make-data-pool)
  "Data pool containing arguments that are passed to a debug draw
  object. It basically contains the self references to the CLOS debug
  draw object.")

(define-condition debug-draw-invalid (simple-warning) ())

(defclass debug-draw ()
  ((data-pool-id :reader :id)
   (foreign-debug-draw :reader foreign-debug-draw)))

(defgeneric draw-line (draw from to color))
(defgeneric draw-sphere (draw p radius color))
(defgeneric draw-triangle (draw v-0 v-1 v-2 color alpha))
(defgeneric draw-box (draw box-min box-max color alpha))
(defgeneric draw-aabb (draw from to color))
(defgeneric draw-transform (draw transform ortho-len))
(defgeneric draw-arc (draw center normal axis radius-a radius-b
                           min-angle max-angle color draw-sect
                           step-degrees))
(defgeneric draw-sphere-patch (draw center up axis radius min-th max-th
                                    min-ps max-ps color ste-degrees))
(defgeneric draw-contact-point (draw point-on-b normal-on-b distance life-time color))
(defgeneric report-error-warning (draw warning-string))
(defgeneric draw-3d-text (location string))

(defmethod initialize-instance :after ((self debug-draw) &key)
  (with-slots (data-pool-id foreign-debug-draw)
      self
    (setf data-pool-id
          (cut:new-pool-value
           *debug-arg-pool* (tg:make-weak-pointer self)))
    (with-foreign-object (callbacks 'debug-draw-callbacks)
      (with-foreign-slots ((draw-line
                            draw-sphere
                            draw-triangle
                            draw-box
                            draw-aabb
                            draw-transform
                            draw-arc
                            draw-sphere-patch
                            draw-contact-point
                            report-error-warning
                            draw-3d-text)
                           callbacks debug-draw-callbacks)
        (setf draw-line (get-callback 'draw-line-cb)
              draw-sphere (get-callback 'draw-sphere)
              draw-triangle (get-callback 'draw-triangle)
              draw-box (get-callback 'draw-box)
              draw-aabb (get-callback 'draw-aabb)
              draw-transform (get-callback 'draw-transform)
              draw-arc (get-callback 'draw-arc)
              draw-sphere-patch (get-callback 'draw-sphere-patch)
              draw-contact-point (get-callback 'draw-contact-point)
              report-error-warning (get-callback 'report-error-warning)
              draw-3d-text (get-callback 'draw-3d-text))
        (setf foreign-debug-draw (new-cl-bullet-debug-draw callbacks data-pool-id))
        (tg:finalize self (let ((id data-pool-id)
                                (foreign-obj foreign-debug-draw))
                            (lambda ()
                              (format t "finalize~%")
                              (delete-cl-bullet-debug-draw foreign-obj)
                              (cut:delete-pool-value *debug-arg-pool* id))))))))

(defun %call-debug-draw-method (pool-id method &rest args)
  (let ((draw (tg:weak-pointer-value
               (cut:pool-value *debug-arg-pool* pool-id))))
    (if draw
        (apply method draw args)
        (warn 'debug-draw-invalid
              :format-control "The object with id `~a' has been garbage collected already."
              :format-arguments (list pool-id)))))

(defcallback draw-line-cb :void ((from bt-3d-vector) (to bt-3d-vector) (color bt-3d-vector) (pool-id :int))
  (%call-debug-draw-method pool-id #'draw-line from to color))

(defcallback draw-sphere-cb :void ((p bt-3d-vector) (radius :double) (color bt-3d-vector) (pool-id :int))
  (%call-debug-draw-method pool-id #'draw-sphere p radius color))

(defcallback draw-triangle-cb :void ((v-0 bt-3d-vector) (v-1 bt-3d-vector) (v-2 bt-3d-vector)
                                     (color bt-3d-vector) (alpha :double) (pool-id :int))
  (%call-debug-draw-method pool-id #'draw-triangle v-0 v-1 v-2 color alpha))

(defcallback draw-box-cb :void ((box-min bt-3d-vector) (box-max bt-3d-vector) (color bt-3d-vector)
                                (alpha :double) (pool-id :int))
  (%call-debug-draw-method pool-id #'draw-box box-min box-max color alpha))

(defcallback draw-aabb-cb :void ((from bt-3d-vector) (to bt-3d-vector) (color bt-3d-vector) (pool-id :int))
  (%call-debug-draw-method pool-id #'draw-aabb from to color))

(defcallback draw-transform-cb :void ((transform bt-transform) (ortho-len :double) (pool-id :int))
  (%call-debug-draw-method pool-id #'draw-transform transform ortho-len))

(defcallback draw-arc-cb :void ((center bt-3d-vector) (normal bt-3d-vector) (axis bt-3d-vector)
                                (radius-a :double) (radius-b :double) (min-angle :double)
                                (max-angle :double) (color bt-3d-vector) (draw-sect :boolean)
                                (step-degrees :double) (pool-id :int))
  (%call-debug-draw-method
   pool-id #'draw-arc
   center normal axis radius-a radius-b min-angle max-angle
   color draw-sect step-degrees))

(defcallback draw-sphere-patch-cb :void ((center bt-3d-vector) (up bt-3d-vector)
                                         (axis bt-3d-vector) (radius :double)
                                         (min-th :double) (max-th :double)
                                         (min-ps :double) (max-ps :double)
                                         (color bt-3d-vector) (step-degrees :double)
                                         (pool-id :int))
  (%call-debug-draw-method
   pool-id #'draw-sphere-patch
   center up axis radius min-th
   max-th min-ps max-ps color
   step-degrees))

(defcallback draw-contact-point-cb :void ((point-on-b bt-3d-vector) (normal-on-b bt-3d-vector)
                                          (distance :double) (life-time :int) (color bt-3d-vector)
                                          (pool-id :int))
  (%call-debug-draw-method
   pool-id #'draw-contact-point
   point-on-b normal-on-b distance
   life-time color))

(defcallback report-error-warning-cb :void ((warning-string :string) (pool-id :int))
  (%call-debug-draw-method pool-id #'report-error-warning warning-string))

(defcallback draw-3d-text-cb :void ((location bt-3d-vector) (string :string) (pool-id :int))
  (%call-debug-draw-method pool-id #'draw-3d-text location string))
