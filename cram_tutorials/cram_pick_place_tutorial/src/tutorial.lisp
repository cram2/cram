;;;
;;; Copyright (c) 2019, Arthur Niedzwiecki <niedzwiecki@uni-bremen.de>
;;;                     Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;                     Amar Fayaz <amar@uni-bremen.de>
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

(in-package :pp-tut)

(defun get-kitchen-urdf ()
  (slot-value
   (btr:object btr:*current-bullet-world* :kitchen)
   'cram-bullet-reasoning:urdf))

(defun move-kitchen-joint (&key (joint-name "iai_fridge_door_joint")
                             (joint-angle 0.2d0) (kitchen-name :kitchen))
  (btr:set-robot-state-from-joints
   `((,joint-name  ,joint-angle))
   (btr:object btr:*current-bullet-world* kitchen-name)))

(defun add-objects-to-mesh-list (&optional (ros-package "cram_bullet_world_tutorial"))
  (mapcar (lambda (object-filename-and-object-extension)
            (declare (type list object-filename-and-object-extension))
            (destructuring-bind (object-filename object-extension)
                object-filename-and-object-extension
              (let ((lisp-name (roslisp-utilities:lispify-ros-underscore-name
                                object-filename :keyword)))
                (pushnew (list lisp-name
                               (format nil "package://~a/resource/~a.~a"
                                       ros-package object-filename object-extension)
                               nil)
                         btr::*mesh-files*
                         :key #'car)
                lisp-name)))
          (mapcar (lambda (pathname)
                    (list (pathname-name pathname) (pathname-type pathname)))
                  (directory (physics-utils:parse-uri
                              (format nil "package://~a/resource/*.*" ros-package))))))


(defun spawn-object (spawn-pose &optional (obj-type :bottle) (obj-name 'bottle-1) (obj-color '(1 0 0)))
  (unless (assoc obj-type btr::*mesh-files*)
    (add-objects-to-mesh-list))
  (btr-utils:spawn-object obj-name obj-type :color obj-color :pose spawn-pose)
  (btr:simulate btr:*current-bullet-world* 10))


(defun make-pose (reference-frame pose)
  (cl-transforms-stamped:make-pose-stamped
   reference-frame 0.0
   (apply #'cl-transforms:make-3d-vector (first pose))
   (apply #'cl-transforms:make-quaternion (second pose))))

(defun park-arms ()
  (pp-plans::park-arms))

(defun park-arm (arm)
  (pp-plans::park-arms :arm arm))

(defmacro handle-failure (errors program-body &body error-handler-body)
  `(progn
    (cpl:with-failure-handling
        ((,errors (e)
           (print e)
           ,@error-handler-body))
    ,@program-body)))
