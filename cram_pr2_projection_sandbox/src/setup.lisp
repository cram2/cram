;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :proj-sand)

;; roslaunch cram_pr2_projection_sandbox sandbox.launch

(defun init-projection ()
;; (def-fact-group costmap-metadata ()
;;   (<- (location-costmap:costmap-size 12 12))
;;   (<- (location-costmap:costmap-origin -6 -6))
;;   (<- (location-costmap:costmap-resolution 0.05))

;;   (<- (location-costmap:costmap-padding 0.2))
;;   (<- (location-costmap:costmap-manipulation-padding 0.2))
;;   (<- (location-costmap:costmap-in-reach-distance 0.9))
;;   (<- (location-costmap:costmap-reach-minimal-distance 0.1)))

  ;; (setf cram-bullet-reasoning-belief-state:*robot-parameter* "robot_description")

  (cram-occasions-events:clear-belief)

  (setf cram-tf:*tf-default-timeout* 2.0)

  (setf prolog:*break-on-lisp-errors* t)
  )

(roslisp-utilities:register-ros-init-function init-projection)

(defun add-objects-to-mesh-list (&optional (ros-package "cram_pr2_projection_sandbox"))
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

(defun spawn-objects ()
  (let ((objects (add-objects-to-mesh-list)))
    (mapcar (lambda (object-type)
              (btr-utils:spawn-object (format nil "~a1" object-type) object-type))
            objects)))

;; todo: create places where objects can theoretically be spawned: in shelves, on counters.
