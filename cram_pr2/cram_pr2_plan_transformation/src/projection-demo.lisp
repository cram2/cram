;;;
;;; Copyright (c) 2019, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;                     Arthur Niedzwiecki <niedzwiecki@uni-bremen.de>
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

(in-package :plt)

;; This implementation is mostly copied from cram-pr2-pick-place-demo

(defparameter *sink-nav-goal*
  (cl-transforms-stamped:make-pose-stamped
   "map"
   0.0
   (cl-transforms:make-3d-vector 0.75d0 0.70d0 0.0)
   (cl-transforms:make-identity-rotation)))

(defparameter *island-nav-goal*
  (cl-transforms-stamped:make-pose-stamped
   "map"
   0.0
   (cl-transforms:make-3d-vector -0.2d0 1.5d0 0.0)
   (cl-transforms:make-quaternion 0 0 1 0)))

(defparameter *object-spawning-poses*
  `((:breakfast-cereal . ((1.4 0.4 0.85)
                          ,(with-slots (cl-tf:w cl-tf:z) (cl-tf:euler->quaternion :az pi)
                             (list 0 0 cl-tf:z cl-tf:w))))
    (:cup . ((1.3 0.6 0.9) (0 0 0 1)))
    (:bowl . ((1.4 0.8 0.87) (0 0 0 1)))
    (:spoon . ((1.43 0.9 0.74132) (0 0 0 1)))
    (:milk . ((1.4 0.62 0.95) (0 0 1 0)))))

(defparameter *object-grasping-arms*
  '((:breakfast-cereal . :left)
    (:bowl . :right)
    (:cup . :left)
    (:spoon . :right)
    (:milk . :right)))

(defparameter *object-placing-poses*
  `((:breakfast-cereal . ((-0.78 0.8 0.95)
                          ,(if (assoc :breakfast-cereal *object-grasping-arms*)
                               (case (cdr (assoc :breakfast-cereal *object-grasping-arms*))
                                 (:left
                                  (with-slots (cl-tf:w cl-tf:z)
                                      (cl-tf:euler->quaternion :az (/ pi -3.0))
                                    (list 0 0 cl-tf:z cl-tf:w)))
                                 (:right
                                  (with-slots (cl-tf:w cl-tf:z)
                                      (cl-tf:euler->quaternion :az (/ pi 3.0))
                                    (list 0 0 cl-tf:z cl-tf:w)))
                                 (otherwise '(0 0 0 1)))
                               '(0 0 0 1))))
    (:cup . ((-0.79 1.35 0.9) (0 0 0.7071 0.7071)))
    (:bowl . ((-0.76 1.19 0.88) (0 0 0.7071 0.7071)))
    (:spoon . ((-0.78 1.5 0.86) (0 0 0 1)))
    (:milk . ((-0.75 1.7 0.95) (0 0 0.7071 0.7071)))))

(defun spawn-objects-on-sink-counter (&optional (spawning-poses *object-spawning-poses*))
  (btr-utils:kill-all-objects)
  (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo")
  (btr:detach-all-objects (btr:get-robot-object))
  (let ((object-types '(:breakfast-cereal
                        ;; :cup
                        :bowl
                        ;; :spoon :milk
                        )))
    ;; spawn objects at default poses
    (let ((objects (mapcar (lambda (object-type)
                             (btr-utils:spawn-object
                              (intern (format nil "~a-1" object-type) :keyword)
                              object-type
                              :pose (cdr (assoc object-type spawning-poses))))
                           object-types)))
      ;; stabilize world
      (btr:simulate btr:*current-bullet-world* 100)
      objects))
  (let ((spoon (btr:object btr:*current-bullet-world* :spoon-1)))    
    (when spoon
      (btr:attach-object (btr:object btr:*current-bullet-world* :kitchen)
                         spoon
                         "sink_area_left_upper_drawer_main"))))
