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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
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

(in-package :pr2-navigation-process-module)

(defparameter *navigation-enabled* t)

(defvar *base-twist-pub* nil)

(defun init-pr2-navigation-process-module ()
  (setf *base-twist-pub* (roslisp:advertise "/base_controller/command"
                                            "geometry_msgs/Twist")))

(roslisp-utilities:register-ros-init-function
 init-pr2-navigation-process-module)

(defun execute-navigation (desig)
  (let* ((target-pose-odom-combined (moveit:ensure-pose-stamped-transformed
                                     (reference desig) "/odom_combined"))
         (target-x (tf:x (tf:origin target-pose-odom-combined)))
         (target-y (tf:y (tf:origin target-pose-odom-combined)))
         (target-theta (tf:z (tf:orientation target-pose-odom-combined))))
    (let (;(controller-x (make-p-controller target-x 1.0 :max 0.5))
          ;(controller-y (make-p-controller target-y 1.0 :max 0.5))
          ;(controller-theta (make-p-controller target-theta 1.0 :max 0.5))
          (controller-complete
            (make-controller
             (lambda (x y theta)
               (let* ((sin-th (sin theta))
                      (cos-th (cos theta))
                      (cossinsqsm (+ (* sin-th sin-th) (* cos-th cos-th)))
                      (gains '(0.5 0.5 1.0))
                      (diff-x (- target-x x))
                      (diff-y (- target-y y))
                      (diff-theta (- target-theta theta)))
                 (list (* (first gains) (+ (/ (* cos-th diff-x) cossinsqsm)
                                           (/ (* sin-th diff-y) cossinsqsm)))
                       (* (second gains) (+ (- (/ (* sin-th diff-x) cossinsqsm))
                                            (/ (* cos-th diff-y) cossinsqsm)))
                       (* (third gains) diff-theta))))
             3 3))
          (plant (make-plant
                  (lambda (dx dy dtheta)
                    (let ((msg (roslisp:make-message
                                "geometry_msgs/Twist"
                                (x linear) dx
                                (y linear) dy
                                (z angular) dtheta)))
                      (roslisp:publish *base-twist-pub* msg)))
                  (lambda ()
                    (let* ((trafo (moveit::tf-transformation "/base_footprint"))
                           (x (tf:x (tf:translation trafo)))
                           (y (tf:y (tf:translation trafo)))
                           (theta (tf:z (tf:rotation trafo))))
                      `(,x ,y ,theta)))
                  3 3)))
      (control plant `(,controller-complete) ;;`(,controller-x ,controller-y ,controller-theta)
               (lambda (plant)
                 (let ((plant-state (funcall (state-function plant))))
                   (and (< (abs (- (first plant-state) target-x)) 0.02)
                        (< (abs (- (second plant-state) target-y)) 0.02)
                        (< (abs (- (third plant-state) target-theta)) 0.1))))
               0.01))))

(def-process-module pr2-navigation-process-module (goal)
  (when *navigation-enabled*
    (unwind-protect
         (progn
           (roslisp:ros-info (pr2-nav process-module)
                             "Using nav-pcontroller.")
           (execute-navigation (reference goal)))
      (roslisp:ros-info (pr2-nav process-module) "Navigation finished.")
      (cram-plan-knowledge:on-event (make-instance 'cram-plan-knowledge:robot-state-changed)))))
