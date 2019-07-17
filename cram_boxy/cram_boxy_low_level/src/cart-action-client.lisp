;;;
;;; Copyright (c) 2018, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
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

(in-package :boxy-ll)

(defparameter *cart-action-timeout* 10.0
  "How many seconds to wait before returning from cart action.")

(defparameter *cart-max-translational-vel* 0.2)
(defparameter *cart-ee-mass* 2.2)
(defparameter *cart-ee-center-of-gravity* '(0.0 0.0 0.19))
(defparameter *cart-impedance-list* '(400 400 400 400 400 400))

(actionlib-client:make-simple-action-client
 'dlr-cart-action
 "left_arm_cart"
 "iai_dlr_msgs/CartesianAction"
 *cart-action-timeout*)

(defun make-cart-action-goal (pose-stamped)
  (declare (type cl-transforms-stamped:pose-stamped pose-stamped))
  (roslisp:make-message
   'iai_dlr_msgs-msg:CartesianGoal
   :goal_pose (cl-transforms-stamped:to-msg pose-stamped)
   :max_trans_vel *cart-max-translational-vel*
   :ee_mass *cart-ee-mass*
   :ee_cog (apply #'vector *cart-ee-center-of-gravity*)
   :cart_imp (apply #'vector *cart-impedance-list*)))

(defun move-arm-cartesian (&key
                             goal-pose-stamped-left
                             goal-pose-stamped-right
                             (action-timeout *cart-action-timeout*))
  (declare (ignore goal-pose-stamped-right))
  (multiple-value-bind (result status)
      (actionlib-client:call-simple-action-client
       'dlr-cart-action
       :action-goal (make-cart-action-goal goal-pose-stamped-left)
       :action-timeout action-timeout)
    (roslisp:ros-info (cart-action) "left arm action finished.")
    (values result status)))

(defun move-arm-cartesian-example ()
  (move-arm-cartesian
   :goal-pose-stamped-left
   (cl-transforms-stamped:pose->pose-stamped
    "left_arm_0_link"
    0.0
    (cram-tf:list->pose
     '((-0.34 -0.03 0.7)
       (0.03797708824277d0 -0.31851065158844d0 -0.04735903814435d0 0.9459651708603d0))))))
