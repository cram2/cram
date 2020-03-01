;;;
;;; Copyright (c) 2016, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :giskard)

(defun make-giskard-neck-cartesian-action-goal (poi base-frame camera-frame)
  (declare (type cl-tf:point-stamped poi))
  (roslisp:make-message
   'giskard_msgs-msg:MoveGoal
   :type (roslisp:symbol-code 'giskard_msgs-msg:MoveGoal :plan_and_execute)
   :cmd_seq (vector
             (roslisp:make-message
              'giskard_msgs-msg:movecmd
              :constraints
              (vector (roslisp:make-message
                       'giskard_msgs-msg:constraint
                       :type
                       "Pointing"
                       :parameter_value_pair
                       (let ((stream (make-string-output-stream)))
                         (yason:encode
                          (alexandria:alist-hash-table
                           `(("root" . ,base-frame)
                             ("tip" . ,camera-frame)
                             ("goal_point" . ,(yason:encode-object poi)))
                           :test #'equalp)
                          stream)
                         (get-output-stream-string stream))))
              :collisions (vector (roslisp:make-message
                                   'giskard_msgs-msg:collisionentry
                                   :type (roslisp:symbol-code
                                          'giskard_msgs-msg:collisionentry
                                          :avoid_all_collisions)))))))

(defun ensure-giskard-neck-poi-goal-reached (result status goal-poi
                                              goal-frame convergence-delta-xy)
  (when (eql status :preempted)
    (roslisp:ros-warn (low-level giskard) "Giskard action preempted with result ~a" result)
    (return-from ensure-giskard-neck-poi-goal-reached))
  (when (eql status :timeout)
    (roslisp:ros-warn (low-level giskard-cart) "Giskard action timed out."))
  (when (eql status :aborted)
    (roslisp:ros-warn (low-level giskard-cart) "Giskard action aborted! With result ~a" result))
  (when goal-poi
    (let* ((poi-in-frame
             (cram-tf:ensure-pose-in-frame
              (point-stamped->pose-stamped goal-poi)
              goal-frame
              :use-zero-time t))
           (goal-dist (max (abs (cl-transforms:x (cl-transforms:origin poi-in-frame)))
                           (abs (cl-transforms:y (cl-transforms:origin poi-in-frame))))))
    (unless (<= goal-dist convergence-delta-xy)
      (cpl:fail 'common-fail:manipulation-goal-not-reached
                :description (format nil "Giskard did not converge to goal:
~a should have been at ~a with delta-xy of ~a."
                                     goal-frame goal-poi
                                     convergence-delta-xy))))))

(defun call-giskard-poi-neck-action (&key poi action-timeout
                                       (pose-base-frame cram-tf:*odom-frame*)
                                       (convergence-delta-xy *giskard-convergence-delta-xy*))
  (declare (type (or null cl-transforms-stamped:pose-stamped) poi)
           (type (or null number) action-timeout convergence-delta-xy)
           (type (or null string) pose-base-frame))
  (if poi
      (let* ((goal-pose (cram-tf:ensure-pose-in-frame poi pose-base-frame :use-zero-time T))
             (goal-poi (cl-tf:make-point-stamped (cl-tf:frame-id goal-pose)
                                                   (cl-tf:stamp goal-pose)
                                                   (cl-tf:origin goal-pose)))
             (camera-tool-frame (cut:var-value '?frame
                                               (cut:lazy-car
                                                (prolog:prolog `(and (rob-int:robot ?robot)
                                                                     (rob-int:camera-frame ?robot ?frame)))))))
        (cram-tf:visualize-marker goal-pose :r-g-b-list '(1 0 1))
        (multiple-value-bind (result status)
            (let ((goal (make-giskard-neck-cartesian-action-goal
                         goal-poi
                         pose-base-frame
                         camera-tool-frame)))
              (actionlib-client:call-simple-action-client
               'giskard-action
               :action-goal goal
               :action-timeout action-timeout))
          (ensure-giskard-neck-poi-goal-reached result status goal-poi camera-tool-frame
                                                convergence-delta-xy)
          (values result status)
          ;; return the joint state, which is our observation
          (joints:full-joint-states-as-hash-table)))
      ;; return NIL as observation if the goal is empty
      (and (roslisp:ros-info (giskard-pm giskard-cart) "Got an empty goal...")
           NIL)))

(defmethod yason:encode-object ((point cl-transforms-stamped:point-stamped))
  (yason:encode
   (alexandria:alist-hash-table
    `(("header"
       .
       ,(alexandria:alist-hash-table
         `(("stamp"
            .
            ,(alexandria:alist-hash-table
              `(("secs" . 0)
                ("nsecs" . 0))
              :test #'equal))
           ("frame_id" . ,(cl-tf:frame-id point))
           ("seq" . 0))
         :test #'equal))
      ("point"
       .
       ,(alexandria:alist-hash-table
         `(("x" . ,(cl-tf:x point))
           ("y" . ,(cl-tf:y point))
           ("z" . ,(cl-tf:z point)))
         :test #'equal))))))

(defun point-stamped->pose-stamped (poi)
  (cl-tf:make-pose-stamped
   (cl-tf:frame-id poi) (cl-tf:stamp poi)
   (cl-tf:make-3d-vector (cl-tf:x poi) (cl-tf:y poi) (cl-tf:z poi))
   (cl-tf:make-identity-rotation)))
