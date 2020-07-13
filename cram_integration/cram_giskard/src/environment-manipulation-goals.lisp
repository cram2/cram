;;;
;;; Copyright (c) 2020, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(defun make-environment-manipulation-goal (arm-left-or-right-or-both
                                           handle-link)
  (declare (type (or list keyword) arm-left-or-right-or-both)
           (type string handle-link))
  (unless (listp arm-left-or-right-or-both)
    (setf arm-left-or-right-or-both (list arm-left-or-right-or-both)))
  (roslisp:make-message
   'giskard_msgs-msg:movegoal
   :type (roslisp:symbol-code 'giskard_msgs-msg:MoveGoal :plan_only)
   :cmd_seq (vector
             (roslisp:make-message
              'giskard_msgs-msg:movecmd
              :constraints
              (map
               'vector #'identity
               (cons
                (roslisp:make-message
                 'giskard_msgs-msg:constraint
                 :type
                 "UpdateGodMap"
                 :parameter_value_pair
                 (let ((stream (make-string-output-stream)))
                   (yason:encode
                    (cut:recursive-alist-hash-table
                     `(("updates"
                        . (("rosparam"
                            . (("general_options"
                                . (("joint_weights"
                                    . (("odom_x_joint" . 0.0001)
                                       ("odom_y_joint" . 0.0001)
                                       ("odom_z_joint" . 0.0001))))))))))
                     :test #'equal)
                    stream)
                   (get-output-stream-string stream)))
                (mapcar (lambda (arm)
                          (let ((tool-frame
                                  (cut:var-value
                                   '?frame
                                   (car (prolog:prolog
                                         `(and (rob-int:robot ?robot)
                                               (rob-int:robot-tool-frame
                                                ?robot ,arm ?frame)))))))
                            (when (cut:is-var tool-frame)
                              (error "[giskard] Tool frame was not defined."))
                            (roslisp:make-message
                             'giskard_msgs-msg:constraint
                             :type
                             "Open"
                             :parameter_value_pair
                             (let ((stream (make-string-output-stream)))
                               (yason:encode
                                (cut:recursive-alist-hash-table
                                 `(("object_name"
                                    . ,(roslisp-utilities:rosify-underscores-lisp-name
                                        (man-int:current-environment-symbol)))
                                   ("tip"
                                    . ,tool-frame)
                                   ("handle_link"
                                    . ,handle-link))
                                 :test #'equal)
                                stream)
                               (get-output-stream-string stream)))))
                        arm-left-or-right-or-both)))))))

(defun make-grasp-bar-goal (arm-left-or-right-or-both
                            tip-grasp-axis bar-axis
                            bar-center
                            bar-length
                            root-link)
  (declare (type (or list keyword) arm-left-or-right-or-both)
           (type cl-transforms-stamped:vector-stamped tip-grasp-axis bar-axis)
           (type cl-transforms-stamped:point-stamped bar-center)
           (type string root-link)
           (type number bar-length))
  (unless (listp arm-left-or-right-or-both)
    (setf arm-left-or-right-or-both (list arm-left-or-right-or-both)))
  (roslisp:make-message
   'giskard_msgs-msg:movegoal
   :type (roslisp:symbol-code 'giskard_msgs-msg:MoveGoal :plan_only)
   :cmd_seq (vector
             (roslisp:make-message
              'giskard_msgs-msg:movecmd
              :constraints
              (map
               'vector #'identity
               (cons
                (roslisp:make-message
                 'giskard_msgs-msg:constraint
                 :type
                 "UpdateGodMap"
                 :parameter_value_pair
                 (let ((stream (make-string-output-stream)))
                   (yason:encode
                    (cut:recursive-alist-hash-table
                     `(("updates"
                        . (("rosparam"
                            . (("general_options"
                                . (("joint_weights"
                                    . (("odom_x_joint" . 0.001)
                                       ("odom_y_joint" . 0.001)
                                       ("odom_z_joint" . 0.001))))))))))
                     :test #'equal)
                    stream)
                   (get-output-stream-string stream)))
                (mapcar (lambda (arm)
                          (let ((tool-frame
                                  (cut:var-value
                                   '?frame
                                   (car (prolog:prolog
                                         `(and (rob-int:robot ?robot)
                                               (rob-int:robot-tool-frame
                                                ?robot ,arm ?frame)))))))
                            (when (cut:is-var tool-frame)
                              (error "[giskard] Tool frame was not defined."))
                            (roslisp:make-message
                             'giskard_msgs-msg:constraint
                             :type
                             "GraspBar"
                             :parameter_value_pair
                             (let ((stream (make-string-output-stream)))
                               (yason:encode
                                (cut:recursive-alist-hash-table
                                 `(("tip_grasp_axis"
                                    . ,(to-hash-table tip-grasp-axis))
                                   ("bar_axis"
                                    . ,(to-hash-table bar-axis))
                                   ("bar_center"
                                    . ,(to-hash-table bar-center))
                                   ("tip"
                                    . ,tool-frame)
                                   ("bar_length"
                                    . ,bar-length)
                                   ("root"
                                    . ,root-link))
                                 :test #'equal)
                                stream)
                               (get-output-stream-string stream)))))
                        arm-left-or-right-or-both)))))))


(defun ensure-goal-reached (status)
  (when (eql status :preempted)
    (roslisp:ros-warn (giskard env-manip) "Giskard action preempted.")
    (return-from ensure-goal-reached))
  (when (eql status :timeout)
    (roslisp:ros-warn (giskard env-manip) "Giskard action timed out.")))

(defun call-environment-manipulation-action (&key
                                               (arm :right)
                                               (handle-link
                                                "iai_fridge_door_handle")
                                               action-timeout)
  (declare (type (or keyword list) arm)
           (type string handle-link)
           (type (or null number) action-timeout))
  (multiple-value-bind (result status)
      (actionlib-client:call-simple-action-client
       'giskard-action
       :action-goal (print (make-environment-manipulation-goal arm handle-link))
       :action-timeout action-timeout)
    (ensure-goal-reached status)
    (values result status)
    ;; return the joint state, which is our observation
    ;; (joints:full-joint-states-as-hash-table)
    ))

(defun call-grasp-bar-action (&key
                                (arm
                                 :right)
                                (tip-grasp-axis
                                 (cl-transforms-stamped:make-vector-stamped
                                  cram-tf:*robot-right-tool-frame* 0.0
                                  (cl-transforms:make-3d-vector 0 0 1)))
                                (bar-axis
                                 (cl-transforms-stamped:make-vector-stamped
                                  "iai_kitchen/iai_fridge_door_handle" 0.0
                                  (cl-transforms:make-3d-vector 0 0 1)))
                                (bar-center
                                 (cl-transforms-stamped:make-point-stamped
                                  "iai_kitchen/iai_fridge_door_handle" 0.0
                                  (cl-transforms:make-identity-vector)))
                                (bar-length
                                 0.4)
                                (root-link
                                 cram-tf:*odom-frame*)
                                action-timeout)
  (declare (type (or keyword list) arm)
           (type (or cl-transforms-stamped:vector-stamped null)
                 tip-grasp-axis bar-axis)
           (type (or cl-transforms-stamped:point-stamped null) bar-center)
           (type (or string null) root-link)
           (type (or number null) bar-length action-timeout))
  (multiple-value-bind (result status)
      (actionlib-client:call-simple-action-client
       'giskard-action
       :action-goal (print
                     (make-grasp-bar-goal arm tip-grasp-axis bar-axis bar-center
                                          bar-length root-link))
       :action-timeout action-timeout)
    (ensure-goal-reached status)
    (values result status)
    ;; return the joint state, which is our observation
    ;; (joints:full-joint-states-as-hash-table)
    ))
