;;; Copyright (c) 2013, Georg Bartels <georg.bartels@cs.uni-bremen.de>
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

(in-package :pr2-manip-pm)

(defvar *left-feature-constraints-config-pub* nil)
(defvar *left-feature-constraints-command-pub* nil)
(defvar *left-feature-controller-state-subscriber* nil)
(defvar *right-feature-constraints-config-pub* nil)
(defvar *right-feature-constraints-command-pub* nil)
(defvar *right-feature-controller-state-subscriber* nil)

(defvar *left-feature-controller-state-active-fluent* nil)
(defvar *right-feature-controller-state-active-fluent* nil)

(defun init-feature-constraints-controller ()
  "Initializes all relevant ROS communication channels to command feature constraints to the feature constraint controllers of the left and right arm of the PR2. As usual, will be called during 'cram-roslisp-common:startup-ros' because it should be registered as an init-function."
  (setf *left-feature-constraints-command-pub*
        (roslisp:advertise
         "/left_arm_feature_controller/constraint_command"
         "constraint_msgs/ConstraintCommand"))
  (setf *left-feature-constraints-config-pub*
        (roslisp:advertise
         "/left_arm_feature_controller/constraint_config"
         "constraint_msgs/ConstraintConfig"
         :latch t))
  (setf *left-feature-controller-state-active-fluent*
        (cram-language:make-fluent :name :left-feature-controller-state-fluent
                                   :allow-tracing nil))
  (setf *left-feature-controller-state-subscriber*
        (roslisp:subscribe
         "/left_arm_feature_controller/constraint_state"
         "constraint_msgs/ConstraintState"
         #'left-feature-constraints-controller-state-callback))
  (setf *right-feature-constraints-command-pub*
        (roslisp:advertise
         "/right_arm_feature_controller/constraint_command"
         "constraint_msgs/ConstraintCommand"))
  (setf *right-feature-constraints-config-pub*
        (roslisp:advertise
         "/right_arm_feature_controller/constraint_config"
         "constraint_msgs/ConstraintConfig"
         :latch t))
  (setf *right-feature-controller-state-active-fluent*
        (cram-language:make-fluent :name :right-feature-controller-state-fluent
                                   :allow-tracing nil))
  (setf *right-feature-controller-state-subscriber*
        (roslisp:subscribe
         "/right_arm_feature_controller/constraint_state"
         "constraint_msgs/ConstraintState"
         #'right-feature-constraints-controller-state-callback)))

(register-ros-init-function init-feature-constraints-controller)

(defun left-feature-constraints-controller-state-callback (msg)
  "Checks whether all weight entries in 'msg' are smaller than 1.0. If yes a pulse on the fluent for the feature constraints controller of the left arm is sent."
  (roslisp:with-fields (weights) msg
    (let ((max-weight (loop for i from 0 below (length weights)
                            for weight = (elt weights i)
                            maximizing weight into max-weight
                            finally (return max-weight))))
      (cond ((< max-weight 1.0)
             ;; All weights are < 1.0, meaning that all constraints are
             ;; satisfied.
             (setf (cram-language:value *left-feature-controller-state-active-fluent*) T)
             (cram-language:pulse *left-feature-controller-state-active-fluent*))
            (t (setf (cram-language:value
                      *left-feature-controller-state-active-fluent*) nil))))))

(defun right-feature-constraints-controller-state-callback (msg)
  "Checks whether all weight entries in 'msg' are smaller than 1.0. If yes a pulse on the fluent for the feature constraints controller of the right arm is sent."
  (roslisp:with-fields (weights) msg
    (let ((max-weight (loop for i from 0 below (length weights)
                            for weight = (elt weights i)
                            maximizing weight into max-weight
                            finally (return max-weight))))
      (cond ((< max-weight 1.0)
             ;; All weights are < 1.0, meaning that all constraints are
             ;; satisfied.
             (setf (cram-language:value *right-feature-controller-state-active-fluent*) T)
             (cram-language:pulse *right-feature-controller-state-active-fluent*))
            (t (setf (cram-language:value
                      *right-feature-controller-state-active-fluent*) nil))))))

(defun wait-for-feature-controller (side &optional (timeout nil))
  "Waits 'timeout' seconds for the fluent watching the feature constraints controller of arm 'side' to become true. Returns nil if a timeout occured, otherwise returns something non-nil."
  (let ((fluent (ecase side
                  (:left *left-feature-controller-state-active-fluent*)
                  (:right *right-feature-controller-state-active-fluent*))))
    (cram-language:wait-for fluent :timeout timeout)))

(defun send-constraints-config (constraints side)
  "Takes a list of constraints 'constraints' and sends the resulting configuration-msg to the feature constraints controller of arm 'side' to prepare it for a subsequent command."
  (let ((publisher (ecase side
                     (:left *left-feature-constraints-config-pub*)
                     (:right *right-feature-constraints-config-pub*)))
        (controller-id (ecase side
                         (:left "pr2_left_arm_feature_controller")
                         (:right "pr2_right_arm_feature_controller"))))
    (roslisp:publish
     publisher
     (cram-fccl:feature-constraints->config-msg constraints controller-id))))

(defun send-constraints-command (constraints side)
  "Takes a list of constraints 'constraints' and sends the resulting command-msg to the feature constraints controller of arm 'side' to start the controller."
  (let ((publisher (ecase side
                     (:left *left-feature-constraints-command-pub*)
                     (:right *right-feature-constraints-command-pub*)))
        (controller-id (ecase side
                         (:left "pr2_left_arm_feature_controller")
                         (:right "pr2_right_arm_feature_controller"))))
    (roslisp:publish
     publisher
     (cram-fccl:feature-constraints->command-msg constraints controller-id))))

(defun start-velocity-resolved-controllers (side)
  "Makes the pr2_controller_manager start the controller plugin for the velocity-resolved arm controller on side 'side', and stop the standard position-resolved controller plugin on the same arm."
  (let ((pre (ecase side
               (:left "l")
               (:right "r"))))
    (switch-controller `(,(concatenate 'string pre "_arm_vel"))
                       `(,(concatenate 'string pre "_arm_controller")))))

(defun shutdown-velocity-resolved-controllers (side)
  "Makes the pr2_controller_manager stop the controller plugin for the velocity-resolved arm controller on side 'side', and start the standard position-resolved controller plugin on the same arm."
  (let ((pre (ecase side
               (:left "l")
               (:right "r"))))
    (switch-controller `(,(concatenate 'string pre "_arm_controller"))
                       `(,(concatenate 'string pre "_arm_vel")))))

(defun stop-velocity-resolved-controllers (side)
  "Stops the velocity-resolved controllers of arm 'side' by sending them a zero-velocity command. Should be used as a quasi-safety-stop because this stop will not consider maximum velocities limits of the arm."
  ;TODO(Georg): Implement me.
  (declare (ignore side)))

(defun stop-feature-controller (side)
  "Gets the current dimensions feature constraints controller or arm 'side', and sets the weights of all constraints to zero to deactive the controller."
  ;TODO(Georg): Implement me.
  (declare (ignore side)))

(defun execute-constraints-motion (constraints side &optional (shutdown-controllers-afterwards nil))
  "Takes a set of 'constraints' and configures, starts, waits-for-finish, and stops the feature constraints controller of arm 'side' with said constraints."
  (declare (type list constraints))
  (multiple-value-bind (undefined-features unknown-frames)
      (get-unknown-frame-ids-of-features constraints :timeout 3.0 :time 0.0)
    (declare (ignore undefined-features)
             (type list unknown-frames))
    (when unknown-frames
      (error 
       'simple-error
       :format-control "Can't execute constraint-based motion. The TF-queries for the following frame-ids timed out: ~a."
       :format-arguments unknown-frames))
    (send-constraints-config constraints side)
    (send-constraints-command constraints side)
    (start-velocity-resolved-controllers side)
    (sleep 0.3) ;;another hack screaming for an action interface
    (wait-for-feature-controller side)
    (when shutdown-controllers-afterwards
      (shutdown-velocity-resolved-controllers side))))

(defun get-unknown-frame-ids-of-features (constraints &key (source-frame "/base_link") (timeout 1.0) (time (roslisp:ros-time)))
  "Takes a list of 'constraints' and checks whether any of the contained geometric features have frame-ids for which a TF query at 'time' from 'source-frame' timed out after 'timeout' seconds. Returns a list of the features with such frame-ids and a list of the unknown frame-ids. If all features are well-defined nil and nil are returned."
  (declare (type list constraints)
           (type string source-frame))
  (let* ((features 
           (remove-duplicates
            (concatenate
             'list
             (mapcar #'cram-feature-constraints:tool-feature constraints)
             (mapcar #'cram-feature-constraints:world-feature constraints))))
         (frame-ids
           (mapcar #'cram-feature-constraints:frame-id
                   (remove-duplicates features
                                      :test #'string-equal
                                      :key #'cram-feature-constraints:frame-id)))
         (unknown-frame-ids
           (remove-if #'(lambda (frame)
                          (cl-tf:wait-for-transform
                           *tf* :timeout timeout :time time
                                :source-frame source-frame :target-frame frame))
                      frame-ids))
         (undefined-features
           (remove-if-not
            #'(lambda (feature)
                (some (lambda (frame-id)
                        (string-equal frame-id 
                                      (cram-feature-constraints:frame-id feature)))
                      unknown-frame-ids))
            features)))
    (values undefined-features unknown-frame-ids)))
