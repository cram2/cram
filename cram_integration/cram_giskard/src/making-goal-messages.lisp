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

(defparameter *avoid-joint-limits-percentage* 40)
(defparameter *prefer-base-low-cost* 0.001)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; UTILS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun make-constraints-vector (&rest entries)
  (if (every #'listp entries)
      (apply #'vector (alexandria:flatten entries))
      (apply #'vector (remove NIL entries))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; JSON CONSTRAINTS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun make-avoid-joint-limits-constraint (&optional (percentage
                                                      *avoid-joint-limits-percentage*))
  (declare (type number percentage))
  (roslisp:make-message
   'giskard_msgs-msg:constraint
   :type
   "AvoidJointLimits"
   :parameter_value_pair
   (alist->json-string
    `(("percentage" . ,percentage)))))

(defun make-prefer-base-constraint (&optional (base-weight
                                               *prefer-base-low-cost*))
  (roslisp:make-message
   'giskard_msgs-msg:constraint
   :type
   "UpdateGodMap"
   :parameter_value_pair
   (alist->json-string
    `(("updates"
       . (("rosparam"
           . (("general_options"
               . (("joint_weights"
                   . (("odom_x_joint" . ,base-weight)
                      ("odom_y_joint" . ,base-weight)
                      ("odom_z_joint" . ,base-weight)))))))))))))

(defun make-align-planes-constraint (root-frame tip-frame root-vector tip-vector)
  (declare (type string root-frame tip-frame)
           (type cl-transforms-stamped:vector-stamped root-vector tip-vector))
  (roslisp:make-message
   'giskard_msgs-msg:constraint
   :type
   "AlignPlanes"
   :parameter_value_pair
   (alist->json-string
    `(("root" . ,root-frame)
      ("tip" . ,tip-frame)
      ("root_normal" . ,(to-hash-table root-vector))
      ("tip_normal" . ,(to-hash-table tip-vector))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; NON-JSON CONSTRAINTS ;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun make-simple-cartesian-constraint (root-link tip-link pose-stamped)
  (declare (type string root-link tip-link)
           (type cl-transforms-stamped:pose-stamped pose-stamped))
  (list (roslisp:make-message
         'giskard_msgs-msg:cartesianconstraint
         :type (roslisp:symbol-code
                'giskard_msgs-msg:cartesianconstraint
                :translation_3d)
         :root_link root-link
         :tip_link tip-link
         :goal (cl-transforms-stamped:to-msg pose-stamped))
        (roslisp:make-message
         'giskard_msgs-msg:cartesianconstraint
         :type (roslisp:symbol-code
                'giskard_msgs-msg:cartesianconstraint
                :rotation_3d)
         :root_link root-link
         :tip_link tip-link
         :goal (cl-transforms-stamped:to-msg pose-stamped))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; COLLISIONS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun make-allow-all-collision ()
  (roslisp:make-message
   'giskard_msgs-msg:collisionentry
   :type (roslisp:symbol-code
          'giskard_msgs-msg:collisionentry
          :allow_all_collisions)))

(defun make-avoid-all-collision (minimal-distance)
  (declare (type number minimal-distance))
  (roslisp:make-message
   'giskard_msgs-msg:collisionentry
   :type (roslisp:symbol-code
          'giskard_msgs-msg:collisionentry
          :avoid_all_collisions)
   :min_dist minimal-distance))

(defun make-allow-hand-collision (hands body-b body-b-link)
  (declare (type list hands)
           (type (or null keyword) body-b body-b-link))
  (roslisp:make-message
   'giskard_msgs-msg:collisionentry
   :type (roslisp:symbol-code
          'giskard_msgs-msg:collisionentry
          :allow_collision)
   :robot_links (make-constraints-vector
                 (mapcan (lambda (hand)
                           (cut:var-value
                            '?hand-links
                            (car
                             (prolog:prolog
                              `(and (rob-int:robot ?robot)
                                    (rob-int:hand-links ?robot ,hand ?hand-links))))))
                         hands))
   :body_b (if body-b
               (roslisp-utilities:rosify-underscores-lisp-name body-b)
               (roslisp:symbol-code 'giskard_msgs-msg:collisionentry :all))
   :link_bs (vector
             (if body-b-link
                 (roslisp-utilities:rosify-underscores-lisp-name body-b-link)
                 (roslisp:symbol-code 'giskard_msgs-msg:collisionentry :all)))))
