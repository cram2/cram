;;;
;;; Copyright (c) 2020, Arthur Niedzwiecki <aniedz@cs.uni-bremen.de>
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

(defparameter *joint-constraint-default-weight* 9000)

(defun symbols->collision-objects (symbols)
  "Take symbol or list of symbols. Coerce to a vector of rosified names."
  (if symbols
      (if (listp symbols)
          (apply #'vector
                 (mapcar #'roslisp-utilities:rosify-underscores-lisp-name
                         symbols))
          (vector (roslisp-utilities:rosify-underscores-lisp-name
                   symbols)))
      (vector (roslisp:symbol-code
               'giskard_msgs-msg:collisionentry
               :all))))

(defun constraint-cartesian (pose tool-frame pose-base-frame)
  (list (roslisp:make-message
         'giskard_msgs-msg:cartesianconstraint
         :type (roslisp:symbol-code
                'giskard_msgs-msg:cartesianconstraint
                :translation_3d)
         :root_link pose-base-frame
         :tip_link tool-frame
         :goal (cl-transforms-stamped:to-msg pose))
        (roslisp:make-message
         'giskard_msgs-msg:cartesianconstraint
         :type (roslisp:symbol-code
                'giskard_msgs-msg:cartesianconstraint
                :rotation_3d)
         :root_link pose-base-frame
         :tip_link tool-frame
         :goal (cl-transforms-stamped:to-msg pose))))

(defun constraint-jointposition (joint-states &optional weights)
  (declare (type (or null list) joint-states)
           (type (or null list) weights))
  "Creates JointPosition constraint message for given joint-states and weights.
Accepts the following:
'((joint-1 joint-2 ...) (state-1 state-2 ...)) constraining to given states, or
'(joint-1 joint-2 ...) constraining to the current joint state
Returns empty vector when NIL joint-states is given.
Throws error when joints and states are of different length or a joint can't be found."
  (if (and joint-states (car joint-states))
      (flet ((list-typep (list type)
               (when (listp list)
                 (not (find nil (mapcar (alexandria:rcurry #'typep type) list))))))
        (let* (joints states)
          (cond (;; lock joints at given positions
                 (and (list-typep joint-states 'list)
                      (list-typep (first joint-states) 'string)
                      (list-typep (second joint-states) 'number))
                 (setf joints (first joint-states))
                 (setf states (second joint-states))
                 (unless (eq (length joints) (length states))
                   (error "[GISKARD CONSTRAINTS] Joints ~a and states ~a are not of the same length."
                          joints states)))
                ;; lock joints in current state if no positions are given
                ((list-typep joint-states 'string) 
                 (setf joints joint-states)
                 (setf states (joints:joint-positions joints))
                 (unless states
                   (error "[GISKARD CONSTRAINTS] Can't get joint state. Is the rosnode running?"))))
          (when (position 'nil (joints:joint-positions joints))
            (error "[GISKARD CONSTRAINTS] Joint ~a not found." (nth (position 'nil (joints:joint-positions joints))
                                                                    joints)))
          (let ((weights ;; Fill or strip weights list such that joints and weights have same length.
                  (if (< (length weights) (length joints))
                      (append weights (make-list (- (length joints) (length weights))
                                                 :initial-element *joint-constraint-default-weight*))
                      (subseq weights 0 (length joints)))))
            (if (eq 1 (length (remove-duplicates (mapcar #'length (list joints states weights)))))
               (coerce
                 (loop for joint in joints
                       for state in states
                       for weight in weights
                       collect (roslisp:make-message
                                'giskard_msgs-msg:constraint
                                :type
                                "JointPosition"
                                :parameter_value_pair
                                (let ((stream (make-string-output-stream)))
                                  (yason:encode
                                   (alexandria:alist-hash-table
                                    `(("joint_name" . ,joint)
                                      ("goal" . ,state)
                                      ("weight" . ,weight))
                                    :test #'equalp)
                                   stream)
                                  (get-output-stream-string stream))))
                 'vector)))))
      (progn (roslisp:ros-info (giskard constraint-joints) "No joints constrained.")
             (vector))))

(defun constraint-collision-allow-all ()
  (roslisp:make-message
   'giskard_msgs-msg:collisionentry
   :type (roslisp:symbol-code
          'giskard_msgs-msg:collisionentry
          :allow_all_collisions)))

(defun constraint-collision-avoid-all (&key (min-dist 0.1))
  (declare (type float min-dist))
  (roslisp:make-message
   'giskard_msgs-msg:collisionentry
   :type (roslisp:symbol-code
          'giskard_msgs-msg:collisionentry
          :avoid_all_collisions)
   :min_dist min-dist))

(defun constraint-collision-allow-hand (left right body-b &key link-bs)
  "Returns a giskard collision entry message.
Allow collision between left and/or right gripper and the given body.
`left'/`right': If not nil, the links of their respective gripper ignore collision.
`body-bs': List of body names for which to ignore collision with
`link-bs': List of links of the body, for which to ignore collision with.
           Avoid all links if nil."
  (declare (type (or null cl-tf:pose-stamped) left right)
           (type (or null list) link-bs)
           (type symbol body-b))
  (let (;; (link-bs-formatted
        ;;   (if link-bs
        ;;       (apply (alexandria:compose #'vector #'roslisp-utilities:rosify-underscores-lisp-name)
        ;;              link-bs)
        ;;       (vector (roslisp:symbol-code 'giskard_msgs-msg:collisionentry :all))))
        (robot-links (apply
                      #'vector
                      (append
                       (when left
                         (cut:var-value
                          '?hand-links
                          (car (prolog:prolog
                                `(and (rob-int:robot ?robot)
                                      (rob-int:hand-links ?robot :left
                                                          ?hand-links))))))
                       (when right
                         (cut:var-value
                          '?hand-links
                          (car (prolog:prolog
                                `(and (rob-int:robot ?robot)
                                      (rob-int:hand-links ?robot :right
                                                          ?hand-links))))))))))
    (roslisp:make-message
     'giskard_msgs-msg:collisionentry
     :type (roslisp:symbol-code 'giskard_msgs-msg:collisionentry :allow_collision)
     :robot_links robot-links
     :body_b (if body-b
                 (roslisp-utilities:rosify-underscores-lisp-name body-b)
                 (roslisp:symbol-code 'giskard_msgs-msg:collisionentry :all))
     :link_bs (symbols->collision-objects link-bs))))

(defun constraint-collision-allow-fingers (left right body-b &key link-bs)
  "Returns a giskard collision entry message.
Allow collision between left and/or right gripper and the given body.
`left'/`right': If not nil, the links of their respective gripper ignore collision.
`body-bs': List of body names for which to ignore collision with
`link-bs': List of links of the body, for which to ignore collision with.
           Avoid all links if nil."
  (declare (type (or null cl-tf:pose-stamped) left right)
           (type (or null list) link-bs)
           (type symbol body-b))
  (let (;; (link-bs-formatted
        ;;   (if link-bs
        ;;       (apply (alexandria:compose #'vector #'roslisp-utilities:rosify-underscores-lisp-name)
        ;;              link-bs)
        ;;       (vector (roslisp:symbol-code 'giskard_msgs-msg:collisionentry :all))))
        (robot-links (apply
                      #'vector
                      (append
                       (when left
                         (mapcar (alexandria:rcurry #'alexandria:assoc-value '?link)
                                 (cut:force-ll
                                  (prolog:prolog
                                   `(and (rob-int:robot ?robot)
                                         (rob-int:hand-links ?robot :left ?hand-links) 
                                         (member ?link ?hand-links)
                                         (rob-int:gripper-finger-link ?robot :left ?link))))))
                       (when right
                         (mapcar (alexandria:rcurry #'alexandria:assoc-value '?link)
                                 (cut:force-ll
                                  (prolog:prolog
                                   `(and (rob-int:robot ?robot)
                                         (rob-int:hand-links ?robot :right ?hand-links) 
                                         (member ?link ?hand-links)
                                         (rob-int:gripper-finger-link ?robot :right ?link))))))))))
    (roslisp:make-message
     'giskard_msgs-msg:collisionentry
     :type (roslisp:symbol-code 'giskard_msgs-msg:collisionentry :allow_collision)
     :robot_links robot-links
     :body_b (if body-b
                 (roslisp-utilities:rosify-underscores-lisp-name body-b)
                 (roslisp:symbol-code 'giskard_msgs-msg:collisionentry :all))
     :link_bs (symbols->collision-objects link-bs))))

(defun constraint-collision-allow-attached (attached-objects body-to-allow links-of-body-to-allow)
  "Returns a giskard collision entry message.
Allow collision between object attached to gripper and the given body, with specified link.
`attached-objects' Names of objects attached to the gripper.
`body-to-allow': Name of object to allow collision with.
`links-of-body-to-allow': List of links of the body, for which to ignore collision with.
                         Avoid all links if nil."
  (declare (type (or null symbol) body-to-allow)
           (type (or null symbol list) attached-objects links-of-body-to-allow))
  (roslisp:make-message
   'giskard_msgs-msg:collisionentry
   :type (roslisp:symbol-code
          'giskard_msgs-msg:collisionentry
          :allow_collision)
   :robot_links (symbols->collision-objects attached-objects)
   :body_b (if body-to-allow
               (roslisp-utilities:rosify-underscores-lisp-name body-to-allow)
               "kitchen")
   :link_bs (symbols->collision-objects links-of-body-to-allow)))


                     

(defun constraint-alignplanes ()
  :TODO
;; header:
;;   seq: 1
;;   stamp:
;;     secs: 1582889286
;;     nsecs: 621740102
;;   frame_id: ''
;; goal_id:
;;   stamp:
;;     secs: 1582889286
;;     nsecs: 621718883
;;   id: "/asdf-1-1582889286.622"
;; goal:
;;   type: 1
;;   cmd_seq:
;;     -
;;       constraints:
;;         -
;;           type: "AlignPlanes"
;;           parameter_value_pair: "{\"root\": \"odom_combined\", \"tip\": \"r_gripper_tool_frame\", \"tip_normal\":\
;;   \ {\"header\": {\"stamp\": {\"secs\": 0, \"nsecs\": 0}, \"frame_id\": \"r_gripper_tool_frame\"\
;;   , \"seq\": 0}, \"vector\": {\"y\": 0.0, \"x\": 0.0, \"z\": 1}}, \"root_normal\"\
;;   : {\"header\": {\"stamp\": {\"secs\": 0, \"nsecs\": 0}, \"frame_id\": \"odom_combined\"\
;;   , \"seq\": 0}, \"vector\": {\"y\": 0.0, \"x\": 0.0, \"z\": 1}}}"
;;       joint_constraints: []
;;       cartesian_constraints: []
;;       collisions: []

  )

(defun constraint-pointing (point)
  :TODO
  ;; tranfer code from neck-cartesian-interface
  )
  
