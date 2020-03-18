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
  
