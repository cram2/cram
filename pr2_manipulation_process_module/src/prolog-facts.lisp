;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(defvar *semantic-map* nil)

(defun get-semantic-map ()
  (or *semantic-map*
      (setf *semantic-map* (sem-map-utils:get-semantic-map))))

(defun get-connecting-joint (part)
  "Returns the connecting joint of the part with name `part-name'."
  (when part
    (or (find-if (lambda (part)
                   (typep part 'sem-map-utils:semantic-map-joint))
                 (sem-map-utils:sub-parts part))
        (get-connecting-joint (sem-map-utils:parent part)))))

(defun get-articulated-position (part-name relative-joint-position)
  "Returns the pose of `part' if its connecting joint is set to
  `relative-joint-position' * <joint maximal value>."
  (let ((part (sem-map-utils:semantic-map-part
                (get-semantic-map) part-name :recursive t)))
    (when part
      (let ((joint (get-connecting-joint part)))
        (when joint
          (cl-transforms:transform->pose
           (cl-transforms:transform*
            (cl-transforms:pose->transform (sem-map-utils:pose part))
            (cl-transforms:make-transform
             (cl-transforms:v* (sem-map-utils:joint-direction joint)
                               (* relative-joint-position
                                  (sem-map-utils:joint-maximal-value joint)))
             (cl-transforms:make-identity-rotation)))))))))

(defun get-articulated-gripper-position (part-name relative-joint-position
                                         &key (tool-length *grasp-distance*))
  "Gets the position of the gripper to reach `part-name'. Uses its
  connecting joint and
  `relative-joint-position'. `relative-joint-position' is a value
  between 0.0 and 1.0 and the joint's maximal value is multiplied by
  it to get the actual pose to grasp."
  (let ((object-pose
          (get-articulated-position part-name relative-joint-position)))
    (when object-pose
      (cl-transforms:transform->pose
       (cl-transforms:transform*
        (cl-transforms:pose->transform object-pose)
        (cl-transforms:make-transform
         (cl-transforms:make-3d-vector (- tool-length) 0 0)
         (cl-transforms:make-quaternion 0 0 1 0)))))))

(def-fact-group manipulation (trajectory-point)

  (<- (trajectory-point ?designator ?point)
    (trajectory-desig? ?designator)
    (desig-prop ?designator (to open))
    (desig-prop ?designator (handle ?handle))
    (desig-prop ?handle (name ?handle-name))
    (member ?joint-pose (0.0 1.0))
    (lisp-fun get-articulated-gripper-position
              ?handle-name ?joint-pose ?point)))

