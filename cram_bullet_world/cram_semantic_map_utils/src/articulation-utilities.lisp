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

(in-package :sem-map-utils)

(defun get-connecting-joint (part)
  "Returns the connecting joint of the part with name `part-name'."
  (when part
    (or (find-if (lambda (part)
                   (typep part 'semantic-map-joint))
                 (sub-parts part))
        (get-connecting-joint (parent part)))))

(defun get-articulated-position (semantic-map part-name joint-position &key relative)
  "Returns the pose of `part' based on its connecting joint position
  `joint-position'. If `relative' is T, the actual value is calculated
  by `joint-position' * <joint maximal value>. This method returns two
  values, the new pose of the object and the joint that was changed."
  (let ((part (semantic-map-part
               semantic-map part-name :recursive t)))
    (when (and part (typep part 'semantic-map-geom))
      (let ((joint (get-connecting-joint part)))
        (when joint
          (values
           (cl-transforms:transform->pose
            (cl-transforms:transform*
             (cl-transforms:pose->transform (pose part))
             (cl-transforms:make-transform
              (cl-transforms:v*
               (joint-direction joint)
               (-
                (if relative
                    (* joint-position
                       (joint-maximal-value joint))
                    joint-position)
                (joint-position joint)))
              (cl-transforms:make-identity-rotation))))
           joint))))))

(defun get-connecting-joint-limits (semantic-map part-name)
  "Returns the limits of the connecting joints as a list. The first
element is the minimal joint limit, the second the maximal limit."
  (let ((part (semantic-map-part
               semantic-map part-name :recursive t)))
    (when part
      (let ((joint (get-connecting-joint part)))
        (list (joint-minimal-value joint)
              (joint-maximal-value joint))))))

(defun update-articulated-object-poses (semantic-map part-name joint-position &key relative)
  "Updates all object poses that belong to the articulated object
`part-name' is part of according to the joint values specified "
  (let ((articulated-object (get-top-level-object
                             (semantic-map-part
                              semantic-map part-name :recursive t))))
    (when articulated-object
      (mapcar (lambda (sub-object)
                (multiple-value-bind
                      (new-pose joint)
                    (get-articulated-position
                     semantic-map (name sub-object) joint-position
                     :relative relative)
                  (when new-pose
                    (update-pose sub-object new-pose :recursive t)
                    (setf (joint-position joint) joint-position))))
              (sub-parts articulated-object)))))
