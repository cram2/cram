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
                   (typep part 'sem-map-utils:semantic-map-joint))
                 (sem-map-utils:sub-parts part))
        (get-connecting-joint (sem-map-utils:parent part)))))

(defun get-articulated-position (semantic-map part-name joint-position &key relative)
  "Returns the pose of `part' based on its connecting joint position
  `joint-position'. If `relative' is T, the actual value is calculated
  by `joint-position' * <joint maximal value>."
  (let ((part (sem-map-utils:semantic-map-part
               semantic-map part-name :recursive t)))
    (when part
      (let ((joint (get-connecting-joint part)))
        (when joint
          (cl-transforms:transform->pose
           (cl-transforms:transform*
            (cl-transforms:pose->transform (sem-map-utils:pose part))
            (cl-transforms:make-transform
             (cl-transforms:v*
              (sem-map-utils:joint-direction joint)
              (if relative
                  (* joint-position
                     (sem-map-utils:joint-maximal-value joint))
                  joint-position))
             (cl-transforms:make-identity-rotation)))))))))

(defun update-articulated-object-poses (semantic-map part-name joint-position &key relative)
  "Updates all object poses that belong to the articulated object
`part-name' is part of according to the joint values specified "
  (let ((articulated-object (sem-map-utils:get-top-level-object
                             (sem-map-utils:semantic-map-part
                              semantic-map part-name :recursive t))))
    (when articulated-object
      (mapcar (lambda (sub-object)
                (sem-map-utils:update-pose
                 sub-object (sem-map-utils:get-articulated-position
                             (get-semantic-map) (sem-map-utils:name sub-object)
                             joint-position :relative relative)
                 :recursive t))
              (sem-map-utils:sub-parts articulated-object)))))
