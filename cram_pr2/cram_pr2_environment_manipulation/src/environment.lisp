;;;
;;; Copyright (c) 2018, Christopher Pollok <cpollok@uni-bremen.de>
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

(in-package :pr2-em)

(defun get-current-environment ()
  (btr:object btr:*current-bullet-world* :kitchen)
  ;; (find 'btr::urdf-semantic-map-object
  ;;       (btr:objects btr:*current-bullet-world*)
  ;;       :key #'type-of)
  )

(defun get-urdf-link-pose (name btr-environment)
  (when (symbolp name)
    (setf name
          (roslisp-utilities:rosify-underscores-lisp-name name)))
  (btr:pose
   (btr:rigid-body
    (btr:object btr:*current-bullet-world*
                btr-environment)
    (btr::make-rigid-body-name
     (string-upcase btr-environment)
     name))))

(defun get-container-link (container-name btr-environment)
  (when (symbolp container-name)
    (setf container-name
          (roslisp-utilities:rosify-underscores-lisp-name container-name)))
  (gethash container-name
           (cl-urdf:links
            (btr:urdf
             (btr:object btr:*current-bullet-world*
                         btr-environment)))))

(defun get-container-joint-type (container-name btr-environment)
  (find-container-joint-type-under-joint
   (cl-urdf:from-joint
    (get-container-link container-name
                        btr-environment))))

(defun find-container-joint-type-under-joint (joint)
  "Return the first joint type different from :FIXED under the given JOINT."
  (if (eq :FIXED (cl-urdf:joint-type joint))
      (find-container-joint-type-under-joint
       (car (cl-urdf:to-joints (cl-urdf:child joint))))
      (cl-urdf:joint-type joint)))

(defun get-handle-link (container-name btr-environment)
  (when (symbolp container-name)
    (setf container-name
          (roslisp-utilities:rosify-underscores-lisp-name container-name)))
  (find-handle-under-link
   (get-container-link container-name btr-environment)))

(defun find-handle-under-link (link)
  (if (search "handle" (cl-urdf:name link))
      link
      (find-handle-under-link (cl-urdf:child
                     (car (cl-urdf:to-joints link))))))

(defun get-joint-position (joint btr-environment)
  (gethash (cl-urdf:name joint)
           (btr:joint-states (btr:object btr:*current-bullet-world* btr-environment))))

(defun get-connecting-joint (part)
  "Returns the connecting (moveable) joint of `part', which can be either
  a link or a joint of an URDF."
  (when part
    (if (typep part 'cl-urdf:joint)
        (or
         (when (not (eql (cl-urdf:joint-type part) :FIXED))
           part)
         (get-connecting-joint (cl-urdf:parent part)))
        (when (typep part 'cl-urdf:link)
          (get-connecting-joint (cl-urdf:from-joint part))))))


(defun get-manipulated-pose (link-name joint-position btr-environment &key relative)
  "Returns the pose of a link based on its connection joint position
  `joint-position'. If `relative' is T, the actual value is calculated
  by `joint-position' * <joint maximal value>. This function returns two
  values, the new pose of the link and the joint that was changed."
  (let ((link (get-container-link link-name btr-environment)))
    (when (typep link 'cl-urdf:link)
      (let ((joint (get-connecting-joint link)))
        (when joint
          (values
           (case (cl-urdf:joint-type joint)
             (:prismatic
              (cl-transforms:transform->pose
               (cl-transforms:transform*
                (cl-transforms:pose->transform (get-urdf-link-pose link-name btr-environment))
                (cl-transforms:make-transform
                 (cl-transforms:v*
                  (cl-urdf:axis joint)
                  (-
                   (if relative
                       (* joint-position
                          (cl-urdf:upper (cl-urdf:limits joint)))
                       joint-position)
                   (get-joint-position joint btr-environment)))
                 (cl-transforms:make-identity-rotation)))))
             (:revolute
              (let* ((rotation
                       (cl-transforms:axis-angle->quaternion
                        (cl-transforms:make-3d-vector 0 0 1) ;; might be some other axis
                        (if relative
                            (* joint-position
                               (cl-urdf:upper (cl-urdf:limits joint)))
                            joint-position)))
                     (link-transform
                       (cl-transforms:pose->transform
                        (get-urdf-link-pose link-name btr-environment)))
                     (joint-transform
                       (cl-transforms:pose->transform
                        (get-urdf-link-pose (cl-urdf:name (cl-urdf:child joint)) btr-environment)))
                     (joint-to-handle
                       (cl-transforms:transform-diff
                        link-transform
                        joint-transform)))
                (cl-transforms:transform-pose
                 (cl-transforms:make-transform
                  (cl-transforms:rotate
                   rotation
                   (cl-transforms:translation joint-to-handle))
                  (cl-transforms:make-identity-rotation))
                 (get-urdf-link-pose link-name btr-environment))
              )))
           joint))))))
