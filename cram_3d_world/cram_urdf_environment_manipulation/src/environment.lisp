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

(in-package :env-man)

(defun get-current-environment ()
  (btr:object btr:*current-bullet-world* :kitchen)
  ;; (find 'btr::urdf-semantic-map-object
  ;;       (btr:objects btr:*current-bullet-world*)
  ;;       :key #'type-of)
  )

(defun get-container-pose-and-transform (name btr-environment)
  "Return a list of the pose-stamped and transform-stamped of the object named
NAME in the environment BTR-ENVIRONMENT."
  (when (symbolp name)
    (setf name
          (roslisp-utilities:rosify-underscores-lisp-name name)))
  (let* ((urdf-pose (get-urdf-link-pose name btr-environment))
         (pose (cram-tf:ensure-pose-in-frame
                (cl-transforms-stamped:pose->pose-stamped
                 cram-tf:*fixed-frame*
                 0.0
                 urdf-pose)
                cram-tf:*robot-base-frame*
                :use-zero-time t))
         (transform (cram-tf:pose-stamped->transform-stamped pose name)))
    (list pose transform)))

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
      (reduce (lambda (&optional x y) (or x y))
              (mapcar 'find-handle-under-link
                      (mapcar 'cl-urdf:child (cl-urdf:to-joints link))))))

(defun get-joint-position (joint btr-environment)
  (gethash (cl-urdf:name joint)
           (btr:joint-states (btr:object btr:*current-bullet-world* btr-environment))))

(defun get-connecting-joint (part)
  "Returns the connecting (moveable) joint of `part', which can be either
   a link or a joint of an URDF."
  (or
   (get-connecting-joint-below part)
   (get-connecting-joint-above part)))

(defun get-connecting-joint-below (part)
  "Traverse the URDF downwards to find a connecting joint."
  (when part
    (if (typep part 'cl-urdf:joint)
        (or
         (when (not (eql (cl-urdf:joint-type part) :FIXED))
           part)
         (get-connecting-joint-below (cl-urdf:child part)))

        (when (typep part 'cl-urdf:link)
          (find-if
           (lambda (joint)
             (get-connecting-joint-below joint))
           (cl-urdf:to-joints part))))))

(defun get-connecting-joint-above (part)
  "Traverse the URDF upwards to find a connecting joint."
  (when part
    (if (typep part 'cl-urdf:joint)
        (or
         (when (not (eql (cl-urdf:joint-type part) :FIXED))
           part)
         (get-connecting-joint-above (cl-urdf:parent part)))

        (when (typep part 'cl-urdf:link)
          (get-connecting-joint-above (cl-urdf:from-joint part))))))

(defun get-manipulated-link (part)
  "Returns the link under the connecting joint. Which is the one actuallz being manipulated."
  (cl-urdf:child
   (get-connecting-joint part)))

(defun get-manipulated-pose (link-name joint-position btr-environment &key relative)
  "Returns the pose of a link based on its connection joint position
  JOINT-POSITION. If RELATIVE is T, the actual value is calculated
  by JOINT-POSITION * <joint maximal value>. This function returns two
  values, the new pose of the link and the joint that was changed."
  (when (not (floatp joint-position))
    (setf joint-position (float joint-position)))
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
                 (get-urdf-link-pose (cl-urdf:name (cl-urdf:child joint)) btr-environment))
              )))
           joint))))))

(defun get-handle-axis (container-designator)
  "Return either a vector with (1 0 0) for horizontal handles or (0 0 1) for
vertical handles on the container described by CONTAINER-DESIGNATOR."
  ;; Check for exceptions based on name.
  (let ((name-exception
          (alexandria:switch ((roslisp-utilities:rosify-underscores-lisp-name
                               (desig:desig-prop-value
                                container-designator :urdf-name))
                              :test 'equal)
            ("oven_area_area_left_drawer_main"
             (cl-transforms:make-3d-vector 0 0 1))
            ("oven_area_area_right_drawer_main"
             (cl-transforms:make-3d-vector 0 0 1)))))
    (if name-exception
        name-exception
        ;; Use prolog to find out which supertype fits.
        (alexandria:switch
            ((desig:desig-prop-value container-designator :type)
             :test (lambda (type super)
                     (prolog:prolog `(man-int:object-type-subtype ,super ,type))))
          (:container-prismatic (cl-transforms:make-3d-vector 1 0 0))
          (:container-revolute (cl-transforms:make-3d-vector 0 0 1))
          (T (progn
               (roslisp:ros-warn (environment-manipulation get-handle-axis)
                                 "Could not get a handle-axis for ~a
Using a default (1 0 0)."
                                         container-designator)
               (cl-transforms:make-3d-vector 1 0 0)))))))
