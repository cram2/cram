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
  (btr:get-environment-object)
  ;; (find 'btr::urdf-semantic-map-object
  ;;       (btr:objects btr:*current-bullet-world*)
  ;;       :key #'type-of)
  )

(defun get-container-pose-and-transform (name btr-environment)
  "Return a list of the pose-stamped and transform-stamped of the object named
`name' in the environment `btr-environment'.
The frame-id will be cram-tf:*robot-base-frame* and the child-frame-id will be the `name'.
That is, returns (base-P-name-stamped base-T-name-stamped)."
  (declare (type (or string symbol) name)
           (type keyword btr-environment))
  (when (symbolp name)
    (setf name (roslisp-utilities:rosify-underscores-lisp-name name)))
  (let* ((map-P-name (get-urdf-link-pose name btr-environment))
         (map-P-base (btr:pose (btr:get-robot-object)))
         (map-T-base (cl-transforms:pose->transform map-P-base))
         (base-T-map (cl-transforms:transform-inv map-T-base))
         (base-P-name (cl-transforms:transform base-T-map map-P-name))
         (base-P-name-stamped (cl-transforms-stamped:pose->pose-stamped
                                cram-tf:*robot-base-frame*
                                0.0
                                base-P-name))
         (base-T-name-stamped (cram-tf:pose-stamped->transform-stamped
                               base-P-name-stamped name)))
    (list
     (the cl-transforms-stamped:pose-stamped base-P-name-stamped)
     (the cl-transforms-stamped:transform-stamped base-T-name-stamped))))

(defun get-urdf-link-pose (name btr-environment)
  "Return the pose in map of the object named `name'
in the object named `btr-environment', i.e., mapPname."
  (declare (type (or string symbol) name)
           (type keyword btr-environment))
  (when (symbolp name)
    (setf name
          (roslisp-utilities:rosify-underscores-lisp-name name)))
  (btr:link-pose (btr:object btr:*current-bullet-world* btr-environment) name))

(defun get-urdf-link-aabb (name btr-environment)
  "Return the axis-aligned bounding box of the object with `name' in `environment'."
  (declare (type (or string symbol) name)
           (type keyword btr-environment))
  (when (symbolp name)
    (setf name
          (roslisp-utilities:rosify-underscores-lisp-name name)))
  (btr:aabb
   (btr:rigid-body
    (btr:object btr:*current-bullet-world*
                btr-environment)
    (btr::make-rigid-body-name
     (string-upcase btr-environment)
     name))))

(defun get-aabb-at-joint-state (name btr-environment joint-state)
  "Return the axis-aligned bounding box of the object with `name' in `environment'
at a specific `joint-state'. First the joint is set to the specified state, then
the bounding box is retrieved and finally the joint is reset."
  (declare (type (or string symbol) name)
           (type keyword btr-environment))
  (let* ((joint (get-connecting-joint
                          (get-environment-link name btr-environment)))
         (original-state (get-joint-position
                          joint
                          btr-environment)))
    (setf (btr:joint-state
           (btr:object btr:*current-bullet-world*
                       btr-environment)
           (cl-urdf:name joint))
          joint-state)
    (let ((aabb (get-urdf-link-aabb (cl-urdf:name (cl-urdf:child joint)) btr-environment)))
      (setf (btr:joint-state
             (btr:object btr:*current-bullet-world*
                         btr-environment)
             (cl-urdf:name joint))
            original-state)
      aabb)))

(defun get-aabbs-at-joint-states (name btr-environment)
  "Return a list of axis-aligned bounding boxes at minimum, middle and maximum of the joint's
limit."
  (declare (type (or string symbol) name)
           (type keyword btr-environment))
  (let* ((joint (get-connecting-joint
                 (get-environment-link name btr-environment)))
         (limits (cl-urdf:limits joint))
         (middle (+ (cl-urdf:lower limits) (/ (cl-urdf:upper limits) 2))))
    (list
     (get-aabb-at-joint-state name btr-environment (cl-urdf:lower limits))
     (get-aabb-at-joint-state name btr-environment middle)
     (get-aabb-at-joint-state name btr-environment (cl-urdf:upper limits)))))

(defun get-environment-link (link-name btr-environment)
  "Return the link with name `link-name' in the urdf object named `btr-environment'."
  (declare (type (or string symbol) link-name)
           (type keyword btr-environment))
  (when (symbolp link-name)
    (setf link-name
          (roslisp-utilities:rosify-underscores-lisp-name link-name)))
  (the cl-urdf:link
       (gethash link-name
                (cl-urdf:links
                 (btr:urdf
                  (btr:object btr:*current-bullet-world*
                              btr-environment))))))

(defun get-container-joint-type (container-name btr-environment)
  "Return the joint-type of the container with `container-name' in the `btr-environment'."
  (declare (type (or string symbol) container-name)
           (type keyword btr-environment))
  (find-container-joint-type-under-joint
   (cl-urdf:from-joint
    (get-environment-link container-name
                        btr-environment))))

(defun find-container-joint-type-under-joint (joint)
  "Return the first joint type different from :FIXED under the given `joint'."
  (declare (type cl-urdf:joint joint))
  (if (eq :FIXED (cl-urdf:joint-type joint))
      (find-container-joint-type-under-joint
       (car (cl-urdf:to-joints (cl-urdf:child joint))))
      (cl-urdf:joint-type joint)))

(defun get-handle-link (container-name btr-environment)
  "Return the link object of the handle of the container with `container-name' in
the `btr-environment' (of type cl-urdf:link)."
  (declare (type (or string symbol) container-name)
           (type keyword btr-environment))
  (when (symbolp container-name)
    (setf container-name
          (roslisp-utilities:rosify-underscores-lisp-name container-name)))
  (find-handle-under-link
   (get-environment-link container-name btr-environment)))

(defun find-handle-under-link (link)
  "Return the link object of the handle under the given `link' object."
  (declare (type (or null cl-urdf:link) link))
  (the (or null cl-urdf:link)
       (if (search "handle" (cl-urdf:name link))
           link
           (reduce (lambda (&optional x y) (or x y))
                   (mapcar 'find-handle-under-link
                           (mapcar 'cl-urdf:child (cl-urdf:to-joints link)))))))

(defun get-joint-position (joint btr-environment)
  "Return the value of the `joint' in the `btr-environment'."
  (declare (type cl-urdf:joint joint)
           (type keyword btr-environment))
  (the number
       (gethash (cl-urdf:name joint)
                (btr:joint-states (btr:object btr:*current-bullet-world* btr-environment)))))

(defun get-connecting-joint (part)
  "Returns the connecting (moveable) joint of `part' of type cl-urdf:joint."
  (declare (type (or cl-urdf:joint cl-urdf:link) part))
  (or (get-connecting-joint-below part)
      (get-connecting-joint-above part)))

(defun get-connecting-joint-below (part)
  "Traverse the URDF downwards to find a connecting joint and returns it.
The type of return value is cl-urdf:joint.
See `get-connecting-joint' for more info."
  (declare (type (or cl-urdf:joint cl-urdf:link) part))
  (when part
    (if (typep part 'cl-urdf:joint)
        (or (when (not (eql (cl-urdf:joint-type part) :FIXED))
              part)
            (get-connecting-joint-below (cl-urdf:child part)))
        (when (typep part 'cl-urdf:link)
          (find-if
           (lambda (joint)
             (get-connecting-joint-below joint))
           (cl-urdf:to-joints part))))))

(defun get-connecting-joint-above (part)
  "Traverse the URDF upwards to find a connecting joint and returns it.
The type of return value is cl-urdf:joint.
See `get-connecting-joint' for more info."
  (declare (type (or cl-urdf:joint cl-urdf:link) part))
  (when part
    (if (typep part 'cl-urdf:joint)
        (or (when (not (eql (cl-urdf:joint-type part) :FIXED))
              part)
            (get-connecting-joint-above (cl-urdf:parent part)))
        (when (typep part 'cl-urdf:link)
          (get-connecting-joint-above (cl-urdf:from-joint part))))))

(defun get-manipulated-link (part)
  "Returns the link under the connecting joint of `part' of type cl-urdf:link.
Which is the one actually being manipulated."
  (declare (type (or cl-urdf:joint cl-urdf:link) part))
  (cl-urdf:child
   (get-connecting-joint part)))

(defun get-manipulated-pose (link-name joint-position btr-environment &key relative)
  "Returns the pose of the link with `link-name' based on its connection joint position
`joint-position'. If `relative' is T, the actual value is calculated
by `joint-position' * <joint maximal value>. This function returns two
values, the new pose of the link and the joint object that was changed.
`btr-environment' is the name of the bullet environment (eg. :kitchen)."
  (declare (type (or string symbol) link-name)
           (type number joint-position)
           (type keyword btr-environment)
           (type boolean relative))
  (when (not (floatp joint-position))
    (setf joint-position (float joint-position)))
  (let ((link (get-environment-link link-name btr-environment)))
    (when (typep link 'cl-urdf:link)
      (let ((joint (get-connecting-joint link)))
        (when joint
          (values
           (case (cl-urdf:joint-type joint)
             (:prismatic
              (the cl-transforms:pose
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
                      (cl-transforms:make-identity-rotation))))))
             (:revolute
              (let* ((rotation
                       (cl-transforms:axis-angle->quaternion
                        (cl-transforms:make-3d-vector 0 0 1) ;; might be some other axis
                        (-
                         (if relative
                             (* joint-position
                                (cl-urdf:upper (cl-urdf:limits joint)))
                             joint-position)
                         (get-joint-position joint btr-environment))))
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
                (the cl-transforms:pose
                     (cl-transforms:transform-pose
                      (cl-transforms:make-transform
                       (cl-transforms:rotate
                        rotation
                        (cl-transforms:translation joint-to-handle))
                       (cl-transforms:make-identity-rotation))
                      (get-urdf-link-pose (cl-urdf:name (cl-urdf:child joint)) btr-environment))))))
           joint))))))

(defun get-manipulated-pose-by-setting-joint (link-name joint-position btr-environment
                                              &key relative)
  (declare (type (or string symbol) link-name)
           (type number joint-position)
           (type keyword btr-environment)
           (type boolean relative))
  (when (not (floatp joint-position))
    (setf joint-position (float joint-position)))
  (let ((link (get-environment-link link-name btr-environment)))
    (when (typep link 'cl-urdf:link)
      (let ((joint (get-connecting-joint link)))
        (when joint
          (when relative
            (setf joint-position
                  (+
                   (cl-urdf:lower (cl-urdf:limits joint))
                   (* joint-position
                      (- (cl-urdf:upper (cl-urdf:limits joint))
                         (cl-urdf:lower (cl-urdf:limits joint)))))))
          (values
           (let ((original-joint-position (get-joint-position joint btr-environment)))
             (setf (btr:joint-state
                    (btr:object btr:*current-bullet-world*
                                btr-environment)
                    (cl-urdf:name joint))
                   joint-position)
             (let ((manipulated-joint-pose
                     (get-urdf-link-pose
                      link-name
                      btr-environment)))
               (setf (btr:joint-state
                      (btr:object btr:*current-bullet-world*
                                  btr-environment)
                      (cl-urdf:name joint))
                     original-joint-position)
               manipulated-joint-pose))
           joint))))))

(defun get-positive-joint-state (joint btr-environment)
  (declare (type cl-urdf:joint joint)
           (type keyword btr-environment))
  (mod (btr:joint-state
        (btr:object btr:*current-bullet-world*
                    btr-environment)
        (cl-urdf:name joint))
       (* 2 pi)))

(defun get-connecting-joint-state-secure (container-name btr-environment)
  (let* ((joint (get-connecting-joint
                 (get-environment-link container-name
                                     btr-environment)))
         (type (find-container-joint-type-under-joint joint))
         (state (btr:joint-state
                     (btr:object btr:*current-bullet-world*
                                 btr-environment)
                     (cl-urdf:name joint))))
    (ecase type
      (:revolute
       (mod state (* 2 pi)))
      (:prismatic
       state))))

(defun get-relative-distance (container-name btr-environment distance action-type)
  (declare (type (or string symbol) container-name)
           (type keyword btr-environment)
           (type number distance)
           (type keyword action-type))
  (let ((state (get-connecting-joint-state-secure container-name btr-environment)))
    (ecase action-type
      (:opening
       (- distance state))
      (:closing
       (- (- distance state))))))

;; Important note: This assumes a relative distance, because the trajectory calculations
;; were implemented in a relative manner.
(defun clip-distance (container-name btr-environment distance action-type)
  "Return a relative distance that stays inside the joint's limits."
  (declare (type (or string symbol) container-name)
           (type keyword btr-environment)
           (type number distance)
           (type keyword action-type))
  (let ((joint (get-connecting-joint
                (get-environment-link container-name
                                    btr-environment))))
    (let ((upper-limit (cl-urdf:upper (cl-urdf:limits joint)))
          (lower-limit (cl-urdf:lower (cl-urdf:limits joint)))
          (state (get-connecting-joint-state-secure container-name btr-environment)))
      (ecase action-type
        (:opening
         (if (> (+ state distance) upper-limit)
             (- upper-limit state)
             (if (< (+ state distance) lower-limit)
                 (- (- state lower-limit))
                 distance)))
        (:closing
         (if (< (- state distance) lower-limit)
             (- state lower-limit)
             (if (> (- state distance) upper-limit)
                 (- (- upper-limit state))
                 distance)))))))
