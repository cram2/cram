;;;
;;; Copyright (c) 2016, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :cram-tf)

(defun poses-equal-p (pose-1 pose-2 dist-sigma ang-sigma)
  (and (<= (cl-transforms:v-dist (cl-transforms:origin pose-1)
                                 (cl-transforms:origin pose-2))
           dist-sigma)
       (<= (cl-transforms:angle-between-quaternions
            (cl-transforms:orientation pose-1)
            (cl-transforms:orientation pose-2))
           ang-sigma)))

(defun frame-to-pose-in-fixed-frame (frame-name)
  (when *transformer*
    (transform-pose-stamped
     *transformer*
     :timeout *tf-default-timeout*
     :pose (make-pose-stamped frame-name 0.0
                              (make-identity-vector)
                              (make-identity-rotation))
     :target-frame *fixed-frame*)))

(defun 3d-vector->list (3d-vector)
  (let ((x (cl-transforms:x 3d-vector))
        (y (cl-transforms:y 3d-vector))
        (z (cl-transforms:z 3d-vector)))
    (list x y z)))

(defun list->3d-vector (position-list)
  (destructuring-bind (x y z)
      position-list
    (cl-transforms:make-3d-vector x y z)))

(defun pose->flat-list (pose)
  (let* ((xyz (cl-transforms:origin pose))
         (qqqw (cl-transforms:orientation pose))
         (x (cl-transforms:x xyz))
         (y (cl-transforms:y xyz))
         (z (cl-transforms:z xyz))
         (q1 (cl-transforms:x qqqw))
         (q2 (cl-transforms:y qqqw))
         (q3 (cl-transforms:z qqqw))
         (w (cl-transforms:w qqqw)))
    (list x y z q1 q2 q3 w)))

(defun pose->flat-list-w-first (pose)
  (let* ((xyz (cl-transforms:origin pose))
         (qqqw (cl-transforms:orientation pose))
         (x (cl-transforms:x xyz))
         (y (cl-transforms:y xyz))
         (z (cl-transforms:z xyz))
         (q1 (cl-transforms:x qqqw))
         (q2 (cl-transforms:y qqqw))
         (q3 (cl-transforms:z qqqw))
         (w (cl-transforms:w qqqw)))
    (list x y z w q1 q2 q3)))

(defun pose->list (pose)
  (let* ((xyz (cl-transforms:origin pose))
         (qqqw (cl-transforms:orientation pose))
         (x (cl-transforms:x xyz))
         (y (cl-transforms:y xyz))
         (z (cl-transforms:z xyz))
         (q1 (cl-transforms:x qqqw))
         (q2 (cl-transforms:y qqqw))
         (q3 (cl-transforms:z qqqw))
         (w (cl-transforms:w qqqw)))
    `((,x ,y ,z) (,q1 ,q2 ,q3 ,w))))

(defun flat-list->pose (pose-list)
  (destructuring-bind (x y z q1 q2 q3 w)
      pose-list
    (cl-transforms:make-pose
     (cl-transforms:make-3d-vector x y z)
     (cl-transforms:make-quaternion q1 q2 q3 w))))

(defun flat-list->transform (pose-list)
  (destructuring-bind (x y z q1 q2 q3 w)
      pose-list
    (cl-transforms:make-transform
     (cl-transforms:make-3d-vector x y z)
     (cl-transforms:make-quaternion q1 q2 q3 w))))

(defun flat-list-w-first->pose (pose-list)
  (destructuring-bind (x y z w q1 q2 q3)
      pose-list
    (cl-transforms:make-pose
     (cl-transforms:make-3d-vector x y z)
     (cl-transforms:make-quaternion q1 q2 q3 w))))

(defun list->pose (pose-list)
  (destructuring-bind ((x y z) (q1 q2 q3 w))
      pose-list
    (cl-transforms:make-pose
     (cl-transforms:make-3d-vector x y z)
     (cl-transforms:make-quaternion q1 q2 q3 w))))

(defun list->transform (pose-list)
  (destructuring-bind ((x y z) (q1 q2 q3 w))
      pose-list
    (cl-transforms:make-transform
     (cl-transforms:make-3d-vector x y z)
     (cl-transforms:make-quaternion q1 q2 q3 w))))

(defun ensure-pose-in-frame (pose frame &key use-current-ros-time use-zero-time)
  (declare (type (or null cl-transforms:pose cl-transforms-stamped:pose-stamped)))
  (when pose
    (cl-transforms-stamped:transform-pose-stamped
     *transformer*
     :pose (let ((pose-stamped
                   (cl-transforms-stamped:ensure-pose-stamped pose frame 0.0)))
             (if use-zero-time
                 (cl-transforms-stamped:copy-pose-stamped pose-stamped :stamp 0.0)
                 pose-stamped))
     :target-frame frame
     :timeout *tf-default-timeout*
     :use-current-ros-time use-current-ros-time)))

(defun ensure-point-in-frame (point frame &key use-current-ros-time use-zero-time)
  (declare (type (or cl-transforms:point cl-transforms-stamped:point-stamped)))
  (cl-transforms-stamped:transform-point-stamped
   *transformer*
   :point (if (typep point 'cl-transforms-stamped:point-stamped)
              (if use-zero-time
                  (with-slots (frame-id origin) point
                    (cl-transforms-stamped:make-point-stamped frame-id 0.0 origin))
                  point)
              (cl-transforms-stamped:make-point-stamped
               frame 0.0 point))
   :target-frame frame
   :timeout *tf-default-timeout*
   :use-current-ros-time use-current-ros-time))

(defun translate-pose (pose &key (x-offset 0.0) (y-offset 0.0) (z-offset 0.0))
  (let* ((pose-origin
           (cl-transforms:origin pose))
         (new-origin
           (cl-transforms:copy-3d-vector
            pose-origin
            :x (let ((x-pose-origin (cl-transforms:x pose-origin)))
                 (+ x-pose-origin x-offset))
            :y (let ((y-pose-origin (cl-transforms:y pose-origin)))
                 (+ y-pose-origin y-offset))
            :z (let ((z-pose-origin (cl-transforms:z pose-origin)))
                 (+ z-pose-origin z-offset)))))
    (etypecase pose
      (cl-transforms-stamped:pose-stamped
       (cl-transforms-stamped:copy-pose-stamped pose :origin new-origin))
      (cl-transforms:pose
       (cl-transforms:copy-pose pose :origin new-origin)))))

(defun rotate-pose (pose axis angle)
  (let* ((pose-orientation
           (cl-transforms:orientation pose))
         (new-orientation
           (cl-transforms:q*
            (cl-transforms:axis-angle->quaternion
             (case axis
               (:x (cl-transforms:make-3d-vector 1 0 0))
               (:y (cl-transforms:make-3d-vector 0 1 0))
               (:z (cl-transforms:make-3d-vector 0 0 1))
               (t (error "[CRAM-TF:ROTATE-POSE] axis ~a not specified properly" axis)))
             angle)
            pose-orientation)))
    (etypecase pose
      (cl-transforms-stamped:pose-stamped
       (cl-transforms-stamped:copy-pose-stamped pose :orientation new-orientation))
      (cl-transforms:pose
       (cl-transforms:copy-pose pose :orientation new-orientation)))))

(defun rotate-pose-in-own-frame (pose axis angle)
  (let* ((pose-orientation
           (cl-transforms:orientation pose))
         (new-orientation
           (cl-transforms:q*
            pose-orientation
            (cl-transforms:axis-angle->quaternion
             (case axis
               (:x (cl-transforms:make-3d-vector 1 0 0))
               (:y (cl-transforms:make-3d-vector 0 1 0))
               (:z (cl-transforms:make-3d-vector 0 0 1))
               (t (error "[CRAM-TF:ROTATE-POSE] axis ~a not specified properly" axis)))
             angle))))
    (etypecase pose
      (cl-transforms-stamped:pose-stamped
       (cl-transforms-stamped:copy-pose-stamped pose :orientation new-orientation))
      (cl-transforms:pose
       (cl-transforms:copy-pose pose :orientation new-orientation)))))

(defun rotate-transform-in-own-frame (transform axis angle)
  (let* ((transform-rotation
           (cl-transforms:rotation transform))
         (new-rotation
           (cl-transforms:q*
            transform-rotation
            (cl-transforms:axis-angle->quaternion
             (case axis
               (:x (cl-transforms:make-3d-vector 1 0 0))
               (:y (cl-transforms:make-3d-vector 0 1 0))
               (:z (cl-transforms:make-3d-vector 0 0 1))
               (t (error "[CRAM-TF:ROTATE-POSE] axis ~a not specified properly" axis)))
             angle))))
    (etypecase transform
      (cl-transforms-stamped:transform-stamped
       (copy-transform-stamped transform :rotation new-rotation))
      (cl-transforms:transform
       (cl-transforms:copy-transform transform :rotation new-rotation)))))

(defun tf-frame-converged (goal-frame goal-pose-stamped delta-xy delta-theta)
  (let* ((pose-in-frame
           (cram-tf:ensure-pose-in-frame
            goal-pose-stamped
            goal-frame
            :use-zero-time t))
         (goal-dist (max (abs (cl-transforms:x (cl-transforms:origin pose-in-frame)))
                         (abs (cl-transforms:y (cl-transforms:origin pose-in-frame)))))
         (goal-angle (cl-transforms:normalize-angle
                      (cl-transforms:get-yaw
                       (cl-transforms:orientation pose-in-frame)))))
    (and (<= goal-dist delta-xy)
         (<= (abs goal-angle) delta-theta))))

(defun pose->transform-stamped (parent-frame child-frame stamp pose)
  (let ((translation (cl-transforms:origin pose))
        (rotation (cl-transforms:orientation pose)))
    (cl-transforms-stamped:make-transform-stamped
     parent-frame child-frame stamp translation rotation)))

(defun transform->pose-stamped (parent-frame stamp transform)
  (let ((origin (cl-transforms:translation transform))
        (orientation (cl-transforms:rotation transform)))
    (cl-transforms-stamped:make-pose-stamped
     parent-frame stamp origin orientation)))

(defun transform-stamped-inv (transform-stamped)
  (let ((frame-id (cl-transforms-stamped:frame-id transform-stamped))
        (child-frame-id (cl-transforms-stamped:child-frame-id transform-stamped))
        (stamp (cl-transforms-stamped:stamp transform-stamped)))
    (cl-transforms-stamped:transform->transform-stamped
     child-frame-id
     frame-id
     stamp
     (cl-transforms:transform-inv transform-stamped))))

(defun multiply-transform-stampeds (x-frame z-frame
                                    x-y-transform y-z-transform
                                    &key (result-as-pose-or-transform :transform))
  (declare (type cl-transforms-stamped:transform-stamped
                 x-y-transform y-z-transform)
           (type keyword result-as-pose-or-transform)
           (type string x-frame z-frame))
  "Returns a pose stamped representing xTz -- transfrom from x-frame to z-frame.

Take xTy, ensure it's from x-frame.
Multiply from the right with the yTz transform -- xTy * yTz == xTz."

  (unless (string-equal (cl-transforms-stamped:frame-id x-y-transform) x-frame)
    (warn "~%~%~%~%!!!!!~%~%~%In multiply-transform-stampeds X-Y-TRANSFORM did not have ~
              correct parent frame: ~a and ~a"
           (cl-transforms-stamped:frame-id x-y-transform) x-frame))

  (unless (string-equal (cl-transforms-stamped:child-frame-id y-z-transform) z-frame)
    (warn "~%~%~%~%!!!!!~%~%~%In multiply-transform-stampeds Y-Z-TRANSFORM did not have ~
              correct child frame: ~a and ~a"
           (cl-transforms-stamped:child-frame-id y-z-transform) z-frame))

  (unless (string-equal (cl-transforms-stamped:child-frame-id x-y-transform)
                        (cl-transforms-stamped:frame-id y-z-transform))
    (warn "~%~%~%~%!!!!!~%~%~%In multiply-transform-stampeds X-Y-TRANSFORM and ~
              Y-Z-TRANSFORM did not have equal corresponding frames: ~a and ~a"
           (cl-transforms-stamped:child-frame-id x-y-transform)
           (cl-transforms-stamped:frame-id y-z-transform)))

  (let ((multiplied-transforms
          (cl-transforms:transform* x-y-transform y-z-transform))
        (timestamp (min (cl-transforms-stamped:stamp x-y-transform)
                        (cl-transforms-stamped:stamp y-z-transform))))
    (ecase result-as-pose-or-transform
      (:pose
       (cl-transforms-stamped:pose->pose-stamped
        x-frame
        timestamp
        (cl-transforms:transform->pose multiplied-transforms)))
      (:transform
       (cl-transforms-stamped:transform->transform-stamped
        x-frame
        z-frame
        timestamp
        multiplied-transforms)))))

(defun strip-transform-stamped (transform-stamped)
  (cl-transforms-stamped:make-pose-stamped
   (cl-transforms-stamped:frame-id transform-stamped)
   (cl-transforms-stamped:stamp transform-stamped)
   (cl-transforms-stamped:translation transform-stamped)
   (cl-transforms:rotation transform-stamped)))

(defun copy-transform-stamped (transform-stamped &key frame-id child-frame-id stamp
                                                   translation rotation)
  (cl-transforms-stamped:make-transform-stamped
   (or frame-id (cl-transforms-stamped:frame-id transform-stamped))
   (or child-frame-id (cl-transforms-stamped:child-frame-id transform-stamped))
   (or stamp (cl-transforms-stamped:stamp transform-stamped))
   (or translation (cl-transforms-stamped:translation transform-stamped))
   (or rotation (cl-transforms-stamped:rotation transform-stamped))))

(defun translate-transform-stamped (transform
                                    &key (x-offset 0.0) (y-offset 0.0) (z-offset 0.0))
  (copy-transform-stamped
   transform
   :translation
   (let ((transform-translation (cl-transforms:translation transform)))
     (cl-transforms:copy-3d-vector
      transform-translation
      :x (let ((x-transform-translation (cl-transforms:x transform-translation)))
           (+ x-transform-translation x-offset))
      :y (let ((y-transform-translation (cl-transforms:y transform-translation)))
           (+ y-transform-translation y-offset))
      :z (let ((z-transform-translation (cl-transforms:z transform-translation)))
           (+ z-transform-translation z-offset))))))

(defun pose-stamped->transform-stamped (pose-stamped child-frame-id)
  (cl-transforms-stamped:make-transform-stamped
   (cl-transforms-stamped:frame-id pose-stamped)
   child-frame-id
   (cl-transforms-stamped:stamp pose-stamped)
   (cl-transforms-stamped:origin pose-stamped)
   (cl-transforms-stamped:orientation pose-stamped)))

(defun apply-transform (left-hand-side-transform right-hand-side-transform)
  (cram-tf:multiply-transform-stampeds
   (cl-transforms-stamped:frame-id left-hand-side-transform)
   (cl-transforms-stamped:child-frame-id right-hand-side-transform)
   left-hand-side-transform
   right-hand-side-transform))


(defun values-converged (values goal-values deltas)
  (flet ((value-converged (value goal-value delta)
           (<= (abs (- value goal-value)) delta)))
    ;; correct arguments
    (if (listp values)
        (if (or (atom goal-values)
                (not (= (length values) (length goal-values))))
            (error "GOAL-VALUES (~a) and VALUES (~a) should be of same length."
                   goal-values values)
            (if (atom deltas)
                (setf deltas (make-list (length values) :initial-element deltas))
                (unless (= (length values) (length deltas))
                  (error "DELTAS (~a) and VALUES (~a) should be of same length."
                         deltas values))))
        (if (or (listp goal-values) (listp deltas))
            (error "All arguments should be of same length")
            (setf values (list values)
                  goal-values (list goal-values)
                  deltas (list deltas))))
    ;; actually compare
    (every #'value-converged values goal-values deltas)))


(defun map-axis-aligned-axis (input-quaternion &optional (map-axis 2))
  (declare (type cl-transforms:quaternion input-quaternion))
  "Returns a list with (axis-index projection-positive),
where axis-index is a number 0, 1 or 2 (for x, y or z correspondingly),
depending on which axis is aligned with the given fixed frame `map-axis',
and projection-positive is T if the axis-index axis is pointing in the direction
of the `map-axis' or NIL if it looks in the opposite direction.
For example, if `map-axis' is 2, then we want to find which axis of
`input-quaternion' is aligned with map Z
and the projection-positive will T if axis-index looks up and NIL if it looks down.
projection-on-axis is a number between [-1; -0.57) V (0.57; 1],
which is the length of the projection of the `map-axis'
onto the correspondingly aligned object axis,
projection-positive is calculated based on the sing of projection-on-axis."
  (let* ((rotation-matrix
           (cl-transforms:quaternion->matrix input-quaternion))
         (map-axis-in-input-frame
           `(,(aref rotation-matrix map-axis 0)
             ,(aref rotation-matrix map-axis 1)
             ,(aref rotation-matrix map-axis 2)))
         (axis-index
           0)
         (projection-on-axis-abs
           (abs (nth axis-index map-axis-in-input-frame)))
         (projection-on-axis
           (nth axis-index map-axis-in-input-frame)))
    (loop for i from 1 to 2
          do (when (> (abs (nth i map-axis-in-input-frame))
                      projection-on-axis-abs)
               (setf projection-on-axis-abs
                     (abs (nth i map-axis-in-input-frame))
                     projection-on-axis
                     (nth i map-axis-in-input-frame)
                     axis-index
                     i)))
    (list axis-index (> projection-on-axis 0))))

(defun map-axis-aligned-orientation (input-quaternion)
  (declare (type cl-transforms:quaternion input-quaternion))
  "The returned orientation is of type cl-transforms:quaternion
and has its axes aligned with the identity axes of map (Z up, X forward, Y left),
i.e. the rotation matrix is a permutation and possibly negation of the identity.
For example, if the object is not flatly lying on the horizontal plane,
the axis-aligned orientation will put it on the plane and also correct
the angle around the map Z axis, such that the other two axes are aligned
with the X and Y of the map"
  (destructuring-bind (axis-facing-up-index axis-positive-p)
      (map-axis-aligned-axis input-quaternion)
    (if axis-positive-p
        (ecase axis-facing-up-index
          (0
           ;; x is up, so align with y, i.e. y of object looks to the left
           (cl-transforms:axis-angle->quaternion
            (cl-transforms:make-3d-vector 0 1 0) (/ pi -2)))
          (1
           ;; y is up, so align with x, i.e. x of object looks forward
           (cl-transforms:axis-angle->quaternion
            (cl-transforms:make-3d-vector 1 0 0) (/ pi 2)))
          (2
           ;; z is up, so align with x and y
           (cl-transforms:make-identity-rotation)))
        (ecase axis-facing-up-index
          (0
           ;; x is down, so align with y, i.e. y of object looks to the left
           (cl-transforms:axis-angle->quaternion
            (cl-transforms:make-3d-vector 0 1 0) (/ pi 2)))
          (1
           ;; y is down, so align with x, i.e. x of object looks forward
           (cl-transforms:axis-angle->quaternion
            (cl-transforms:make-3d-vector 1 0 0) (/ pi -2)))
          (2
           ;; z is down, so align with x and y
           (cl-transforms:axis-angle->quaternion
            (cl-transforms:make-3d-vector 0 1 0) pi))))))

(defun angle-around-map-z (input-quaternion)
  (declare (type cl-transforms:quaternion input-quaternion))
  "Calculates the angle between `input-quaternion' and
the corresponding axis-aligned orientation as per MAP-AXIS-ALIGNED-ORIENTATION,
assuming that the object is flatly lying on a horizontal plane,
i.e. that the angles around X and Y are 0."
  ;; find out which axis in the `input-quaternion' is aligned with map Z
  (destructuring-bind (axis-facing-up-index axis-positive-p)
      (map-axis-aligned-axis input-quaternion)
    ;; find the map axis-aligned orientation for `input-quaternion'
    (let ((map-axis-aligned-quaternion
            (map-axis-aligned-orientation input-quaternion)))
      ;; find the angle between `input-quaternion' and map-axis-aligned-quaternion
      ;; this angle is the orientation of input frame in axis-aligned frame
      (multiple-value-bind (axis-aligned-R-input-angle axis-aligned-R-input-axis)
          (cl-transforms:angle-between-quaternions
           map-axis-aligned-quaternion
           input-quaternion)
        ;; angle-between-quaternions returns an angle and an axis,
        ;; around which to rotate with the angle.
        ;; the axis is going to be in axis-aligned coordinate frame
        ;; and the angle might be clockwise or counterclockwise around that axis,
        ;; so normalize it according to the right-hand rule
        (let* ((axis-aligned-R-input-angle-normalized
                 (if (> (nth axis-facing-up-index
                             (3d-vector->list axis-aligned-R-input-axis))
                        0)
                     axis-aligned-R-input-angle
                     (* -1 axis-aligned-R-input-angle)))
               ;; now that we know the angle between axis-aligned and input,
               ;; we need to put that angle around map's Z axis.
               ;; for that, we need to know if the vertical axis of `input-quaternion'
               ;; is pointing up or down, if it points down we need to go clockwise
               (angle-around-map-positive-Z
                 (if axis-positive-p
                     axis-aligned-R-input-angle-normalized
                     (* -1 axis-aligned-R-input-angle-normalized))))
          angle-around-map-positive-Z)))))
