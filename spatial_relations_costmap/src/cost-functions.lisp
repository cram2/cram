;;; Copyright (c) 2012, Gayane Kazhoyan <kazhoyan@in.tum.de>
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

(in-package :spatial-relations-costmap)

;;; UTIL FUNCTIONS
;; used by (left-of-costmap ...)
;; (defun calculate-left-of-pose (pose distance)
;;   ;; (let ((offset (cl-transforms:make-pose
;;   ;;                (cl-transforms:make-3d-vector 0.0 distance 0.0)
;;   ;;                (cl-transforms:make-identity-rotation))))
;;   ;;   (format t "ACHTUNG~%pose = ~a~%dist-pose = ~a~%" pose offset)
;;   ;;   (cl-transforms:transform-pose
;;   ;;    (cl-transforms:pose->transform pose)
;;   ;;    offset))
;; (cl-transforms:make-pose
;;                  (cl-transforms:make-3d-vector 0.0 0.0 0.0)
;;                  (cl-transforms:make-identity-rotation))
;;   )

;; used in make-supporting-obj-alligned-orientation-generator
(defparameter *orientation-samples* 1)
(defparameter *orientation-sample-step* (/ pi 18))

;; used for near and far-from desig-props
(defun get-aabb-min-length (object)
  (let* ((dims (bt:bounding-box-dimensions (aabb object))))
    (min (cl-transforms:x dims) (cl-transforms:y dims))))

;; used for near and far-from desig-props
(defun get-aabb-circle-diameter (object)
  (cl-transforms:x (bt:bounding-box-dimensions (aabb object))))

;; used for near and far-from desig-props
;; we assume the radius of the oval is the max of its two radia
(defun get-aabb-oval-diameter (object)
  (let ((dims (bt:bounding-box-dimensions (aabb object))))
    (max (cl-transforms:x dims) (cl-transforms:y dims))))

;; used for near desig-prop
;; the radius of the costmap is the distance from the center of the ref-obj
;; to the center of for-obj
(defun calculate-near-costmap-radius (ref-obj-size for-obj-size
                                      ref-obj-padding for-obj-padding)
  (+ (/ ref-obj-size 2.0d0) ref-obj-padding for-obj-padding (/ for-obj-size 2.0d0)))

;; used for near desig-prop
;; the radius of the costmap is the distance from the center of the ref-obj
;; to the center of for-obj
;; we suppose that far relation means that between the two objects it is possible
;; to put another one of the two objects - the one which has bigger size
;; ref-sz/2 + ref-padding + max-padding + max-sz + max-padding + for-padding + for-sz/2
(defun calculate-far-costmap-radius (ref-obj-size for-obj-size
                                     ref-obj-padding for-obj-padding)
  (if (> ref-obj-size for-obj-size)
      (+ (* ref-obj-size 1.5) (* ref-obj-padding 3) for-obj-padding (/ for-obj-size 2))
      (+ (/ ref-obj-size 2) ref-obj-padding (* for-obj-padding 3) (* for-obj-size 1.5))))

;; used for near and far-from desig-props
(defun calculate-costmap-width/2 (ref-obj-size for-obj-size costmap-width-percentage)
  (* (+ ref-obj-size for-obj-size) 0.25d0 costmap-width-percentage))


(defun get-y-of-pose (pose)
  (cl-transforms:y (cl-transforms:origin pose)))

(defun get-x-of-pose (pose)
  (cl-transforms:x (cl-transforms:origin pose)))

(defun set-z-of-pose (pose obj)
  (let ((z 0.86d0))
    ;; z should be chosen such that obj is a little bit floating over the table
    ;; white_counter_top_surface_z = 0.853000011
    ;; pot_bottom_z = 0.853659636
    (cl-transforms:make-pose
     (cl-transforms:make-3d-vector
      (cl-transforms:x (cl-transforms:origin pose))
      (cl-transforms:y (cl-transforms:origin pose))
      (+ z (/ (cl-transforms:z (cl-bullet:bounding-box-dimensions (aabb obj))) 2)))
     (cl-transforms:orientation pose))))

(defun pose-on-table ()
  (let ((x (- (+ 1.74d0 (random 0.4d0))))
        (y (+ 0.95 (random 1.3d0)))
        (z 1.0d0))
    (cl-transforms:make-pose
     (cl-transforms:make-3d-vector x y z)
     (cl-transforms:make-identity-rotation))))

(defun pose-on-table-near-world-right-edge ()
  (let ((x (- (+ 1.74d0 (random 0.4d0))))
        (y 0.9d0)
        (z 1.0d0))
    (cl-transforms:make-pose
     (cl-transforms:make-3d-vector x y z)
     (cl-transforms:make-identity-rotation))))
    

;; used in potential-field-costmap prolog pred
(defun get-sem-map-part (sem-map urdf-name)
  (let ((owl-name (sem-map-utils:urdf-name->obj-name urdf-name)))
    (sem-map-utils:semantic-map-part (semantic-map sem-map) owl-name)))

;; TODO: maybe include bb into deciding not just the pose and ratio
(defun get-closest-edge (obj-pose supp-obj-pose supp-obj-dims)
  "The supp-obj is supposed to be rectangular and have 4 edges
with y axis looking to the left and x - to the back.
`obj-pose' should be in the world frame.
The function returns one of the following keys: :front, :back, :left, :right."
  (declare (type cl-transforms:pose obj-pose supp-obj-pose)
           (type cl-transforms:3d-vector supp-obj-dims))
  (flet ((check-relation-p (dimensions/2 coords pred-1 pred-2 ratio-x ratio-y)
           (< (* (funcall pred-1 (cl-transforms:x dimensions/2) 
                          (cl-transforms:x coords))
                 ratio-x)
              (* (funcall pred-2 (cl-transforms:y dimensions/2)
                          (cl-transforms:y coords))
                 ratio-y)))
         (get-quarter-in-supp-obj (coords)
           (if (> (cl-transforms:x coords) 0)
               (if (> (cl-transforms:y coords) 0)
                   :back-left :back-right)
               (if (> (cl-transforms:y coords) 0)
                   :front-left :front-right))))
    (let* ((transform (cl-transforms:pose->transform supp-obj-pose))
           (world->supp-transform (cl-transforms:transform-inv transform))
           (obj-coords-in-supp (cl-transforms:transform-point
                                world->supp-transform
                                (cl-transforms:origin obj-pose)))
           (dimensions/2 (cl-transforms:v* supp-obj-dims 0.5))
           (ratio-x 1.0d0)
           (ratio-y 1.0d0)
           (quarter (get-quarter-in-supp-obj obj-coords-in-supp)))
      ;; find which edges of supp-obj are longer
      ;; longer edges are more preferred, ratio decides how much more preferred
      (if (> (cl-transforms:x dimensions/2) (cl-transforms:y dimensions/2))
          (setf ratio-x (/ (cl-transforms:x dimensions/2)
                           (cl-transforms:y dimensions/2)))
          (setf ratio-y (/ (cl-transforms:y dimensions/2)
                           (cl-transforms:x dimensions/2))))
      ;; find the edge of supp-obj to which obj is the closest
      ;; first find in which quarter of supp obj it is and then compare the 2 edges
      (ecase quarter
        (:back-left
         (if (check-relation-p dimensions/2 obj-coords-in-supp #'- #'- ratio-x ratio-y)
             :back :left))
        (:back-right ; obj.y < 0
         (if (check-relation-p dimensions/2 obj-coords-in-supp #'- #'+ ratio-x ratio-y)
             :back :right))
        (:front-left ; obj.x < 0
         (if (check-relation-p dimensions/2 obj-coords-in-supp #'+ #'- ratio-x ratio-y)
             :front :left))
        (:front-right ; obj.x < 0 and obj.y < 0
         (if (check-relation-p dimensions/2 obj-coords-in-supp #'+ #'+ ratio-x ratio-y)
             :front :right))))))
;;;


(defun make-potential-field-cost-function (axis ref-x ref-y supp-pose pred
                                           &optional (threshold 0.0d0))
  "This function is used for resolving spatial relations such as left-of,
in-front-of etc.
Returns a lambda function which for any (x y) gives a value in [0; 1].
`axis' is either :x (for relations in-front-of and behind) or :y (for left and right).
`ref-x' and `ref-y' are the coordinates of the reference point according to which the
relation is resolved.
`pred' is either #'< (in which case the non-zero values are assigned to points on the
negative side of the `axis') or #'>."
  (let* ((z (cl-transforms:z (cl-transforms:origin supp-pose)))
         (translated-supp-pose (cl-transforms:make-transform
                                (cl-transforms:make-3d-vector ref-x ref-y z)
                                (cl-transforms:orientation supp-pose)))
         (world->supp-trans (cl-transforms:transform-inv translated-supp-pose)))
    (lambda (x y)
      (let* ((point (cl-transforms:transform-point world->supp-trans
                                                   (cl-transforms:make-3d-vector x y z)))
             (coord (ecase axis
                      (:x (cl-transforms:x point))
                      (:y (cl-transforms:y point))))
             (module (sqrt (+ (* (cl-transforms:x point) (cl-transforms:x point))
                              (* (cl-transforms:y point) (cl-transforms:y point))))))
        (if (funcall pred coord 0.0d0)
            ;; return the cos of the angle between (x, y) and (ref-x, ref-y)
            ;; (using the correct axis)
            (if (> (abs (/ coord module)) threshold)
                (abs (/ coord module))
                0.0d0)
            0.0d0)))))

(defun make-objects-bounding-box-costmap-generator (objs &key (invert nil)
                                                           (padding 0.0d0))
  "This costmap generator is used for collision avoidance.
Returns a lambda function which for each (x y) gives 1.0 if one of the objects in `objs'
covers (x y) with its bounding box, and 0.0 if (x y) is free of objects.
`objs' is a list of btr:object's.
If `invert' is t, the 1.0 and 0.0 from the original definition are exchanged.
`padding' extends the bounding box of all the objects in `objs' on 2 * padding."
  (when objs
    (let ((aabbs (loop for obj in (cut:force-ll objs)
                       collecting (aabb obj))))
      (lambda (x y)
        (block nil
          (dolist (bounding-box aabbs (if invert 1.0d0 0.0d0))
            (let* ((bb-center (cl-bullet:bounding-box-center bounding-box))
                   (dimensions-x/2
                     (+ (/ (cl-transforms:x (bt:bounding-box-dimensions bounding-box)) 2)
                        padding))
                   (dimensions-y/2
                     (+ (/ (cl-transforms:y (bt:bounding-box-dimensions bounding-box)) 2)
                        padding)))
              (when (and
                     (< x (+ (cl-transforms:x bb-center) dimensions-x/2))
                     (> x (- (cl-transforms:x bb-center) dimensions-x/2))
                     (< y (+ (cl-transforms:y bb-center) dimensions-y/2))
                     (> y (- (cl-transforms:y bb-center) dimensions-y/2)))
                (return (if invert 0.0d0 1.0d0))))))))))

(defun supporting-obj-alligned-direction (x y supp-obj-pose supp-obj-dims
                                          &key ref-obj-dependent ref-obj-pose)
  "This function returns an angle (in world coordinate system)
perpendicular to that edge of the supporting object to which pose is the closest.
If `ref-obj-dependent' is t then pose will be `ref-obj-pose',
otherwise it will be defined by (x, y)."
  (declare (type cl-transforms:pose supp-obj-pose ref-obj-pose)
           (type cl-transforms:3d-vector supp-obj-dims)
           (type boolean ref-obj-dependent))
  (let* ((pose (cond (ref-obj-dependent
                      (unless ref-obj-pose (format t "In supporting-obj-alligned-direction ref-obj-pose is nil~%"))
                      ref-obj-pose)
                     (t (cl-transforms:make-pose
                         (cl-transforms:make-3d-vector x y 0.8399999737739563d0)
                         (cl-transforms:make-identity-rotation)))))
         (edge (get-closest-edge pose supp-obj-pose supp-obj-dims))
         (angle-in-supp (ecase edge
                          (:front 0.0d0)
                          (:back pi)
                          (:left (* pi 1.5d0))
                          (:right (/ pi 2.))))
         ;; This is only correct if supp-obj is rotated only around Z axis, i.e.
         ;; it is parallel to the floor
         ;; (supp->world-transform (cl-transforms:pose->transform supp-obj-pose))
         ;; (pose-in-world (cl-transforms:transform-pose
         ;;                 supp->world-transform
         ;;                 (cl-transforms:make-pose
         ;;                  (cl-transforms:make-identity-vector)
         ;;                  (cl-transforms:euler->quaternion :az angle-in-supp))))
         ;; (angle (* 2 (acos (cl-transforms:w (cl-transforms:orientation pose-in-world)))))
         (supp-angle (* 2 (acos (cl-transforms:w (cl-transforms:orientation
                                                  supp-obj-pose)))))) 
    ;; cos is symmetric so we get only angles between 0 and pi
    (when (< (cl-transforms:z (cl-transforms:orientation supp-obj-pose)) 0)
      (setf supp-angle (- (* 2 pi) supp-angle)))
    ;; (format t "edge = ~a~%angle = ~a~%" edge (/ (* (+ supp-angle angle-in-supp) 180) pi))
    (+ supp-angle angle-in-supp)))