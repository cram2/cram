;;; Copyright (c) 2012, Gayane Kazhoyan <kazhoyan@in.tum.de>
;;;                     Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :btr-spatial-cm)

;;;;;;;;;;;;;;;;;;;;;;;;;;; NEAR and FAR calculations

(defun get-aabb-min-length (object)
  (let ((dims (btr:calculate-bb-dims object)))
    (min (cl-transforms:x dims) (cl-transforms:y dims))))

(defun get-aabb-circle-diameter (object)
  (cl-transforms:x (btr:calculate-bb-dims object)))

(defun get-aabb-oval-diameter (object)
  "We assume the radius of the oval is the max of its two radia"
  (let ((dims (btr:calculate-bb-dims object)))
    (max (cl-transforms:x dims) (cl-transforms:y dims))))

(defun calculate-near-costmap-min-radius (ref-obj-size for-obj-size
                                          ref-obj-padding for-obj-padding)
  "The radius of the costmap is the distance from the center of the `ref-obj'
to the center of `for-obj'"
  (+ (/ ref-obj-size 2.0d0) ref-obj-padding for-obj-padding (/ for-obj-size 2.0d0)))

(defun calculate-far-costmap-min-radius (ref-obj-size for-obj-size
                                         ref-obj-padding for-obj-padding)
  "The radius of the costmap is the distance from the center of the `ref-obj'
to the center of `for-obj'.
We assume that far relation means that between the two objects it is possible
to put another one of the two objects - the one which has bigger size:
ref-sz/2 + ref-padding + max-padding + max-sz + max-padding + for-padding + for-sz/2"
  (if (> ref-obj-size for-obj-size)
      (+ (* ref-obj-size 1.5) (* ref-obj-padding 3) for-obj-padding (/ for-obj-size 2))
      (+ (/ ref-obj-size 2) ref-obj-padding (* for-obj-padding 3) (* for-obj-size 1.5))))

(defun calculate-costmap-width (ref-obj-size for-obj-size costmap-width-percentage)
  (* (+ ref-obj-size for-obj-size) 0.5d0 costmap-width-percentage))

;;;;;;;;;;;;;;;;;;;;;;;;;;; SUPPORTING OBJECT calculations

;; (defun get-sem-map-part (environment-object urdf-name)
;;   (let ((owl-name (sem-map-utils:urdf-name->obj-name urdf-name)))
;;     (sem-map-utils:semantic-map-part (btr:semantic-map environment-object) owl-name)))

(defun get-link-rigid-body (environment-object link-name)
  (gethash (if (symbolp link-name)
               (roslisp-utilities:rosify-underscores-lisp-name link-name)
               link-name)
           (btr:links environment-object)))

(defun get-rigid-body-aabb-top-z (rigid-body)
  (when rigid-body
    (let ((body-aabb (cl-bullet:aabb rigid-body)))
      (+ (cl-transforms:z (cl-bullet:bounding-box-center body-aabb))
         (/ (cl-transforms:z (cl-bullet:bounding-box-dimensions body-aabb)) 2)))))

(defun pose-within-aabb (object-pose rigid-body)
  (labels ((inside-aabb (min max pt)
             "Checks if `pt' lies in the axis-alligned bounding box specified by
  the two points `min' and `max'"
             (let ((x (cl-transforms:x pt))
                   (y (cl-transforms:y pt))
                   ;; (z (cl-transforms:z pt))
                   )
               (declare (type double-float x y ;; z
                              ))
               (and (>= x (cl-transforms:x min))
                    (<= x (cl-transforms:x max))
                    (>= y (cl-transforms:y min))
                    (<= y (cl-transforms:y max))
                    ;; (>= z (cl-transforms:z min))
                    ;; (<= z (cl-transforms:z max))
                    )))
           (2d-object-bb (aabb)
             (let ((center (cl-bullet:bounding-box-center aabb))
                   (dim/2 (cl-transforms:v* (cl-bullet:bounding-box-dimensions aabb) 0.5)))
               (list (cl-transforms:v- center dim/2)
                     (cl-transforms:v+ center dim/2)))))
    (let* ((aabb (btr:aabb rigid-body))
           (bb (2d-object-bb aabb))
           (transform (cl-transforms:make-transform
                       (cl-bullet:bounding-box-center aabb)
                       (cl-transforms:make-identity-rotation)))
           (pt (cl-transforms:make-3d-vector
                (cl-transforms:x (cl-transforms:origin object-pose))
                (cl-transforms:y (cl-transforms:origin object-pose))
                (cl-transforms:z (cl-transforms:translation transform)))))
      (inside-aabb (first bb) (second bb) pt))))

(defun get-highest-rigid-body-below-object (rigid-bodies object)
  (loop for body in rigid-bodies
        for current-z = (get-rigid-body-aabb-top-z body)
        with object-z = (get-rigid-body-aabb-top-z object)
        and highest-z = 0.0 and highest-body
        do (when (and (>= current-z highest-z)
                         (< current-z object-z))
                (setf highest-z current-z)
                (setf highest-body body))
        finally (return highest-body)))



;;;;;;;;;;;;;;;;;;;;;;;; COSTMAPS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun make-object-bounding-box-costmap-generator (object)
  (let* ((bounding-box-dims (btr:calculate-bb-dims object))
         (dimensions-x/2 (/ (cl-transforms:x bounding-box-dims) 2))
         (dimensions-y/2 (/ (cl-transforms:y bounding-box-dims) 2))
         (center-x (cl-transforms:x (cl-transforms:origin (btr:pose object))))
         (center-y (cl-transforms:y (cl-transforms:origin (btr:pose object)))))
    (lambda (x y)
      (if (and
           (< x (+ center-x dimensions-x/2))
           (> x (- center-x dimensions-x/2))
           (< y (+ center-y dimensions-y/2))
           (> y (- center-y dimensions-y/2)))
          1.0 0.0))))

;;; TODO: maybe include bb into deciding not just the pose and ratio
(defun get-closest-edge (obj-pose supp-obj-pose supp-obj-dims)
  "The supp-obj is supposed to be rectangular (and have 4 edges obviously)
with y axis (in table coordinate system) pointing towards its left edge
and x - to the back. `obj-pose' should be in the world frame.
The function returns one of the following keys: :front, :back, :left, :right."
  (declare (type cl-transforms:pose obj-pose supp-obj-pose)
           (type cl-transforms:3d-vector supp-obj-dims))
  (flet ((check-relation-p (dimensions/2 coords pred-1 pred-2 ratio-x ratio-y)
           (< (* (funcall pred-1
                          (cl-transforms:x dimensions/2)
                          (cl-transforms:x coords))
                 ratio-x)
              (* (funcall pred-2
                          (cl-transforms:y dimensions/2)
                          (cl-transforms:y coords))
                 ratio-y)))
         (get-quarter-in-supp-obj (coords)
           (if (> (cl-transforms:x coords) 0)
               (if (> (cl-transforms:y coords) 0)
                   :back-left :back-right)
               (if (> (cl-transforms:y coords) 0)
                   :front-left :front-right))))
    (let* ((world->supp-transform
             (cl-transforms:pose->transform supp-obj-pose))
           (supp->world-transform
             (cl-transforms:transform-inv world->supp-transform))
           (obj-coords-in-supp
             (cl-transforms:transform-point
              supp->world-transform
              (cl-transforms:origin obj-pose)))
           (dimensions/2
             (cl-transforms:v* supp-obj-dims 0.5))
           (ratio-x
             1.0d0)
           (ratio-y
             1.0d0)
           (quarter
             (get-quarter-in-supp-obj obj-coords-in-supp)))
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

(defun make-potential-field-cost-function (axis ref-pose supp-pose pred
                                           &optional (threshold 0.2d0))
  "This function is used for resolving spatial relations such as left-of, behind etc.
   Returns a lambda function which for any (x y) gives a value in [0; 1].
   `axis' is either :x (for relations in-front-of and behind) or :y (for left and right).
   `ref-pose' is the coordinate of the reference point according to which the
   relation is resolved.
   `pred' is either #'< (in which case non-zero values are assigned to points on the
   negative side of the `axis') or #'>.
   If `threshold' is specified all values generated by the function that are below the
   threshold  will be assigned 0."
  (let* ((ref-x
           (cl-transforms:x (cl-transforms:origin ref-pose)))
         (ref-y
           (cl-transforms:y (cl-transforms:origin ref-pose)))
         (translated-supp-pose
           (cl-transforms:make-transform
            (cl-transforms:make-3d-vector ref-x ref-y 0)
            ;; costmaps are 2D and all the calculations of this
            ;; potential fields as well, so we pick a random z
            ;; coordinate, namely 0.
            (cl-transforms:orientation supp-pose)))
         (supp->world-trans
           (cl-transforms:transform-inv translated-supp-pose)))
    (lambda (x y)
      (let* ((point
               (cl-transforms:transform-point
                supp->world-trans
                (cl-transforms:make-3d-vector x y 0)))
             (coord
               (ecase axis
                 (:x (cl-transforms:x point))
                 (:y (cl-transforms:y point))))
             (mode
               (sqrt (+ (* (cl-transforms:x point) (cl-transforms:x point))
                        (* (cl-transforms:y point) (cl-transforms:y point))))))
        (if (funcall pred coord 0.0d0)
            ;; projects the vector to the point (x, y) onto the `axis'
            ;; returns the ratio between the lengths of projection and the vector itself
            (if (> (abs (/ coord mode)) threshold)
                (abs (/ coord mode))
                0.0d0)
            0.0d0)))))

(defun make-aabbs-costmap-generator (objs &key (invert nil) (padding 0.0d0))
  " Returns a lambda function which for each (x y) gives 1.0 if one of the objects in
   `objs' covers (x y) with its bounding box, and 0.0 if (x y) is free of objects.
   `objs' is a lazy list of btr:object's.
   If `invert' is t, the 1.0 and 0.0 from the original definition are exchanged.
   `padding' extends the bounding box of all the objects in `objs' on 2 * padding."
  (when objs
    (let ((bbcenterx-bbcentery-dimx/2-dimy/2-lists
            (loop for obj in (cut:force-ll objs)
                  collecting
                  (let* ((bb-center (cl-transforms:origin (btr:pose obj)))
                         (bb-center-x (cl-transforms:x bb-center))
                         (bb-center-y (cl-transforms:y bb-center))
                         (bb-dim (cl-bullet:bounding-box-dimensions (btr:aabb obj)))
                         (dimensions-x/2 (+ (/ (cl-transforms:x bb-dim) 2) padding))
                         (dimensions-y/2 (+ (/ (cl-transforms:y bb-dim) 2) padding)))
                    (list bb-center-x bb-center-y dimensions-x/2 dimensions-y/2)))))
      (lambda (x y)
        (block nil
          (dolist (info-list bbcenterx-bbcentery-dimx/2-dimy/2-lists (if invert 1.0d0 0.0d0))
            (destructuring-bind (bb-center-x bb-center-y dimensions-x/2 dimensions-y/2)
                info-list
              (when (and
                     (< x (+ bb-center-x dimensions-x/2))
                     (> x (- bb-center-x dimensions-x/2))
                     (< y (+ bb-center-y dimensions-y/2))
                     (> y (- bb-center-y dimensions-y/2)))
                (return (if invert 0.0d0 1.0d0))))))))))

(defun make-slot-cost-function (supp-object paddings-list
                                preferred-supporting-object-side object-count
                                max-slot-size min-slot-size position-deviation-threshold)
  "'Divides' the supporting object into `object-count' number of slots and assigns 1s to
   the centers of those slots and all the points that are not further from the center than
   `position-deviation-threshold'.
   _____________
   |_x_|_x_|_x_|  <- example for object-count = 5
   |__x__|__x__|     the supporting object is divided into 5 slots. 'x' - center of slot

   The resulting slot sizes are restricted by `max-slot-size' and `min-slot-size', i.e.
   everything which is out of the boundaries gets truncated to the max or min.
   `preferred-supporting-object-size' is either :+ (for positive direction of axis) or :-.
   `paddings-list' is of format '(padd-x+ padd-x- padd-y+ padd-y-).
   The `supp-object' is supposed to be rectangular with y axis pointing to the left
   and x - to the back."
  (declare (type cl-bullet:rigid-body supp-object)
           (type list paddings-list)
           (type keyword preferred-supporting-object-side)
           (type integer object-count)
           (type real max-slot-size min-slot-size position-deviation-threshold))
  (flet ((calculate-points (distance point-count longer-side-axis coord-on-shorter-axis)
           (loop repeat point-count
                 for next-coord = (* (/ (1- point-count) 2) distance)
                   then (- next-coord distance)
                 collecting
                 (cond
                   ((eql longer-side-axis #'cl-transforms:x)
                    (list next-coord coord-on-shorter-axis))
                   ((eql longer-side-axis #'cl-transforms:y)
                    (list coord-on-shorter-axis next-coord))))))
    (let* ((supp-obj-dims-in-sem-map-coords
             (btr:calculate-bb-dims supp-object))
           (padded-supp-obj-dims-in-sem-map-coords
             (cl-transforms:v-
              supp-obj-dims-in-sem-map-coords
              (cl-transforms:make-3d-vector (+ (first paddings-list) (second paddings-list))
                                            (+ (third paddings-list) (fourth paddings-list))
                                            0.0d0)))
           ;; inverse transform of the origin of the center of the padded region:
           (padding-center->zero-in-sem-map-coords
             (cl-transforms:make-transform
              (cl-transforms:make-3d-vector
               (/ (- (first paddings-list) (second paddings-list)) 2)
               (/ (- (third paddings-list) (fourth paddings-list)) 2)
               0) ; it's the inverse, therefore (- first second)
              (cl-transforms:make-identity-rotation)))
           ;; set initial values for the axis and switch if needed
           (longer-side-axis #'cl-transforms:x)
           (shorter-side-axis #'cl-transforms:y)
           (longer-side-length nil)
           (max-possible-object-count nil))
      (when (> (cl-transforms:y padded-supp-obj-dims-in-sem-map-coords)
               (cl-transforms:x padded-supp-obj-dims-in-sem-map-coords))
        (rotatef longer-side-axis shorter-side-axis))
      (setf longer-side-length
            (funcall longer-side-axis padded-supp-obj-dims-in-sem-map-coords))
      (setf max-possible-object-count
            (* (floor longer-side-length min-slot-size) 2))
      (when (> object-count max-possible-object-count)
        (setf object-count max-possible-object-count))
      (let* ((object-count-on-preferred-side
               (ceiling object-count 2))
             (object-count-on-other-side
               (floor object-count 2))
             (coord-on-shorter-axis
               (- (/ (funcall shorter-side-axis padded-supp-obj-dims-in-sem-map-coords)
                     2.0)
                  (/ min-slot-size
                     2.0)))
             (distance
               (/ longer-side-length object-count-on-preferred-side)))
        (when (> distance max-slot-size)
          (setf distance max-slot-size))
        (ecase preferred-supporting-object-side
          (:+ nil)
          (:- (setf coord-on-shorter-axis (- coord-on-shorter-axis))))
        (let* ((placement-points
                 (concatenate
                  'list
                  (calculate-points distance object-count-on-preferred-side
                                    longer-side-axis coord-on-shorter-axis)
                  (calculate-points distance object-count-on-other-side
                                    longer-side-axis (- coord-on-shorter-axis))))
               (supp-obj-pose
                 (btr:pose supp-object))
               (world->supp-transform
                 (cl-transforms:pose->transform supp-obj-pose))
               (supp->world-transform
                 (cl-transforms:transform-inv world->supp-transform)))
          (lambda (x y)
            (let* ((point
                     (cl-transforms:transform-point
                      padding-center->zero-in-sem-map-coords
                      (cl-transforms:transform-point
                       supp->world-transform
                       (cl-transforms:make-3d-vector x y 0))))
                   (min-dist
                     (reduce #'min (mapcar
                                    (lambda (x-and-y)
                                      (sqrt
                                       (+ (expt (- (cl-transforms:x point)
                                                   (first x-and-y)) 2)
                                          (expt (- (cl-transforms:y point)
                                                   (second x-and-y)) 2))))
                                    placement-points))))
              (if (<= min-dist position-deviation-threshold)
                  1.0
                  0.0))))))))


(defun make-side-costmap-generator (obj axis sign)
  "Returns a lambda function which for each (x y) gives 1.0
if it is on the sign side of the axis. "
  (when obj
    (let* ((pose-origin (cl-transforms:origin (btr:pose obj)))
           (bb-x (cl-transforms:x pose-origin))
           (bb-y (cl-transforms:y pose-origin)))
      (lambda (x y)
        (if (case axis
              (:x (funcall sign x bb-x))
              (:y (funcall sign y bb-y)))
            1.0
            0.0)))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; HEIGHT GENERATORS ;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *board-thickness* 0.035)

(defun make-object-bounding-box-height-generator (object &optional (tag :on))
  (constantly (list
               (ecase tag
                 (:on (+ (cl-transforms:z
                          (cl-transforms:origin (btr:pose object)))
                         (/ (cl-transforms:z
                             (btr:calculate-bb-dims object))
                            2.0)))
                 (:in (+ (cl-transforms:z
                          (cl-transforms:origin (btr:pose object)))
                         (- (/ (cl-transforms:z
                                (btr:calculate-bb-dims object))
                               2.0))
                         *board-thickness*))))))

(defun make-object-on-object-bb-height-generator (environment-objects for-object)
  (let* ((environment-object-top
           (apply #'max
                  (mapcar (lambda (environment-object)
                            (+ (cl-transforms:z
                                (cl-transforms:origin (btr:pose environment-object)))
                               (/ (cl-transforms:z (btr:calculate-bb-dims environment-object))
                                  2.0)))
                          (if (listp environment-objects)
                              environment-objects
                              (list environment-objects)))))
         (for-object-height
           (cl-transforms:z (btr:calculate-bb-dims for-object)))
         (for-object-z
           (+ environment-object-top (/ for-object-height 2))))
    (constantly
     (list for-object-z))))

;;;;;;;;;;;;;;;;;;;;;;;;;; ORIENTATION GENERATORS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun supporting-obj-aligned-direction (x y supp-obj-pose supp-obj-dims supp-obj-z
                                          &key ref-obj-dependent ref-obj-pose)
  "This function returns an angle (in world coordinate system)
   perpendicular to the edge of the supporting object, to which _pose_ is the closest.
   If `ref-obj-dependent' is t then _pose_ will be `ref-obj-pose',
   otherwise it will be defined by (x, y)."
  (declare (type cl-transforms:pose supp-obj-pose ref-obj-pose)
           (type cl-transforms:3d-vector supp-obj-dims)
           (type boolean ref-obj-dependent))
  (let* ((pose
           (cond (ref-obj-dependent
                  ref-obj-pose)
                 (t (cl-transforms:make-pose
                     (cl-transforms:make-3d-vector x y supp-obj-z)
                     (cl-transforms:make-identity-rotation)))))
         (edge
           (get-closest-edge pose supp-obj-pose supp-obj-dims))
         (angle-in-supp
           (ecase edge
             (:front 0.0d0)
             (:back pi)
             (:left (* pi 1.5d0))
             (:right (/ pi 2.))))
         ;; This is only correct if supp-obj is rotated only around Z axis, i.e.
         ;; it is parallel to the floor
         (supp-angle
           (* 2 (acos (cl-transforms:w (cl-transforms:orientation supp-obj-pose))))))
    ;; acos only gives values between 0 and pi
    ;; checks the sign of the cos to see what sign the angle should have
    (when (< (cl-transforms:z (cl-transforms:orientation supp-obj-pose)) 0)
      (setf supp-angle (- (* 2 pi) supp-angle)))
    (+ supp-angle angle-in-supp)))
