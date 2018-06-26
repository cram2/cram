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

(in-package :pr2-reachability-costmap)

(defparameter *package-name* "pr2_reachability_costmap")
(defparameter *reachability-map-files*
  '((:left . "resource/pr2-reachability-map-left-5cm.map")
    (:right . "resource/pr2-reachability-map-right-5cm.map")))
(defparameter *ik-reference-frame* "torso_lift_link")

(defvar *reachability-maps* nil)

(defun get-reachability-map (side)
  (let ((map (cdr (assoc side *reachability-maps*)))
        (relative-filename (cdr (assoc side *reachability-map-files*))))
    (assert relative-filename)
    (or map
        (let ((reachability-map
                (make-instance 'reachability-map
                  :filename (concatenate
                             'string
                             (namestring (ros-load:ros-package-path
                                           *package-name*))
                             relative-filename))))
          (push (cons side reachability-map) *reachability-maps*)
          reachability-map))))

(defun make-inverse-reachability-matrix
    (reachability-map-matrix z-index orientation-indices)
  (declare (type simple-array reachability-map-matrix)
           (type fixnum z-index)
           (type list orientation-indices))
  (assert (> (list-length orientation-indices) 0))
  (let ((result (cma:make-double-matrix
                 (array-dimension reachability-map-matrix 2)
                 (array-dimension reachability-map-matrix 1))))
    (dotimes (y (array-dimension reachability-map-matrix 1))
      (dotimes (x (array-dimension reachability-map-matrix 2))
        (dolist (orientation-index orientation-indices)
          (incf (aref result y x) (aref reachability-map-matrix
                                        z-index y x orientation-index)))))
    (cma:m./ result (float (list-length orientation-indices) 0.0d0))))

(defun ensure-point-stamped (pose-specification)
  (etypecase pose-specification
    (cl-transforms-stamped:point-stamped pose-specification)
    (cl-transforms-stamped:pose-stamped
     (cl-transforms-stamped:make-point-stamped
      (cl-transforms-stamped:frame-id pose-specification)
      (cl-transforms-stamped:stamp pose-specification)
      (cl-transforms:origin pose-specification)))
    (cl-transforms:3d-vector
     (cl-transforms-stamped:make-point-stamped
      *fixed-frame*
      0.0
      pose-specification))
    (cl-transforms:pose
     (cl-transforms-stamped:make-point-stamped
      *fixed-frame*
      0.0
      (cl-transforms:origin pose-specification)))))

(defun get-closest-orientation (pose orientations)
  "Returns the orientation in `orientation' that is closest to the
  orientation of `pose'. If `pose' is of type TF:POSE-STAMPED,
  transforms it first into the fixed frame."
  (let ((pose (etypecase pose
                (cl-transforms-stamped:pose-stamped
                 (cl-transforms-stamped:transform-pose-stamped
                  *transformer*
                  :pose pose :target-frame *fixed-frame* :timeout *tf-default-timeout*))
                (cl-transforms:pose pose))))
    (find-closest-orientation
     (cl-transforms:orientation pose) orientations)))

(defun make-inverse-reachability-costmap (sides pose-specification
                                          &key orientations)
  "Returns a generator that uses an inverse reachability map to
generate poses from which `poses' are reachable. `sides' indicates the
arms to use. Multiple size lead to an OR like combination of costmaps
of the sides. 

`pose-specification' can either be a TF:POSE-STAMPED or a
TF:POINT-STAMPED. If the parameter is a pose-stamped, the closest
orientation in the corresponding reachability-map is used. If it is a
point-stamped, all orientations are used."
  (flet ((get-orientation-indices (reachability-map orientations)
           (remove nil 
                   (loop for orientation in orientations
                         collecting (position
                                     orientation (orientations reachability-map)
                                     :test (lambda (orientation-1 orientation-2)
                                             (< (cl-transforms:angle-between-quaternions
                                                 orientation-1 orientation-2)
                                                1e-6)))))))
    (when (and (typep pose-specification 'cl-transforms:pose) orientations)
      (error 'simple-error
             :format-control "`orientations' cannot be specified in combination with a CL-TRANSFORMS:POSE."))
    (let* ((point (ensure-point-stamped pose-specification))
           (point-in-map (cl-transforms-stamped:transform-point-stamped
                          *transformer*
                          :point point
                          :target-frame *fixed-frame*
                          :timeout *tf-default-timeout*))
           (point-in-ik-frame (cl-transforms-stamped:transform-point-stamped
                               *transformer*
                               :point point
                               :target-frame *ik-reference-frame*
                               :timeout *tf-default-timeout*))
           (functions (mapcar
                       (lambda (side)
                         (let* ((reachability-map (get-reachability-map side))
                                (origin (inverse-map-origin reachability-map))
                                (orientations
                                  (etypecase pose-specification
                                    (cl-transforms:3d-vector
                                     (or orientations (orientations reachability-map)))
                                    (cl-transforms:pose
                                     (list (get-closest-orientation
                                            pose-specification
                                            (orientations reachability-map)))))))
                           (generator-function
                            (make-matrix-cost-function
                             (+ (cl-transforms:x origin) (cl-transforms:x point-in-map))
                             (+ (cl-transforms:y origin) (cl-transforms:y point-in-map))
                             ;; TODO(moesenle) verify resolution
                             (cl-transforms:x (resolution reachability-map))
                             (make-inverse-reachability-matrix
                              (inverse-reachability-map reachability-map)
                              (map-coordinate->array-index
                               (cl-transforms:z point-in-ik-frame)
                               (cl-transforms:z (resolution reachability-map))
                               (cl-transforms:z (origin reachability-map)))
                              (get-orientation-indices
                               reachability-map orientations))))))
                       sides)))
      (make-instance 'map-costmap-generator
        :generator-function (lambda (costmap-metadata matrix)
                              (reduce (lambda (previous-matrix function)
                                        (funcall
                                         function costmap-metadata
                                         previous-matrix))
                                      functions :initial-value matrix))
        :name :inverse-reachability-costmap))))
