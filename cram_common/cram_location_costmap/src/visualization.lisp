;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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
;;;

(in-package :location-costmap)

(defvar *location-costmap-publisher* nil)
(defvar *marker-publisher* nil)
(defvar *occupancy-grid-publisher* nil)

(defparameter *z-padding* 1.0)
(defparameter *last-published-marker-index* nil)

(defun location-costmap-vis-init ()
  (setf *location-costmap-publisher*
        (advertise "cram_location_costmap" "visualization_msgs/MarkerArray"))
  (setf *occupancy-grid-publisher*
        (advertise "cram_location_occupancy_grid" "nav_msgs/OccupancyGrid"))
  (setf *marker-publisher*
        (advertise "cram_location_marker" "visualization_msgs/Marker")))

(roslisp-utilities:register-ros-init-function location-costmap-vis-init)

(defun hsv->rgb (h s v)
  "Converts a given set of HSV values `h', `s', `v' into RGB
coordinates. The return value is of type vector (three entries, `r',
`g'. `b'). The output RGB values are in the range [0, 1], and the
input HSV values are in the ranges h = [0, 360], and s, v = [0, 1],
respectively."
  (let* ((c (* v s)) ;; Chroma
         (h-prime (mod (/ h 60.0) 6))
         (x (* c (- 1 (abs (- (mod h-prime 2) 1)))))
         (m (- v c))
         (pre (cond ((and (<= 0 h-prime) (< h-prime 1)) (vector c x 0))
                    ((and (<= 1 h-prime) (< h-prime 2)) (vector x c 0))
                    ((and (<= 2 h-prime) (< h-prime 3)) (vector 0 c x))
                    ((and (<= 3 h-prime) (< h-prime 4)) (vector 0 x c))
                    ((and (<= 4 h-prime) (< h-prime 5)) (vector x 0 c))
                    ((and (<= 5 h-prime) (< h-prime 6)) (vector c 0 x))
                    (t (vector 0 0 0)))))
    (map 'vector (lambda (value) (+ value m)) pre)))

(defun rgb->hsv (r g b)
  "Converts a given set of RGB values `r', `g', `b' into HSV
coordinates. The return value is of type vector (three entries, `h',
`s'. `v'). The input RGB values are in the range [0, 1], and the
output HSV values are in the ranges h = [0, 360], and s, v = [0, 1],
respectively."
  (let* ((c-max (max r g b))
         (c-min (min r g b))
         (delta (- c-max c-min)))
    (cond ((> delta 0)
           (let ((h (cond ((= c-max r)
                           (* 60 (mod (/ (- g b) delta) 6)))
                          ((= c-max g)
                           (* 60 (+ (/ (- b r) delta) 2)))
                          ((= c-max b)
                           (* 60 (+ (/ (- r g) delta) 4)))))
                 (s (cond ((= delta 0) 0)
                          (t (/ delta c-max))))
                 (v c-max))
             (vector h s v)))
          (t (vector 0 0 c-max)))))

(defun location-costmap->marker-array (map &key
                                             (frame-id *fixed-frame*)
                                             (threshold 0.0005) (z *z-padding*)
                                             hsv-colormap
                                             (intensity-colormap nil)
                                             (base-color (vector 0 0 1))
                                             (elevate-costmap nil))
  (with-slots (origin-x origin-y resolution) map
    (let* ((map-array (get-cost-map map))
           (boxes nil)
           (max-val (loop for y from 0 below (array-dimension map-array 0)
                          maximizing (loop for x from 0 below (array-dimension
                                                               map-array 1)
                                           maximizing (aref map-array y x)))))
      (declare (type cma:double-matrix map-array))
      (let ((index 0))
        (dotimes (row (array-dimension map-array 0))
          (dotimes (col (array-dimension map-array 1))
            (let ((curr-val (/ (aref map-array row col) max-val)))
              (when (> curr-val threshold)
                (let ((pose (cl-transforms:make-pose
                             (cl-transforms:make-3d-vector
                              (+ (* col resolution) origin-x)
                              (+ (* row resolution) origin-y)
                              (+ z (or (when elevate-costmap curr-val)
                                       0.0)))
                             (cl-transforms:axis-angle->quaternion
                              (cl-transforms:make-3d-vector 1.0 0.0 0.0) 0.0)))
                      (color (cond (hsv-colormap
                                    (hsv->rgb (* 360 curr-val)
                                              0.5 0.5))
                                   (intensity-colormap
                                    (let* ((hsv-color (rgb->hsv
                                                       (elt base-color 0)
                                                       (elt base-color 1)
                                                       (elt base-color 2)))
                                           (mod-hsv
                                             (vector (elt hsv-color 0)
                                                     (elt hsv-color 1)
                                                     curr-val)))
                                      (hsv->rgb
                                       (elt mod-hsv 0)
                                       (elt mod-hsv 1)
                                       (elt mod-hsv 2))))
                                   (t base-color))))
                  (push (make-message "visualization_msgs/Marker"
                                      (frame_id header) frame-id
                                      (stamp header) (ros-time)
                                      (ns) ""
                                      (id) index
                                      (type) (roslisp-msg-protocol:symbol-code
                                              'visualization_msgs-msg:marker
                                              :cube)
                                      (action) (roslisp-msg-protocol:symbol-code
                                                'visualization_msgs-msg:marker
                                                :add)
                                      (pose) (to-msg pose)
                                      (x scale) resolution
                                      (y scale) resolution
                                      (z scale) resolution
                                      (r color) (elt color 0)
                                      (g color) (elt color 1)
                                      (b color) (elt color 2)
                                      (a color) 0.5)
                        boxes)
                  (incf index))))))
        (values (make-message "visualization_msgs/MarkerArray"
                              (markers) (map 'vector #'identity boxes))
                index)))))

(defun occupancy-grid->grid-cells-msg (grid &key (frame-id *fixed-frame*) (z *z-padding*))
  (with-slots (origin-x origin-y width height resolution) grid
    (let ((grid-arr (grid grid))
          (resolution/2 (/ resolution 2)))
      (declare (type (simple-array fixnum 2) grid-arr))
      (make-message
       "nav_msgs/GridCells"
       (frame_id header) frame-id
       (stamp header) (ros-time)
       cell_width resolution
       cell_height resolution
       cells (map 'vector #'identity
                  (loop for row below (array-dimension grid-arr 0)
                        nconcing (loop for col below (array-dimension grid-arr 1)
                                       when (eql (aref grid-arr row col) 1)
                                         collect
                                         (make-message
                                          "geometry_msgs/Point"
                                          x (+ (* col resolution) resolution/2 origin-x)
                                          y (+ (* row resolution) resolution/2 origin-y)
                                          z z))))))))

(defun remove-markers-up-to-index (index)
  (let ((removers
          (loop for i from 0 to index
                collect (make-message "visualization_msgs/Marker"
                                      (frame_id header) *fixed-frame*
                                      (ns) ""
                                      (id) i
                                      (action) (roslisp-msg-protocol:symbol-code
                                                'visualization_msgs-msg:marker
                                                :delete)))))
    (when removers
      (publish *location-costmap-publisher* 
               (make-message
                "visualization_msgs/MarkerArray"
                (markers) (map 'vector #'identity removers))))))

(defun publish-location-costmap (map &key (frame-id *fixed-frame*)
                                       (threshold 0.0005) (z *z-padding*))
  (when *location-costmap-publisher*
    (multiple-value-bind (markers last-index)
        (location-costmap->marker-array
         map :frame-id frame-id
             :threshold threshold
             :z z
             :hsv-colormap t)
      (when *last-published-marker-index*
        (remove-markers-up-to-index *last-published-marker-index*))
      (setf *last-published-marker-index* last-index)
      (publish *location-costmap-publisher*
               markers))))

(defun publish-point (point &key id)
  (let ((current-index 0))
    (when *marker-publisher*
      (publish *marker-publisher*
               (make-message "visualization_msgs/Marker"
                             (stamp header) (ros-time)
                             (frame_id header) *fixed-frame*
                             ns "kipla_locations"
                             id (or id (incf current-index))
                             type (symbol-code
                                   'visualization_msgs-msg:<marker> :sphere)
                             action (symbol-code
                                     'visualization_msgs-msg:<marker> :add)
                             (x position pose) (cl-transforms:x point)
                             (y position pose) (cl-transforms:y point)
                             (z position pose) (cl-transforms:z point)
                             (w orientation pose) 1.0
                             (x scale) 0.15
                             (y scale) 0.15
                             (z scale) 0.15
                             (r color) 1.0 ; (random 1.0)
                             (g color) 0.0 ; (random 1.0)
                             (b color) 0.0 ; (random 1.0)
                             (a color) 0.9)))))

(defun publish-pose (pose &key id)
  (let ((point (cl-transforms:origin pose))
        (rot (cl-transforms:orientation pose))
        (current-index 0))
    (when *marker-publisher*
      (publish *marker-publisher*
               (make-message "visualization_msgs/Marker"
                             (stamp header) (ros-time)
                             (frame_id header)
                             (typecase pose
                               (pose-stamped (frame-id pose))
                               (t *fixed-frame*))
                             ns "kipla_locations"
                             id (or id (incf current-index))
                             type (symbol-code
                                   'visualization_msgs-msg:<marker> :arrow)
                             action (symbol-code
                                     'visualization_msgs-msg:<marker> :add)
                             (x position pose) (cl-transforms:x point)
                             (y position pose) (cl-transforms:y point)
                             (z position pose) (cl-transforms:z point)
                             (x orientation pose) (cl-transforms:x rot)
                             (y orientation pose) (cl-transforms:y rot)
                             (z orientation pose) (cl-transforms:z rot)
                             (w orientation pose) (cl-transforms:w rot)
                             (x scale) 0.15
                             (y scale) 0.15
                             (z scale) 0.15
                             (r color) 1.0
                             (g color) 0.0
                             (b color) 0.0
                             (a color) 1.0)))))

(defmethod on-visualize-costmap-sample rviz ((point cl-transforms:3d-vector))
  (publish-point point))

(defmethod on-visualize-costmap rviz ((map location-costmap))
  (publish-location-costmap map :threshold *costmap-valid-solution-threshold*))
