
(in-package :kipla)

(defvar *location-costmap-grid-publisher* nil)
(defvar *marker-publisher* nil)

(defun location-costmap-vis-init ()
  (setf *location-costmap-grid-publisher*
        (roslisp:advertise "/kipla/location_costmap" "nav_msgs/GridCells"))
  (setf *marker-publisher*
        (roslisp:advertise "/kipla/location_marker" "visualization_msgs/Marker")))

(register-ros-init-function location-costmap-vis-init)

(defun location-costmap->grid-cells-msg (map &key (frame-id "/map"))
  (with-slots (origin-x origin-y resolution) map
    (let* ((map-array (get-cost-map map))
           (grid-cells nil)
           (max-val (loop for y from 0 below (array-dimension map-array 1)
                          maximizing (loop for x from 0 below (array-dimension map-array 1)
                                           maximizing (aref map-array x y)))))
      (loop for y from 0 below (array-dimension map-array 1)
                             do (loop for x from 0 below (array-dimension map-array 0)
                                      do (when (> (aref map-array x y) 0.0)
                                           (push (make-message "geometry_msgs/Point"
                                                        x (+ (* x resolution) origin-x)
                                                        y (+ (* y resolution) origin-y)
                                                        z (/ (aref map-array x y) max-val))
                                                 grid-cells))))
      (make-message "nav_msgs/GridCells"
                    (frame_id header) frame-id
                    (stamp header) (ros-time)
                    cell_width resolution
                    cell_height resolution
                    cells (map 'vector #'identity grid-cells)))))

(defun publish-location-costmap (map &key (frame-id "/map"))
  (publish *location-costmap-grid-publisher* (location-costmap->grid-cells-msg
                                              map :frame-id frame-id)))

(let ((current-index 0))
  
  (defun publish-point (point)
    (publish *marker-publisher*
             (make-message "visualization_msgs/Marker"
                           (stamp header) (ros-time)
                           (frame_id header) "/map"
                           ns "kipla_locations"
                           id (incf current-index)
                           type (symbol-code 'visualization_msgs-msg:<marker> :sphere)
                           action (symbol-code 'visualization_msgs-msg:<marker> :add)
                           (x position pose) (cl-transforms:x point)
                           (y position pose) (cl-transforms:y point)
                           (z position pose) (cl-transforms:z point)
                           (w orientation pose) 0
                           (x scale) 0.1
                           (y scale) 0.1
                           (z scale) 0.1
                           (r color) (random 1.0)
                           (g color) (random 1.0)
                           (b color) (random 1.0)
                           (a color) 1))))

