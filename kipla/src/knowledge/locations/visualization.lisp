
(in-package :kipla)

(defvar *location-costmap-grid-publisher* nil)
(defvar *marker-publisher* nil)
(defvar *occupancy-grid-publisher* nil)

(defun location-costmap-vis-init ()
  (setf *location-costmap-grid-publisher*
        (roslisp:advertise "/kipla/location_costmap" "nav_msgs/GridCells"))
  (setf *occupancy-grid-publisher*
        (roslisp:advertise "/kipla/location_occupancy_grid" "nav_msgs/OccupancyGrid"))  
  (setf *marker-publisher*
        (roslisp:advertise "/kipla/location_marker" "visualization_msgs/Marker")))

(register-ros-init-function location-costmap-vis-init)

(defun location-costmap->grid-cells-msg (map &key (frame-id "/map") (threshold 0.0005))
  (with-slots (origin-x origin-y resolution) map
    (let* ((map-array (get-cost-map map))
           (grid-cells nil)
           (max-val (loop for y from 0 below (array-dimension map-array 1)
                          maximizing (loop for x from 0 below (array-dimension map-array 1)
                                           maximizing (aref map-array x y)))))
      (loop for y from 0 below (array-dimension map-array 1)
                             do (loop for x from 0 below (array-dimension map-array 0)
                                      do (when (> (aref map-array x y) threshold)
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

(defun publish-location-costmap (map &key (frame-id "/map") (threshold 0.0005))
  (publish *location-costmap-grid-publisher* (location-costmap->grid-cells-msg
                                              map :frame-id frame-id :threshold threshold)))

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
                           (w orientation pose) 1
                           (x scale) 0.05
                           (y scale) 0.05
                           (z scale) 0.05
                           (r color) (random 1.0)
                           (g color) (random 1.0)
                           (b color) (random 1.0)
                           (a color) 1)))

  (defun publish-pose (pose)
    (let ((point (cl-transforms:origin pose))
          (rot (cl-transforms:orientation pose)))
      (publish *marker-publisher*
               (make-message "visualization_msgs/Marker"
                             (stamp header) (ros-time)
                             (frame_id header) "/map"
                             ns "kipla_locations"
                             id (incf current-index)
                             type (symbol-code 'visualization_msgs-msg:<marker> :arrow)
                             action (symbol-code 'visualization_msgs-msg:<marker> :add)
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
                             (r color) 1
                             (g color) 0
                             (b color) 0
                             (a color) 1)))))
