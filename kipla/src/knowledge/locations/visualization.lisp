
(in-package :kipla)

(defvar *location-costmap-publisher* nil)
(defvar *marker-publisher* nil)
(defvar *occupancy-grid-publisher* nil)

(defun location-costmap-vis-init ()
  (setf *location-costmap-publisher*
        (roslisp:advertise "/kipla/location_costmap" "mapping_msgs/CollisionMap"))
  (setf *occupancy-grid-publisher*
        (roslisp:advertise "/kipla/location_occupancy_grid" "nav_msgs/OccupancyGrid"))  
  (setf *marker-publisher*
        (roslisp:advertise "/kipla/location_marker" "visualization_msgs/Marker")))

(register-ros-init-function location-costmap-vis-init)

(defun location-costmap->collision-map (map &key (frame-id "/map") (threshold 0.0005))
  (with-slots (origin-x origin-y resolution) map
    (let* ((map-array (get-cost-map map))
           (boxes nil)
           (max-val (loop for y from 0 below (array-dimension map-array 1)
                          maximizing (loop for x from 0 below (array-dimension map-array 1)
                                           maximizing (aref map-array y x)))))
      (declare (type cma:double-matrix map-array))
      (dotimes (row (array-dimension map-array 0))
        (dotimes (col (array-dimension map-array 0))
          (when (> (aref map-array row col) threshold)
            (push (make-message "mapping_msgs/OrientedBoundingBox"
                                (x center) (+ (* col resolution) origin-x)
                                (y center) (+ (* row resolution) origin-y)
                                (z center) (/ (aref map-array row col) max-val)
                                (x extents) resolution
                                (y extents) resolution
                                (z extents) resolution
                                (x axis) 1
                                (y axis) 0
                                (z axis) 0
                                angle 0)
                  boxes))))
      (make-message "mapping_msgs/CollisionMap"
                    (frame_id header) frame-id
                    (stamp header) (ros-time)
                    boxes (map 'vector #'identity boxes)))))

(defun occupancy-grid->grid-cells-msg (grid &key (frame-id "/map") (z 0.0))
  (with-slots (origin-x origin-y width height resolution) grid
    (let ((grid-arr (grid grid))
          (resolution/2 (/ resolution 2)))
      (declare (type (simple-array fixnum 2) grid-arr))
      (make-message "nav_msgs/GridCells"
                    (frame_id header) frame-id
                    (stamp header) (ros-time)
                    cell_width resolution
                    cell_height resolution
                    cells (map 'vector #'identity
                               (loop for row below (array-dimension grid-arr 0)
                                     nconcing (loop for col below (array-dimension grid-arr 1)
                                                    when (eql (aref grid-arr row col) 1) collect
                                                      (make-message "geometry_msgs/Point"
                                                           x (+ (* col resolution) resolution/2 origin-x)
                                                           y (+ (* row resolution) resolution/2 origin-y)
                                                           z z))))))))

(defun publish-location-costmap (map &key (frame-id "/map") (threshold 0.0005))
  (publish *location-costmap-publisher* (location-costmap->collision-map
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
                           (x scale) 0.15
                           (y scale) 0.15
                           (z scale) 0.15
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

(defun publish-location-desig-cost-function (desig)
  (reference desig)
  (let ((cm (find-if (rcurry #'typep 'kipla-reasoning:costmap-location-proxy)
                     (slot-value desig 'data))))
    (assert cm () "No location costmap found. Cannot visualize.")
    (publish-location-costmap (kipla-reasoning::costmap cm))))
