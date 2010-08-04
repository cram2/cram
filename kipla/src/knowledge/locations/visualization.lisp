
(in-package :kipla)

(defvar *location-costmap-grid-publisher* nil)

(defun location-costmap-vis-init ()
  (setf *location-costmap-grid-publisher*
        (roslisp:advertise "/kipla/location_costmap" "nav_msgs/GridCells")))

(register-ros-init-function location-costmap-vis-init)

(defun location-costmap->grid-cells-msg (map &key (z-scaling-factor 1.0) (frame-id "/map"))
  (with-slots (origin-x origin-y resolution) map
    (let* ((map-array (get-cost-map map))
           (grid-cells nil))
      (loop for y from 0 below (array-dimension map-array 1)
                             do (loop for x from 0 below (array-dimension map-array 0)
                                      do (when (> (aref map-array x y) 0.0)
                                           (push (make-message "geometry_msgs/Point"
                                                        x (+ (* x resolution) origin-x)
                                                        y (+ (* y resolution) origin-y)
                                                        z (* (aref map-array x y) z-scaling-factor))
                                                 grid-cells))))
      (make-message "nav_msgs/GridCells"
                    (frame_id header) frame-id
                    (stamp header) (ros-time)
                    cell_width resolution
                    cell_height resolution
                    cells (map 'vector #'identity grid-cells)))))

(defun publish-location-costmap (map &key (z-scaling-factor 1.0) (frame-id "/map"))
  (publish *location-costmap-grid-publisher* (location-costmap->grid-cells-msg
                                              map
                                              :z-scaling-factor z-scaling-factor
                                              :frame-id frame-id)))
