
(in-package :table-costmap)

(defmethod costmap-generator-name->score ((name (eql 'location-neighborhood)))
  5)

(defmethod costmap-generator-name->score ((name (eql 'named-table)))
  5)

(defmethod costmap-generator-name->score ((name (eql 'all-tables)))
  10)

(defmethod costmap-generator-name->score ((name (eql 'tables-occupied)))
  8)

(defmethod costmap-generator-name->score ((name (eql 'static-occupied)))
  9)

(defmethod costmap-generator-name->score ((name (eql 'free-space)))
  10)

(defun make-robot-pos-generator (threshold &key (n-solutions 1))
  "Returns a function that returns the position of the robot in the
map if its probability value in the corresponding costmap is greater
than threshold * highest-probability."
  (flet ((max-map-value (map-arr)
           (declare (type cma:double-matrix map-arr))
           (let ((max-value 0.0d0))
             (dotimes (row (array-dimension map-arr 0))
               (dotimes (col (array-dimension map-arr 1))
                 (when (> (aref map-arr row col) max-value)
                   (setq max-value (aref map-arr row col)))))
             max-value)))
    (let ((solution-cnt 0))
      (lambda (map)
        (unless (and (>= solution-cnt n-solutions) *tf*)
          (incf solution-cnt)
          (when (cl-tf:can-transform *tf* :target-frame "/map" :source-frame "/base_link")
            (let* ((robot-pos (cl-transforms:translation
                               (cl-tf:lookup-transform
                                *tf* :target-frame "/map" :source-frame "/base_link")))
                   (x (cl-transforms:x robot-pos))
                   (y (cl-transforms:y robot-pos))
                   (map-arr (get-cost-map map))
                   (max-value (max-map-value map-arr)))
              (declare (type cma:double-matrix map-arr))
              (when (> (get-map-value map x y) (* threshold max-value))
                robot-pos))))))))

(defun nav-angle-to-point (p p-ref)
  "Calculates the angle from `p-ref' to face at `p'"
  (cl-transforms:axis-angle->quaternion
   (cl-transforms:make-3d-vector 0 0 1)
   (let ((p-rel (cl-transforms:v- p p-ref)))
     (atan (cl-transforms:y p-rel) (cl-transforms:x p-rel)))))

(def-fact-group table-costmap (desig-costmap
                               desig-orientation
                               desig-z-value
                               drivable-location-costmap)

  (<- (drivable-location-costmap ?cm ?padding)
    (global-fluent-value *map-fl* ?map)
    (global-fluent-value *table-grid-cells-fl* ?tables)
    (inverted-occupancy-grid ?map ?free-space)
    (occupancy-grid ?map ?static-occupied (padding ?padding))
    (occupancy-grid ?tables ?tables-occupied (padding ?padding))
    (costmap ?cm)
    (lisp-fun cl-transforms:make-3d-vector 0 0 0 ?map-origin)
    (costmap-add-function free-space (make-occupancy-grid-cost-function ?free-space) ?cm)
    (costmap-add-function static-occupied (make-occupancy-grid-cost-function ?static-occupied :invert t)
                          ?cm)
    (costmap-add-function tables-occupied (make-occupancy-grid-cost-function ?tables-occupied :invert t)
                          ?cm)
    (costmap-add-generator (make-robot-pos-generator 0.1) ?cm))
  
  (<- (desig-costmap ?desig ?cm)
    (desig-prop ?desig (on table))
    (desig-prop ?desig (name ?table-name))
    (annotated-point ?table-name ?_)
    (global-fluent-value *table-grid-cells-fl* ?table-msg)
    (occupancy-grid ?table-msg ?table-costmap)
    (costmap ?cm)
    (costmap-add-function named-table (make-table-cost-function ?table-name 1.0) ?cm)
    (costmap-add-function all-tables (make-occupancy-grid-cost-function ?table-costmap) ?cm))

  (<- (desig-costmap ?desig ?cm)
    (desig-prop ?desig (on table))
    (not (desig-prop ?desig (name ?_)))
    (global-fluent-value *table-grid-cells-fl* ?table-msg)
    (occupancy-grid ?table-msg ?table-costmap)
    (costmap ?cm)
    (costmap-add-function all-tables (make-occupancy-grid-cost-function ?table-costmap) ?cm))
  
  (<- (desig-costmap ?desig ?cm)
    (desig-prop ?desig (to see))
    (desig-location-prop ?desig ?loc)
    (costmap ?cm)
    (costmap-padding ?padding)
    (drivable-location-costmap ?cm ?padding)
    (costmap-add-function location-neighborhood (make-location-cost-function ?loc 0.5) ?cm))

  (<- (desig-costmap ?desig ?cm)
    (desig-prop ?desig (to reach))
    (desig-location-prop ?desig ?loc)
    (costmap ?cm)
    (costmap-manipulation-padding ?padding)
    (drivable-location-costmap ?cm ?padding)
    (costmap-add-function location-neighborhood (make-location-cost-function ?loc 0.4) ?cm))

    ;; Missing: (for ?obj-type)

  (<- (desig-z-value ?desig ?point ?z)
    (global-fluent-value *table-height-map-fl* ?table-heightmap)
    (lisp-fun cl-transforms:x ?point ?x)
    (lisp-fun cl-transforms:y ?point ?y)    
    (lisp-fun height-map-lookup ?table-heightmap ?x ?y ?z))

  (<- (desig-orientation ?desig ?point ?orientation)
    (or (desig-prop ?desig (to reach))
        (desig-prop ?desig (to see)))
    (desig-location-prop ?desig ?loc)
    (lisp-fun cl-transforms:origin ?loc ?loc-p)
    (lisp-fun nav-angle-to-point ?loc-p ?point ?orientation)))
