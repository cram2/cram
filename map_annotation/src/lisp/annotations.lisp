
(in-package :map-annotation)

(defvar *annotated-points* nil
  "alist of annotations and the corresponding points.")

(defun init-annotated-points ()
  (unless (roslisp:wait-for-service "/table_annotations/get_annotated_points" 1.0)
    (error
     'simple-error :format-control
     "Could not connect to service '/table_annotations/get_annotated_points'"))
  (let ((srv-result (roslisp:call-service "/table_annotations/get_annotated_points"
                                          "map_annotation/GetAnnotatedPoints")))
    (map 'list (lambda (label point)
                 (with-fields ((x x) (y y) (z z))
                     point
                   (cons (lispify-ros-name label (find-package :map-annotation))
                         (make-instance 'cl-transforms:3d-vector
                                        :x x :y y :z z))))
         (map_annotation-srv:labels-val srv-result)
         (map_annotation-srv:points-val srv-result))))

(defun get-closest-annotation (point)
  (labels ((find-closest-point (points closest dist)
             (if (not points)
                 closest
                 (let ((curr-dist (cl-transforms:v-dist point (cdar points))))
                   (if (< curr-dist dist)
                       (find-closest-point (cdr points) (car points) curr-dist)
                       (find-closest-point (cdr points) closest dist))))))
    
    (check-type point cl-transforms:point)
    (unless *annotated-points*
      (setf *annotated-points* (init-annotated-points)))
    (when *annotated-points*
      (car (find-closest-point (cdr *annotated-points*) (car *annotated-points*)
                               (cl-transforms:v-dist (cdar *annotated-points*) point))))))

(defun get-annotated-point (name)
  (unless *annotated-points*
    (setf *annotated-points* (init-annotated-points)))
  (cdr (assoc (symbol-name name) *annotated-points* :key #'symbol-name :test #'equal)))

(defun get-annotation-names ()
  (unless *annotated-points*
    (setf *annotated-points* (init-annotated-points)))
  (mapcar #'car *annotated-points*))
