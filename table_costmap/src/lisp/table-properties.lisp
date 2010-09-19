
(in-package :table-costmap)

(defvar *initial-stddev* 1.0
  "Initial std deviation for table-properties.")

(defvar *accept-threshold* 0.1
  "Threshold above which a point is accepted to belong to a specific
  table.")

(defclass table-properties ()
  ((mean :reader mean)
   (cov :reader cov :initarg :cov
        :initform (make-array
                   2 :element-type 'double-float
                   :initial-contents `((,*initial-stddev* 0)
                                       (0 ,*initial-stddev*))))
   (annotation :reader annotation :initarg annotation)))

(defmethod initialize-instance :after ((props table-properties) &key
                                       annotation)
  (setf (slot-value props 'mean)
        (get-annotated-point annotation)))

(defun 3d-vector->mean (vec)
  (let ((result (make-array 2 :element-type 'double-float)))
    (declare (type (simple-array double-float (2)) result))
    (setf (aref result 0) (cl-transforms:x vec)
          (aref result 1) (cl-transforms:y vec))
    result))

(defun points-mean (pts)
  "Returns the mean vector of all points in `pts'"
  (let ((x 0.0d0)
        (y 0.0d0)
        (z 0.0d0)
        (n 0))
    (map 'nil (lambda (pt)
                (incf n)
                (incf x (cl-transforms:x pt))
                (incf y (cl-transforms:y pt))
                (incf z (cl-transforms:z pt)))
         pts)
    (cl-transforms:make-3d-vector (/ x n)
                                  (/ y n)
                                  (/ z n))))

(defun points-cov (pts &optional (mean (points-mean pts)))
  "Returns the covariance of `pts'."
  (let ((cov (make-array
              '(3 3)
              :element-type 'double-float
              :initial-element 0.0d0))
        (n 0)
        (accessors (make-array 3 :initial-contents (list #'cl-transforms:x
                                                         #'cl-transforms:y
                                                         #'cl-transforms:z))))
    (declare (type (simple-array double-float (3 3)) cov)
             (type (simple-array t (3)) accessors))
    (map 'nil (lambda (pt)
                (dotimes (y 3)
                  (dotimes (x y)
                    (let ((val (* (- (funcall (aref accessors x) pt)
                                     (funcall (aref accessors x) mean))
                                  (- (funcall (aref accessors y) pt)
                                     (funcall (aref accessors y) mean)))))
                      (incf (aref cov y x) val)
                      (incf n)))))
         pts)
    (dotimes (y 3)
      (dotimes (x 3)
        (setf (aref cov y x) (/ (aref cov y x) n))
        (unless (eql x y)
          (setf (aref cov x y) (aref cov y x)))))
    cov))

(defun process-table-grid-cells-msg (properties msg)
  "Takes a grid cells message and returns the updated list of
  TABLE-PROPERTIES. `properties' is the list of TABLE-PROPERTIES to be
  updated."
  (labels ((find-closest-table (pt table-props &optional
                                   (best (car properties))
                                   (best-dist (cl-transforms:v-dist
                                               pt (mean (car properties)))))
             (if (not table-props)
                 best
                 (let ((dist (cl-transforms:v-dist
                              pt (mean (car table-props)))))
                   (if (< dist best-dist)
                       (find-closest-table (cdr table-props) pt dist)
                       (find-closest-table (cdr table-props) best best-dist)))))
           (cluster-points (points gaussians)
             (let ((annotated-points (make-hash-table)))
               (dolist (pt points)
                 (let ((pt-3d (cl-transforms:make-3d-vector
                               (geometry_msgs-msg:x-val pt)
                               (geometry_msgs-msg:y-val pt)
                               (geometry_msgs-msg:z-val pt))))
                   (let ((closest (find-closest-table
                                   pt-3d properties)))
                     (when (< (funcall (gethash (annotation closest) gaussians)
                                       (cl-transforms:x pt-3d)
                                       (cl-transforms:y pt-3d))
                              *accept-threshold*)
                       (push pt-3d (gethash (annotation closest) annotated-points))))))
               annotated-points)))
    (let ((gaussians (make-hash-table)))
      (dolist (prop properties)
        (with-slots (mean cov annotation) prop
          (setf (gethash annotation gaussians)
                (make-gauss-cost-function (3d-vector->mean mean) cov))))
      (with-fields (cells) msg
        (let ((clustered-points (cluster-points cells gaussians)))
          (mapcar
           (lambda (prop)
             (if (gethash (annotation prop) clustered-points)
                 (let* ((mean (points-mean (gethash (annotation prop)
                                                    clustered-points)))
                        (cov (points-cov (gethash (annotation prop)
                                                  clustered-points)
                                         mean)))
                   (make-instance 'table-properties
                                  :annotation (annotation prop)
                                  :mean mean
                                  :cov cov))
                 prop))
           properties))))))
