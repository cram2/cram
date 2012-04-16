
(in-package :location-costmap)

(defun make-gauss-cost-function (mean covariance)
  (let ((mean (etypecase mean
                (list (make-array 2 :initial-contents mean))
                (array mean)
                (cl-transforms:3d-vector
                 (make-array
                  2 :initial-contents (list
                                       (float (cl-transforms:x mean) 0.0d0)
                                       (float (cl-transforms:y mean) 0.0d0))))))
        (covariance (etypecase covariance
                      (list (make-array '(2 2) :initial-contents covariance))
                      (array covariance))))
    (let ((gauss-fun (cma:gauss (cma:double-matrix-from-array covariance)
                                (cma:double-matrix-from-array mean)))
          (pos (cma:make-double-vector 2)))
      (lambda (x y)
        (setf (aref pos 0 0) x)
        (setf (aref pos 1 0) y)
        (funcall gauss-fun pos)))))

(defun make-location-cost-function (loc std-dev)
  (let ((loc (cl-transforms:origin loc)))
    (make-gauss-cost-function loc `((,(float (* std-dev std-dev) 0.0d0) 0.0d0)
                                    (0.0d0 ,(float (* std-dev std-dev)))))))

(defun make-range-cost-function (point distance &key invert)
  "Returns a costfunction that returns 1 for every point that is not
  further than distance away from point."
  (let ((z (cl-transforms:z point))
        (in-range (if invert 0.0d0 1.0d0))
        (out-range (if invert 1.0d0 0.0d0)))
    (lambda (x y)
      (if (> (cl-transforms:v-dist point (cl-transforms:make-3d-vector x y z))
             distance)
          out-range
          in-range))))

(defun make-axis-boundary-cost-function (axis boundary side)
  "Returns a cost function that has the value 1 if the pose is on the
respective side of `boundary'.

The value of `axis' is either :X or :Y.

`boundary' is a NUMBER.

`side' is either :left or :right. If `side' is :left, the value 1.0 is
returned for poses < `boundary', otherwise poses > `boundary' result
in a value of 1.0"

  (let ((pred (ecase side
                (:left #'<)
                (:right #'>))))
    (lambda (x y)
      (if (funcall
           pred
           (ecase axis
             (:x x)
             (:y y))
           boundary)
          1.0d0
          0.0d0))))

(defun make-occupancy-grid-cost-function (grid &key invert)
  (when grid
    (let* ((grid (if invert
                     (invert-occupancy-grid grid)
                     grid))
           (origin-x (origin-x grid))
           (origin-y (origin-y grid))
           (max-x (+ (width grid) origin-x (- (resolution grid))))
           (max-y (+ (height grid) origin-y (- (resolution grid)))))
      (lambda (x y)
        (if (and (>= x origin-x) (>= y origin-y)
                 (< x max-x) (< y max-y))
            (coerce (get-grid-cell grid x y) 'double-float)
            (if invert 1.0d0 0.0d0))))))

(defun make-padded-costmap-cost-function (costmap padding &key invert)
  (when (and costmap padding)
    (let* ((grid (matrix->occupancy-grid (origin-x costmap)
                                         (origin-y costmap)
                                         (resolution costmap)
                                         (get-cost-map costmap)))
           (padding-mask (make-padding-mask (round (/ padding (resolution costmap)))))
           (pred (if invert
                     (lambda (x) (eql x 0))
                     (lambda (x) (eql x 1))))
           (grid-arr (grid grid))
           (max-row (- (array-dimension grid-arr 0)
                       (truncate (/ padding (resolution costmap)))))
           (max-col (- (array-dimension grid-arr 1)
                       (truncate (/ padding (resolution costmap)))))
           (result-grid (copy-occupancy-grid grid)))
      (do ((row (truncate (/ padding (resolution costmap))) (1+ row)))
          ((>= row max-row))
        (do ((col (truncate (/ padding (resolution costmap))) (1+ col)))
            ((>= col max-col))
          (when (funcall pred (aref grid-arr row col))
            (occupancy-grid-put-mask col row result-grid padding-mask :coords-raw-p t))))
      (make-occupancy-grid-cost-function result-grid :invert invert))))
