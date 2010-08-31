
(in-package :location-costmap)

(defun make-gauss-cost-function (mean cov)
  (let ((gauss-fun (cma:gauss (cma:double-matrix-from-array cov)
                              (cma:double-matrix-from-array mean)))
        (pos (cma:make-double-vector 2)))
    (lambda (x y)
      (setf (aref pos 0 0) x)
      (setf (aref pos 1 0) y)
      (funcall gauss-fun pos))))

(defun make-location-cost-function (loc std-dev)
  (let ((loc (cl-transforms:origin loc)))
    (make-gauss-cost-function (make-array 2 :initial-contents (list (cl-transforms:x loc)
                                                                    (cl-transforms:y loc)))
                              (make-array '(2 2)
                                          :initial-contents `((,(coerce (* std-dev std-dev)
                                                                        'double-float) 0)
                                                              (0 ,(coerce (* std-dev std-dev)
                                                                          'double-float)))))))

(defun make-range-cost-function (point distance)
  "Returns a costfunction that returns 1 for every point that is not
  further than distance away from point."
  (lambda (x y)
    (if (> (cl-transforms:v-dist point (cl-transforms:make-3d-vector x y 0))
           distance)
        0.0d0
        1.0d0)))

(defun make-occupancy-grid-cost-function (grid &key invert)
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
          0.0d0))))
