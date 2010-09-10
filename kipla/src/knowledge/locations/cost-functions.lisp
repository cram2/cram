
(in-package :kipla-reasoning)

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

(defun make-table-cost-function (name &optional (std-dev 1.0))
  (let* ((mean (get-annotated-point name))
         (mean-vec (make-array 2 :initial-contents `(,(cl-transforms:x mean) ,(cl-transforms:y mean))))
         (scaled-cov (make-array '(2 2) :initial-contents `((,std-dev 0) (0 ,std-dev)))))
    (make-gauss-cost-function mean-vec scaled-cov)))

(defun make-range-cost-function (point distance)
  "Returns a costfunction that returns 1 for every point that is not
  further than distance away from point."
  (lambda (x y)
    (if (> (cl-transforms:v-dist point (cl-transforms:make-3d-vector x y 0))
           distance)
        0.0d0
        1.0d0)))

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
