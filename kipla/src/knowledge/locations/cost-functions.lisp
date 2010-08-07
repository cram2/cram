
(in-package :kipla-reasoning)

(defun lisp-array->foreign-array (arr &optional (type (array-element-type arr)))
  (grid:make-foreign-array type
                           :dimensions (array-dimensions arr)
                           :initial-contents (map-tree (rcurry #'coerce type) (grid:contents arr))))

(defun list->foreign-array (l &optional (type (type-of (car l))))
  (grid:make-foreign-array type :dimensions (length l)
                           :initial-contents (mapcar (rcurry #'coerce type) l)))


(defun make-gauss-cost-function (mean cov)
  (let ((gauss-fun (cma:gauss (lisp-array->foreign-array cov 'double-float)
                              (lisp-array->foreign-array mean 'double-float))))
    (lambda (x y)
      (funcall gauss-fun (list->foreign-array `(,x ,y) 'double-float)))))

(defun make-location-cost-function (loc std-dev)
  (let ((loc (cl-transforms:origin loc)))
    (make-gauss-cost-function (make-array 2 :initial-contents (list (cl-transforms:x loc)
                                                                    (cl-transforms:y loc)))
                              (make-array '(2 2)
                                          :initial-contents `((,(coerce (* std-dev std-dev)
                                                                        'float) 0)
                                                              (0 ,(coerce (* std-dev std-dev)
                                                                          'float)))))))

(defun make-table-cost-function (occupancy-grid &optional (magic-scaling-factor 1.0))
  (let* ((mean (occupancy-grid-mean occupancy-grid))
         (cov (occupancy-grid-covariance occupancy-grid :mean mean))
         (mean-vec (make-array 2 :initial-contents `(,(cl-transforms:x mean) ,(cl-transforms:y mean))))
         (scaled-cov (grid:map-grid :source cov :element-function (curry #'* magic-scaling-factor))))
    (make-gauss-cost-function mean-vec scaled-cov)))
