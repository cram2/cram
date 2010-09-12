
(in-package :table-costmap)

(defun make-table-cost-function (name &optional (std-dev 1.0))
  (let* ((mean (get-annotated-point name))
         (mean-vec (make-array 2 :initial-contents `(,(cl-transforms:x mean) ,(cl-transforms:y mean))))
         (scaled-cov (make-array '(2 2) :initial-contents `((,std-dev 0) (0 ,std-dev)))))
    (make-gauss-cost-function mean-vec scaled-cov)))
