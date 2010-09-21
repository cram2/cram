
(in-package :table-costmap)

(defun make-table-cost-function (name)
  (let* ((table-cluster (get-table-cluster name))
         (mean (3d-vector->mean (mean table-cluster)))
         (cov (2d-cov (cov table-cluster))))
    (make-gauss-cost-function mean cov)))
