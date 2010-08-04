
(in-package :cma)

(defun determinant (mat)
  "Invert the matrix."
  (multiple-value-bind (lu out-per sign)
      (gsll:LU-decomposition (gsll:copy mat))
    (declare (ignore out-per))
    (gsll:lu-determinant lu sign)))

(defun gauss (cov mean)
  "Returns a function that calculates the multivariate gauss for `cov'
   and `mean'

   This function works on gsll matrices and vectors."
  (flet ((vector-to-matrix (vec)
           (grid:make-foreign-array (gsll:element-type vec)
                                    :dimensions `(,(gsll:dim0 vec) 1)
                                    :initial-contents (loop for i below (gsll:dim0 vec)
                                                            collecting (list (grid:gref vec i))))))
    (let* ((k (gsll:dim0 mean))
           (det^1/2 (sqrt (determinant cov)))
           (c (/ (* (expt (* 2 pi) (/ k 2)) det^1/2)))
           (cov-inv (gsll:invert-matrix cov)))
      (lambda (x)
        (let ((x-matrix (vector-to-matrix (gsll:elt- x mean))))
          (* c (exp (* -0.5 (grid:gref (gsll:matrix-product (gsll:matrix-transpose x-matrix)
                                                            (gsll:matrix-product cov-inv x-matrix))
                                       0 0)))))))))
