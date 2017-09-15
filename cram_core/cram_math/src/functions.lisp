
(in-package :cma)

(defun determinant (mat)
  "Invert the matrix."
  (multiple-value-bind (lu out-per sign)
      (gsll:LU-decomposition (gsll:copy mat))
    (declare (ignore out-per))
    (gsll:lu-determinant lu sign)))

(defun invert-matrix (matrix)
  "Invert the matrix; return the inverse and determinant."
  (multiple-value-bind (LU permutation signum)
      (gsl:LU-decomposition (grid:copy matrix))
    (values
      (gsl:LU-invert LU permutation)
      (gsl:LU-determinant LU signum))))

(defun gauss (cov mean)
  "Returns a function that calculates the multivariate gauss for `cov'
   and `mean'

   This function works on cram_math matrices"
  (let* ((k (double-vector-size mean))
         (det^1/2 (sqrt (determinant (grid-from-double-matrix cov))))
         (c (/ (* (expt (* 2 pi) (/ k 2)) det^1/2)))
         (cov-inv (double-matrix-from-grid (invert-matrix (grid-from-double-matrix cov))))
         (x-mean-trans (make-double-matrix (double-vector-size mean) 1))
         (cov-x-prod (make-double-vector (height cov)))
         (e-factor-mat (make-double-matrix 1 1))
         (x-mean (make-double-vector (double-vector-size mean))))
    (declare (type double-float c))
    (lambda (x)
      (declare (type cma:double-matrix x))
      (* c (exp (* -0.5 (aref (double-matrix-product
                               (double-matrix-transpose (m- x mean x-mean)
                                                        x-mean-trans)
                               (double-matrix-product cov-inv x-mean cov-x-prod)
                               e-factor-mat)
                              0 0)))))))
