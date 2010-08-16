
(in-package :kipla-reasoning)

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
