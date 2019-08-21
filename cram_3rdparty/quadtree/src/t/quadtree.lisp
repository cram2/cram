#|
  This file is a part of quadtree project.
  Copyright (c) 2015 Masayuki Takagi (kamonama@gmail.com)
|#

(in-package :cl-user)
(defpackage quadtree-test
  (:use :cl
        :prove))
(in-package :quadtree-test)

(plan nil)

(defstruct (point (:constructor make-point (x y)))
  (x 0d0 :read-only t)
  (y 0d0 :read-only t))

(defmethod quadtree:intersect-p (quadtree (point point))
  (destructuring-bind (x0 y0 x1 y1) (quadtree:boundary quadtree)
    (let ((x (point-x point))
          (y (point-y point)))
      (and (<= x0 x x1) (<= y0 y y1)))))

(let ((points (list (make-point 0.0d0 0.0d0)
                    (make-point 1.0d0 1.0d0)
                    (make-point 0.1d0 0.1d0)
                    (make-point 0.1d0 0.2d0)
                    (make-point 0.2d0 0.1d0)
                    (make-point 0.2d0 0.2d0)
                    (make-point 0.2d0 0.3d0)
                    (make-point 0.2d0 0.4d0)
                    (make-point 0.3d0 0.2d0)
                    (make-point 0.3d0 0.3d0))))
  (let ((qt (quadtree:make 0.0d0 0.0d0 1.0d0 1.0d0)))
    ;; build the quadtree
    (loop for point in points
       do (quadtree:insert qt point))
    ;; query the quadtree
    (is (length (quadtree:query qt 0.1 0.1)) 5)))


(finalize)
