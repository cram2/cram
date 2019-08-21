#|
  This file is a part of quadtree project.
  Copyright (c) 2015 Masayuki Takagi (kamonama@gmail.com)
|#

(in-package :cl-user)
(defpackage quadtree
  (:use :cl :alexandria)
  (:export #:make-quadtree-internal
           #:make-quadtree
           #:boundary
           #:insert
           #:query
           #:clear-quadtree
           #:intersect-p
           #:object-equal
           #:same-coords-as))
(in-package :quadtree)

(defvar *max-depth* 50)

(defstruct (quadtree (:constructor %make-quadtree)) ;; % prefix, because one should use make-quadtree instead
  object
  nw ne sw se
  boundary
  depth
  max-depth)

(defstruct (point (:constructor make-point (x y o)))
  (x 0d0 :read-only t)
  (y 0d0 :read-only t)
  (o 0d0 :read-only t))

(defun make-quadtree (x0 y0 x1 y1 &key (max-depth *max-depth*))
  (%make-quadtree-internal x0 y0 x1 y1 0 max-depth))

;; (defmethod initialize-instance :after ((grid occupancy-grid) &key)
;;   (with-slots (width height resolution grid) grid
;;     (setf grid (make-array (list (round (/ height resolution))
;;                                  (round (/ width resolution)))
;;                            :element-type 'fixnum
;;                            :initial-element 0))))

(defun %make-quadtree-internal (x0 y0 x1 y1 depth max-depth)
  (declare (float x0 y0 x1 y1))
  (unless (<= 0 depth max-depth)
    (error "The value ~S is an linvalid value as depth." depth))
  (%make-quadtree :boundary (list x0 y0 x1 y1)
                  :depth depth
                  :max-depth max-depth))

;; (defun copy-quadtree (src)
;;   (when src
;;     ;;(declare (type quadtree src))
;;     (with-slots (object nw ne sw se boundary depth max-depth) src
;;       (let ((copy-src (%make-quadtree-internal (quadtree-x0 boundary)
;;                                                (quadtree-y0 boundary)
;;                                                (quadtree-x1 boundary)
;;                                                (quadtree-y1 boundary)
;;                                                depth
;;                                                max-depth)))
;;         (setf (quadtree-object copy-src) (copy-object object))
;;         (setf (quadtree-nw copy-src) (copy-quadtree nw)
;;               (quadtree-ne copy-src) (copy-quadtree ne)
;;               (quadtree-sw copy-src) (copy-quadtree sw)
;;               (quadtree-se copy-src) (copy-quadtree se))))))

(defun quadtree-x0 (quadtree-boundary)
  (when quadtree-boundary
    (first quadtree-boundary)))

(defun quadtree-y0 (quadtree-boundary)
  (when quadtree-boundary
    (second quadtree-boundary)))

(defun quadtree-x1 (quadtree-boundary)
  (when quadtree-boundary
    (third quadtree-boundary)))

(defun quadtree-y1 (quadtree-boundary)
  (when quadtree-boundary
    (fourth quadtree-boundary)))                                               
                             
(defun boundary (quadtree)
  (quadtree-boundary quadtree))

(defun node-p (quadtree)
  (and (quadtree-nw quadtree)
       t))

(defun leaf-p (quadtree)
  (not (node-p quadtree)))

(defun full-p (quadtree)
  (quadtree-object quadtree))

(defun max-depth-p (quadtree)
  (= (quadtree-depth quadtree)
     (quadtree-max-depth quadtree)))

(defun root-p (quadtree)
  (= (quadtree-depth quadtree) 0))

(defun subtrees (quadtree)
  (when quadtree
    (list (quadtree-nw quadtree) (quadtree-ne quadtree)
          (quadtree-sw quadtree) (quadtree-se quadtree))))

(defun intersects-with-subtrees-p (object subtrees)
  (when (and object subtrees)
    (every #'identity (mapcar (lambda (st) (intersect-p st object)) subtrees))))

(defun middlepoint-of-quadtree-p (object quadtree)
  (intersects-with-subtrees-p object (subtrees quadtree)))
    
(defun subdevide (quadtree)
  (unless (leaf-p quadtree)
    (error "The quadtree ~A is already subdevided." quadtree))
  (destructuring-bind (x0 y0 x1 y1) (boundary quadtree)
    (declare (double-float x0 y0 x1 y1))
    (let ((xmid (/ (+ x1 x0) 2.0))
          (ymid (/ (+ y1 y0) 2.0))
          (depth (1+ (the fixnum (quadtree-depth quadtree))))
          (max-depth (quadtree-max-depth quadtree)))
      (setf (quadtree-nw quadtree) (%make-quadtree-internal x0 ymid xmid y1 depth max-depth)
            (quadtree-ne quadtree) (%make-quadtree-internal xmid ymid x1 y1 depth max-depth)
            (quadtree-sw quadtree) (%make-quadtree-internal x0 y0 xmid ymid depth max-depth)
            (quadtree-se quadtree) (%make-quadtree-internal xmid y0 x1 ymid depth max-depth))))
  (let ((object (quadtree-object quadtree)))
    (insert (quadtree-nw quadtree) object)
    (insert (quadtree-ne quadtree) object)
    (insert (quadtree-sw quadtree) object)
    (insert (quadtree-se quadtree) object))
  t)

(defgeneric intersect-p (quadtree object)
  (:documentation "Returns if the object intersects the quadtree"))

(defmethod quadtree:intersect-p (quadtree (point point))
  (destructuring-bind (x0 y0 x1 y1) (quadtree:boundary quadtree)
    (let ((x (point-x point))
          (y (point-y point)))
      (and (<= x0 x x1) (<= y0 y y1)))))

(defgeneric object-equal (object object)
  (:documentation "Returns if the objects are equal to save space"))

(defmethod quadtree:object-equal ((p1 point) (p2 point))
            (let ((o1 (point-o p1))
                  (o2 (point-o p2)))
              (equal o1 o2)))

(defgeneric same-coords-as (object object)
  (:documentation "Returns if the objects are equal to save space"))

(defmethod quadtree:same-coords-as ((p1 point) (p2 point))
  (let ((x1 (point-x p1))
        (x2 (point-x p2))
        (y1 (point-y p1))
        (y2 (point-y p2)))
    (and (equal x1 x2)
         (equal y1 y2))))

(defun set-in-quadtree (quadtree x y)
  ;;declare TODO
  (insert quadtree (make-point x y 1.0d0)))

(defun get-in-quadtree (quadtree x y &optional neighbor-p)
  (funcall (lambda (queried)
             (if neighbor-p
                 (when (same-coords-as (make-point x y 0.0d0) queried)
                   queried)
                 queried))
           (query quadtree x y)))
         
(defun clear-in-quadtree (quadtree x y &optional remove-subtrees-if-possible)
  (let ((cleared-point (make-point x y 0.0d0)))
    (multiple-value-bind (queried-point queried-trees) (query quadtree x y t)
      (print queried-point)
      (when (same-coords-as cleared-point queried-point)
        (setf (quadtree-object (first (last queried-trees 1))) cleared-point)
        (when remove-subtrees-if-possible
          (let ((parent (first (last queried-trees 2))))
            (when (reduce #'object-equal
                          (remove-if-not #'identity
                                         (mapcar #'quadtree-object (list
                                                                    (quadtree-nw parent)
                                                                    (quadtree-ne parent)
                                                                    (quadtree-sw parent)
                                                                    (quadtree-se parent)))))
              (clear-subtrees-of parent)
              (setf (quadtree-object parent)
                    (make-point 
                     (point-x (quadtree-object parent))
                     (point-y (quadtree-object parent))
                     0.0d0)))))))))

(defun insert (quadtree object) ;; instead of object: x, y, object? instead object has to save x and y 
  (cond
    ;; When the object does not intersect the quadtree, just return nil.
    ((not (if quadtree (intersect-p quadtree object))) nil)
    ;; When the quadtree is a node, recursively insert the object to its children.
    ((node-p quadtree)
     (insert (quadtree-nw quadtree) object)
     (insert (quadtree-ne quadtree) object)
     (insert (quadtree-sw quadtree) object)
     (insert (quadtree-se quadtree) object)
     t)
    ;; Insert the object to the quadtree, if leaf is free.
    ((not (full-p quadtree))
     (setf (quadtree-object quadtree) object)
     t)
    ;; If leaf is not free, but has same coords as object, update the object
    ((same-coords-as (quadtree-object quadtree) object)
     (setf (quadtree-object quadtree) object)
     t)
    ;; When the quadtree is full and is not at its max depth, subdevide it and
    ;; recursively insert the object.
    ((and (not (max-depth-p quadtree))
          (not (object-equal object (quadtree-object quadtree))))
     (subdevide quadtree)
     (insert quadtree object))
    ;; Otherwise if the max-depth-p or the objects were equal (object-equal returned t),
    ;; do nothing, since the object represents accordingly the given object. 
    (t
     t)))

(defun point-intersect-p (quadtree x y)
  (destructuring-bind (x0 y0 x1 y1) (boundary quadtree)
    (and (<= x0 x x1)
         (<= y0 y y1))))

(defun query (quadtree x y &optional trees)
  (let ((ret (query-tree quadtree x y '())))
    (when ret
      (values (quadtree-object (first (last ret)))
              (if trees
                  ret
                  nil)))))

(defun query-tree (quadtree x y path)
  (when (point-intersect-p quadtree x y)
    (if (node-p quadtree)
        (or (query-tree (quadtree-nw quadtree) x y (append path (list (quadtree-nw quadtree))))
            (query-tree (quadtree-ne quadtree) x y (append path (list (quadtree-ne quadtree))))
            (query-tree (quadtree-sw quadtree) x y (append path (list (quadtree-sw quadtree))))
            (query-tree (quadtree-se quadtree) x y (append path (list (quadtree-se quadtree)))))
        (when (quadtree-object quadtree)
          path))))

(defun clear-subtrees-of (quadtree)
  (setf (quadtree-nw quadtree) nil
        (quadtree-ne quadtree) nil
        (quadtree-sw quadtree) nil
        (quadtree-se quadtree) nil)
  t)

(defun clear-quadtree (quadtree)
  (setf (quadtree-object quadtree) nil)
  (clear-subtrees-of quadtree)
  t)
