
(in-package :cut)

(defstruct quad-tree
  (children (make-array 4 :initial-element nil))
  (range nil)
  (points-per-cell 1))

(defstruct quad-tree-node
  (points nil)
  (range nil))

(defstruct quad-tree-range
  min-x max-x
  min-y max-y)

(defstruct quad-tree-point
  x y)

(defun quad-tree (&key (points-per-cell 1) points width height)
  (assert (and width height) () "Width and height of the area to index
  on must be specified.")
  (let ((tree (make-quad-tree :points-per-cell points-per-cell
                              :range (make-quad-tree-range :min-x 0 :max-x width
                                                           :min-y 0 :max-y height))))
    (loop for p in points do (quad-tree-insert tree p))
    tree))

(defun quad-tree-index (range point)
  (let ((min-x (quad-tree-range-min-x range))
        (max-x (quad-tree-range-max-x range))
        (min-y (quad-tree-range-min-y range))
        (max-y (quad-tree-range-max-y range))
        (x (quad-tree-point-x point))
        (y (quad-tree-point-y point)))
    (+ (if (and (>= x min-x)
                (< x (+ min-x (/ (- max-x min-x) 2))))
           0 1)
       (if (and (>= y min-y)
                (< y (+ min-y (/ (- max-y min-y) 2))))
           0 2))))

(defun quad-tree-sub-range (range quadrant-index)
  (let ((old-min-x (quad-tree-range-min-x range))
        (old-max-x (quad-tree-range-max-x range))
        (old-min-y (quad-tree-range-min-y range))
        (old-max-y (quad-tree-range-max-y range)))
    (ecase quadrant-index
      (0 (make-quad-tree-range
          :min-x old-min-x :max-x (+ old-min-x (/ (- old-max-x old-min-x) 2))
          :min-y old-min-y :max-y (+ old-min-y (/ (- old-max-y old-min-y) 2))))
      (1 (make-quad-tree-range
          :min-x (+ old-min-x (/ (- old-max-x old-min-x) 2)) :max-x old-max-x
          :min-y old-min-y :max-y (+ old-min-y (/ (- old-max-y old-min-y) 2))))
      (2 (make-quad-tree-range
          :min-x old-min-x :max-x (+ old-min-x (/ (- old-max-x old-min-x) 2))
          :min-y (+ old-min-y (/ (- old-max-y old-min-y) 2)) :max-y old-max-y))
      (3 (make-quad-tree-range
          :min-x (+ old-min-x (/ (- old-max-x old-min-x) 2)) :max-x old-max-x
          :min-y (+ old-min-y (/ (- old-max-y old-min-y) 2)) :max-y old-max-y)))))

(defun quad-tree-insert (tree point)
  (let* ((quad-index (quad-tree-index (quad-tree-range tree) point))
         (child-node (aref (quad-tree-children tree) quad-index)))
    (etypecase child-node
      (null (setf (aref (quad-tree-children tree) quad-index)
                  (make-quad-tree-node :points (list point)
                                       :range (quad-tree-sub-range (quad-tree-range tree) quad-index))))
      (quad-tree (quad-tree-insert child-node point))
      (quad-tree-node (if (>= (length (quad-tree-node-points child-node))
                              (quad-tree-points-per-cell tree))
                          (let ((sub-tree (make-quad-tree :range (quad-tree-node-range child-node)
                                                          :points-per-cell (quad-tree-points-per-cell tree)))
                                (points (cons point (quad-tree-node-points child-node))))
                            (setf (aref (quad-tree-children tree) quad-index)
                                  sub-tree)
                            (loop for p in points do
                              (quad-tree-insert sub-tree p)))
                          (setf (quad-tree-node-points child-node)
                                (cons point (quad-tree-node-points child-node)))))))
  nil)

(defun quad-tree-points (tree)
  (etypecase tree
    (null nil)
    (quad-tree (loop for sub-tree across (quad-tree-children tree)
                     appending (quad-tree-points sub-tree)))
    (quad-tree-node (quad-tree-node-points tree))))

(defun %quad-tree-pt-distance (p-1 p-2)
  (+ (expt (- (quad-tree-point-x p-1) (quad-tree-point-x p-2)) 2)
     (expt (- (quad-tree-point-y p-1) (quad-tree-point-y p-2)) 2)))

(defun quad-tree-pt-distance (p-1 p-2)
  (sqrt (%quad-tree-pt-distance p-1 p-2)))

(defun quad-tree-range-top-left (range)
  (make-quad-tree-point :x (quad-tree-range-min-x range)
                        :y (quad-tree-range-min-y range)))

(defun quad-tree-range-bot-right (range)
  (make-quad-tree-point :x (quad-tree-range-max-x range)
                        :y (quad-tree-range-max-y range)))

(defun quad-tree-closest-point-in-range (tree point range)
  "Returns all points of the tree that are closer than `range' to
`point'. The euclidean distance is used to measure the distance
between points."
  (let ((closest-pt
         (etypecase tree
           (null nil)
           (quad-tree-node (minimum (quad-tree-node-points tree) :test #'%quad-tree-pt-distance))
           (quad-tree (let* ((index (quad-tree-index (quad-tree-range tree) point))
                             (child-node (aref (quad-tree-children tree) index)))
                        (cond (child-node
                               (quad-tree-closest-point-in-range child-node point range))
                              (t
                               (loop for c across (quad-tree-children tree)
                                     when (and c (or (< (quad-tree-pt-distance (quad-tree-range-top-left
                                                                                (etypecase c
                                                                                  (quad-tree-node (quad-tree-node-range c))
                                                                                  (quad-tree (quad-tree-range c))))
                                                                               point)
                                                        range)
                                                     (< (quad-tree-pt-distance (quad-tree-range-bot-right
                                                                                (etypecase c
                                                                                  (quad-tree-node (quad-tree-node-range c))
                                                                                  (quad-tree (quad-tree-range c))))
                                                                               point)
                                                        range)))
                                       appending (quad-tree-closest-point-in-range c point range))
                               )))))))
    (when (and closest-pt (< (quad-tree-pt-distance point closest-pt)))
      closest-pt)))

(defmethod print-object ((tree quad-tree) strm)
  (print-unreadable-object (tree strm :type t :identity t)))
