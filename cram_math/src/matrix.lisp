
(in-package :cma)

(defclass matrix ()
  ((width :initarg :width :reader width)
   (height :initarg :height :reader height)
   (content :reader content)))

(defmethod initialize-instance :after ((mat matrix) &key initial-contents)
  (with-slots (width height content) mat
    (setf content (if initial-contents
                      (make-array (* width height)
                                  :initial-contents (flatten initial-contents))
                      (make-array (* width height))))))

(defun make-matrix (width height &optional initial-contents)
  (make-instance 'matrix :width width :height height :initial-contents initial-contents))

(defun make-vector (size &optional initial-contents)
  (make-instance 'matrix :width 1 :height size :initial-contents initial-contents))

(defun vector-size (vec)
  (assert (eql (width vec) 1) () "`vec' must be a vector, i.e. have width 1.")
  (height vec))

(defun fill-matrix (mat &key
                    (initial-element nil initial-element-p)
                    (initial-contents nil))
  (cond (initial-contents
         (map-into (content mat) #'identity (flatten initial-contents)))
        (initial-element-p
         (dotimes (i (* (width mat) (height mat)))
           (setf (svref (content mat) i) initial-element))))
  mat)

(defun matrix-from-array (arr &optional out)
  (let ((out (or out
                 (cond ((eql (array-rank arr) 1)
                        (make-vector (array-dimension arr 0)))
                       ((eql (array-rank arr) 2)
                        (make-matrix (array-dimension arr 1)
                                     (array-dimension arr 0)))
                       (t
                        (error 'simple-error :format-control "Array must have rank 1 or 2."))))))
    (map-into (content out) #'identity (make-array (array-total-size arr) :displaced-to arr))
    out))

(defun matrix-from-grid (grid &optional
                         (out (make-matrix (grid:dim1 grid)
                                           (grid:dim0 grid))))
  (fill-matrix out :initial-contents (loop for row below (grid:dim0 grid)
                                           nconcing (loop for col below (grid:dim1 grid)
                                                          collecting (grid:gref grid row col)))))

(defun grid-from-matrix (mat &optional type)
  (let ((type (or type (type-of (svref (content mat) 0)))))
    (grid:make-foreign-array type :dimensions `(,(height mat) ,(width mat))
                             :initial-contents (loop for row below (height mat)
                                                     collecting
                                                  (loop for col below (width mat)
                                                        collecting (coerce (mref mat row col) type))))))

(declaim (inline mref (setf mref)))
(defun mref (mat row col)
  (declare (type matrix mat)
           (type fixnum row col)
           (optimize (speed 3)))
  (with-slots (width height content) mat
    (declare (type fixnum width))
    (svref content (+ (the fixnum (* width row)) col))))

(defun (setf mref) (new-value mat row col)
  (declare (type matrix mat)
           (type fixnum row col)
           (optimize (speed 3)))
  (with-slots (width content) mat
    (declare (type fixnum width))    
    (setf (svref content (+ (the fixnum (* width row)) col)) new-value)))

(defun map-matrix (function &rest matrices)
  (let ((out-matrix (make-matrix (width (car matrices)) (height (car matrices)))))
    (apply #'map-matrix-into out-matrix function matrices)))

(defun map-matrix-into (result function &rest matrices)
  (apply #'map-into (content result) function (mapcar #'content matrices))
  result)

(defun matrix-transpose (mat &optional
                         (out (make-matrix (height mat) (width mat))))
  (declare (optimize (speed 3)))
  (with-slots (width height) mat
    (declare (fixnum width height))
    (dotimes (row height)
      (declare (sb-int:truly-dynamic-extent row))
      (dotimes (col width)
        (declare (sb-int:truly-dynamic-extent col))
        (setf (mref out col row)
              (mref mat row col)))))
  out)

(defun matrix-product (m-1 m-2 &optional
                       (out (make-matrix (width m-2) (height m-1))))
  (dotimes (row (height m-1))
    (dotimes (col (width m-2))
      (setf (mref out row col) 0)
      (dotimes (i (width m-1))
        (setf (mref out row col)
              (+ (mref out row col) (* (mref m-1 row i)
                                       (mref m-2 i col)))))))
  out)

(defmethod print-object ((mat matrix) strm)
  (print-unreadable-object (mat strm :type t :identity t)
    (princ (make-array `(,(height mat) ,(width mat)) :displaced-to (content mat)) strm)))
