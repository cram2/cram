
(in-package :cma)

(deftype double-matrix () '(simple-array double-float 2))

(declaim (ftype (function (double-matrix) fixnum)
                width height))

(defun height (mat)
  (array-dimension mat 0))

(defun width (mat)
  (array-dimension mat 1))

(defun make-double-matrix (width height &key initial-contents (initial-element 0.0d0))
  (let ((result (make-array (list height width) :element-type 'double-float :initial-element initial-element)))
    (when initial-contents
      (fill-double-matrix result :initial-contents initial-contents))
    result))

(defun make-double-vector (size &key initial-contents)
  (make-double-matrix 1 size :initial-contents initial-contents))

(declaim (ftype (function (double-matrix) (array double-float 1)) flatten-double-matrix))
(defun flatten-double-matrix (mat)
  (make-array (array-total-size mat)
              :element-type 'double-float
              :displaced-to mat))  

(declaim (ftype (function (double-matrix) fixnum) double-vector-size))
(defun double-vector-size (vec)
  (assert (eql (width vec) 1) () "`vec' must be a vector, i.e. have width 1.")
  (height vec))

(defun fill-double-matrix (mat &key
                           (initial-element nil initial-element-p)
                           (initial-contents nil))
  (check-type mat double-matrix)
  (let ((flattened-mat (flatten-double-matrix mat)))
    (cond (initial-contents
           (map-into flattened-mat (rcurry #'coerce 'double-float) (flatten initial-contents)))
          (initial-element-p
           (map-into flattened-mat (constantly (coerce initial-element 'double-float))
                     flattened-mat))))
  mat)

(declaim (ftype (function (function &rest double-matrix) double-matrix) map-double-matrix))
(defun map-double-matrix (function &rest matrices)
  (apply #'map-double-matrix-into
         (make-double-matrix (width (car matrices)) (height (car matrices)))
         function matrices))

(define-compiler-macro map-double-matrix (function &rest matrices)
  `(map-double-matrix-into (make-double-matrix (width ,(car matrices)) (height ,(car matrices)))
                           ,function ,@matrices))

(declaim (ftype (function (double-matrix function &rest double-matrix) double-matrix)
                map-double-matrix-into))
(defun map-double-matrix-into (result function &rest matrices)
  (declare (type double-matrix result))
  (apply #'map-into (flatten-double-matrix result) function (mapcar #'flatten-double-matrix matrices))
  result)

(define-compiler-macro map-double-matrix-into (result function &rest matrices)
  `(dotimes (row (height ,result) ,result)
     (dotimes (col (width ,(car matrices)))
       (setf (aref (the double-matrix ,result) row col)
             (funcall ,function ,@(mapcar (lambda (m)
                                            `(aref (the double-matrix ,m) row col))
                                          matrices))))))

(declaim (ftype (function ((array * *) &optional double-matrix) double-matrix) double-matrix-from-array))
(defun double-matrix-from-array (arr &optional out)
  (let ((out (or out
                 (cond ((eql (array-rank arr) 1)
                        (make-double-vector (array-dimension arr 0)))
                       ((eql (array-rank arr) 2)
                        (make-double-matrix (array-dimension arr 1)
                                            (array-dimension arr 0)))
                       (t
                        (error 'simple-error :format-control "Array must have rank 1 or 2."))))))
    (map-into (flatten-double-matrix out) (rcurry #'coerce 'double-float)
              (make-array (array-total-size arr)
                          :element-type (array-element-type arr)
                          :displaced-to arr))
    out))

(defun double-matrix-from-grid (grid &optional
                                (out (make-double-matrix (grid:dim1 grid)
                                                         (grid:dim0 grid))))
  (fill-double-matrix out :initial-contents (loop for row below (grid:dim0 grid)
                                                  nconcing (loop for col below (grid:dim1 grid)
                                                                 collecting (grid:gref grid row col)))))

(defun grid-from-double-matrix (mat)
  (grid:make-foreign-array 'double-float :dimensions (array-dimensions mat)
                           :initial-contents (loop for row below (height mat)
                                                   collecting
                                                (loop for col below (width mat)
                                                      collecting (aref mat row col)))))

(declaim (ftype (function (double-matrix &optional double-matrix) double-matrix)
                double-matrix-transpose))
(defun double-matrix-transpose (mat &optional
                                (out (make-double-matrix (height mat) (width mat))))
  (declare (optimize (speed 3)))
  (declare (type double-matrix mat out))
  (let ((width (width mat))
        (height (height mat)))
    (declare (fixnum width height))
    (dotimes (row height)
      (declare (sb-int:truly-dynamic-extent row))
      (dotimes (col width)
        (declare (sb-int:truly-dynamic-extent col))
        (setf (aref out col row)
              (aref mat row col)))))
  out)

(declaim (ftype (function (double-matrix double-matrix &optional double-matrix) double-matrix)
                double-matrix-product))
(defun double-matrix-product (m-1 m-2 &optional
                              (out (make-double-matrix (width m-2) (height m-1))))
  (declare (optimize (speed 3)))
  (declare (type double-matrix m-1 m-2 out))
  (let ((m-1-height (height m-1))
        (m-1-width (width m-1))
        (m-2-width (width m-2)))
    (declare (type fixnum m-1-height m-1-width m-2-width))
    (dotimes (row m-1-height)
      (dotimes (col m-2-width)
        (setf (aref out row col) 0.0d0)
        (dotimes (i m-1-width)
          (setf (aref out row col)
                (+ (aref out row col) (* (aref m-1 row i)
                                         (aref m-2 i col))))))))
  out)

(defmacro %wrap-matrix-elem-un-op (name op)
  `(progn
     (declaim (ftype (function (double-matrix double-float &optional double-matrix) double-matrix)
                     ,name))
     (defun ,name (m s &optional (out (make-double-matrix (width m) (height m))))
       (declare (type double-matrix m out)
                (type double-float s))
       (flet ((doit (e)
                (declare (optimize (speed 3)))
                (funcall ,op e s)))
         (map-double-matrix-into out #'doit m)))))

(defmacro %wrap-matrix-elem-bin-op (name op)
  `(progn
     (declaim (ftype (function (double-matrix double-matrix &optional double-matrix) double-matrix)
                     ,name))     
     (defun ,name (m-1 m-2 &optional (out (make-double-matrix (width m-1) (height m-2))))
       (declare (type double-matrix m-1 m-2 out))
       (declare (optimize (safety 0) (speed 3)))
       (dotimes (y (height m-1) out)
         (dotimes (x (width m-1) out)
           (setf (aref out y x)
                 (,op (aref m-1 y x) (aref m-2 y x))))))))

(declaim (inline m+ m- m* m/ m.+ m.- m.* m./))

(%wrap-matrix-elem-un-op m.+ #'+)
(%wrap-matrix-elem-un-op m.- #'-)
(%wrap-matrix-elem-un-op m.* #'*)
(%wrap-matrix-elem-un-op m./ #'/)

(%wrap-matrix-elem-bin-op m+ +)
(%wrap-matrix-elem-bin-op m- -)
(%wrap-matrix-elem-bin-op m* *)
(%wrap-matrix-elem-bin-op m/ /)

