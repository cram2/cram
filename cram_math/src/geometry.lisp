
(in-package :cma)

(defclass 2d-point ()
  ((x :reader x :initarg :x)
   (y :reader y :initarg :y)))

(defclass polygon ()
  ((vertices :initarg :vertices
             :reader vertices))
  (:documentation "Stores the vertices of a polygon."))

(defgeneric point-in-polygon (point polygon)
  (:documentation "Returns T if `point' is inside `polygon'"))

(defmethod initialize-instance :after ((poly polygon) &key
                                       (vertices (error 'simple-error :format-control "No vertices specified.")))
  (etypecase vertices
   (list (setf (slot-value poly 'vertices) (make-array (length vertices) :initial-contents vertices)))
   (array (setf (slot-value poly 'vertices) vertices))))

(defmethod point-in-polygon ((point 2d-point) (poly polygon))
  (with-slots (x y) point
    (with-slots (vertices) poly
      (loop for i below (array-dimension vertices 0)
            with j = (1- (array-dimension vertices 0))
            with c = nil
            when (and
                   (not (eq (> (y (aref vertices i)) y)
                            (> (y (aref vertices j)) y)))
                   (< x (+ (/ (* (- (x (aref vertices i)) (x (aref vertices j)))
                                 (- x (y (aref vertices i))))
                              (- (y (aref vertices j)) (y (aref vertices i))))
                           (x (aref vertices i)))))
              do (setf c (not c))
            finally (return c)))))


(defun degrees->radians (degrees)
  (* PI (/ degrees 180)))

(defun radians->degrees (radians)
  (* 180 (/ radians PI)))
