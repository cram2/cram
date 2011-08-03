
(in-package :location-costmap)

(defgeneric 2d-value-map-lookup (map x y)
  (:documentation "Returns the z value of a 2D-point in the height
  map."))

(defclass 2d-value-map (occupancy-grid-metadata)
  ((2d-value-map :reader 2d-value-map)))

(defclass lazy-2d-value-map (2d-value-map)
  ((generator :initarg :generator-fun
              :initform (error
                         'simple-error
                         :format-control "No generator function specified")
              :reader generator))
  (:default-initargs :initial-element -1.0d0))

(defmethod initialize-instance :after ((map 2d-value-map) &key initial-element)
  (with-slots (width height resolution 2d-value-map) map
    (setf 2d-value-map (cma:make-double-matrix
                      (round (/ width resolution))
                      (round (/ height resolution))))
    (when initial-element
      (cma:fill-double-matrix 2d-value-map :initial-element initial-element))))

(defmethod 2d-value-map-lookup ((map 2d-value-map) x y)
  (with-slots (resolution origin-x origin-y 2d-value-map width height) map
    (declare (type cma:double-matrix 2d-value-map))
    (if (or (< (- x origin-x) 0)
            (< (- y origin-y) 0)
            (> (- x origin-x) width)
            (> (- y origin-y) height))
        0.0d0
        (aref 2d-value-map
              (round (/ (- y origin-y) resolution))
              (round (/ (- x origin-x) resolution))))))

(declaim (inline 2d-value-map-set))
(defun 2d-value-map-set (map x y value)
  (with-slots (resolution origin-x origin-y 2d-value-map) map
    (declare (type cma:double-matrix 2d-value-map))
    (setf (aref 2d-value-map
                (round (/ (- y origin-y) resolution))
                (round (/ (- x origin-x) resolution)))
          (float value 0.0d0))))

(defmethod 2d-value-map-lookup ((map lazy-2d-value-map) x y)
  (let ((cached-value (call-next-method)))
    (if (< cached-value 0.0d0)
        (2d-value-map-set map x y (funcall (generator map) x y))
        cached-value)))
