
(in-package :location-costmap)

(defgeneric height-map-lookup (map x y)
  (:documentation "Returns the z value of a 2D-point in the height
  map."))

(defclass height-map (occupancy-grid-metadata)
  ((height-map :reader height-map)))

(defclass lazy-height-map (height-map)
  ((generator :initarg :generator-fun
              :initform (error
                         'simple-error
                         :format-control "No generator function specified")
              :reader generator))
  (:default-initargs :initial-element -1.0d0))

(defmethod initialize-instance :after ((map height-map) &key initial-element)
  (with-slots (width height resolution height-map) map
    (setf height-map (cma:make-double-matrix
                      (round (/ width resolution))
                      (round (/ height resolution))))
    (when initial-element
      (cma:fill-double-matrix height-map :initial-element initial-element))))

(defmethod height-map-lookup ((map height-map) x y)
  (with-slots (resolution origin-x origin-y height-map width height) map
    (declare (type cma:double-matrix height-map))
    (if (or (< (- x origin-x) 0)
            (< (- y origin-y) 0)
            (> (- x origin-x) width)
            (> (- y origin-y) height))
        0.0d0
        (aref height-map
              (round (/ (- y origin-y) resolution))
              (round (/ (- x origin-x) resolution))))))

(declaim (inline height-map-set))
(defun height-map-set (map x y value)
  (with-slots (resolution origin-x origin-y height-map) map
    (declare (type cma:double-matrix height-map))
    (setf (aref height-map
                (round (/ (- y origin-y) resolution))
                (round (/ (- x origin-x) resolution)))
          (float value 0.0d0))))

(defmethod height-map-lookup ((map lazy-height-map) x y)
  (let ((cached-value (call-next-method)))
    (if (< cached-value 0.0d0)
        (height-map-set map x y (funcall (generator map) x y))
        cached-value)))
