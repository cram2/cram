
(in-package :location-costmap)

(defclass height-map (occupancy-grid-metadata)
  ((height-map :reader height-map)))

(defmethod initialize-instance :after ((map height-map) &key)
  (with-slots (width height resolution height-map) map
    (setf height-map (cma:make-double-matrix
                      (round (/ width resolution))
                      (round (/ height resolution))))))

(defun height-map-lookup (map x y)
  (with-slots (resolution origin-x origin-y height-map width height) map
    (declare (type cma:double-matrix height-map))
    (if (or (< (- x origin-x) 0)
            (< (- y origin-y) 0)
            (> (- x origin-x) width)
            (> (- y origin-y) height))
        0
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
          value)))
