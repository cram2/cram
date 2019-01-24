;;; Mapping from the object of the urdf kitchen to the semantic map.
;;; The semantic map kitchen is the one in OpenEase where the data is from.
;;; The URDF kitchen is the one the robot interacts with in the bullet world.
;;; The semantic map has a lot more objects then the URDF since all the cuttlery
;;; the robot interacts with is not present in the semantic map but spawned
;;; additionally.
(in-package :kvr)

(defun get-all-objects-urdf()
  (btr:rigid-body-names
   (btr:object btr:*current-bullet-world* :kitchen)))

(defun get-all-objects-semantic()
  (btr:rigid-body-names
   (btr:object btr:*current-bullet-world* :semantic-map-kitchen)))

;;; semantic map . urdf
(defparameter  *semantic-to-urdf*
  '((|''IslandArea_nhwy''| . :|KITCHEN.kitchen_island|)
    (|''SinkArea_pY2k''| . :|KITCHEN.sink_area|)))

(defun match-kitchens(name)
  (cdr (assoc name *semantic-to-urdf*)))


