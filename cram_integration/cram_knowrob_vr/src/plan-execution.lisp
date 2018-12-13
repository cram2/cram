(in-package :kvr)

(defun execute-pick-and-place (type)
  "Executes the pick and place plan on an object of the given type.
The positions of where the robot looks for the object and where he is placing
it down are the ones extracted from Virtual Reality.
TYPE: the given type of an object on which the pick and place action should be
executed.
RETURNS: "
  (pick-and-place  (set-grasp-base-pose
                    (get-camera-location-at-start-by-object-type
                     (object-type-filter-prolog type)))
                   (set-grasp-look-pose
                    (get-object-location-at-start-by-object-type
                     (object-type-filter-prolog type)))
                   (set-grasp-base-pose
                    (get-camera-location-at-end-by-object-type
                     (object-type-filter-prolog type)))
                   (set-grasp-look-pose
                    (place-pose-btr-island
                     (object-type-filter-prolog type)))
                   (set-grasp-look-pose
                    (place-pose-btr-island
                     (object-type-filter-prolog type)))
  type))

(defun execute-pick-up-object (type)
  "Executes only the picking up action on an object given the type of the object.
TYPE: The type of the object. Could be :koelln-muesli-knusper-honig-nuss for
the cereal box.
RETURNS:"
  (pick-up-object (set-grasp-base-pose
                   (get-camera-location-at-start-by-object-type
                    (object-type-filter-prolog type)))                 

                  (set-grasp-look-pose
                   (get-object-location-at-start-by-object-type
                    (object-type-filter-prolog type)))
                  
                  type))

(defun execute-place-object (?obj-desig type)
  "Executes the placing action given the object designator of the picked up and
held in hand object. The placing pose is the one used in VR for that kind of 
object.
?OBJ-DESIG: The object designator of the object the robot is currently holding
and which should be placed down."
  (place-object (set-grasp-base-pose
                 (get-camera-location-at-end-by-object-type type))
                (set-grasp-look-pose (place-pose-btr-island))
                (set-grasp-look-pose (place-pose-btr-island))
                (cram-projection::projection-environment-result-result ?obj-desig)))

(defun execute-move-to-object (type)
  "Moves the robot to the position where the human was standing in order to
grasp the object."
  (move-to-object (set-grasp-base-pose
                   (get-camera-location-at-start-by-object-type
                    (object-type-filter-prolog type)))
                  (set-grasp-look-pose
                   (get-object-location-at-start-by-object-type
                    (object-type-filter-prolog type)))))
