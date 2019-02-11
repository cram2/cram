(in-package :kvr)

(defun execute-pick-and-place (type)
  "Executes the pick and place plan on an object of the given type.
The positions of where the robot looks for the object and where he is placing
it down are the ones extracted from Virtual Reality.
TYPE: the given type of an object on which the pick and place action should be
executed.
RETURNS: "
  (pick-and-place
   (set-grasp-base-pose
    (umap-T-laser-T-human type))
 
   (set-grasp-look-pose
     (umap-T-robot type))
   
   (set-grasp-base-pose 
    (umap-T-laser-T-place type :end))
   
   (set-grasp-look-pose
    (place-pose type))
   
   (set-grasp-look-pose 
    (place-pose type))
   type))

(defun execute-pick-up-object (type)
  "Executes only the picking up action on an object given the type of the object.
TYPE: The type of the object. Could be 'muesli or 'cup etc. The name is internally
set to CupEcoOrange in a string.
RETURNS:"
  (pick-up-object (set-grasp-base-pose
                   (umap-T-laser-T-human type))                 
                  
                  (set-grasp-look-pose
                   (umap-T-robot type))
                  
                  type))

(defun execute-place-object (?obj-desig type)
  "Executes the placing action given the object designator of the picked up and
held in hand object. The placing pose is the one used in VR for that kind of 
object.
?OBJ-DESIG: The object designator of the object the robot is currently holding
and which should be placed down."
  (place-object
   (set-grasp-base-pose 
    (umap-T-laser-T-place type :end))

   (set-grasp-look-pose
    (place-pose type))
   
   (set-grasp-look-pose
    (place-pose type))

   (cram-projection::projection-environment-result-result ?obj-desig)))

(defun execute-move-to-object (type &optional (event-time :start))
  "Moves the robot to the position where the human was standing in order to
grasp the object or in order to place it."
  
   (if (eq event-time :start)
       (progn
         (move-to-object
          (set-grasp-base-pose
           (umap-T-human type))
          (set-grasp-look-pose
           (umap-T-robot type)))))
  
  (if (eq event-time :end)
      (progn
        (move-to-object
         (set-grasp-base-pose
          (ucamera-T-usurface type :end))
         (set-grasp-look-pose
          (place-pose type))))))
