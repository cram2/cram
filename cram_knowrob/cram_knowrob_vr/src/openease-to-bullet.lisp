;;; This file contains all functions needed to adjust the poses and transforms
;;; from the OpenEase semantic map to the bullet world semantic map,
;;; since the kitches are slightly different and therefore the poses will be off
;;; without the usage of these functions. Also there are variations within the
;;; data representation of OpenEase and Bulelt World, these functions also take
;;; care of that. 
(in-package :kvr)


(defun quaternion-w-flip (pose)
  "Flips the quaternion from OpenEase wxyz notation to the ros xyzw notation.
POSE: is the pose, from which the quaternion will be taken and flipped.
RETURNS: the given pose with the flipped quaternion."
  (let* ((quaternion (cl-tf:rotation pose)))
    (cl-tf:make-transform
     (cl-tf:translation pose)
     (cl-tf:make-quaternion
      (cl-tf:y quaternion)
      (cl-tf:z quaternion)
      (cl-tf:w  quaternion)
      (cl-tf:x quaternion)))))


(defun remove-z (pose)
  "Remove z coordinate in order to move the robot to the x y  coordinates within the map.
POSE: the given pose from which the z coordinate of the translation will be deleted.
RETURNS: a new pose with the z coordinate within the translation  set to 0.0"
  (let* ((translation (cl-tf:translation pose)))
    (cl-tf:make-transform
     (cl-tf:make-3d-vector (cl-tf:x translation) (cl-tf:y translation) 0)
     (cl-tf:rotation pose))))


(defun human-to-robot-hand-transform ()
  "Defines the offset between the human hand from the virtual reality to the
robot standart gripper, which has been calculated manually.
RETURNS: a cl-transform."
  (let ((alpha  0; (/ pi 4)
                ))
      (cl-tf:make-transform
       (cl-tf:make-3d-vector 0.0 -0.07 0.2)
       (cl-tf:matrix->quaternion 
        (make-array '(3 3)
                    :initial-contents
                    `((0                1 0)
                      (,(- (cos alpha)) 0 ,(- (sin alpha)))
                      (,(- (sin alpha)) 0 ,(cos alpha))))))))



(defun place-pose-btr-island ()
  "Calculates the placing pose for an object, relative to the bullet world
kitchen island. This is needed, since the OpenEase kitchen island and the
bullet world kitchen island, are slightly offset to one another, and the offset
fixing the general semantic map offset, is not enough to fix it.
Therefore relative poses are being calculated.
RETURNS: A cl-tf:transform representing the pose and orientation, at which the
robot in the bullet world should place the object currently in hand."
  (let* (table-pose-oe
         table-pose-bullet
         place-pose)
    ; get pose of Table in map frame
    (setq table-pose-oe
          (make-poses "?PoseTable"
                      (cut:lazy-car
                       (prolog-simple "ep_inst(EpInst),
                                       u_occurs(EpInst, EventInst, Start, End),
                                       obj_type(TableInst, knowrob:'IslandArea'),
                                       iri_xml_namespace(TableInst, _, TableShortName),
                                       actor_pose(EpInst, TableShortName, Start, PoseTable)."))))
    ; get pose of table in bullet world
    (setq table-pose-bullet
          (cl-tf:pose->transform
           (btr:pose
            (gethash '|iai_kitchen_kitchen_island|
                     (slot-value
                      (btr:object btr:*current-bullet-world* :kitchen)
                      'cram-bullet-reasoning:rigid-bodies)))))
    
    ; calculate place pose relative to bullet table
    (setq place-pose
          (cl-tf:transform*
           table-pose-bullet
           (cl-tf:transform-inv table-pose-oe)
           (make-poses "?PoseObjEnd")))
    place-pose))
    
