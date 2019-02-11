(in-package :kvr)

(defun move-obj-with-offset (x-offset y-offset object-knowrob object-bullet)
  "Moves an object to a given x y offset from it's starting position. "
  (move-object 
   (cl-tf:make-transform 
    (cl-tf:make-3d-vector
     (+
      x-offset
      (cl-tf:x
       (cl-tf:translation
        (get-object-location-at-start-by-object-type object-knowrob))))
     (+
      y-offset
      (cl-tf:y
       (cl-tf:translation
        (get-object-location-at-start-by-object-type object-knowrob))))
     
     (cl-tf:z
      (cl-tf:translation 
       (get-object-location-at-start-by-object-type object-knowrob))))
    (cl-tf:rotation
     (get-object-location-at-start-by-object-type object-knowrob)))
   object-bullet))


(defun move-head (pose)
  "Moves the head of the robot to the given POSE."
  (prolog:prolog `(and (btr:bullet-world ?world)
                       (cram-robot-interfaces:robot ?robot )
                       (btr:head-pointing-at ?world ?robot ,pose))))

; moves the given object to the given pose.
; usage example: (move-object (pose-lists-parser '|?PoseObjEnd|))
(defun move-object (transform obj)
  "Moves an object to the desired transform.
TRANSFORM: The transform describing the position to which an object should be moved.
OBJ: The object which is supposed to be moved."
  (let* ((pose (cl-tf:transform->pose transform)))
    (prolog:prolog `(and (btr:bullet-world ?world)
                         (assert (btr:object-pose ?world ,obj ,pose))))))


;;(make-poses "?PoseCameraStart")
(defun move-robot (transform)
  "Moves the robot to a given position which is described by the given transform."
  ;; make the transform a viable robot position
  (let* ((pose (cl-tf:transform->pose  (remove-z transform )))
         (quaternion (cl-tf:orientation pose))
         (x nil)
         (y nil))
    ;; make quaternion
    ;; make into matrix, get x and y values
    (setq x (aref (cl-tf:quaternion->matrix quaternion) 0 2))
    (setq y (aref (cl-tf:quaternion->matrix quaternion) 1 2))
    (setq quaternion (cl-tf:axis-angle->quaternion (cl-tf:make-3d-vector 0 0 1) (atan y x)))
    
    (setq pose (cl-tf:make-pose
                (cl-tf:origin pose)
                quaternion))
    
    (prolog:prolog `(and (btr:bullet-world ?world)
                         (assert (btr:object-pose ?world cram-pr2-description:pr2 ,pose))))))

(defun move-toso-up (?angle)
  "Moves up the torso of the robot by the given angle. The robot should do this automatically during picking and placing actions, but for debugging purposes this can be useful. 
?ANGLE: Angle, by which the robot should move up his torso."
   (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
           (cpl:top-level
             (exe:perform
              (desig:a motion (type moving-torso) (joint-angle ?angle))))))
