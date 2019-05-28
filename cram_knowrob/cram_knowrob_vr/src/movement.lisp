(in-package :kvr)

#+this-is-also-not-used-anymore
(
 (defun move-head (pose)
   "Moves the head of the robot to the given POSE."
   (prolog:prolog `(and (btr:bullet-world ?world)
                        (cram-robot-interfaces:robot ?robot )
                        (btr:head-pointing-at ?world ?robot ,pose))))

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

     (setq pose (cl-transforms:make-pose
                 (cl-transforms:origin pose)
                 quaternion))

     (prolog:prolog `(and (btr:bullet-world ?world)
                          (assert (btr:object-pose ?world cram-pr2-description:pr2 ,pose))))))

 (defun move-toso-up (?angle)
   "Moves up the torso of the robot by the given angle. The robot should do this automatically during picking and placing actions, but for debugging purposes this can be useful.
?ANGLE: Angle, by which the robot should move up his torso."
   (proj:with-projection-environment urdf-proj:urdf-bullet-projection-environment
     (cpl:top-level
       (exe:perform
        (desig:a motion (type moving-torso) (joint-angle ?angle))))))
 )
