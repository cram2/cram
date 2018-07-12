(in-package :kvr)

;;; a human feet offset. Since the human feet are a lot smaller then the robots
;;; base, this offset can be added in order to prevent the robot from crashing
;;; into the bases of tables.
;;; in short: moves the robot away from the given pose to avoid collisions.
(defparameter *human-feet-offset* 0.0 ) ;; was 0.3 for Andrei's Data


;; example: (make-poses "?PoseCameraStart")
(defun set-grasp-base-pose (transform)
  "Calculates the transform of where the robot should stand in order to interact
with an object. Based on data from Virtual Reality. This function removes the z
component of the Camera pose of the human and also fixes the quaternion so that
the robot won't tilt into the pane of the floor.
TRANSFORM: The transform of the Camera (head position of the human)  which is
given by the Virtual Reality.
RETURNS: A pose stamped, with removed z components and fixed rotations so that
the robot can be placed at this position."
  
   ; make the transform a viable robot position
  (let* ((pose (cl-tf:transform->pose  (remove-z transform)))
         (quaternion (cl-tf:orientation pose))
         (x nil)
         (y nil)
         (?grasp-base-pose nil))
    ; make quaternion
    ; make into matrix, get x and y values
    (setq x (aref (cl-tf:quaternion->matrix quaternion) 0 2))
    (setq y (aref (cl-tf:quaternion->matrix quaternion) 1 2))
    (setq quaternion
          (cl-tf:axis-angle->quaternion
           (cl-tf:make-3d-vector 0 0 1)
           (atan y x)))
    
    (setq ?grasp-base-pose 
          (cl-transforms-stamped:make-pose-stamped
           "map"
           0.0
           (cl-tf:make-3d-vector
            (if (plusp (cl-tf:x (cl-tf:origin pose)))
                (- (cl-tf:x (cl-tf:origin pose)) *human-feet-offset*)
                (+ (cl-tf:x (cl-tf:origin pose)) *human-feet-offset*))
            (cl-tf:y (cl-tf:origin pose))
            0)
           quaternion))))

;;(make-poses "?PoseObjStart")
(defun set-grasp-look-pose (transform)
  "Transforms the given transform of the position of an object, into a pose
stamped, at which the robot will look for an object.
TRANSFORM: The given transform of the objects assumed/derrived location.
RETURNS: A pose stamped of where the robot should look for an object."
  (let* ((?grasp-look-pose nil))
    (setq ?grasp-look-pose
          (cl-transforms-stamped:make-pose-stamped
           "map"
           0.0
           (cl-tf:origin (cl-tf:transform->pose transform))
           (cl-tf:orientation (cl-tf:transform->pose transform))))))

;; (make-poses "?PoseObjEnd")
(defun set-place-pose (transform)
  "Takes the given transform of the placing the object pose, sets the
quaternion to an identity one and transforms the transform into a pose stamped.
TRANSFORM: The given transform at which the human placed an object.
RETURNS: A pose stamped, with an identity quaternion."
  (let* ((?grasp-look-pose nil))
    (setq ?grasp-look-pose
          (cl-transforms-stamped:make-pose-stamped
           "map"
           0.0
           (cl-tf:origin (cl-tf:transform->pose transform))
           (cl-tf:make-identity-rotation)))))
