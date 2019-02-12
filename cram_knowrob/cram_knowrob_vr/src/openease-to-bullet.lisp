;;; This file contains all functions needed to adjust the poses and transforms
;;; from the OpenEase semantic map to the bullet world semantic map,
;;; since the kitches are slightly different and therefore the poses will be off
;;; without the usage of these functions. Also there are variations within the
;;; data representation of OpenEase and Bulelt World, these functions also take
;;; care of that. 
(in-package :kvr)

(defun apply-bullet-transform (transform)
  "Applies the transform of the VR to bullet world and the rotation."
  (cl-tf:transform*
   (cl-tf:make-transform (cl-tf:make-3d-vector -2.65 -0.7 0.0)
                         (cl-tf:axis-angle->quaternion
                          (cl-tf:make-3d-vector 0 0 1)
                          pi)) 
   transform))

(defun apply-bullet-rotation (transform)
  "Applies only the offset rotation between the VR and the bullet world."
  (cl-tf:transform*
   (cl-tf:make-transform (cl-tf:make-3d-vector 0.0 0.0 0.0)
                         (cl-tf:axis-angle->quaternion
                          (cl-tf:make-3d-vector 0 0 1)
                          pi)) 
   transform))

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
      (cl-tf:w quaternion)
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
  (let ((alpha  0)) ; (/ pi 4)               
      (cl-tf:make-transform
       (cl-tf:make-3d-vector 0.0 -0.07 0.2)
       (cl-tf:matrix->quaternion 
        (make-array '(3 3)
                    :initial-contents
                    `((0                1 0)
                      (,(- (cos alpha)) 0 ,(- (sin alpha)))
                      (,(- (sin alpha)) 0 ,(cos alpha))))))))


