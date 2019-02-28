;;; This file contains all functions needed to adjust the poses and transforms
;;; from the OpenEase semantic map to the bullet world semantic map,
;;; since the kitches are slightly different and therefore the poses will be off
;;; without the usage of these functions. Also there are variations within the
;;; data representation of OpenEase and Bulelt World, these functions also take
;;; care of that.
(in-package :kvr)

#+not-used-anymore
(
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
       (cl-tf:x quaternion))))))
