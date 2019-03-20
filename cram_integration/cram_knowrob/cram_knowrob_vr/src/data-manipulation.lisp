;;; Contains all the functions necessary to extract data from OpenEase into CRAM.
(in-package :kvr)

#+not-used-anymore
(
 (defun make-pose (pose)
   "Makes a proper cl-tf:transform with a 3d-vector for translation and a
quaternion for rotation.
POSE: A list of position and rotation values all in one list, as returned by OpenEase.
RETURNS: A cl-tf:transform consisting of a 3d-vector and a quaternion."
   ;; (apply-bullet-transform
   ;; (quaternion-w-flip ;NOTE Not needed when using Mongo, "might" be needed for OpenEase, depending on version
   (cl-tf:make-transform
    (apply #'cl-tf:make-3d-vector (subseq pose 0 3))
    (apply #'cl-tf:make-quaternion (subseq pose 3 7))))
;;))


 (defun apply-rotation (transform)
   (cl-tf:transform*
    (cl-tf:make-transform (cl-tf:make-3d-vector 0.0 0.0 0.0)
                          (cl-tf:axis-angle->quaternion
                           (cl-tf:make-3d-vector 0 0 1)
                           (/ pi 2)))
    transform))

 (defun add-pos-offset-to-transform (transform x y z)
   "Adds a given offset to a given transform."
   (cl-tf:make-transform
    (cl-tf:make-3d-vector
     (+ x (cl-tf:x (cl-tf:translation transform)))
     (+ y (cl-tf:y (cl-tf:translation transform)))
     (+ z (cl-tf:z (cl-tf:translation transform))))
    (cl-tf:rotation transform)))
 )
