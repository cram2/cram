;;; Contains all the functions necessary to extract data from OpenEase into CRAM.
;;; NOTE: Just the data extraction is here. Its manipulation and adjustments are in the openase-to-bullet.lisp file
(in-package :kvr)

;; TODO edit comment. maybe move to utilities?
(defun make-pose (pose)
  "Makes a proper cl-tf:transform with a 3d-vector for translation and a
quaternion for rotation.
POSE: A list of position and rotation values all in one list, as returned by OpenEase.
RETURNS: A cl-tf:transform consisting of a 3d-vector and a quaternion."
  (apply-bullet-transform
  ; (quaternion-w-flip ;??? deprecated
    (cl-tf:make-transform
     (apply #'cl-tf:make-3d-vector (subseq pose 0 3))
     (apply #'cl-tf:make-quaternion (subseq pose 3 7)))));)
