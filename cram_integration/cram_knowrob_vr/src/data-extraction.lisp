;;; Contains all the functions necessary to extract data from OpenEase into CRAM.
;;; NOTE: Just the data extraction is here. Its manipulation and adjustments are in the openase-to-bullet.lisp file
(in-package :kvr)



(defun make-poses (name &optional (poses-list *poses-list*))
  "Calls make-pose to convert the list of values from poses-list to a
cl-tf:transform, and then applies all the adjustments needed to convert
the pose from the OpenEase map into a proper pose for the bullet world.
NAME: The name of an item in the *poses-list* of which the pose is needed.
The time stamp is included into the name, meaning possible parameters could be: 
OPTIONAL POSES-LIST: A different poses-list can be given if needed. Otherwise
*poses-list* will be used as default.
RETURNS: A cl-tf:transform of the given object at the given timestamp, With all 
the adjustments for the differences between the OpenEase and the bullet world
already beeing made."
  (apply-bullet-transform
   (quaternion-w-flip
     (make-pose (cut:var-value (intern name) poses-list)))))

;; TODO edit comment
(defun make-pose (pose)
  "Makes a proper cl-tf:transform with a 3d-vector for translation and a
quaternion for rotation.
POSE: A list of position and rotation values all in one list, as returned by OpenEase.
RETURNS: A cl-tf:transform consisting of a 3d-vector and a quaternion."
  (apply-bullet-transform
   (quaternion-w-flip
    (cl-tf:make-transform
     (apply #'cl-tf:make-3d-vector (subseq pose 0 3))
     (apply #'cl-tf:make-quaternion (subseq pose 3 7))))))
