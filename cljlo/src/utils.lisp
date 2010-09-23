
(in-package :jlo)

(defgeneric identity-jlo (&key name parent)
  (:documentation "Creates an identity transformation"))

(defgeneric make-jlo-rpy (&key name parent x y z roll pitch yaw)
  (:documentation "Creates a jlo object from x y z and roll pitch yaw."))

(defgeneric inlier? (reference obj &optional threshold)
  (:documentation "Checks if the pose obj lies in the pose covariance
                   ellipsoid of reference."))

(defgeneric euclidean-distance (jlo-1 jlo-2)
  (:documentation "Calculates the euclidean distance between two lo objects"))

(defgeneric component-distance (jlo-1 jlo-2 &key reference axis)
  (:documentation "Returns the destance with respect to just one
                   component, in coordinate frame `reference'. `axis'
                   is one of :X, :Y or :Z.
                   The calculation is jlo-1 - jlo-2"))

(defmethod identity-jlo (&key name (parent (make-jlo :name "/map")))
  (make-jlo :parent parent :name name))

(defmethod make-jlo-rpy (&key name
                         (parent (make-jlo :name "/map"))
                         (x 0) (y 0) (z 0)
                         (roll 0) (pitch 0) (yaw 0))
  (let* ((cos-roll (cos roll))
         (sin-roll (sin roll))
         (cos-pitch (cos pitch))
         (sin-pitch (sin pitch))
         (cos-yaw (cos yaw))
         (sin-yaw (sin yaw))
         (pose (make-array 16 :initial-contents
                                 `(,(* cos-yaw cos-pitch)
                                    ,(- (* cos-yaw sin-pitch sin-roll) (* sin-yaw cos-roll))
                                    ,(+ (* cos-yaw sin-pitch cos-roll) (* sin-yaw sin-roll))
                                    ,x

                                    ,(* sin-yaw cos-pitch)
                                    ,(+ (* sin-yaw sin-pitch sin-roll) (* cos-yaw cos-roll))
                                    ,(- (* sin-yaw sin-pitch cos-roll) (* cos-yaw sin-roll))
                                    ,y

                                    ,(- sin-pitch)
                                    ,(* cos-pitch sin-roll)
                                    ,(* cos-pitch cos-roll)
                                    ,z
                                    0 0 0 1))))
    (make-jlo :name name :parent parent :pose pose)))

(defmethod inlier? ((reference jlo) (obj jlo) &optional (threshold 0.05))
  (and (< (abs (- (cov reference 0 0) (pose obj 0 3))) threshold)
       (< (abs (- (cov reference 1 1) (pose obj 1 3))) threshold)
       (< (abs (- (cov reference 2 2) (pose obj 2 3))) threshold)))

(defmethod euclidean-distance ((jlo-1 jlo) (jlo-2 jlo))
  (let ((distance-lo (frame-query jlo-1 jlo-2)))
    (sqrt (+ (expt (pose distance-lo 0 3) 2)
             (expt (pose distance-lo 1 3) 2)
             (expt (pose distance-lo 2 3) 2)))))

(defmethod component-distance ((jlo-1 jlo) (jlo-2 jlo) &key
                               (reference (make-jlo :id 1))
                               (axis :z))
  (let ((ref-jlo-1 (frame-query reference jlo-1))
        (ref-jlo-2 (frame-query reference jlo-2))
        (index (ccase axis
                 (:x 0)
                 (:y 1)
                 (:z 2))))
    (- (pose ref-jlo-1 index 3)
       (pose ref-jlo-2 index 3))))

