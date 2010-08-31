
(in-package :kipla-utils)

(defgeneric pose->jlo (pose))
(defgeneric jlo->pose (jlo))

(defmethod pose->jlo ((p cl-transforms:pose))
  (let ((p-matrix (cl-transforms:transform->matrix p)))
    (jlo:make-jlo :parent (jlo:make-jlo :name "/map")
                  :pose (make-array (array-total-size p-matrix)
                                    :displaced-to p-matrix
                                    :element-type (array-element-type p-matrix)))))

(defmethod pose->jlo ((p cl-transforms:pose))
  (pose->jlo (cl-transforms:make-transform
              (cl-transforms:origin p)
              (cl-transforms:orientation p))))

(defmethod pose->jlo ((p cl-transforms:transform))
  (let ((p-matrix (cl-transforms:transform->matrix p)))
    (jlo:make-jlo :parent (jlo:make-jlo :name "/map")
                  :pose (make-array (array-total-size p-matrix)
                                    :displaced-to p-matrix
                                    :element-type (array-element-type p-matrix)))))

(defmethod pose->jlo ((p cl-tf:stamped-transform))
  (let ((p-matrix (cl-transforms:transform->matrix p)))
    (jlo:make-jlo :parent (jlo:make-jlo :name (cl-tf:frame-id p))
                  :name (cl-tf:child-frame-id p)
                  :pose (make-array (array-total-size p-matrix)
                                    :displaced-to p-matrix
                                    :element-type (array-element-type p-matrix)))))

(defmethod pose->jlo ((p cl-tf:pose-stamped))
  (let ((p-matrix (cl-transforms:transform->matrix p)))
    (jlo:make-jlo :parent (cl-tf:frame-id p)
                  :pose (make-array (array-total-size p-matrix)
                                    :displaced-to p-matrix
                                    :element-type (array-element-type p-matrix)))))

(defmethod jlo->pose (jlo)
  (cl-transforms:matrix->transform
   (make-array '(4 4) :displaced-to (vision_msgs-msg:pose-val
                                     (jlo:partial-lo jlo)))))
