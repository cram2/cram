
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
  (let ((p-matrix (cl-transforms:pose->matrix p)))
    (jlo:make-jlo :parent (jlo:make-jlo :name (cl-tf:frame-id p))
                  :pose (make-array (array-total-size p-matrix)
                                    :displaced-to p-matrix
                                    :element-type (array-element-type p-matrix)))))

(defmethod jlo->pose (jlo)
  (cl-transforms:matrix->transform
   (make-array '(4 4) :displaced-to (vision_msgs-msg:pose-val
                                     (jlo:partial-lo (jlo:frame-query
                                                      (jlo:make-jlo :id 1)
                                                      jlo))))))
(defun table-cluster->jlo (cluster)
  (flet ((pose-cov->jlo-cov (cov)
           (let ((result (make-array 36 :initial-element 0.0d0)))
             (dotimes (y 3)
               (dotimes (x 3)
                 (setf (aref result (+ (* y 6) x)) (aref cov y x))))
             result)))
    (jlo:make-jlo :name (symbol-name (table-costmap:name cluster))
                  :pose (make-array
                         16 :initial-contents
                         `(1 0 0 ,(cl-transforms:x (table-costmap:mean cluster))
                             0 1 0 ,(cl-transforms:y (table-costmap:mean cluster))
                             0 0 1 ,(cl-transforms:z (table-costmap:mean cluster))
                             0 0 0 1))
                  :cov (let ((cov (pose-cov->jlo-cov (table-costmap:cov cluster))))
                         (dotimes (i 2)
                           (setf (aref cov (+ (* i 6) 2)) 0.0d0)
                           (setf (aref cov (+ (* 2 6) i)) 0.0d0))
                         (setf (aref cov 14) 0.1d0)
                         cov))))