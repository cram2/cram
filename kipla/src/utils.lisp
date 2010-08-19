
(in-package :kipla-utils)

(defun lispify-ros-name (str &optional (package *package*))
  "Returns a lispified symbol correstonding to the string
  `str'. Lispification inserts - signs between camel-cased words and
  makes all characters uppercase."
  (flet ((char-ucase-p (c)
           (find c "ABCDEFGHIJKLMNOPQRSTUVWXYZ")))
    (let ((s (make-string-output-stream)))
      (loop for c across str
            with first = t
            when (and (not first) (char-ucase-p c)) do (write-char #\- s)
              do (write-char c s) (setf first nil))
      (intern (string-upcase (get-output-stream-string s)) package))))

(defun rosify-lisp-name (sym)
  (with-output-to-string (strm)
    (loop for ch across (symbol-name sym)
          with upcase = t
          if (eql ch #\-) do (setf upcase t)
            else do
              (progn
                (if upcase
                    (princ (char-upcase ch) strm)
                    (princ (char-downcase ch) strm))
                (setf upcase nil)))))

(defgeneric pose->jlo (pose))

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
