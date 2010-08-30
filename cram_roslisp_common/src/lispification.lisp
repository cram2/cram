
(in-package :cram-roslisp-common)

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
