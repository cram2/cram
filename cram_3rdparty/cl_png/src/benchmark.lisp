
(in-package #:png-test)

(defparameter *fall-pathname*
  #+asdf (asdf:system-relative-pathname '#:png "Fall.png"))

(defun decode-fall ()
  (with-open-file (input *fall-pathname* :element-type '(unsigned-byte 8))
    (decode input)))
   

(defun time-decode (n)
  (time (dotimes (i n)
	  (decode-fall))))

(defun time-encode (n)
  (let ((im (decode-fall)))
    (time
     (dotimes (i n)
       (with-open-file (output "/tmp/foo.png" :element-type '(unsigned-byte 8)
			       :direction :output :if-exists :supersede)
	 (encode im output))))))

(defun time-16-bit-image (n)
  (let ((im (decode-fall)))
    (time (dotimes (i n)
	    (16-bit-image im)))))

(defun time-8-bit-image (n)
  (let ((im (16-bit-image (decode-fall))))
    (time (dotimes (i n)
	    (8-bit-image im)))))
