(defpackage #:png-example
  (:use #:common-lisp)
  (:export #:rotate))

(in-package #:png-example)

(defun rotate (input-pathname output-pathname)
  "Read a PNG image, rotate it 90 degrees, and write it to a new file."
  (let* ((old (with-open-file (input input-pathname 
				     :element-type '(unsigned-byte 8))
		(png:decode input)))
	 (new (png:make-image (png:image-width old)
			      (png:image-height old)
			      (png:image-channels old)
			      (png:image-bit-depth old)))
	 (m (png:image-width old)))
      (dotimes (i (png:image-height new))
	(dotimes (j (png:image-width new))
	  (dotimes (k (png:image-channels new))
	    (setf (aref new i j k) (aref old j (- m i 1) k)))))
      (with-open-file (output output-pathname :element-type '(unsigned-byte 8)
			      :direction :output :if-exists :supersede)
	(png:encode new output))))
