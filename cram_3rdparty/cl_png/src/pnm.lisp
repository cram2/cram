(defpackage #:pnm
  (:use #:common-lisp #:image)
  (:export #:decode #:encode))

(in-package #:pnm)

(defun whitespace-char-p (char)
  (and (member char '(#\Space #\Newline #\Return #\Tab))
       t))

(defun read-magic-number (input)
  (with-output-to-string (output)
    (loop
     for char = (read-char input)
     do (cond
         ((whitespace-char-p char) (loop-finish))
         ((char-equal #\# char) (read-line input nil nil))
         (t (write-char char output)))
     finally (unread-char char input))))

(defun read-whitespace (input &key (at-least 1))
  (loop
   for char = (read-char input nil nil)
   and count from 0
   do (cond
       ((char-equal #\# char) (read-line input nil nil))
       ((not (whitespace-char-p char)) (loop-finish)))
   finally
   (when char
     (unread-char char input))
   (when (< count at-least)
     (error "Read only ~D whitespace characters; ~D required." 
            count at-least))))

(defun read-whitespace-character (input)
  (let ((char (read-char input)))
    (unless (whitespace-char-p char)
      (error "Expected whitespace character, found ~S." char))))

(defun read-positive-integer (input)
  (parse-integer 
   (with-output-to-string (output)
     (loop
      for char = (read-char input nil nil)
      do (cond
          ((null char) (loop-finish))
          ((digit-char-p char) (write-char char output))
          ((char-equal #\# char) (read-line input nil nil))
          (t (loop-finish)))
      finally (when char
                (unread-char char input))))))

(defun read-pnm-header (input)
  "Read the header if it hasn't been read already, then return it."
  (let* ((magic-number (read-magic-number input))
         (type (cond ((equal magic-number "P4") 'pbm)
                     ((equal magic-number "P5") 'pgm)
                     ((equal magic-number "P6") 'ppm)
                     (t (error "Unknown magic number: ~S" 
                               magic-number))))
         (width (progn 
                  (read-whitespace input)
                  (read-positive-integer input)))
         (height (progn 
                   (read-whitespace input)
                   (read-positive-integer input)))
         (max-value (if (eq type 'pbm) 
                        1
                      (progn
                        (read-whitespace input)
                        (read-positive-integer input)))))
    (values magic-number width height max-value)))

(defun decode (input)
  (multiple-value-bind (magic-number width height max-value)
      (read-pnm-header input)
    (funcall 
     (cond ((equal magic-number "P4") 'read-pbm)
           ((equal magic-number "P5")
            (cond ((<= max-value 255) 'read-pgm-8)
                  ((<= max-value 65535) 'read-pgm-16)
                  (t (error "Invalid max-value for PGM: ~D"
                            max-value))))
           ((equal magic-number "P6")
            (cond ((<= max-value 255) 'read-ppm-8)
                  ((<= max-value 65535) 'read-ppm-16)
                  (t (error "Invalid max-value for PPM: ~D" max-value))))
           (t (error "Invalid magic number for PNM: ~S" magic-number)))
     input width height)))

(defun read-pbm (input width height)
  (let ((row (make-array (ceiling width 8)
                         :element-type '(unsigned-byte 8)))
        (raw-data (make-array (* width height)
                              :element-type 'bit)))
    (dotimes (y height)
      (unless (= (read-sequence row input)
                 (length row))
        (error "Failed to read ~D bytes of raw image data." (length row)))
      (dotimes (x width)
        (setf (aref raw-data (+ (* y width) x)) 
              (ldb (byte 1 (- 7 (mod x 8)))
                   (aref row (floor x 8))))))
    (make-instance 'image :raw-data raw-data
                   :height height :width width :bits 1 :channels 1)))

(defun read-pgm-8 (input width height)
  (let ((raw-data (make-array (* width height) 
                              :element-type '(unsigned-byte 8))))
    (unless (= (read-sequence raw-data input)
               (length raw-data))
      (error "Failed to read ~D bytes of raw image data." (length raw-data)))
    (make-instance 'image :raw-data raw-data :height height :width width 
		   :bits 8 :channels 1)))

(defun read-pgm-16 (input width height)
  (let ((raw-data (make-array (* width height) 
                              :element-type '(unsigned-byte 16)))
        (row (make-array (* width 2) :element-type '(unsigned-byte 8))))
    (dotimes (y height)
      (unless (= (read-sequence row input)
                 (length row))
        (error "Failed to read ~D bytes of raw image data." (length row)))
      (dotimes (x width)
        (setf (aref raw-data (+ (* y width) x))
              (+ (* 256 (aref row (* x 2)))
                 (aref row (1+ (* x 2)))))))
    (make-instance 'image :raw-data raw-data :height height :width width
		   :bits 16 :channels 1)))

(defun read-ppm-8 (input width height)
  (let ((raw-data (make-array (* width height 3)
			      :element-type '(unsigned-byte 8)))
        (row (make-array (* width 3)
                         :element-type '(unsigned-byte 8))))
    (dotimes (y height)
      (unless (= (read-sequence row input) (length row))
        (error "Failed to read ~D bytes of raw image data." (length row)))
      (dotimes (x width)
	(dotimes (c 3)
	  (setf (aref raw-data (+ (* (+ (* y width) x) 3) c))
		(aref row (+ (* x 3) c))))))
    (make-instance 'image :raw-data raw-data :height height :width width
		   :bits 8 :channels 3)))

(defun read-ppm-16 (input width height)
  (let ((raw-data #1=(make-array (* width height 3)
				 :element-type '(unsigned-byte 16)))
        (row (make-array (* width 3 2)
                         :element-type '(unsigned-byte 8))))
    (flet ((row-ref (x c)
             (let ((i (+ (* x 3) c)))
               (+ (* 256 (aref row (* i 2)))
                  (aref row (1+ (* i 2)))))))
      (dotimes (y height)
        (unless (= (read-sequence row input) (length row))
          (error "Failed to read ~D bytes of raw image data." (length row)))
        (dotimes (x width)
	  (dotimes (c 3)
	    (setf (aref raw-data (+ (* (+ (* y width) x) 3) c))
		  (row-ref x c))))))
      (make-instance 'image :raw-data raw-data :height height :width width
		     :bits 16 :channels 3)))

(defun write-pnm-header (image stream type)
  (write-sequence (map 'vector #'char-code 
		       (format nil "~A~%~D ~D~%~D~%" type (image-width image) 
			       (image-height image) 
			       (1- (expt 2 (image-bit-depth image)))))
		  stream))

(defun write-pnm-data-1 (image stream)
  (dotimes (y (image-height image))
    (loop
       with value = 0
       for x below (image-width image)
       do (let ((b (mod x 8)))
            (when (zerop b)
              (unless (= x 0)
                (write-byte value stream)
                (setf value 0)))
            (setf value (dpb (aref image y x)
                             (byte 1 (- 7 b))
                             value)))
       finally (write-byte value stream))))

(defun write-pnm-data-8 (image stream)
  (write-sequence (array-displacement image) stream))

(defun write-pnm-data-16 (image stream)
  (dotimes (i (array-total-size image))
    (multiple-value-bind (msb lsb) (floor (row-major-aref image i) 256)
      (write-byte msb stream)
      (write-byte lsb stream))))

(defun write-pnm-data (image stream)
  (let ((bits (image-bit-depth image)))
    (cond ((= bits 1) (write-pnm-data-1 image stream))
	  ((<= bits 8) (write-pnm-data-8 image stream))
	  ((<= bits 16) (write-pnm-data-16 image stream))
	  (t (error "Cannot write ~D-bit image as PNM." bits))))
  t)
  
(defun encode (image stream)
  (cond ((= (image-bit-depth image) 1) 
	 (write-pnm-header image stream "P4")
	 (write-pnm-data image stream))
	((= (image-channels image) 1)
	 (write-pnm-header image stream "P5")
	 (write-pnm-data image stream))
	((= (image-channels image) 3)
	 (write-pnm-header image stream "P6")
	 (write-pnm-data image stream))
	(t (error "Cannot write image with ~D bits and ~D channels as PNM." 
		  (image-bit-depth image) (image-channels image)))))
