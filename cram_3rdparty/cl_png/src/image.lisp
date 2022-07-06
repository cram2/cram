;;; TODO:
;;; * The displacement messes up the type inference, so making image
;;;   operations efficient is a pain.  Should see if I can just pass
;;;   the 3-D array to the C functions.

(defpackage #:image
  (:documentation "Image representation and manipulation.")
  (:use #:common-lisp)
  (:export
   ;; image.lisp
   #:image
   #:8-bit-image
   #:16-bit-image
   #:grayscale-image
   #:grayscale-alpha-image
   #:rgb-image
   #:rgb-alpha-image
   #:transparent-image
   #:opaque-image
   #:make-image
   #:copy-image
   #:image-height
   #:image-width
   #:image-channels
   #:image-alpha
   #:image-bit-depth
   ;; ops.lisp
   #:mismatched-image-types
   #:mismatched-image-sizes
   #:mismatched-image-dims
   #:make-image-like
   #:size
   #:dims
   #:fillv
   #:channel-min
   #:channel-max
   #:intensity-max
   #:norm2
   #:rotate
   #:flip
   #:mirror
   #:add
   #:add*
   #:subtract
   #:subtract*
   #:threshold-filter
   #:binary-morphological-filter
   #:erosion-filter
   #:dilation-filter
   #:majority-filter
   #:open-filter
   #:close-filter
   #:move-towards
   #:convolve))

(in-package #:image)

(deftype 8-bit-image (&optional height width channels)
  "An IMAGE with element type (UNSIGNED-BYTE 8)."
  `(and (simple-array (unsigned-byte 8) (,height ,width ,channels))))

(deftype 16-bit-image (&optional height width channels)
  "An IMAGE with element type (UNSIGNED-BYTE 16)."
  `(and (simple-array (unsigned-byte 16) (,height ,width ,channels))))

(deftype image (&optional height width channels)
  "A three-dimensional array of (unsigned-byte 8) or (unsigned-byte
16). In the current version, an IMAGE is displaced to a
one-dimensional SIMPLE-ARRAY with the same total number of elements,
but applications should not rely on this implementation detail, as it
is likely to change in future versions. The functions MAKE-IMAGE,
DECODE, COPY-IMAGE, 8-BIT-IMAGE, AND 16-BIT-IMAGE return IMAGEs.

The three dimensions represent row, column, and channel.  In other
words, (aref image i j k) is the intensity in the k-th channel of the
pixel in the i-th row and j-th column of image."
  `(or (8-bit-image ,height ,width ,channels)
       (16-bit-image ,height ,width ,channels)))

(defun even-channels-p (image)
  (evenp (array-dimension image 2)))

(deftype transparent-image (&optional height width)
  "An IMAGE with either two or four channels (i.e. has an alphs
channel)."
  `(and (image ,height ,width)
		(satisfies even-channels-p)))

(deftype opaque-image (&optional height width)
  "An IMAGE with either 1 or 3 channels"
  `(and (image ,height ,width)
        (not (satisfies even-channels-p))))

(defun color-channels-p (image)
  (> (array-dimension image 2) 2))

(deftype rgb-image (&optional height width)
  "An IMAGE with either three or four channels."
  `(and (image ,height ,width)
		(satisfies color-channels-p)))

(defun color-and-alpha-channels-p (image)
  (= (array-dimension image 2) 4))

(deftype rgb-alpha-image (&optional height width)
  "An IMAGE with exactly four channels."
  `(and (image ,height ,width)
		(satisfies color-and-alpha-channels-p)))

(defun grayscale-channels-p (image)
  (<= (array-dimension image 2) 2))

(deftype grayscale-image (&optional height width)
  "An IMAGE with either one or two channels."
  `(and (image ,height ,width)
        (satisfies grayscale-channels-p)))

(defun grayscale-alpha-channels-p (image)
  (= (array-dimension image 2) 2))

(deftype grayscale-alpha-image (&optional height width)
  "An IMAGE with exactly two channels."
  `(and (image ,height ,width)
        (satisfies grayscale-alpha-channels-p)))

(defun make-shareable-array (&rest args)
  #+(or lispworks3 lispworks4 lispworks5.0)
  (sys:in-static-area
    (apply #'make-array args))
  #-(or lispworks3 lispworks4 lispworks5.0)
  (apply #'make-array
	 #+(or lispworks allegro) :allocation
	 #+lispworks :static #+allegro :static-reclaimable
	 args))

(defun make-image (height width channels &optional bit-depth)
  "Make a new IMAGE of the specified height, width, and number of
channels.  The image will be an 8-bit-image or a 16-bit-image depending
on the value of byte-size.  Makes an 8-BIT-IMAGE if BIT-DEPTH is 8 or
NIL and a 16-BIT-IMAGE if BIT-DEPTH is 16.  The contents of the image
are undefined."
  (make-shareable-array (list height width channels)
                        :element-type (ecase bit-depth
                                        ((8 nil) '(unsigned-byte 8))
                                        (16 '(unsigned-byte 16)))))

(defun image-height (image)
  "The height of image, i.e., the number of rows."
  (array-dimension image 0))

(defun image-width (image)
  "The width of IMAGE, i.e., the number of columns."
  (array-dimension image 1))

(defun image-channels (image)
  "The number of channels in IMAGE.  Grayscale images have one
channel, whereas RGB images have three."
  (array-dimension image 2))

(defun image-alpha (image)
  "Returns T if there is an alpha channel, NIL otherwise."
  (evenp (array-dimension image 2)))

(defun image-bit-depth (image)
  "Returns the bit-depth of the image, i.e., the number of bits in the
byte representing each sample. The bit depth is 8 or 16, depending on
whether the image is an 8-bit-image or a 16-bit-image, respectively."
  (etypecase image
    (8-bit-image 8)
    (16-bit-image 16)))

(defun copy-image (image)
  "Creates a copy of IMAGE. The elements of the new image are the same
as the corresponding elements of IMAGE, and the new image has the same
height, width, number of channels, and bit depth as IMAGE."
  (let ((new (make-image (image-height image) (image-width image)
			 (image-channels image) (image-bit-depth image))))
    (dotimes (i (array-total-size image) new)
      (setf (row-major-aref new i) (row-major-aref image i)))))

(defun 8-bit-image (image)
  "If IMAGE is an 8-BIT-IMAGE, return it or a copy of it.  If IMAGE is
a 16-BIT-IMAGE, return an 8-BIT-IMAGE that has the same width, height,
and number of channels as image, but where each element is the
corresponding element in image divided by 257 and rounded to the
nearest integer.  The effect of this division is to compress the
dynamic range of the image so as to fit within the smaller bit depth."
  (etypecase image
    (8-bit-image image)
    (16-bit-image
     (let ((new (make-image (image-height image) (image-width image)
			    (image-channels image) 8)))
       (dotimes (i (array-total-size image) new)
	 (setf (row-major-aref new i) (round (row-major-aref image i) 257)))))))

(defun 16-bit-image (image)
  "If IMAGE is a 16-BIT-IMAGE, return it or a copy of it.  If IMAGE is
an 8-BIT-IMAGE, return a 16-BIT-IMAGE that has the same width, height,
and number of channels as IMAGE, but where each element is the
corresponding element in image multiplied by 257.  The effect of this
multiplication is to stretch the dynamic range of the image to utilize
the increased bit depth."
  (etypecase image
    (16-bit-image image)
    (8-bit-image
     (let ((new (make-image (image-height image) (image-width image)
			    (image-channels image) 16)))
       (dotimes (i (array-total-size image) new)
	 (setf (row-major-aref new i) (* 257 (row-major-aref image i))))))))

(defun grayscale-image (image)
  "If IMAGE is a GRAYSCALE-IMAGE, return it, otherwise return a
GRAYSCALE-IMAGE of the same width and height whose corresponding
elements are the average of the channel intensities of IMAGE.
Strip out any alpha channel present."
  (flet ((convert ()
           (let* ((bit-depth (image-bit-depth image))
		  (gray (make-image (image-height image) (image-width image)
				    1 bit-depth))
		  (tp `(unsigned-byte ,bit-depth)))
             (dotimes (h (image-height image) gray)
               (dotimes (w (image-width image))
                 ;; average the RGB channel intensities
                 (let ((avg (+ (coerce (aref image h w 0) 'float)
                               (coerce (aref image h w 1) 'float)
                               (coerce (aref image h w 2) 'float))))
                   (setf (aref gray h w 0) (coerce (floor avg 3) tp)))))))
	 (strip ()
           (let ((gray (make-image (image-height image) (image-width image)
                                   1 (image-bit-depth image))))
             (dotimes (h (image-height image) gray)
               (dotimes (w (image-width image))
		 (setf (aref gray h w 0) (aref image h w 0)))))))
    (etypecase image
      (grayscale-alpha-image (strip))
      (grayscale-image image)
      (t (convert)))))

(defun grayscale-alpha-image (image)
  "If IMAGE is a GRAYSCALE-ALPHA-IMAGE, return it, otherwise return a
GRAYSCALE-ALPHA-IMAGE of the same width and height whose corresponding
elements are the average of the channel intensities of IMAGE.
Add an alpha channel if needed."
  (flet ((convert ()
           (let* ((bit-depth (image-bit-depth image))
		  (gray (make-image (image-height image) (image-width image)
				    2 bit-depth))
		  (tp `(unsigned-byte ,bit-depth))
		  (use-this-alpha (cond
				    ((image-alpha image) nil)
				    ((= bit-depth 8) 255)
				    ((= bit-depth 16) 65535))))
             (dotimes (h (image-height image) gray)
               (dotimes (w (image-width image))
                 ;; average the RGB channel intensities
                 (let ((avg (+ (coerce (aref image h w 0) 'float)
                               (coerce (aref image h w 1) 'float)
                               (coerce (aref image h w 2) 'float))))
                   (setf (aref gray h w 0) (coerce (floor avg 3) tp)
			 (aref gray h w 1) (or use-this-alpha
					       (aref image h w 3))))))))
	 (add-alpha ()
           (let* ((bit-depth (image-bit-depth image))
		  (gray (make-image (image-height image) (image-width image)
				    2 bit-depth))
		  (use-this-alpha (cond
				    ((= bit-depth 8) 255)
				    ((= bit-depth 16) 65535))))
             (dotimes (h (image-height image) gray)
               (dotimes (w (image-width image))
		 (setf (aref gray h w 0) (aref image h w 0)
		       (aref gray h w 1) use-this-alpha))))))
    (etypecase image
      (grayscale-alpha-image image)
      (grayscale-image (add-alpha))
      (t (convert)))))

(defun rgb-image (image)
  "If IMAGE is an RGB-IMAGE, return it, otherwise return an
RGB-IMAGE of the same width and height whose corresponding
elements are the grayscale value repeated as needed.  Strip
out any alpha channels."
  (flet ((convert ()
           (let ((rgb (make-image (image-height image) (image-width image)
                                   3 (image-bit-depth image))))
             (dotimes (h (image-height image) rgb)
               (dotimes (w (image-width image))
                   (setf (aref rgb h w 0) (aref image h w 0)
			 (aref rgb h w 1) (aref image h w 0)
			 (aref rgb h w 2) (aref image h w 0))))))
	 (strip ()
           (let ((rgb (make-image (image-height image) (image-width image)
                                   3 (image-bit-depth image))))
             (dotimes (h (image-height image) rgb)
               (dotimes (w (image-width image))
                   (setf (aref rgb h w 0) (aref image h w 0)
			 (aref rgb h w 1) (aref image h w 1)
			 (aref rgb h w 2) (aref image h w 2)))))))
    (etypecase image
      (rgb-alpha-image (strip))
      (rgb-image image)
      (t (convert)))))

(defun rgb-alpha-image (image)
  "If IMAGE is a RGB-ALPHA-IMAGE, return it, otherwise return a
RGB-ALPHA-IMAGE of the same width and height whose corresponding
elements are the rgb elements of the original if the original is
an RGB-IMAGE and the repeated grayscale values if the original is
a GRAYSCALE image.  Add an alpha channel if needed."
  (flet ((convert ()
           (let* ((bit-depth (image-bit-depth image))
		  (rgba (make-image (image-height image) (image-width image)
				    4 bit-depth))
		  (use-this-alpha (cond
				    ((image-alpha image) nil)
				    ((= bit-depth 8) 255)
				    ((= bit-depth 16) 65535))))
             (dotimes (h (image-height image) rgba)
               (dotimes (w (image-width image))
                 ;; average the RGB channel intensities
                   (setf (aref rgba h w 0) (aref image h w 0)
			 (aref rgba h w 1) (aref image h w 0)
			 (aref rgba h w 2) (aref image h w 0)
			 (aref rgba h w 3) (or use-this-alpha
					       (aref image h w 1)))))))
	 (add-alpha ()
           (let* ((bit-depth (image-bit-depth image))
		  (rgba (make-image (image-height image) (image-width image)
				    4 bit-depth))
		  (use-this-alpha (cond
				    ((= bit-depth 8) 255)
				    ((= bit-depth 16) 65535))))
             (dotimes (h (image-height image) rgba)
               (dotimes (w (image-width image))
		 (setf (aref rgba h w 0) (aref image h w 0)
		       (aref rgba h w 1) (aref image h w 1)
		       (aref rgba h w 2) (aref image h w 2)
		       (aref rgba h w 3) use-this-alpha))))))
    (etypecase image
      (rgb-alpha-image image)
      (rgb-image (add-alpha))
      (t (convert)))))
