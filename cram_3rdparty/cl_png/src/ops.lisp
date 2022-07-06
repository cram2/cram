(in-package #:image)

;;; Error handlers

(define-condition mismatched-image-types (error)
  ((types :initarg :types :reader mismatched-image-types-types))
  (:report (lambda (condition stream)
             (format stream "Mismatched image types:~%~{  ~s~%~}"
                     (mismatched-image-types-types condition)))))

(define-condition mismatched-image-sizes (error)
  ((sizes :initarg :sizes :reader mismatched-image-sizes))
  (:report (lambda (condition stream)
             (format stream "Mismatched image sizes:~%~{  ~s~%~}"
                     (mismatched-image-sizes condition)))))

(define-condition mismatched-image-dims ()
  ((dims :initarg :dims :reader mismatched-image-dims-dims))
  (:report (lambda (condition stream)
             (format stream "Mismatched image dims:~%~{  ~s~%~}"
                     (mismatched-image-dims-dims condition)))))

;;; Convenience functions

(defun make-image-like (im)
  "Makes an image the same size and bit-depth as IM."
  (destructuring-bind (nm tp (h w c)) (type-of im)
    (declare (ignorable nm))
	(make-image h w c (second tp))))


;;; Additional IMAGE accessors

(defun size (im)
  "The (height x width) of an image as an alist"
  (cons (array-dimension im 0) (array-dimension im 1)))

(defun dims (im)
  "The dimensions (height x width x channels) of an image as a list"
  (list (array-dimension im 0) (array-dimension im 1) (array-dimension im 2)))


;;; Image setting functions

(defun fillv (im val)
  "Fills entire image with constant value. Value must be a list with
  the same number of elements as IM has channels.

Returns IM. "
  (unless (= (length val) (image-channels im))
    (error "Image channels doesn't match fill dimension: ~s" (length val)))
  (dotimes (i (image-height im) im)
      (dotimes (j (image-width im))
        (dotimes (k (image-channels im))
          (setf (aref im i j k) (nth k val))))))



;;; Scalar-valued functions of images

(defun channel-max (im)
  "Finds the max values over all the pixel channels (separately)
in the image. "
  (let ((mx (make-array (image-channels im) :initial-element 0)))
    (dotimes (i (image-height im) mx)
      (dotimes (j (image-width im))
        (dotimes (k (image-channels im))
          (when (< (aref mx k) (aref im i j k))
            (setf (aref mx k) (aref im i j k))))))))

(defun channel-min (im)
  "Finds the min values over all the pixel channels (separately)
in the image. "
  (let ((mn (make-array (image-channels im) :initial-element 255)))
    (dotimes (i (image-height im) mn)
      (dotimes (j (image-width im))
        (dotimes (k (image-channels im))
          (when (> (aref mn k) (aref im i j k))
            (setf (aref mn k) (aref im i j k))))))))

(defun intensity-max (im)
  "Finds the max intensities over all the pixels (sum of all the
channels) in the image. "
  (let ((mx 0))
    (dotimes (i (image-height im) mx)
      (dotimes (j (image-width im))
        (let ((acc 0))
          (dotimes (k (image-channels im))
            (incf acc (aref im i j k)))
          (when (< mx acc)
            (setf mx acc)))))))


(defun norm2 (im)
  "Calculates the sum of the squared intensities of all the pixels in
  the image. "
  (let ((acc 0))
    (dotimes (i (image-height im) acc)
      (dotimes (j (image-width im))
        (dotimes (k (image-channels im))
          (incf acc (* (aref im i j k) (aref im i j k))))))))


;;; Image-valued operations on single images

(defun rotate (old)
  "Returns a new image which is rotated counter-clockwise 90-degrees
from the old image"
  (let* ((new (make-image (image-width old) (image-height old)
                          (image-channels old) (image-bit-depth old)))
         (m (image-width old)))
    (dotimes (i (image-height new) new)
      (dotimes (j (image-width new))
        (dotimes (k (image-channels new))
          (setf (aref new i j k) (aref old j (- m i 1) k)))))))


(defun flip (old)
  "Returns a new image which is flipped vertically from the old image"
  (let ((new (make-image-like old))
        (m (image-height old)))
    (dotimes (h (image-height new) new)
      (dotimes (w (image-width new))
        (dotimes (c (image-channels new))
          (setf (aref new h w c) (aref old (- m h 1) w c)))))))


(defun mirror (old)
  "Returns a new image which is mirrored horizontally from the old
image"
  (let ((new (make-image-like old))
        (m (image-width old)))
    (dotimes (h (image-height new) new)
      (dotimes (w (image-width new))
        (dotimes (c (image-channels new))
          (setf (aref new h w c) (aref old h (- m w 1) c)))))))


;;; Image-valued operations on multiple images

(defun subtract (im1 im2)
  "Subtracts image IM2 from image IM1 and returns the resulting image
difference without modifying either IM1 or IM2. Both images must be
the same type and size.

Clips pixel intensity to 0 when necessary. "
  (unless (equalp (type-of im1) (type-of im2))
    (error 'mismatched-image-types :types (list (type-of im1) (type-of im2))))
  (let ((new (make-image-like im1)))
        (dotimes (i (image-height new) new)
          (dotimes (j (image-width new))
            (dotimes (k (image-channels new))
              (setf (aref new i j k)
                    (if (> (aref im1 i j k) (aref im2 i j k))
                        (-  (aref im1 i j k) (aref im2 i j k))
                        0)))))))

(defun subtract* (im1 im2)
  "Destructively subtracts image IM2 from image IM1, leaving the
resulting image difference in im1. Both images must be the same type
and size.

Clips pixel intensity to 0 when necessary. "
  (unless (equalp (type-of im1) (type-of im2))
    (error 'mismatched-image-types :types (list (type-of im1) (type-of im2))))
  (dotimes (i (image-height im1) im1)
	(dotimes (j (image-width im1))
	  (dotimes (k (image-channels im1))
        (if (> (aref im1 i j k) (aref im2 i j k))
            (decf (aref im1 i j k) (aref im2 i j k))
            (setf (aref im1 i j k) 0))))))


(defun add (im1 im2)
  "Adds image IM2 from image IM1 and returns the resulting image sum
without modifying either IM1 or IM2. Both images must be the same type
and size.

Clips to maximum intensity in each channel if exceeded. "
  (unless (equalp (type-of im1) (type-of im2))
    (error 'mismatched-image-types :types (list (type-of im1) (type-of im2))))
  (let ((lim (1- (expt 2 (image-bit-depth im1))))
        (new (make-image-like im1)))
        (dotimes (i (image-height new) new)
          (dotimes (j (image-width new))
            (dotimes (k (image-channels new))
              (let ((sum (+ (coerce (aref im1 i j k) '(unsigned-byte 16))
                            (coerce (aref im2 i j k) '(unsigned-byte 16)))))
                (setf (aref new i j k) (if (> sum lim) lim sum))))))))


(defun add* (im1 im2)
  "Destructively adds image IM2 from image IM1, leaving the resulting
image sum in im1. Both images must be the same type and size.
Clips to maximum intensity in each channel if exceeded"
  (unless (equalp (type-of im1) (type-of im2))
    (error 'mismatched-image-types :types (list (type-of im1) (type-of im2))))
  (let ((lim (1- (expt 2 (image-bit-depth im1)))))
    (dotimes (i (image-height im1) im1)
      (dotimes (j (image-width im1))
        (dotimes (k (image-channels im1))
          (if (< (aref im1 i j k) (- lim (aref im2 i j k)))
              (incf (aref im1 i j k) (aref im2 i j k))
              (setf (aref im1 i j k) lim)))))))


(defun threshold-filter (image threshold)
  "Returns a GRAYSCALE-IMAGE the same dimensions as IMAGE whose
  corresponding elements are white if they exceed a threshold, or
  black if they do not. "
  (let* ((img (grayscale-image image))
         (new (image::make-image-like img))
         (maxval (if (= 8 (image-bit-depth img)) #xff #xffff)))
    (dotimes (i (array-total-size img) new)
      (setf (row-major-aref new i) (if (< (row-major-aref img i) threshold)
                                       0 maxval)))))

(defun generate-mask (name 1side)
  "Generates a binary mask matrix of '(UNSIGNED-BYTE 8) elements whose
sides are both (2*1SIDE+1) pixels. Valid mask shapes are specified by
NAME, currently :square and :cross. An invalid name will signal an
error.
"
  (let* ((width (1+ (* 2 1side)))
         (mask (make-array (list width width)
                           :element-type '(unsigned-byte 8)
                           :initial-element 1)))
    (ecase name
      (:square mask)
      (:cross
       (multiple-value-bind (d r) (floor width 3)
         (declare (ignorable r))
         ;; Notch out the corners
         (dotimes (r d mask)
           (dotimes (w d)
             (setf (aref mask r w) 0)
             (setf (aref mask r (- width w 1)) 0)
             (setf (aref mask (- width r 1) w) 0)
             (setf (aref mask (- width r 1) (- width w 1)) 0))))))))


(defun view-mask (mask)
  "Prints MASK to *standard-output* for quick examination. "
  (let ((dims  (array-dimensions mask)))
    (format t "~&")
    (dotimes (h (first dims))
      (dotimes (w (second dims))
        (format t "~:[ ~;1~]" (> (aref mask h w) 0)))
      (terpri))))

;; (view-mask (generate-mask :cross 3))
;; (view-mask (apply #'generate-mask '(:cross 4)))

(defun binary-morph-kernel (image r c mask op)
  "Impliments a binary morphological filter using binary MASK centered
at R,C on IMAGE. OP works as follows:
* OP = :or  (erosion) any mask pixel set results in output set,
* OP = :and (dilation), all mask pixels set results in output set,
* OP = :maj (median), majority of mask pixels set results in output set
"
  (let* ((dim (array-dimension mask 0))
         (1side (floor dim 2))
         (ro (- r 1side))
         (co (- c 1side))
         (rv (if (= 8 (image-bit-depth image)) #xff #xffff)))
    (cond ((eql op :or)
           (dotimes (r dim 0)
             (dotimes (c dim)
               (when (> (aref mask r c) 0)
                 (when (> (aref image (+ ro r) (+ co c) 0) 0)
                   (return-from binary-morph-kernel rv))))))
          ((eql op :and)
           (dotimes (r dim rv)
             (dotimes (c dim)
               (when (> (aref mask r c) 0)
                 (when (= (aref image (+ ro r ) (+ co c) 0) 0)
                   (return-from binary-morph-kernel 0))))))
          ((eql op :maj)
           (let ((acc 0))
             (dotimes (r dim)
               (dotimes (c dim)
                 (when (and (> (aref image (+ ro r ) (+ co c) 0) 0)
                            (> (aref mask r c) 0))
                   (incf acc))))
             (if (> acc (floor (array-total-size mask) 2))
                 rv
                 0))))))

(defun binary-morphological-filter (image operation mask fill)
  "Returns a binary-valued GRAYSCALE-IMAGE the same dimensions as
  IMAGE whose corresponding elements have been calculated using a
  binary morphological filter. The filter kernel centers a MASK on
  each pixel in IMAGE and looks at all the pixels in IMAGE which line
  up with those set in the MASK. OPERATION=:or sets the filtered pixel
  if any of these is set; :and sets the filtered pixel only if all of
  these are set; and :maj sets the filtered pixel if a majority of
  them are set. A border region can't be reached by the center of the
  mask while keeping the mask within IMAGE. New pixels in the border
  region are filled with FILL."
  (let* ((img    (grayscale-image image))
         (new    (image::make-image-like img))
         (1side  (floor (array-dimension mask 0) 2))
         (newrows (- (image-height image) 1side 1side))
         (newcols (- (image-width image) 1side 1side)))
    ;; Fill it first so the borders are taken care of
    (image::fillv new fill)
    (dotimes (r newrows new)
      (dotimes (c newcols)
        (let ((rv (binary-morph-kernel image (+ r 1side) (+ c 1side) mask operation)))
          ;; (format t " returns rv=~a~%" rv)
          (setf (aref new (+ r 1side) (+ c 1side) 0) rv))))))


(defun erosion-filter (image &key pattern mask (fill '(0)))
  "Returns a binary GRAYSCALE-IMAGE produced by a morphological filter
  with an AND operation on the mask kernel.

  The mask may be either specified by PATTERN or provided by MASK, a
  2-d array of (unsigned-byte 8) values set to 0 or 1.

  The format of PATTERN is '(:square 3) '(:cross 4), where the
  2*size+1 gives the total width and height of the mask.
"
  (let ((mask (if mask mask (apply #'generate-mask pattern))))
    (binary-morphological-filter image :and mask fill)))


(defun dilation-filter (image &key pattern mask (fill '(0)))
  "Returns a binary GRAYSCALE-IMAGE produced by a morphological filter
  with an OR operation on the given mask kernel.

  The mask may be either specified by PATTERN or provided by MASK, a
  2-d array of (unsigned-byte 8) values set to 0 or 1.

  The format of PATTERN is '(:square 3) '(:cross 4), where the
  2*size+1 gives the total width and height of the mask.
"
  (let ((mask (if mask mask (apply #'generate-mask pattern))))
    (binary-morphological-filter image :or mask fill)))


(defun majority-filter (image &key pattern mask (fill '(0)))
  "Returns a binary GRAYSCALE-IMAGE produced by a morphological filter
  with an MAJ operation on the given mask kernel (majority).

  The mask may be either specified by PATTERN or provided by MASK, a
  2-d array of (unsigned-byte 8) values set to 0 or 1.

  The format of PATTERN is '(:square 3) '(:cross 4), where the
  2*size+1 gives the total width and height of the mask.
"
    (let ((mask (if mask mask (apply #'generate-mask pattern))))
    (binary-morphological-filter image :maj mask fill)))


(defun open-filter (image &key pattern mask (fill '(0)))
  "Returns a binary GRAYSCALE-IMAGE produced by a cascade of an
erosion filter followed by a dilation filter, both using the same
mask. See those filters for descriptions of their properties.

The distinctive property of the cascade is size preservation of large
scale features.
"
  (let* ((mask (if mask mask (apply #'generate-mask pattern))))
    (binary-morphological-filter
     (binary-morphological-filter image :and mask fill) :or mask fill)))


(defun close-filter (image &key pattern mask (fill '(0)))
  "Returns a binary GRAYSCALE-IMAGE produced by a cascade of a
dilation filter followed by an erosion filter, both using the same
mask. See those filters for descriptions of their properties.

The distinctive property of the cascade is size preservation of large
scale features.
"
  (let* ((mask (if mask mask (apply #'generate-mask pattern))))
    (binary-morphological-filter
     (binary-morphological-filter image :or mask fill) :and mask fill)))


(defun move-towards (im1 im2 step)
  "Moves the pixels values of binary GRAYSCALE-IMAGE IM1 towards IM2
  by at most STEP. IM1 is altered, IM2 is not. Step can be 0-255 for
  8-bit depths and 0-65535 for 16-bit depths. "
  (dotimes (i (image-height im1) im1)
	(dotimes (j (image-width im1))
      (let ((src (aref im1 i j 0))
            (dst (aref im2 i j 0)))
        (setf (aref im1 i j 0) (+ src (* (min (abs (- dst src)) step)
                                         (if (< dst src) -1 1))))))))


(defun color-channels (image)
  (etypecase image
    (grayscale-image 1)
    (rgb-image 3)))

(defun max-color-index (image)
  (ecase (image-bit-depth image) (8 255) (16 65535)))

(defun pixel-type (image) (cadr (type-of image)))


(defun convolve (image kernel &key fill)
  "Returns a new image of type IMAGE produced by convolving KERNEL
  with IMAGE. This is not a circular convolution and no values are
  computed in the border region where the kernel is not completely
  contained in IMAGE.

  FILL controls the output image size: if set, it returns an image the
  same size as IMAGE, if NIL, it returns an image which is shrunk to
  exclude the uncomputed border region (the dimensions of KERNEL-1).

  KERNEL must be a 2-d square mask with odd dimensions (e.g. 3x3), a
  simple-array of type FLOAT. IMAGE can be GRAYSCALE-IMAGE or an
  RGB-IMAGE - in the latter case the channels are convolved with the
  KERNEL seperately.

  If FILL is set, it must consist of a list of values of the same type
  as the pixels of IMAGE. For example, an rgb-image with 8-bit pixels
  might have a fill of '(#x7f #x7f #x7f), whereas a grayscale-image
  would have a fill of '(#x7f). If FILL is not set the image will shrink.
"
  (declare (optimize (speed 2) (compilation-speed 0) (safety 0) (debug 0)))
  (declare (type (simple-array float (* *)) kernel))
  ;; (declare (type (simple-array (unsigned-byte 8) (* * *)) image))
  (let* ((dim     (array-dimension kernel 0))
         (1side   (floor dim 2))
         (colors  (the fixnum (color-channels image)))
         (maxval  (the fixnum (max-color-index image)))
         (width   (the fixnum (image-width image)))
         (height  (the fixnum (image-height image)))
         ;; (typ     (pixel-type image))
         )
    (flet ((innerprod (ro co k) ;; (ro,co) are the corner points of the mask
             (declare (type fixnum ro co k))
             (let ((accum 0.e0))
               (declare (type float accum))
               (dotimes (r dim)
                 (dotimes (c dim)
                   (incf accum (* (aref kernel r c)
                                  (the fixnum (aref image (+ ro r) (+ co c) k))))))
               (the fixnum (max 0 (min (the fixnum (floor accum)) maxval))))))
      (let ((newrows (the fixnum (- height 1side 1side)))
            (newcols (the fixnum (- width  1side 1side))))
        (if fill
            ;; Convolution preserving size with fill
            (let ((new (make-image-like image)))
              (dotimes (r 1side)
                (dotimes (c width)
                  (dotimes (k colors)
                    (setf (aref new r c k) (nth k fill)
                          (aref new (- height r 1) c k) (nth k fill)))))
              (dotimes (c 1side)
                (dotimes (r height)
                  (dotimes (k colors)
                    (setf (aref new r c k) (nth k fill)
                          (aref new r (- width c 1) k) (nth k fill)))))
              (dotimes (r newrows new)
                (dotimes (c newcols)
                  (dotimes (k colors)
                    (setf (the fixnum (aref new (+ r 1side) (+ c 1side) k)) (innerprod r c k))))))
            ;; Shrinking convolution
            (let ((new (make-image newrows newcols (image-channels image) (image-bit-depth image))))
              (dotimes (r newrows new)
                (dotimes (c newcols)
                  (dotimes (k colors)
                    (setf (the fixnum (aref new r c k)) (innerprod r c k)))))))))))
