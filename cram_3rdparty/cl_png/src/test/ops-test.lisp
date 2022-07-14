;;;# Unit Tests for Image Operations
;;;
;;;## Requirements
;;;
;;; This test suite depends on lisp-unit.lisp, obtained from
;;; http://www.cliki.net/lisp-unit, but also included herein.
;;;
;;; Everything should be loaded correctly by loading ops-test with
;;; asdf, provided the asd links have been established:

;; (asdf:oos 'asdf:load-op :ops-test)

;;; Some of the tests refer to provided images which must be located
;;; as defined in *BMPIMAGES-PATHNAME* as set below.
;;;
(defpackage #:ops-test
  (:use #:common-lisp #:lisp-unit #:image #:png)
  (:export #:*images-pathname*))
(in-package #:ops-test)

(defparameter *images-pathname*
  #+asdf (merge-pathnames "images/"
                          (truename (asdf:system-definition-pathname
                                     '#:ops-test))))
;;;## Convenience functions
(defun make-name (bn tp)
  (merge-pathnames (make-pathname :name bn :type tp) *images-pathname*))

(defun decode-pngimage (basename)
  (let ((pathname (make-name basename "png")))
    (with-open-file (input pathname :element-type '(unsigned-byte 8))
      (png:decode input))))

(defun decode-bmpimage (basename &key strip-alpha)
  (let ((pathname (make-name basename "bmp")))
    (with-open-file (input pathname :element-type '(unsigned-byte 8))
      (bmp:decode input :strip-alpha strip-alpha))))

(defun max-diff (im1 im2)
  (image:intensity-max (image:subtract im1 im2)))

(defun list2array (lst)
  (make-array (length lst) :initial-contents lst))

(defun fill-graded (a)
  (dotimes (h (image-height a) a)
    (dotimes (w (image-width a))
      (dotimes (c (image-channels a))
        (let ((val (+ (* (image-width a) h) w c)))
          (setf (aref a h w c) val))))))

;;;# Basic Tests
;;;
;;; Basic tests are those which will subsequently be used to help
;;; verify other functions.
;;;
;;;## Accessor Tests
(define-test accessor-test
  (let ((a (make-image 111 222 3)))
    (assert-equal '(111 . 222) (image:size a))
    (assert-equal '(111 222 3) (image:dims a))))

(run-tests accessor-test)

(defparameter *simple-set* '(((10 11 1 8)  grayscale-image (99))
                             ((12 13 1 16) grayscale-image (88))
                             ((14 15 3 8)  rgb-image (99 88 77))
                             ((16 17 3 16) rgb-image (66 55 44))
                             ((18 19 4 8)  rgb-image (111 122 133 144))
                             ((20 21 4 16) rgb-image (122 133 144 155))))

;;;## Test FILLV
;;;
;;; Creates an image of each type, fills it with an arbitrary pixel
;;; value, then checks to see that all are set correctly. Also checks
;;; the check function by changing one pixel.
(define-test fillv-test
  (flet ((check-for-diff (im val)
           (block found-diff
             (dotimes (r (image-height im) NIL)
               (dotimes (c (image-width im))
                 (dotimes (ch (image-channels im))
                   (unless (= (aref im r c ch) (nth ch val))
                     (return-from found-diff T))))))))
    (dolist (tst *simple-set*)
      (let ((img (apply #'make-image (car tst)))
            (typ (cadr tst))
            (val (caddr tst)))
        (assert-true (typep img typ))
        (image:fillv img val)
        (assert-false (check-for-diff img val))
        (decf (aref img 2 3 0))
        (assert-true (check-for-diff img val))))))

(run-tests fillv-test)

;;;## Test INTENSITY-MAX and CHANNEL-MAX
;;;
;;;
(define-test max-test
  (labels ((intensity (pix &optional (sum 0))
             (if (null pix)
                 sum
                 (intensity (cdr pix) (+ sum (car pix))))))
    (dolist (tst *simple-set*)
      (let ((img (apply #'make-image (car tst)))
            (val (caddr tst)))
        (image:fillv img val)
        (assert-equalp (list2array val) (channel-max img))
        (assert-equal (intensity val) (intensity-max img))
        ))))

(run-tests max-test)

;;;# Functions that Modify a Single Image
;;;

;;;## SCALE
;;;
;;; TBD

;;;# Functions that Combine Images
;;;
;;;## ADD, ADD*
;;;
;;; Depends on FLIP, MIRROR, CHANNEL-MIN, CHANNEL-MAX

(defparameter *add-set* '(((5 6 1 8)  grayscale-image)
                          ((5 6 1 16) grayscale-image)
                          ((5 6 3 8)  rgb-image)
                          ((5 6 3 16) rgb-image)
                          ((5 6 4 8)  rgba-image)
                          ((5 6 4 16) rgba-image)))

(define-test image-add-test
  (dolist (tst *add-set*)
    (let* ((a (fill-graded (apply #'make-image (car tst))))
           (b (flip a))
           (c (add a b))
           (d (image::mirror c))
           (e (add c d)))
      ;; test non-destructive version
      (assert-equalp (image::channel-min e) (channel-max e))
      ;; test destructive version
      (add* a b)
      (setf c (image::mirror a))
      (add* a c)
      (assert-equalp (image::channel-min a) (channel-max a)))))

(run-tests image-add-test)

(define-test image-sub-test
  (dolist (tst *add-set*)
    (let* ((a (fill-graded (apply #'make-image (car tst))))
           (b (flip a))
           (c (add a b))
           (d (subtract c b)))
      (assert-equalp a d)
      ;; test destructive version
      (subtract* c b)
      (assert-equalp a c))))

(run-tests image-sub-test)


;;;## MOVE-TO
;;;
(define-test image-move-to-test
    (let* ((a (image:fillv (make-image 4 4 1) '(100)))
           (b (image:fillv (make-image 4 4 1) '(10)))
           (c (image:fillv (make-image 4 4 1) '(80)))
           (d (image:fillv (make-image 4 4 1) '(30))))
      (assert-equalp c (image::move-towards a b 20))
      (assert-equalp d (image::move-towards b a 20))))

(run-tests image-move-to-test)




;;;# Other Scalar Valued Functions
;;;
;;;## IMAGE-NORM2
;;;
(define-test image-norm2
  (let ((a (make-image 10 10 3))
        (b (make-image 10 10 3)))
    (image:fillv a '(1 1 1))
    (image:fillv b '(2 2 2))
    (assert-equal  300 (image:norm2 a))
    (assert-equal 1200 (image:norm2 b))))

(run-tests image-norm2)



;;;# Edge Detector
;;;
;;; We will use a convolutional filter with this kernel:
;;;
;;;  0 -1  0
;;; -1  4 -1
;;;  0 -1  0
;;;

(defparameter *edge-kernel*
  (let ((mask (make-array (list 3 3)
                          :element-type 'float :initial-element 0.e0)))
    (setf (aref mask 0 1) -1.e0
          (aref mask 1 0) -1.e0
          (aref mask 1 2) -1.e0
          (aref mask 2 1) -1.e0
          (aref mask 1 1)  4.e0)
    mask))
(print *edge-kernel*)

;; Disabled because the input file butterfly8-gray.png is missing.
#+ignore
(define-test convolve-edge
  (let* ((a (png:decode-file (merge-pathnames "butterfly8-gray.png" *images-pathname*)))
         (c (png:decode-file (merge-pathnames "butterfly8.png" *images-pathname*)))
         (ea)
         (ec))
    (time
     (setf ea (image::convolve a *edge-kernel* :fill '(#x7f))))
    (png:encode-file ea "test-ea.png")

    (time
     (setf ec (image::convolve c *edge-kernel*)))
    (format t "ec size:~s~%" (size ec))
    (png:encode-file ec "test-ec.png")

    (time
     (setf ec (image::convolve c *edge-kernel* :fill '(#x7f #x7f #x7f))))
    (format t "ec size:~s~%" (size ec))
    (png:encode-file ec "test-ecb.png")

    (assert-true (typep ec 'rgb-image))))

;;(run-tests convolve-edge)

;; (run-tests)
