;;;# Unit Tests for Image Operations
;;;
;;; RUN-TESTS to run tests, either individually, multiply, or all at
;;; once:
;;;
;; (run-tests test1)              ; runs one test
;; (run-tests test1 test2)        ; runs two tests
;; (run-tests)                    ; runs all defined tests
;;;
;;;## Requirements
;;;
;;; These unit tests depend on the lisp-unit library, obtained from
;;; http://www.cliki.net/lisp-unit. The package defined in
;;; bmp-test.asd will load them.
;;;
;;; Test images are provided which must located relative to this file
;;; location as defined in *BMPIMAGES-PATHNAME* below
;;;
(defpackage #:image-test
  (:use #:common-lisp #:lisp-unit #:image)
  (:export #:*images-pathname*))

(in-package #:image-test)

(defparameter *images-pathname*
  #+asdf (merge-pathnames "images/" 
                          (truename (asdf:system-definition-pathname 
                                     '#:image-test))))

;;;## Utility functions
;;;
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


;;;# Basic Tests
;;;


(in-package #:image-test)

(define-test channel-order
  (let ((rgb (decode-bmpimage "tagged-RGB"))
        (argb (decode-bmpimage "tagged-ARGB")))
    (assert-true (typep rgb 'rgb-image))
    (assert-true (typep rgb 'opaque-image))
    (assert-true (typep argb 'rgb-image))
    (assert-false (typep argb 'opaque-image))
    (assert-true (typep argb 'transparent-image))))

(define-test fill-tests
  (let ((a (make-image 10 10 3)))
    (assert-error 'error (image:fillv a '(99 99)))
    (assert-error 'error (image:fillv a '(1 1 2 2 2)))
    (image:fillv a '(99 88 77))
    (assert-equalp #(99 88 77) (image:channel-max a))))

(run-tests channel-order)
(run-tests fill-tests)

  
;;; Scalar-Valued Functions of One Image
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


;;;# Functions of One Image
;;;
;;;## IMAGE-SCALE

;;;# Functions of Two Images
;;;

;;;## IMAGE-SUB
;;;
(define-test image-sub-test
  (let ((a (decode-bmpimage "intrepid-rgb"))
        (b (decode-bmpimage "scene"))
        (c (decode-bmpimage "intrepid-argb"))
        (d (decode-bmpimage "intrepid-argb" :strip-alpha T))
        (err))
    (setf err (image:norm2 (image:subtract a d)))
    (assert-error 'error (image:subtract a b))
    (assert-error 'error (image:subtract a c))
    (assert-equal 0 err)
    (let* ((f (image:subtract a a))
           (g (make-image-like c))
           (h (make-image-like c)))
      (assert-equal 0 (image:norm2 f))
      (image:subtract* a d)
      (assert-equal 0 (image:norm2 a))
      (image:fillv g '(40 40 40 40))
      (setf h (image:subtract c g))
      (image:subtract* c g)
      (assert-equalp c h))))

(run-tests image-sub-test)


;;; ## IMAGE-ADD
(define-test image-add-test
  (let* ((a (decode-bmpimage "butterfly1"))
         ;; (a-maxch (image-channel-max a))
         (a-maxin (image:intensity-max a))
         (b (make-image-like a))
         (val '(40 30 20))
         (submax #(215 225 235))
         (c)
         (d))
    (image:fillv b val)
    (setf c (image:subtract a b))
    (assert-equal (- a-maxin (apply #'+ val)) (image:intensity-max c))
    (assert-equalp submax (image:channel-max c))
    (setf d (image:add c b))
    (image:subtract* d a)            ; contains bottom end clipping residue
    ;; (format t "maxchans d = ~a~%" (image-channel-max d))
    ))
;; (still working on this one)

(run-tests image-add-test)


;;;# Statistical Functions
;;;