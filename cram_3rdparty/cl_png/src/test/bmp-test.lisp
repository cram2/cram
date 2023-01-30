;;;# Unit Tests for BMP file interface
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
;;; Depends on the lisp-unit, obtained from
;;; http://www.cliki.net/lisp-unit. The package defined in
;;; bmp-test.asd will load and run them.
;;;
;;; Test images are provided which must located as defined in
;;; *BMPIMAGES-PATHNAME* below.
;;;
(defpackage #:bmp-test
  (:use #:common-lisp #:lisp-unit #:image)
  (:export #:*images-pathname*))

(in-package #:bmp-test)

(defparameter *images-pathname*
  #+asdf (merge-pathnames "images/"
                          (truename (asdf:system-definition-pathname
                                     '#:bmp-test))))
(run-tests )
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

(defun encode-decode (im &key strip-alpha keep-tmp)
  (let ((pathname (make-name "tmp" "bmp")))
    (ignore-errors (delete-file pathname))
    (bmp::encode-file im pathname :strip-alpha strip-alpha)
    (prog1
        (bmp::decode-file pathname)
      (unless keep-tmp
        (delete-file pathname)))))

(defun max-diff (im1 im2)
  (image:intensity-max (image:subtract im1 im2)))


;;;# Basic Tests
;;;
;;; Tests that decode function produces correct type of image and
;;; fails on those whose modes aren't supported.
(define-test image-type-test
  (let ((rgb  (decode-bmpimage "tagged-RGB"))
        (argb (decode-bmpimage "tagged-ARGB"))
        (gray (decode-bmpimage "tagged-gray")))
    (assert-true (typep rgb  'rgb-image))
    (assert-true (typep gray 'grayscale-image))
    (assert-error 'bmp::unhandled-compression (decode-bmpimage "intrepid-xrgb"))
    (assert-error 'bmp::unhandled-compression (decode-bmpimage "intrepid-r5b6g5"))
    (assert-error 'bmp::unhandled-compression (decode-bmpimage "intrepid-a1r5b5g5"))
    (assert-error 'bmp::unhandled-bitcount    (decode-bmpimage "intrepid-x1r5b5g5"))))

;;(run-tests '(image-type-test))

;;; Test that the row padding is being handled properly.  There are
;;; four cases: images whose widths modulo 4 result in 0,1,2,3.
;;;
;;; The methodology is to decode a file into an image, re-encode the
;;; image back to a file, then decode it again and compare the two
;;; decoded images.

(define-test encode-modulo-0
  (let ((a (decode-bmpimage "scene")))
    (assert-equal 0 (max-diff a (encode-decode a)))))

(define-test encode-modulo-1
  (let ((a (decode-bmpimage "scene-w765"))
        (b (decode-bmpimage "scene-bw765")))
    (assert-true (typep a 'rgb-image))
    (assert-equal 0 (max-diff a (encode-decode a :keep-tmp t)))
    (assert-true (typep b 'grayscale-image))
    (assert-equal 0 (max-diff b (encode-decode b)))))

(define-test encode-modulo-2
  (let ((a (decode-bmpimage "scene-w766")))
    (assert-equal 0  (image:intensity-max (image:subtract a (encode-decode a))))))

(define-test encode-modulo-3
  (let ((a (decode-bmpimage "scene-w767"))
        (b (decode-bmpimage "scene-bw767")))
    (assert-true (typep a 'rgb-image))
    (assert-equal 0 (max-diff a (encode-decode a)))
    (assert-true (typep b 'grayscale-image))
    (assert-equal 0 (max-diff b (encode-decode b)))))

;;(run-tests '(encode-modulo-0 encode-modulo-2))
;;(run-tests '(encode-modulo-1 encode-modulo-3))
;; (run-tests)


(define-test decode-strip-alpha
    (let* ((a (decode-bmpimage "intrepid-rgb"))
           (b (decode-bmpimage "intrepid-argb"))
           (c (decode-bmpimage "intrepid-argb" :strip-alpha T)))
    (assert-true (typep a 'rgb-image))
    (assert-true (typep c 'rgb-image))
    (assert-equal 0 (max-diff a c))))

(define-test encode-strip-alpha
    (let* ((a (decode-bmpimage "intrepid-argb"))
           (b (encode-decode a :strip-alpha T :keep-tmp T))
           (c (decode-bmpimage "tmp")))
      (assert-true (typep b 'rgb-image))
      (assert-true (typep c 'rgb-image))
      (assert-equal 0 (max-diff b c))))

;;(run-tests '(decode-strip-alpha))
;;(run-tests '(encode-strip-alpha))
