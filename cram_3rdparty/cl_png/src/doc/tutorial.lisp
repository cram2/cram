(defpackage #:tutorial
  (:use #:common-lisp #:lisp-unit #:png)
  (:export #:*images-pathname*))
(in-package #:tutorial)

;;;# IMAGE arrays
;;;
;;; An image is a 3-dimensional array of row, column, and channel
;;; representing pixels. 
;;;
;;; The BIT-DEPTH is the size of unsigned-byte used representing the
;;; array. Currently, this is restricted to 8 or 16.
;;;
;;;## Image types
;;;
;;; The following image types are defined: 8-BIT-IMAGE, 16-BIT-IMAGE,
;;; GRAY-SCALE-IMAGE, RGB-IMAGE, ARGB-IMAGE. 
;;;
;;; Note that the Lisp TYPE-OF function returns the size as well as
;;; the element type for an array. This means that if you compare
;;; types you are also comparing image sizes.

(let ((a (make-image 200 300 3 8))
      (b (make-image 200 300 3 16))
      (c (make-image 200 301 3 8))
      (d (make-image 200 300 3 8)))
  (format t "~&TYPE-OF A: ~s~%" (type-of a))
  (format t "  A equals B: ~a~%" (equal (type-of a) (type-of b)))
  (format t "  A equals C: ~a~%" (equal (type-of a) (type-of c)))
  (format t "  A equals D: ~a~%" (equal (type-of a) (type-of d))))

;; TYPE-OF A: (SIMPLE-ARRAY (UNSIGNED-BYTE 8) (200 300 3))
;;   A equals B: NIL
;;   A equals C: NIL
;;   A equals D: T

;;;# Operations on IMAGEs
;;;
;;;



;;;# Encoders and Decoders
;;;
;;;## PNG
;;;
;;; TBD
;;;
;;;### Example
;;;
(defun rotate-file (input-pathname output-pathname)
  "Read a PNG image, rotate it 90 degrees CCW, and write it to a new file."
  (let* ((old (with-open-file (input input-pathname :element-type '(unsigned-byte 8))
                (decode input)))
         (new (rotate old)))
    (with-open-file (output output-pathname :element-type '(unsigned-byte 8)
                            :direction :output :if-exists :supersede)
      (encode new output))))


;;;## BMP
;;;
;;; TBD
;;;
;;;### Example
;;;
;;; TBD

;;;## TIF, JPEG, and other formats
;;;
;;; Planned but not implemented yet.
;;;

