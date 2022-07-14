(in-package #:cl-user)

(defpackage #:png
  (:documentation "Read and write PNG (Portable Network Graphics) files.")
  (:use #:common-lisp #:cffi #:image)
  (:export
   #:image
   #:8-bit-image
   #:16-bit-image
   #:grayscale-image
   #:grayscale-alpha-image
   #:rgb-image
   #:rgb-alpha-image
   #:make-image
   #:copy-image
   #:image-height
   #:image-width
   #:image-channels
   #:image-bit-depth
   #:decode
   #:decode-file
   #:encode
   #:encode-file
   #:decode-bmp
   #:decode-bmp-file
   ))
