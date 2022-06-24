(in-package #:cl-user)

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
   #:convolve
   ))
