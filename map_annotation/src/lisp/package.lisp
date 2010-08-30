
(in-package :cl-user)

(defpackage map-annotation
    (:use :cl :roslisp :cram-roslisp-common)
  (:export #:get-closest-annotation
           #:get-annotated-point))
