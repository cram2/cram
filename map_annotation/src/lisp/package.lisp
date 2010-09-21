
(in-package :cl-user)

(defpackage map-annotation
    (:use :cl :roslisp :cram-roslisp-common :crs)
  (:export #:get-closest-annotation
           #:get-annotated-point
           #:annotated-point
           #:get-annotation-names))
