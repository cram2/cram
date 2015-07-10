
(in-package :cl-user)

(defpackage map-annotation
    (:use :cl :roslisp :cram-roslisp-common :prolog)
  (:export #:get-closest-annotation
           #:get-annotated-point
           #:annotated-point
           #:get-annotation-names))
