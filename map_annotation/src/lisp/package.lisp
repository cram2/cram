
(in-package :cl-user)

(defpackage map-annotation
    (:use :cl :roslisp :prolog)
  (:export #:get-closest-annotation
           #:get-annotated-point
           #:annotated-point
           #:get-annotation-names))
