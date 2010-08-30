
(in-package :cl-user)

(defpackage cram-roslisp-common
    (:use :cl :roslisp)
  (:export #:register-ros-init-function
           #:register-ros-cleanup-function
           #:startup-ros
           #:shutdown-ros
           #:lispify-ros-name
           #:rosify-lisp-name))
