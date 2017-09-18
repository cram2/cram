(in-package :cl-user)

(defpackage cram-intermediate-tutorial
  (:nicknames :tuti)
  (:use #:common-lisp)
  (:export #:init-cram-moveit-tutorial
           #:*cube-mesh* #:*pose-mid* #:*pose-cube* #:*pose-right*
           #:*pose-right-msg* #:*start-robot-state* #:*planned-trajectory*))
