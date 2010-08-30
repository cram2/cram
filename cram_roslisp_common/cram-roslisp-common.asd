;;;; -*- Mode: LISP -*-

(defsystem "cram-roslisp-common"
  :depends-on ("roslisp")
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "ros-node" :depends-on ("package"))))))
