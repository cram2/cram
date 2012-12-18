;;;; -*- Mode: LISP -*-

(defsystem "cram-roslisp-common"
  :depends-on ("roslisp" "cl-tf" "cram-utilities")
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "ros-node" :depends-on ("package"))
             (:file "time" :depends-on ("package" "ros-node"))
             (:file "lispification" :depends-on ("package"))
             (:file "tf" :depends-on ("package" "ros-node"))
             (:file "sbcl-hotpatches")))))
