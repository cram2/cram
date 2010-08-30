;;;; -*- Mode: LISP -*-

(defsystem "cram-roslisp-common"
  :depends-on ("roslisp" "cl-tf")
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "ros-node" :depends-on ("package"))
             (:file "lispification" :depends-on ("package"))
             (:file "tf" :depends-on ("package" "ros-node"))))))
