;;;; -*- Mode: LISP -*-

(defsystem "cram-roslisp-common"
  :depends-on ("roslisp" "cl-tf" "cram-utilities" "roslisp-utilities")
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "time" :depends-on ("package"))
             (:file "lispification" :depends-on ("package"))
             (:file "tf" :depends-on ("package"))
             (:file "sbcl-hotpatches")))))
