; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(asdf:defsystem cram-executive
  :name "cram-executive"
  :author "Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>"
  :maintainer "Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>"
  :licence "BSD"
  :description "Utility constructs from writing CRAM-based executives."
  :depends-on (:alexandria :cram-language :cram-designators
                           :cram-process-modules :cram-occasions-events)

  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "perform" :depends-on ("package"))))))
