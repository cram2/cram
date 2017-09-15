; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(asdf:defsystem cram-tf
  :name "cram-tf"
  :author "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :maintainer "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :licence "BSD"
  :description "Coordinate-frame transformation specific stuff for designators."
  :depends-on (:cram-designators
               :cl-transforms-stamped
               :cl-transforms
               :cram-utilities
               :cram-prolog
               :cram-robot-interfaces
               :cl-tf
               :roslisp-utilities
               :roslisp
               :cram-designator-specification)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "setup" :depends-on ("package"))
             (:file "designator-extensions" :depends-on ("package" "setup"))
             (:file "robot-current-pose" :depends-on ("package" "setup"))
             (:file "designator-filters" :depends-on ("package"))
             (:file "utilities" :depends-on ("package" "setup"))
             (:file "facts" :depends-on ("package" "utilities"))))))
