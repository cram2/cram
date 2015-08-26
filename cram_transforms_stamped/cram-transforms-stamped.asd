; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(asdf:defsystem cram-transforms-stamped
  :name "cram-transforms-stamped"
  :author "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :maintainer "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :licence "BSD"
  :description "Coordinate-frame transformation specific stuff for designators."
  :depends-on (:alexandria
               :cram-designators
               :cl-transforms-stamped
               :cram-utilities
               :cram-prolog
               :cram-roslisp-common
               :roslisp)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "designator-extensions" :depends-on ("package"))
             (:file "designator-filters" :depends-on ("package"))
             (:file "robot-current-pose" :depends-on ("package"))
             (:file "facts" :depends-on ("package"))))))
