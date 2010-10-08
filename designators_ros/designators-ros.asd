; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(asdf:defsystem designators-ros
  :name "designators-ros"
  :author "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :maintainer "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :licence "BSD"
  :description "Ros specific stuff for designators."
  :depends-on (:alexandria
               :designators
               :cl-tf
               :cram-utilities
               :cram-reasoning
               :cram-roslisp-common)

  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "location-proxy" :depends-on ("package"))))))
