; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(asdf:defsystem cram-math
  :name "cram-math"
  :author "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :maintainer "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :licence "BSD"
  :description "Some math utilities"
  :depends-on (alexandria gsll)

  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "random" :depends-on ("package"))
             (:file "matrix" :depends-on ("package"))
             (:file "functions" :depends-on ("package" "matrix"))
             (:file "geometry" :depends-on ("package"))))))
