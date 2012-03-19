; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(asdf:defsystem process-modules
  :name "process-modules"
  :author "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :version "0.1"
  :maintainer "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :licence "BSD"
  :description "Process modules."
  :long-description "To apply transformational planning, we need to
                     separate the high-level plan from low-level
                     components. High-level plans can be reasoned
                     about and transformed whereas the low level
                     components are somehow atomic. This is in
                     particular useful for implementing projection
                     mechanisms since they represent the continous
                     processes in the world."
  :depends-on (:alexandria :cram-language :designators)

  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "process-module-protocol" :depends-on ("package"))
             (:file "process-module" :depends-on ("package" "process-module-protocol"))))))
