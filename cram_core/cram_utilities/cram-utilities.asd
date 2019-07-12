; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(asdf:defsystem cram-utilities
  :name "cram-utilities"
  :author "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :maintainer "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :licence "BSD"
  :description "Pattern matching and other utilities."
  :long-description "Provides basic pattern matching, binding
                     handling, and other utilities used by other cram
                     components."
  :depends-on (sb-concurrency
               ;; synchronization-tools
               alexandria)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "utils" :depends-on ("package"))
             (:file "macros" :depends-on ("package"))
             (:file "threads" :depends-on ("package"))
             (:file "lazy" :depends-on ("package"))
             (:file "patmatch" :depends-on ("package"))
             (:file "data-pool" :depends-on ("package"))
             (:file "clos" :depends-on ("package"))
             (:file "time" :depends-on ("package"))
             (:file "quad-tree" :depends-on ("package"))
             (:file "deprecation" :depends-on ("package"))
             (:file "file-cache" :depends-on ("package"))))))

(defmethod asdf:perform ((o asdf:test-op)
                         (c (eql (asdf:find-system 'cram-utilities))))
  (asdf:operate 'asdf:load-op 'cram-utilities-tests)
  (asdf:operate 'asdf:test-op 'cram-utilities-tests))
