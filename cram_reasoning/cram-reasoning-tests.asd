; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(asdf:defsystem cram-reasoning-tests
  :name "cram-reasoning-tests"
  :author "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :version "0.1"
  :maintainer "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :licence "BSD"
  :description "Tests for cram-reasoning"
  :depends-on (alexandria
               lisp-unit
               cram-utilities
               cram-reasoning)
  :components
  ((:module "tests"
            :components
            ((:file "package")
             (:file "utilities" :depends-on ("package"))
             (:file "rete" :depends-on ("package" "utilities"))
             (:file "unify")
             (:file "prolog")))))

(defmethod asdf:perform ((o asdf:test-op)
                         (c (eql (asdf:find-system 'cram-reasoning-tests))))
  (flet ((symbol (pkg name)
           (intern (string name) (find-package pkg))))
    (funcall (symbol :cram-reasoning-tests :run-cram-reasoning-tests))))
