; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(asdf:defsystem cram-reasoning-tests
  :name "cram-reasoning-tests"
  :author "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :version "0.1"
  :maintainer "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :licence "BSD"
  :description "Tests for cram-reasoning"
  :depends-on (alexandria
               fiveam
               cram-test-utilities
               cram-utilities
               cram-reasoning
               #+sbcl sb-rt
               #-sbcl rtest)

  :components
  ((:module "tests"
            :components
            ((:file "package")
             (:file "rete")
             (:file "suite")
             (:file "unify")
             (:file "prolog"))
            :serial t)))

(defmethod asdf:perform ((o asdf:test-op)
                         (c (eql (asdf:find-system 'cram-reasoning-tests))))
  (flet ((symbol (pkg name)
           (intern (string name) (find-package pkg))))
    (funcall (symbol :sb-rt :do-tests))
    (funcall (symbol :5am :run!) (symbol :crs-tests :reasoning))))
