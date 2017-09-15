; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(asdf:defsystem cram-prolog-tests
  :name "cram-prolog-tests"
  :author "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :maintainer "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :licence "BSD"
  :description "Tests for cram-prolog"
  :depends-on (alexandria
               lisp-unit
               cram-utilities
               cram-prolog)
  :components
  ((:module "tests"
            :components
            ((:file "package")
             (:file "utilities" :depends-on ("package"))
             (:file "rete" :depends-on ("package" "utilities"))
             (:file "unify" :depends-on ("package"))
             (:file "prolog" :depends-on ("package"))))))

(defmethod asdf:perform ((o asdf:test-op)
                         (c (eql (asdf:find-system 'cram-prolog-tests))))
  (flet ((symbol (pkg name)
           (intern (string name) (find-package pkg))))
    (funcall (symbol :cram-prolog-tests :run-cram-prolog-tests))))
