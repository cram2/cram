;;; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(in-package :cl-user)

(asdf:defsystem cram-utilities-tests
  :name "cram-utilities-tests"
  :author "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :maintainer "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :licence "BSD"
  :description "Test-suite for cram-utilities"

  :depends-on (cram-utilities
               lisp-unit
               alexandria)
  :components
  ((:module "tests"
            :components ((:file "package")
                         (:file "utilities" :depends-on ("package"))
                         (:file "patmatch" :depends-on ("package"))
                         (:file "lazy-lists" :depends-on ("package"))))))

(defmethod asdf:perform ((o asdf:test-op)
                         (c (eql (asdf:find-system 'cram-utilities-tests))))
  (flet ((symbol (pkg name)
           (intern (string name) (find-package pkg))))
    (funcall (symbol :cut-tests :run-cut-tests))))
