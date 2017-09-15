;;; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(in-package :cl-user)

(asdf:defsystem cram-test-utilities-tests
  :name "cram-test-utilities-tests"
  :author "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :maintainer "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :licence "BSD"
  :description "Test-suite for cram-test-utilities"

  :depends-on (cram-test-utilities
               fiveam
               alexandria)
  :components
  ((:module "tests"
            :components ((:file "package")
                         (:file "suite")
                         (:file "utils"))
            :serial t)))

(defmethod asdf:perform ((o asdf:test-op)
                         (c (eql (asdf:find-system 'cram-test-utilities-tests))))
  (flet ((symbol (pkg name)
           (intern (string name) (find-package pkg))))
    (funcall (symbol :5am :run!) (symbol :cram-test-utilities-tests :test-utilities))))
