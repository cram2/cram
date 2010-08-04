;;; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(in-package :cl-user)

(asdf:defsystem cram-utilities-tests
  :name "cram-utilities-tests"
  :author "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :version "0.1"
  :maintainer "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :licence "BSD"
  :description "Test-suite for cram-utilities"

  :depends-on (cram-test-utilities
               cram-utilities
               fiveam
               alexandria)
  :components
  ((:module "tests"
            :components ((:file "package")
                         (:file "suite")
                         (:file "patmatch"))
            :serial t)))

(defmethod asdf:perform ((o asdf:test-op)
                         (c (eql (asdf:find-system 'cram-utilities-tests))))
  (flet ((symbol (pkg name)
           (intern (string name) (find-package pkg))))
    (funcall (symbol :5am :run!) (symbol :cut-tests :utilities))))
