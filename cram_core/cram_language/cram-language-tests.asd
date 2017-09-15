;;; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(in-package :cl-user)

(asdf:defsystem cram-language-tests
  :name "cram-language-tests"
  :author "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :maintainer "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :licence "BSD"
  :description "Test-suite for cram-language"

  :depends-on (cram-language cram-utilities cram-test-utilities
               fiveam alexandria)
  :components
  ((:module "tests"
    :components
    ((:file "package")
     (:file "suite"                 :depends-on ("package"))
     (:file "test-infrastructure"   :depends-on ("package"))
     (:file "task-utils"            :depends-on ("package"))
     (:file "fluent-utils"          :depends-on ("package"))
     (:file "walker-tests"          :depends-on ("package" "suite" "test-infrastructure"))
     (:file "task-tests"            :depends-on ("package" "suite" "test-infrastructure"
                                                           "task-utils" "fluent-utils"))
     (:file "fluent-tests"          :depends-on ("package" "suite" "test-infrastructure"
                                                           "task-utils" "fluent-utils"))
     (:file "language-tests"        :depends-on ("package" "suite" "test-infrastructure"
                                                           "task-utils" "fluent-utils"))
     (:file "execution-trace-tests" :depends-on ("package" "suite" "test-infrastructure"))))))

(defmethod asdf:perform ((o asdf:test-op)
                         (c (eql (asdf:find-system 'cram-language-tests))))
  (flet ((symbol (pkg name)
           (intern (string name) (find-package pkg))))
    (funcall (symbol :5am :run!) (symbol :cpl-tests :language))))
