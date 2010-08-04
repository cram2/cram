;;; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(in-package :cl-user)

(asdf:defsystem cram-language
  :name "cram-language"
  :author "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :version "0.1"
  :maintainer "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :licence "BSD"
  :description "Coginitive plan language"
  :long-description "cram-language is a new plan language."

  :depends-on (trivial-garbage
               alexandria
               cram-utilities)
  :components
  ((:module "src"
    :components
    ((:file "packages")
     (:file "utils" :depends-on ("packages"))
     (:file "task" :depends-on ("packages" "utils"))
     (:file "failures" :depends-on ("packages"))
     (:file "task-tree" :depends-on ("packages" "task"))
     (:file "task.implementation"
            :depends-on ("packages" "fluents" "task" "failures"))
     (:file "base" :depends-on ("packages" "task" "walker" "task-tree"))
     (:file "plans" :depends-on ("packages" "task" "task-tree"))
     (:file "goals" :depends-on ("packages" "task-tree"))
     (:file "language" :depends-on ("packages" "fluents"))
     (:file "swank-indentation")
     (:module "fluents"
              :depends-on ("packages" "task")
              :components
              ((:file "fluent")
               (:file "value-fluent" :depends-on ("fluent"))
               (:file "fluent-net" :depends-on ("fluent"))
               (:file "pulse-fluent" :depends-on ("fluent"))))
     (:module "walker"
              :depends-on ("packages")
              :components
              ((:file "augment-environment-sbcl-patch")
               (:file "env")
               (:file "env-impl-specific")
               (:file "plan-tree")
               (:file "walker")
               (:file "interface"))
              :serial t)))))

(defmethod asdf:perform ((o asdf:test-op)
                         (c (eql (asdf:find-system 'cram-language))))
  (asdf:operate 'asdf:load-op 'cram-language-tests)
  (asdf:operate 'asdf:test-op 'cram-language-tests))
