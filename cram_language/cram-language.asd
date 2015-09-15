;;; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(in-package :cl-user)

(asdf:defsystem cram-language
  :name "cram-language"
  :author "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :maintainer "Lorenz Moesenlechner <moesenle@cs.tum.edu>"
  :licence "BSD"
  :description "Coginitive plan language"
  :long-description "cram-language is a new plan language."

  :depends-on (trivial-garbage
               alexandria
               cram-utilities
               sb-cltl2)
  :components
  ((:module "src"
    :components
    (;; Roots of the dependency graph
     (:file "packages")
     (:file "utils" :depends-on ("packages"))
     (:file "task-interface" :depends-on ("packages" "utils"))
     (:file "fluent-interface" :depends-on ("packages"))
     (:file "logging" :depends-on ("packages" "task-interface"))
     ;; WITH-POLICY, implementation
     (:file "with-policy" :depends-on ("packages"))
     (:file "default-policies" :depends-on ("packages" "with-policy"))
     ;; TASKS, implementation
     (:module "tasks"
      :depends-on ("packages" "task-interface" "fluent-interface" "utils" "logging")
      :components
      ((:file "failures" :depends-on ())
       (:file "task" :depends-on ("failures"))
       (:file "task-tree" :depends-on ("task"))))
     ;; FLUENTS, implementation
     (:module "fluents"
      :depends-on ("packages" "fluent-interface" "task-interface" "logging")
      :components
      ((:file "fluent")
       (:file "value-fluent" :depends-on ("fluent"))
       (:file "fluent-net" :depends-on ("fluent"))
       (:file "pulse-fluent" :depends-on ("fluent"))))
     ;; WALKER
     (:module "walker"
      :depends-on ("packages")
      :components
      ((:file "env")
       (:file "env-impl-specific")
       (:file "plan-tree")
       (:file "walker")
       (:file "interface"))
      :serial t)
     ;; CRAM, The Language
     (:file "language" :depends-on ("packages" "walker" "tasks" "fluents" "logging" "with-policy" "default-policies"))
     (:file "plans" :depends-on ("packages" "tasks"))
     (:file "goals" :depends-on ("packages" "tasks"))
     (:file "fluent-operators" :depends-on ("packages" "fluents"))
     (:file "swank-indentation" :depends-on ("packages"))
     ;; Some CRAM vs. SBCL scheduling related stuff
     #+sbcl (:file "sbcl-hotpatches")))))

(defmethod asdf:perform ((o asdf:test-op)
                         (c (eql (asdf:find-system 'cram-language))))
  (asdf:operate 'asdf:load-op 'cram-language-tests)
  (asdf:operate 'asdf:test-op 'cram-language-tests))
