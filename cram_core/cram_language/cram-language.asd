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
     (:file "language" :depends-on ("packages" "walker" "tasks" "fluents" "logging" "with-policy"))
     (:file "plans" :depends-on ("packages" "tasks"))
     (:file "goals" :depends-on ("packages" "tasks"))
     (:file "fluent-operators" :depends-on ("packages" "fluents"))
     (:file "swank-indentation" :depends-on ("packages"))
     ;; Some CRAM vs. SBCL scheduling related stuff
     #+sbcl (:file "sbcl-hotpatches"))))

  :perform (asdf:test-op (operation cram-language-component)
                         (asdf:operate 'asdf:load-op 'cram-language-tests)
                         (asdf:operate 'asdf:test-op 'cram-language-tests))

  ;; Some deadline-related things depend on the SBCL version, so push the version
  ;; to *features* to have version-dependent code at compile time
  :perform (asdf:prepare-op :after (operation cram-language-component)
                            (let* ((sbcl-version-string
                                     (lisp-implementation-version))
                                   (major-version
                                     (parse-integer sbcl-version-string :junk-allowed t))
                                   (major-version-dot-index
                                     (position #\. sbcl-version-string))
                                   (minor-version
                                     (parse-integer (subseq sbcl-version-string
                                                            (1+ major-version-dot-index))
                                                    :junk-allowed t))
                                   (minor-version-dot-index
                                     (position #\. sbcl-version-string
                                               :start (1+ major-version-dot-index)))
                                   (patch-version
                                     (parse-integer (subseq sbcl-version-string
                                                            (1+ minor-version-dot-index))
                                                    :junk-allowed t)))
                              (when (or (> major-version 1)
                                        (and (= major-version 1)
                                             (or (> minor-version 4)
                                                 (and (= minor-version 4)
                                                      (>= patch-version 3)))))
                                (pushnew :sbcl-1.4.3+ *features*)))))
