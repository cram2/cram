;;; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(in-package :cl-user)

(asdf:defsystem cram-execution-trace
  :name "cram-execution-trace"
  :author "Nikolaus Demmel <demmeln@cs.tum.edu>"
  :maintainer "Nikolaus Demmel <demmeln@cs.tum.edu>"
  :licence "BSD"
  :description "Execution trace for CPL."
  :long-description "The execution trace provides facilities for tracing and
    all state changes during plan execution, serializing this episode
    information and accessing it in various ways."
  :depends-on (cl-store
               alexandria
               cram-utilities
               cram-language)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "utils" :depends-on ("package"))
     (:file "fluent-tracing" :depends-on ("package" "episode-knowledge" "utils"))
     (:file "auto-tracing" :depends-on ("package" "fluent-tracing"))
     (:file "offline-task" :depends-on ("package"))
     (:file "episode-knowledge-backend" :depends-on ("package" "episode-knowledge" "offline-task"))
     (:file "serialize" :depends-on ("package" "episode-knowledge" "episode-knowledge-backend"))
     (:file "interface" :depends-on ("package" "episode-knowledge" "serialize"))
     (:module "episode-knowledge"
              :depends-on ("package" "utils")
              :components
              ((:file "episode-knowledge")
               (:file "live-episode-knowledge" :depends-on ("episode-knowledge"))
               (:file "offline-episode-knowledge" :depends-on ("episode-knowledge"))))))))

(defmethod asdf:perform ((o asdf:test-op)
                         (c (eql (asdf:find-system 'cram-execution-trace))))
  (asdf:operate 'asdf:load-op 'cram-execution-trace-tests)
  (asdf:operate 'asdf:test-op 'cram-execution-trace-tests))
