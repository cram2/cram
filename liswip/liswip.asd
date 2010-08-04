;;;; -*- Mode: LISP -*-

(in-package :asdf)

(defsystem "liswip"
  :author "Lars Kunze <kunzel@in.tum.de>"
  :version "0.1"
  :maintainer "Lars Kunze <kunzel@in.tum.de>"
  :license "BSD"
  :description "CFFI-Based Common Lisp <-> SWI Prolog interface."

  :depends-on (:cffi :cram-utilities :trivial-garbage :sb-posix)

  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "cffi")
             (:file "lisp-blobs")
             (:file "liswip")
             (:file "swi-predicates"))
            :serial t)))
