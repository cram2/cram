;;;; -*- Mode: Lisp; -*-

(asdf:defsystem :ops-test
  :components ((:file "ops-test" :depends-on ("lisp-unit"))
	       (:file "lisp-unit")
	       )
  :depends-on (#:png))

