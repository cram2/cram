;;;; -*- Mode: Lisp; -*-

(asdf:defsystem #:image-test
  :components ((:file "image-test" :depends-on ("lisp-unit"))
	       (:file "lisp-unit")
	       )
  :depends-on (#:png))
