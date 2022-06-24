;;;; -*- Mode: Lisp; -*-

(asdf:defsystem :bmp-test
  :components ((:file "bmp-test" :depends-on ("lisp-unit"))
	       (:file "lisp-unit")
	       )
  :depends-on (#:png))
