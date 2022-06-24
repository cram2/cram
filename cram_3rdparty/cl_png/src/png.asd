;;;; -*- Mode: Lisp; -*-

(in-package #:cl-user)

(asdf:defsystem #:png
  :description "Read and write PNG (Portable Network Graphics) files."
  :perform (asdf:load-op :after (op png)
                         (pushnew :png *features*))
  :components ((:file "png-package" :depends-on ("image"))
               (:file "compat" :depends-on ("png-package"))
               (:file "libpng" :depends-on ("grovel" "compat" "png-package" "wrappers"))
               (:cffi-grovel-file "grovel")
               (:cffi-wrapper-file "wrappers")
               (:file "image")
               (:file "bmp" :depends-on ("image"))
               (:file "pnm" :depends-on ("image"))
               (:file "ops" :depends-on ("image")))
  :depends-on (#:cffi)
  :defsystem-depends-on ("cffi-grovel"))
