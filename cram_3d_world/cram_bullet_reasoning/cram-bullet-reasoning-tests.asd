(defsystem cram-bullet-reasoning-tests
  :depends-on (cram-bullet-reasoning
               cram-bullet-reasoning-utilities
               lisp-unit
               cl-transforms
               roslisp)
  :components ((:module "tests"
                :components
                ((:file "package")
                 (:file "items-tests" :depends-on ("package"))
                 (:file "objects-tests" :depends-on ("package"))))))
