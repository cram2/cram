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
                 (:file "objects-tests" :depends-on ("package"))
                 (:file "bounding-box-tests" :depends-on ("package"))
                 (:file "timeline-tests" :depends-on ("package"))
                 (:file "copy-world-tests" :depends-on ("package"))
                 (:file "moveit-tests" :depends-on ("package"))))))
