(defsystem cram-bullet-reasoning-tests
  :depends-on (cram-bullet-reasoning
               cram-bullet-reasoning-utilities
               lisp-unit
               cl-transforms
               cram-pr2-description
               ;;cram-boxy-description
               roslisp)
  :components ((:module "tests"
                :components
                ((:file "package")
                 (:file "items-tests" :depends-on ("package"))
                 (:file "objects-tests" :depends-on ("package"))
                 (:file "robot-model-tests" :depends-on ("package"))
                 (:file "robot-model-utils-tests" :depends-on ("package" "robot-model-tests" "objects-tests"))
                 (:file "bounding-box-tests" :depends-on ("package"))
                 (:file "timeline-tests" :depends-on ("package"))
                 (:file "copy-world-tests" :depends-on ("package"))
                 (:file "moveit-tests" :depends-on ("package"))))))
