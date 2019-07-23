(defsystem cram-beginner-tutorial-tests
  :depends-on (cram-my-beginner-tutorial 
               lisp-unit
               cl-transforms
               roslisp
               turtlesim-msg)
  :components ((:module "tests"
                :components
                ((:file "package")
                 (:file "simple-plans-tests" :depends-on ("package")))))
  :perform (test-op (operation component)
                    (symbol-call :lisp-unit '#:run-tests :all :cram-beginner-tutorial-tests)))
