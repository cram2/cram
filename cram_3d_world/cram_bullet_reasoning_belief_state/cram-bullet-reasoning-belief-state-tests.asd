(defsystem cram-bullet-reasoning-belief-state-tests
  :depends-on (cram-bullet-reasoning-belief-state
               lisp-unit)
  :components ((:module "tests"
                :components
                ((:file "package")
                 (:file "event-handlers-tests" :depends-on ("package"))))))
  ;; :perform (test-op (operation component)
  ;;                   (symbol-call :lisp-unit '#:run-tests :all
  ;;                                :cram-bulet-reasoning-belief-state-tests)))
