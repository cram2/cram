(defsystem tutorial-process-module
  :author "Jan Winkler"
  :license "BSD"
  
  :depends-on (cram-tf
               cram-process-modules
               cram-roslisp-common
               cram-reasoning
               cram-plan-failures
               cram-plan-occasions-events
               cram-projection
               trivial-garbage
               alexandria)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "designators" :depends-on ("package"))
             (:file "action-handlers" :depends-on ("package" "designators"))
             (:file "process-module"
              :depends-on ("package" "designators" "action-handlers"))))))
