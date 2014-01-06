(defsystem tutorial-process-module
  :author "Jan Winkler"
  :license "BSD"
  
  :depends-on (designators-ros
               process-modules
               cram-roslisp-common
               cram-reasoning
               cram-plan-failures
               cram-plan-knowledge
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
