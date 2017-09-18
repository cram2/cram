(defsystem cram-tutorial-planlib
  :author "Jan Winkler <winkler@cs.uni-bremen.de>"
  :license "BSD"
  :description "A roslisp package called 'cram-tutorial-planlib'"
  
  :depends-on (roslisp
               cram-tf
               cram-roslisp-common
               cram-plan-library
               cram-reasoning
               cram-plan-occasions-events
               cram-bullet-reasoning-belief-state
               alexandria
               cram-plan-failures
               cram-designators
               cram-language
               cram-process-modules)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "cram-tutorial-planlib" :depends-on ("package"))))))
