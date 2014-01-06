(defsystem cram-tutorial-executive
  :author "Jan Winkler <winkler@cs.uni-bremen.de>"
  :license "BSD"
  :description "A roslisp package called 'cram-tutorial-executive'"
  
  :depends-on (roslisp
               designators-ros
               cram-roslisp-common
               cram-plan-library
               cram-reasoning
               cram-plan-knowledge
               cram-environment-representation
               alexandria
               designators-ros
               cram-language
               tutorial-process-module
               cram-tutorial-planlib)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "cram-tutorial-executive" :depends-on ("package"))))))
