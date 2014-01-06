(defsystem cram-tutorial-planlib
  :author "Jan Winkler <winkler@cs.uni-bremen.de>"
  :license "BSD"
  :description "A roslisp package called 'cram-tutorial-planlib'"
  
  :depends-on (roslisp
               designators-ros
               cram-roslisp-common
               cram-plan-library
               cram-reasoning
               cram-plan-knowledge
               cram-environment-representation
               alexandria
	       cram-plan-failures
               designators
               cram-language
	       process-modules)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "cram-tutorial-planlib" :depends-on ("package"))))))
