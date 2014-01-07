(defsystem cram-beginner-tutorial
  :depends-on (roslisp cram-language turtlesim-msg cl-transforms geometry_msgs-msg designators cram-reasoning actionlib actionlib_tutorials-msg process-modules cram-plan-library cram-plan-failures
        cram-plan-knowledge turtle_actionlib-msg)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "turtle-action-client" :depends-on  ("package"))
             (:file "tutorial-designators" :depends-on ("package" "turtle-action-client"))
             (:file "tutorial" :depends-on ("package" "turtle-action-client" "tutorial-designators"))))))
