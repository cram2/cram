(defsystem cram-beginner-tutorial
  :depends-on (cram-language roslisp turtlesim-msg geometry_msgs-msg cl-transforms
                             cram-designators cram-prolog
                             actionlib actionlib_msgs-msg turtle_actionlib-msg
                             cram-process-modules cram-language-designator-support)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "control-turtlesim" :depends-on ("package"))
             (:file "simple-plans" :depends-on ("package" "control-turtlesim"))
             (:file "action-designators" :depends-on ("package"))
             (:file "turtle-action-client" :depends-on ("package"))
             (:file "location-designators" :depends-on ("package"))
             (:file "process-modules" :depends-on ("package"
                                                   "control-turtlesim"
                                                   "simple-plans"
                                                   "action-designators"
                                                   "turtle-action-client"))
             (:file "selecting-process-modules" :depends-on ("package"
                                                             "action-designators"
                                                             "location-designators"
                                                             "process-modules"))))))
