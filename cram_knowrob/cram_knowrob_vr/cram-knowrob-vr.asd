(asdf:defsystem cram-knowrob-vr
	:depends-on (roslisp
               cram-language
               cram-json-prolog
               std_msgs-msg
               cl-transforms
               cl-transforms-stamped
               cl-tf
               cram-bullet-world-tutorial
               cram-knowrob-pick-place
               cram-object-interfaces)
	:components
	
	((:module "src"
	  :components
	  ((:file "package")
     
     (:file "items" :depends-on ("package"))
     (:file "utils" :depends-on ("package"))
     (:file "data-extraction" :depends-on ("package"))
     (:file "robot-positions-calculations" :depends-on ("package"))
     (:file "init" :depends-on ("package"
                                "utils")) ; initialisation    
	   (:file "queries" :depends-on ("package"
                                   "utils"))
     (:file "demo-preparation" :depends-on ("package"
                                            "queries"
                                            "utils"))
     (:file "openease-to-bullet" :depends-on ("package"
                                              "utils"))
     (:file "demo-plans" :depends-on ("package"
                                      "queries")) ; plans for demonstrations     
     (:file "designators" :depends-on ("package"))
     (:file "plans" :depends-on ("package"
                                 "designators"
                                 "utils"))
     (:file "plan-execution" :depends-on ("package"
                                          "plans"
                                          "utils"))
     (:file "grasping" :depends-on ("package")) ; specifies how to grasp obj
     (:file "gaussian" :depends-on ("package"))
     
     ;; TODO remove?
     (:file "utility-queries" :depends-on ("package")) 
     
     ;(:file "reference-queries" :depends-on ("package")) can this jsut be ignored?
     ))))
