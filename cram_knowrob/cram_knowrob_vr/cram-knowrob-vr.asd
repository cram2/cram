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
     (:file "utility-queries" :depends-on ("package")) 
     (:file "items" :depends-on ("package"))
     (:file "openease-to-bullet" :depends-on ("package")) 
     (:file "data-manipulation" :depends-on ("package"))
         
     (:file "designators" :depends-on ("package" "items"))
     (:file "init" :depends-on ("package" "utility-queries" "items")) ; initialisation     
     (:file "queries" :depends-on ("package" "data-manipulation"))
     (:file "movement" :depends-on ("package" "designators" "queries"))
     (:file "utils" :depends-on ("package" "items" "movement")) 
     (:file "robot-positions-calculations" :depends-on ("package" "queries"))
     (:file "grasping" :depends-on ("package" "queries" "openease-to-bullet")) ; specifies how to grasp obj  
     (:file "plans" :depends-on ("package" "designators" "utils" "queries"))     
     (:file "demo-preparation" :depends-on ("package" "utils"))
     (:file "plan-execution" :depends-on ("package" "plans" "utils" "queries" "movement"))  
     (:file "demo-plans" :depends-on ("package" "queries" "plans" "plan-execution")) ; plans for demonstrations     

     (:file "gaussian" :depends-on ("package"))
     (:file "debugging-utils" :depends-on ("package" "movement" "queries" "openease-to-bullet" "init"))
     ))))
