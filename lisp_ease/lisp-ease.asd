(asdf:defsystem lisp-ease
	:depends-on (roslisp
               cram-language
               cram-json-prolog
               std_msgs-msg
               cl-transforms
               cl-transforms-stamped
               cl-tf
               cram-bullet-world-tutorial)
	:components
	
	((:module "src"
	  :components
	  ((:file "package")
	   (:file "queries" :depends-on ("package"))
     (:file "basic-queries")           
     (:file "utils" :depends-on ("package"))
     (:file "demo-plans" :depends-on ("package"))
     
     (:file "init" :depends-on ("package"))
     (:file "openease-to-bullet" :depends-on ("package"))
     (:file "items" :depends-on ("package"))
     (:file "data-extraction" :depends-on ("package"))
     (:file "map-calibration" :depends-on ("package"))
     (:file "plans" :depends-on ("package"))
     (:file "plan-execution" :depends-on ("package"))
     (:file "robot-positions-calculations" :depends-on ("package"))
     (:file "designators" :depends-on ("package"))
     (:file "grasping" :depends-on ("package")) ;; maybe add more documentation
     (:file "gaussian" :depends-on ("package"))
     (:file "demo-preparation" :depends-on ("package"))
     ))))
