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
	
	((:module "queries"
	  :components
	  ((:file "package")
	   (:file "queries" :depends-on ("package"))
     	   (:file "basic-queries")
     	   (:file "combined-queries")
     	   (:file "bullet_simulation" :depends-on ("package"))
     	   (:file "items" :depends-on ("package"))))))
