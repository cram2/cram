(defsystem bullet-reasoning-interface
  :depends-on (roslisp
               cram-language
               bullet-reasoning
               spatial-relations-demo
               cram-designators
               cl-tf2
               cram-prolog
               bullet_reasoning_interface-srv
               bullet_reasoning_interface-msg)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "reasoning-service" :depends-on ("package" "output" "facts" "operations" "utils"))
     (:file "facts" :depends-on ("package" "output"))
     (:file "operations" :depends-on ("package" "output"))
     (:file "output" :depends-on ("package"))
     (:file "utils" :depends-on ("package" "output"))))))
