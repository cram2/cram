(defsystem bullet-reasoning-interface
  :depends-on (roslisp
               cram-language
               bullet-reasoning
               spatial-relations-demo
               designators
               cl-tf2
               cram-reasoning
               bullet_reasoning_interface-srv
               bullet_reasoning_interface-msg)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "reasoning-service" :depends-on ("package"))))))
