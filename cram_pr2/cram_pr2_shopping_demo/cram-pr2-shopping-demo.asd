(defsystem cram-pr2-shopping-demo
    :depends-on (roslisp-utilities
                 cl-transforms 
                 cl-transforms-stamped
                 cl-tf
                 cram-tf
                 cram-language
                 cram-executive
                 cram-designators
                 cram-prolog
                 cram-projection
                 cram-common-failures
                 cram-physics-utils
                 cl-bullet
                 cram-bullet-reasoning
                 cram-bullet-reasoning-belief-state
                 cram-bullet-reasoning-utilities
                 cram-btr-visibility-costmap
                 cram-location-costmap
                 cram-pr2-projection
                 cram-mobile-pick-place-plans
                 cram-pr2-description)

    :components
    ((:module "src"
      :components
      ((:file "package")
       (:file "setup" :depends-on ("package"))
       (:file "utils" :depends-on ("package"))
       (:file "plans" :depends-on ("package"))
       (:file "grasping" :depends-on ("package"))
       ))))
