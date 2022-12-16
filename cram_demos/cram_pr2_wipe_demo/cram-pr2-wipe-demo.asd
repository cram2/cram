(defsystem cram-pr2-wipe-demo
  :author "Felix Krause"

  :depends-on (roslisp-utilities 

               cl-transforms
               cl-transforms-stamped
               cl-tf
               cl-bullet
               cram-tf

               cram-language
               cram-executive
               cram-designators
               cram-prolog
               cram-projection
               cram-occasions-events
               cram-utilities 

               cram-common-failures
               cram-mobile-pick-place-plans
               cram-object-knowledge

               cram-cloud-logger

               cram-physics-utils     
               cl-bullet 
               cram-bullet-reasoning
               cram-bullet-reasoning-belief-state
               cram-bullet-reasoning-utilities
               cram-btr-visibility-costmap
               cram-btr-spatial-relations-costmap

        
               cram-robot-pose-gaussian-costmap
               cram-occupancy-grid-costmap
               cram-location-costmap

               cram-urdf-projection     
               cram-urdf-projection-reasoning 
               cram-pr2-description
               cram-fetch-deliver-plans
               cram-urdf-environment-manipulation
               cram-mobile-cut-pour-plans

               cram-plan-occasions-events
               cram-manipulation-interfaces
               cram-robot-interfaces
               )

  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "demo" :depends-on ("package"))
     (:file "setup" :depends-on ("package"))
     (:file "trajectories" :depends-on ("package"))
     (:file "wipe-designators" :depends-on ("package"))
     (:file "wiping" :depends-on ("package"))
     ))))
