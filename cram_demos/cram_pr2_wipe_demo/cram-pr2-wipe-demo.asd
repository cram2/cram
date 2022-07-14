(defsystem cram-pr2-wipe-demo
  :author "Felix Krause"

  :depends-on (roslisp-utilities ; for ros-init-function

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
               cram-utilities ; for EQUALIZE-LISTS-OF-LISTS-LENGTHS

               cram-common-failures
               cram-mobile-pick-place-plans
               cram-object-knowledge

               cram-cloud-logger

               cram-physics-utils     ; for reading "package://" paths
               cl-bullet ; for handling BOUNDING-BOX datastructures
               cram-bullet-reasoning
               cram-bullet-reasoning-belief-state
               cram-bullet-reasoning-utilities
               cram-btr-visibility-costmap
               cram-btr-spatial-relations-costmap

               ;; cram-semantic-map-costmap
               cram-robot-pose-gaussian-costmap
               cram-occupancy-grid-costmap
               cram-location-costmap

               cram-urdf-projection      ; for with-simulated-robot
               cram-urdf-projection-reasoning ; to set projection reasoning to T
               cram-pr2-description
               cram-fetch-deliver-plans
               cram-urdf-environment-manipulation
               cram-mobile-cut-pour-plans)

  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "demo" :depends-on ("package"))
     (:file "setup" :depends-on ("package"))
     ))))
