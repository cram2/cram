(in-package :le)

; initializes the episode data by connecting to OpenEase and loads the Episode data within OpenEase
; contains paths to the Episode data and SemanticMap
; which might need to be adjusted, if other episode data is to be loaded. 

;; care to give the right rcg stamp
;; rcg_c is for finding out the displacement
;; rcg_f has the to date better grasps
;; eval2 has best full set pick and place
;; rcg_d different grasps
(defun init-episode ()
  (start-ros-node "lisp_ease")
  (register-ros-package "knowrob_robcog")
  (u-load-episodes "/media/hasu/Exte/episodes/Own-Episodes/set-clean-table/rcg_eval2/Episodes/")
  (owl-parse "/media/hasu/Exte/episodes/Own-Episodes/set-clean-table/rcg_eval2/SemanticMap.owl")
  (connect-to-db "Own-Episodes_set-clean-table")  
  (map-marker-init))


; initializes the bullet world environment based on the bullet-world-tutorial
(defun init-bullet-world ()
  ;;; append own meshes to meshes list so that they can be loaded.
  (append-meshes-to-list)
  
  ;;; init tf early. Otherwise there will be exceptions.
  (cram-tf::init-tf)
  
  ;;; set costmap parameters
  (prolog:def-fact-group costmap-metadata ()
    (prolog:<- (location-costmap:costmap-size 12 12))
    (prolog:<- (location-costmap:costmap-origin -6 -6))
    (prolog:<- (location-costmap:costmap-resolution 0.05))

    (prolog:<- (location-costmap:costmap-padding 0.2))
    (prolog:<- (location-costmap:costmap-manipulation-padding 0.2))
    (prolog:<- (location-costmap:costmap-in-reach-distance 0.6))
    (prolog:<- (location-costmap:costmap-reach-minimal-distance 0.2)))
  ;;; set params
  (setf cram-bullet-reasoning-belief-state:*robot-parameter* "robot_description")
  (setf cram-bullet-reasoning-belief-state:*kitchen-parameter* "no_kitchen_description")

  (sem-map:get-semantic-map)

  (cram-occasions-events:clear-belief)

  (setf cram-tf:*tf-default-timeout* 2.0)

  (setf prolog:*break-on-lisp-errors* t)
  
  ;;; initialization from the tutorial
  (prolog:prolog '(and (btr:bullet-world ?world)
                              (assert (btr:object ?world :static-plane :floor ((0 0 0) (0 0 0 1))
                                                  :normal (0 0 1) :constant 0))))
  (cram-occupancy-grid-costmap::init-occupancy-grid-costmap)
  (cram-bullet-reasoning-belief-state::ros-time-init)
  (cram-location-costmap::location-costmap-vis-init)
 
  ;(prolog:prolog '(btr:bullet-world ?world))
  (prolog:prolog '(and (btr:bullet-world ?world)
                   (btr:debug-window ?world)))
  ;;;load robot description
  (let ((robot-urdf
                   (cl-urdf:parse-urdf
                    (roslisp:get-param "robot_description"))))
             (prolog:prolog
              `(and (btr:bullet-world ?world)
                    (cram-robot-interfaces:robot ?robot)
                    (assert (btr:object ?world :urdf ?robot ((0 0 0) (0 0 0 1)) :urdf ,robot-urdf))
                    (cram-robot-interfaces:robot-arms-carrying-joint-states ?robot ?joint-states)
                    (assert (btr:joint-state ?world ?robot ?joint-states))
                    (assert (btr:joint-state ?world ?robot (("torso_lift_joint" 0.15d0)))))))

 ;;; spawn kitchen as urdf
 ;;  (let ((kitchen-urdf 
 ;;                 (cl-urdf:parse-urdf 
 ;;                  (roslisp:get-param "kitchen_description"))))
 ;;             (prolog:prolog
 ;;              `(and (btr:bullet-world ?world)
 ;;                    (assert (btr:object ?world :semantic-map no-urdf-kitchen ((0 0 0) (0 0 0 1)) )))))
 )


(defun init-reset-sim ()
  "Resets the simulation, but can also be used to initialize it (although I prefer to initilize it manually so I can see where it fails if it does.) Spawns all the objects which are necessary for the current scenario (Meaning: Kitchen, Robot, Muesli, Milk, Cup, Bowl, Fork and 3 Axis objects for debugging."
  (init-episode)
  (init-bullet-world)
  (add-bowl)
  (add-muesli)
  (add-axes)
  (add-fork)
  (add-cup)
  (add-milk)
  
  ;;axes 3
  (prolog:prolog '(and (btr:bullet-world ?world)
                   (assert (btr:object ?world :mesh ba-axes3 ((2 2 1) (0 0 0 1))
                            :mass 0.2 :color (1 0 0) :mesh :ba-axes))))
  ;; axes 2 
  (prolog:prolog '(and (btr:bullet-world ?world)
                   (assert (btr:object ?world :mesh ba-axes2 ((2 2 1) (0 0 0 1))
                            :mass 0.2 :color (0 1 0) :mesh :ba-axes)))))

