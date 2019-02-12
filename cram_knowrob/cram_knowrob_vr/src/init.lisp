;;; initializes the episode data by connecting to OpenEase and loads the Episode data within OpenEase
;;; contains paths to the Episode data and SemanticMap
;;; which might need to be adjusted, if other episode data is to be loaded. 

;;; care to give the right rcg stamp
;;; rcg_c is for finding out the displacement
;;; rcg_f has the to date better grasps
;;; eval2 has best full set pick and place
;;; rcg_d different grasps
(in-package :kvr)

;;; Sets the local path of where the episode data is located
(defvar *episode-path* "/home/hasu/ros_workspace/episode_data/episodes/Own-Episodes/set-clean-table/")

;;; epdir: name of the directory of the episode which is to be loaded. The path is
;;; individual and therefore hardcoded one
(defun init-episode (&optional (namedir "p4_island_rotated"))
  "Initializes the node and loads the episode data from knowrob via json_prolog.
The path of the episode files is set in the *episode-path* variable.
namedir: (parameter) is the name of the episode file directory which is to be loaded. "
  
  (ros-info (kvr) "initializing the episode data and connecting to database...")
  (start-ros-node "cram_knowrob_vr")
  (register-ros-package "knowrob_robcog")
  (u-load-episodes
   (concatenate 'string
                 *episode-path* namedir "/Episodes/"))
  (owl-parse
   (concatenate 'string
                *episode-path*  namedir "/SemanticMap.owl"))
  (connect-to-db "Own-Episodes_set-clean-table")  
  (map-marker-init))


;;; initializes the bullet world environment based on the bullet-world-tutorial
(defun init-bullet-world ()
  "Initializes the bullet world. The robot spawns in the white urdf kitchen,
while the semantic map kitchen is spawned right next to the urdf kitchen,
representing the Virtual Reality world, and how the kitchen was set up there."
  ;; reset bullet world
  (setq btr:*current-bullet-world* nil)
  
  ;; append own meshes to meshes list so that they can be loaded.
  (btr:add-objects-to-mesh-list "cram_knowrob_vr")

  (setf btr:*mesh-path-whitelist* *mesh-path-whitelist-unreal-kitchen*)
  
  ;; init tf early. Otherwise there will be exceptions.
  (cram-tf::init-tf)
  (setf cram-tf:*tf-default-timeout* 2.0)
  (setf prolog:*break-on-lisp-errors* t)

  (cram-occupancy-grid-costmap::init-occupancy-grid-costmap)
  (cram-bullet-reasoning-belief-state::ros-time-init)
  (cram-location-costmap::location-costmap-vis-init)
  
  ;; set costmap parameters
  (prolog:def-fact-group costmap-metadata ()
    (prolog:<- (location-costmap:costmap-size 12 12))
    (prolog:<- (location-costmap:costmap-origin -6 -6))
    (prolog:<- (location-costmap:costmap-resolution 0.05))

    (prolog:<- (location-costmap:costmap-padding 0.2))
    (prolog:<- (location-costmap:costmap-manipulation-padding 0.2))
    (prolog:<- (location-costmap:costmap-in-reach-distance 0.6))
    (prolog:<- (location-costmap:costmap-reach-minimal-distance 0.2)))

  ;;; initialization from the tutorial
  (prolog:prolog '(and (btr:bullet-world ?world)
                   (assert
                    (btr:object ?world :static-plane :floor ((0 0 0) (0 0 0 1))
                                                     :normal (0 0 1) :constant 0))))
  (prolog:prolog '(and (btr:bullet-world ?world)
                   (btr:debug-window ?world)))

  ;;; load robot description
  (let ((robot-urdf
          (cl-urdf:parse-urdf
           (roslisp:get-param btr-belief:*robot-parameter*))))
    (prolog:prolog
     `(and (btr:bullet-world ?world)
           (cram-robot-interfaces:robot ?robot)
           (assert (btr:object ?world :urdf ?robot ((0 0 0) (0 0 0 1)) :urdf ,robot-urdf))
           (rob-int:robot-joint-states ?robot :arm :left :park ?left-joint-states)
           (assert (btr:joint-state ?world ?robot ?left-joint-states))
           (rob-int:robot-joint-states ?robot :arm :right :park ?right-joint-states)
           (assert (btr:joint-state ?world ?robot ?right-joint-states))
           (rob-int:robot-torso-link-joint ?robot ?_ ?torso-joint)
           (rob-int:joint-lower-limit ?robot ?torso-joint ?lower-limit)
           (rob-int:joint-upper-limit ?robot ?torso-joint ?upper-limit)
           (lisp-fun average ?lower-limit ?upper-limit ?average-joint-value)
           (assert (btr:joint-state ?world ?robot
                                    ((?torso-joint ?average-joint-value)))))))

  (ros-info (kvr) "spawning semantic-map kitchen...")

  (sem-map:get-semantic-map)

;(cram-occasions-events:clear-belief)

    ;; spawning semantic map kitchen
  (prolog:prolog
   `(and (btr:bullet-world ?world)
         (assert (btr:object ?world :semantic-map :semantic-map-kitchen ((0 -3 0) (0 0 0 1))))))

  ;; spawning urdf kitchen
  (prolog:prolog
   `(and (btr:bullet-world ?world)
         (assert (btr:object ?world :urdf :kitchen ((0 0 0) (0 0 0 1))
                                          :collision-group :static-filter
                                          :collision-mask (:default-filter :character-filter)
                                          :compound t
                                          :urdf ,(cl-urdf:parse-urdf (roslisp:get-param "kitchen_description")))))))


(defun init-items ()
  "Spawns all the objects the robot interacts with."
  (ros-info (kvr) "spawning objects for experiments...")
  (add-bowl)
  (add-muesli)
  (add-axes)
  (add-fork)
  (add-cup)
  (add-milk)
  (add-spoon)
  (add-axes :axes2)
  (add-axes :axes3))

(defun init-full-simulation ()
  "Spawns all the objects which are necessary for the current
scenario (Meaning: Kitchen, Robot, Muesli, Milk, Cup, Bowl, Fork and 3 Axis
objects for debugging."
  (init-episode)2
  (init-bullet-world)
  (init-items)
  (init-spawn-objects-in-semantic))

(defun init-spawn-objects-in-semantic ()
  "Spawns all objects of the current Episode on their original position in the
semantic map kitchen."
  (add-bowl :edeka-red-bowl2)
  (add-muesli :koelln-muesli-knusper-honig-nuss2)
  (add-fork :fork-blue-plastic2)
  (add-cup :cup-eco-orange2)
  (add-milk :weide-milch-small2)

  ;; the offset is the same offset as in the semantic map kitchen
  (move-obj-with-offset 0.0 -3.0 "IkeaBowl" :edeka-red-bowl2)
  (move-obj-with-offset 0.0 -3.0 "KoellnMuesliKnusperHonigNuss" :koelln-muesli-knusper-honig-nuss2)
  (move-obj-with-offset 0.0 -3.0 "PlasticBlueFork" :fork-blue-plastic2)
  (move-obj-with-offset 0.0 -3.0 "CupEcoOrange" :cup-eco-orange2)
  (move-obj-with-offset 0.0 -3.0 "MilramButtermilchErdbeere" :weide-milch-small2))
