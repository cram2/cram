;;; Contains functions which are usefull for debugging any errors within the code

(in-package :kvr)

(defun reset-simulation ()
  "Resets the simulation and belief state. Re-spawns the objects at their
initial position."
  (cram-occasions-events:clear-belief)
  (spawn-urdf-items))

;; killing object from bullet world. ex: 'axes
(defun kill-obj (object)
  "Removes an object from the bullet world."
  (btr-utils:kill-object object))

(defun make-transform-hand-std-pr2 (object)
  "Make a transform from human hand to the standart pr2"
  (cl-tf:transform* (get-hand-location-at-start-by-object-type object)
                    (human-to-robot-hand-transform)
                    cram-pr2-description::*standard-to-pr2-gripper-transform*))

(defun get-robot-in-map-pose ()
  "Get the position of the robot within the map frame."
  (cl-tf:transform->transform-stamped "map" "base_footprint" 0.0
                                      (cl-tf:pose->transform
                                       (btr:pose
                                        (btr:get-robot-object)))))

;; look up transfrom from tf. ex: "l_wrist_roll_link" "l_gripper_l_finger_tip_link" 
(defun lookup-tf-transform (parent_frame child_frame)
  "Looks up the tf transform."
  (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment 
    (cram-tf::lookup-transform cram-tf::*transformer* parent_frame child_frame)))


;;- more bullet checking/utils functions
(defun check-obj-in-world (object-name)
  "Check if the object is in the current world."
  (btr:object btr:*current-bullet-world* object-name))


(defun run-simulation-physics ()
  "simulates the world for a second."
  (prolog:prolog '(and (btr:bullet-world ?world)
                   (btr:simulate ?world 10))))

(defun check-stability-of-sim ()
  "checks if the simulation is stable, or if run for a longer time, some objects would change their position. If the result is anything but NIL, the world is stable."
  (prolog:prolog '(and (btr:bullet-world ?world)
                   (btr:simulate ?world 100))))


(defun set-axes (object)
  "Sets the axes to the robots hands and to the position of the grasping pose of the human. "
  (let* ((transf_r)
         (transf_l))
    (setq transf_r (car
                    (cram-projection::projection-environment-result-result
                     (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment 
                       (cram-tf::lookup-transform cram-tf::*transformer* "map" "r_gripper_r_finger_tip_link" )))))
    (setq transf_l (car
                    (cram-projection::projection-environment-result-result
                     (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment 
                       (cram-tf::lookup-transform cram-tf::*transformer* "map" "l_gripper_l_finger_tip_link" )))))
    
    (setq transf_r
          (cl-tf:make-transform
           (cl-tf:translation transf_r)
           (cl-tf:rotation transf_r)))
    (move-object transf_r 'axes)
    (setq transf_l
          (cl-tf:make-transform
           (cl-tf:translation transf_l)
           (cl-tf:rotation transf_l)))
    
    (move-object transf_r 'axes)
    (move-object transf_l 'axes2)
    (move-object
     (get-hand-location-at-start-by-object-type
      (object-type-filter-prolog object)) 'axes3)))


(defun reset-robot ()
  (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
    (cpl:top-level
      (cpl:seq
        (exe:perform
         (desig:an action
                   (type positioning-arm)
                   (left-configuration park)
                   (right-configuration park)))))))


;;if in back 'cereal-5
(defun is-in-view (name-of-object)
  "Checks if the object is in view of the robot.
NAME-OF-OBJECT: The name of the object instance, for which it should be checked if it is still in view. "
  (prolog:prolog `(and (btr:bullet-world ?world)
                              (cram-robot-interfaces:robot ?robot)
                              (btr:visible ?world ?robot ,name-of-object))))
