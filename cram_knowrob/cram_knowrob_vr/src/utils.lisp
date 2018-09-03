(in-package :kvr)

(defun object-type-filter-prolog (object-type)
  "Maps the simple name of an object, e.g. cup to the one known in the database
for that object, e.g. CupEcoOrange."
  ;;do some filtering for exact object names
    (case object-type
      (muesli (setq object-type "KoellnMuesliKnusperHonigNuss"))
      (cup (setq object-type "CupEcoOrange"))
      (bowl (setq object-type "IkeaBowl"))
      (milk (setq object-type "MilramButtermilchErdbeere"))
      (fork (setq object-type "PlasticBlueFork"))
      (spoon (setq object-type "PlasticBlueSpoon"))
      (t (ros-warn nil "Unknown object type. Known types are: muesli, cup, bowl, milk, fork, spoon"))))

(defun object-type-filter-bullet(object-type)
  "Maps the simple name of an object, e.g. cup to the one known in the database
for that object, e.g. CupEcoOrange."
  ;;do some filtering for exact object names
    (case object-type
      (muesli (setq object-type :koelln-muesli-knusper-honig-nuss))
      (cup (setq object-type :cup-eco-orange))
      (bowl (setq object-type :edeka-red-bowl))
      (milk (setq object-type :weide-milch-small))
      (fork (setq object-type :fork-blue-plastic))
      (spoon (setq object-type :spoon-blue-plastic))
      (t (ros-warn nil "Unknown object type. Known types are: muesli, cup, bowl, milk, fork, spoon"))))

(defun object-type-fixer(object-type)
  (case object-type
    (:weide-milch-small (setq object-type :milram-buttermilch-erdbeere))
    (:edeka-red-bowl (setq object-type :ikea-bowl))
    (:fork-blue-plastic (setq object-type :plastic-blue-fork))
    (:spoon-blue-plastic (setq object-type :plastic-blue-spoon))
    (t object-type)))


;; killing object from bullet world. ex: 'axes
(defun kill-obj (object)
  "Removes an object from the bullet world."
  (btr-utils:kill-object object)) 

;; all kind of transform utils

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

(defun make-transf-manually ()
  "Calculating the grasping transform manually for debugging puposes."
  ;; inverse of map to object, therefore object to map
  ;; object T map
  (cram-tf:transform-stamped-inv
   (cl-tf:transform->stamped-transform "map"
                                       "ba_muesli"
                                       0.0
                                       (get-object-location-at-start-by-object-type
                                        (object-type-filter-prolog 'muesli))))

  ;; map T unreal hand
   (cl-tf:transform->transform-stamped "map"
                                       "hand"
                                       0.0
                                       (get-hand-location-at-start-by-object-type
                                        (object-type-filter-prolog 'muesli)))

  ;; unreal hand T standard gripper
  (human-to-robot-hand-transform))

;; look up transfrom from tf. ex: "l_wrist_roll_link" "l_gripper_l_finger_tip_link" 
(defun lookup-tf-transform (parent_frame child_frame)
  "Looks up the tf transform."
  (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment 
    (cram-tf::lookup-transform cram-tf::*transformer* parent_frame child_frame)))


(defun alternative-demo (object)
  "Moves the object and robot to their respective locations at the beginning of
the episode. . "
  (move-object (get-object-location-at-start-by-object-type
                (object-type-filter-prolog object))
               (object-type-filter-bullet object)))



(defun apply-bullet-transform (transform)
  "Applies the transform of the VR to bullet world and the rotation."
  (cl-tf:transform*
   (cl-tf:make-transform (cl-tf:make-3d-vector -2.65 -0.7 0.0)
                         (cl-tf:axis-angle->quaternion
                          (cl-tf:make-3d-vector 0 0 1)
                          pi)) 
   transform))

(defun apply-bullet-rotation (transform)
  "Applies only the offset rotation between the VR and the bullet world."
  (cl-tf:transform*
   (cl-tf:make-transform (cl-tf:make-3d-vector 0.0 0.0 0.0)
                         (cl-tf:axis-angle->quaternion
                          (cl-tf:make-3d-vector 0 0 1)
                          pi)) 
   transform))



(defun apply-rotation (transform)
  (cl-tf:transform*
   (cl-tf:make-transform (cl-tf:make-3d-vector 0.0 0.0 0.0)
                         (cl-tf:axis-angle->quaternion
                          (cl-tf:make-3d-vector 0 0 1)
                          (/ pi 2))) 
   transform))



;;- more bullet checking/utils stuff
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

                                        ;splits the list of the pose into pose and quaternion
                                        ;for specific usecase test function



;;; evaluation
(defun move-obj-with-offset (x-offset y-offset object)
  "Moves an object to a given x y offset from it's starting position. "
  (move-object 
   (cl-tf:make-transform 
    (cl-tf:make-3d-vector
     (+
      x-offset
      (cl-tf:x
       (cl-tf:translation
        (get-object-location-at-start-by-object-type
         (object-type-filter-prolog object)))))
     (+
      y-offset
      (cl-tf:y
       (cl-tf:translation
        (get-object-location-at-start-by-object-type
         (object-type-filter-prolog object)))))
     (cl-tf:z
      (cl-tf:translation 
       (get-object-location-at-start-by-object-type
        (object-type-filter-prolog object)))))
    (cl-tf:rotation
     (get-object-location-at-start-by-object-type
      (object-type-filter-prolog object))))
   (object-type-filter-bullet object)))
;;--------------------------------------------------------------




(defun add-pos-offset-to-transform (transform x y z)
  "Adds a given offset to a given transform."
  (cl-tf:make-transform
     (cl-tf:make-3d-vector
      (+ x (cl-tf:x (cl-tf:translation transform)))
      (+ y (cl-tf:y (cl-tf:translation transform)))
      (+ z (cl-tf:z (cl-tf:translation transform))))
     (cl-tf:rotation transform)))

(defun move-head (pose)
  "Moves the head of the robot to the given POSE."
  (prolog:prolog `(and (btr:bullet-world ?world)
                       (cram-robot-interfaces:robot ?robot )
                       (btr:head-pointing-at ?world ?robot ,pose))))

;;if in back 'cereal-5
(defun is-in-view (name-of-object)
  "Checks if the object is in view of the robot.
NAME-OF-OBJECT: The name of the object instance, for which it should be checked if it is still in view. "
  (prolog:prolog `(and (btr:bullet-world ?world)
                              (cram-robot-interfaces:robot ?robot)
                              (btr:visible ?world ?robot ,name-of-object))))

; moves the given object to the given pose.
; usage example: (move-object (pose-lists-parser '|?PoseObjEnd|))
(defun move-object (transform obj)
  "Moves an object to the desired transform.
TRANSFORM: The transform describing the position to which an object should be moved.
OBJ: The object which is supposed to be moved."
  (let* ((pose (cl-tf:transform->pose transform)))
    (prolog:prolog `(and (btr:bullet-world ?world)
                         (assert (btr:object-pose ?world ,obj ,pose))))))


;;(make-poses "?PoseCameraStart")
(defun move-robot (transform)
  "Moves the robot to a given position which is described by the given transform."
  ;; make the transform a viable robot position
  (let* ((pose (cl-tf:transform->pose  (remove-z transform )))
         (quaternion (cl-tf:orientation pose))
         (x nil)
         (y nil))
    ;; make quaternion
    ;; make into matrix, get x and y values
    (setq x (aref (cl-tf:quaternion->matrix quaternion) 0 2))
    (setq y (aref (cl-tf:quaternion->matrix quaternion) 1 2))
    (setq quaternion (cl-tf:axis-angle->quaternion (cl-tf:make-3d-vector 0 0 1) (atan y x)))
    
    (setq pose (cl-tf:make-pose
                (cl-tf:origin pose)
                quaternion))
    
    (prolog:prolog `(and (btr:bullet-world ?world)
                         (assert (btr:object-pose ?world cram-pr2-description:pr2 ,pose))))))



 
(defun place-offset-transform ()
 "Creates a transform which describes and offset for placing an object. "
  (let ()
      (cl-tf:make-transform
       (cl-tf:make-3d-vector 0.0 0.2 0.0)
       (cl-tf:make-identity-rotation))))


(defun parse-str (str)
  "parses the output from knowrob to a proper string which prolog can use."
  (concatenate 'string "'"  (remove #\' (string str)) "'"))



(defun transform-to-pose-stamped (transform)
  "Takes a transform and returnes a pose stamped without any changes."
  (cl-tf:make-pose-stamped
   "map"
   0.0
   (cl-tf:translation transform)
   (cl-tf:rotation transform)))


;;;------
(defun move-toso-up (?angle)
  "Moves up the torso of the robot by the given angle. The robot should do this automatically during picking and placing actions, but for debugging purposes this can be useful. 
?ANGLE: Angle, by which the robot should move up his torso."
   (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
           (cpl:top-level
             (exe:perform
              (desig:a motion (type moving-torso) (joint-angle ?angle))))))


