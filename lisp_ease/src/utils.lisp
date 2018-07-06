(in-package :le)
;; usefull bullet world infos

;; get-info of an object
(defun get-info (infoObj)
  "returns contents of a specific field from the read out data."
  (cut:var-value (intern infoObj)  *poses-list*))

;; returns the hand used in the curretnly loaded episode
(defun get-hand ()
  "returns which hand was used to interact with the object"
  (if (search "Left" (string (get-info "?HandInstShortName")))
      :left
      (if (search "Right" (string (get-info "?HandInstShortName")))
          :right
          NIL)))


;; killing object from bullet world. ex: 'axes
(defun kill-obj (object)
  "Removes an object from the bullet world."
  (btr-utils:kill-object object)) 

;; all kind of transform utils

(defun make-transform-hand-std-pr2 ()
  "Make a transform from human hand to the standart pr2"
  (cl-tf:transform* (make-poses "?PoseHandStart")
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
  (cram-tf:transform-stamped-inv  (cl-tf:transform->stamped-transform "map" "ba_muesli"  0.0 (make-poses "?PoseObjStart")))

  ;; map T unreal hand
  (cl-tf:transform->transform-stamped "map" "hand" 0.0 (make-poses "?PoseHandStart"))

  ;; unreal hand T standard gripper
  (human-to-robot-hand-transform)
  )

;; look up transfrom from tf. ex: "l_wrist_roll_link" "l_gripper_l_finger_tip_link" 
(defun lookup-tf-transform (parent_frame child_frame)
  "Looks up the tf transform."
  (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment 
    (cram-tf::lookup-transform cram-tf::*transformer* parent_frame child_frame)))


(defun alternative-demo (object)
  "Moves the object and robot to their respective locations at the beginning of the episode. . "
  (move-object  (make-poses "?PoseObjStart") object)
  ;; (move-to-object (set-grasp-base-pose (make-poses "?PoseCameraStart")) (set-grasp-look-pose (make-poses "?PoseObjStart")))
  )


;; new now! maybe rename...
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


(defun start-sim ()
  "simulates the world for a second."
  (prolog:prolog '(and (btr:bullet-world ?world)
                   (btr:simulate ?world 10))))

(defun check-stability-of-sim ()
  "checks if the simulation is stable, or if run for a longer time, some objects would change their position. If the result is anything but NIL, the world is stable."
  (prolog:prolog '(and (btr:bullet-world ?world)
                   (btr:simulate ?world 100))))



(defun set-axes ()
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
    (move-object (make-poses "?PoseHandStart") 'axes3)))

                                        ;splits the list of the pose into pose and quaternion
                                        ;for specific usecase test function

;; for spawning boxes on the edges of the table
;; or generally to be used when more then one object of one kind needs to be spawned somewhere. 
(defun table-calibration (max)
  "Function used to calibrate offset between the open ease and the bullet world. "
  (let (temp-name)
    (get-grasp-something-poses)
    (dotimes (c max )
      (get-next-obj-poses c)
      (setf temp-name (intern (concatenate 'string  "koelln-muesli-knusper-honig-nuss" (write-to-string c))))
      (add-muesli temp-name)
      (format nil "added: ~D out of: ~D ~%" c max)
      (move-object (make-poses "?PoseCameraStart") temp-name)))
)


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
          (make-poses "?PoseObjStart"))))
       (+
        y-offset
        (cl-tf:y
         (cl-tf:translation
          (make-poses "?PoseObjStart"))))
       (cl-tf:z
        (cl-tf:translation 
         (make-poses "?PoseObjStart"))))
      (cl-tf:rotation
       (make-poses "?PoseObjStart"))) object))
;;--------------------------------------------------------------




(defun add-pos-offset-to-transform (transform x y z)
  "Adds a given offset to a given transform."
  (cl-tf:make-transform
     (cl-tf:make-3d-vector
      (+ x (cl-tf:x (cl-tf:translation transform)))
      (+ y (cl-tf:y (cl-tf:translation transform)))
      (+ z (cl-tf:z (cl-tf:translation transform))))
     (cl-tf:rotation transform)))

;; name "?PoseObjStart"
(defun make-poses-without-transform (name &optional (poses-list *poses-list*))
  "Makes poses without the application of the transform, which is needed to display them in the bullet world correctly. This is used to demonstrate the offset between the two worlds. "
  (make-pose (cut:var-value (intern name) poses-list)))


(defun make-poses-with-quaternion (name &optional (poses-list *poses-list*))
  "Makes poses with the fixed quaternion."
  (quaternion-w-flip
   (make-pose (cut:var-value (intern name) poses-list))))



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


