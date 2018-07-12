(in-package :kvr)
;;------------------- from tutorials ----------------------
;;example call: (move-to-object (set-grasp-base-pose
;; (make-poses "?PoseCameraStart")) (set-grasp-look-pose (make-poses "?PoseObjStart")))
(defun move-to-object (?grasping-base-pose ?grasping-look-pose)
  "Moves the robot into the position which the human had when interacting
with an object. The robot is placed at the spot where the human was standing and
is looking at the spot where the object was in Virtual Reality.
?GRASPING-BASE-POSE: The position for the robot base. Aka, where the human feet
were. This transform is being calculated by the set-grasp-base-pose function
?GRASPING-LOOK-POSE: The position which the object had in Virtual Reality, and
where the robot should be looking at. This position is calculated by the
grasp-look-pose function.
RETURNS: Errors or a successfull movement action of the robot."
  (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
  (cpl:top-level 
    (cpl:seq
      (pp-plans::park-arms)
      ;; move the robot to location
      (exe:perform
       (desig:an action
                 (type going)
                 (target (desig:a location (pose ?grasping-base-pose)))))
      ;; move the head to look at location
      (exe:perform
       (desig:an action
                 (type looking)
                 (target (desig:a location (pose ?grasping-look-pose)))))))))


;; ---------------------------------------------------------------------------------------
;; pick up an object function       ------------------------------------------------------
(defun pick-up-obj (?type)
  "Picks up an object of the given type.
?TYPE: The type of the object that is to be picked up.
RETURNS: Errors or a successfull movement action of the robot."
  (let* ((?obj-desig nil)
         (?arm (get-hand)))
    (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
      (cpl:top-level
        (setf ?obj-desig
              (exe:perform (desig:an action
                                     (type detecting)
                                     (object (desig:an object (type ?type))))))
        ; TODO replace with ros-warn or remove complety after testing.
        (print  (desig:reference
                 (desig:an action
                           (type picking-up)
                           (arm ?arm)
                           (object ?obj-desig))))
        (exe:perform 
         (desig:an action
                   (type picking-up)
                   (arm ?arm)
                   (object ?obj-desig)))))))


(defun pick-up-object (?grasping-base-pose ?grasping-look-pose ?type)
  "A plan to pick up an object.
?GRASPING-BASE-POSE: The pose at which the human stood to pick up the object.
?GRASPING-LOOK-POSE: The pose at which the object was standing when picked up,
and at which the robot will look for it.
?TYPE: the type of the object the robot should look for and which to pick up.
RETURNS: The object designator of the object that has been picked up in this plan."
  (let* ((?obj-desig nil)
         (?arm (get-hand)))
    (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
      (cpl:top-level
        ; make sure the arms are not in the way
        (pp-plans::park-arms)
        ; move the robot to location
        (exe:perform
         (desig:an action
                   (type going)
                   (target
                    (desig:a
                     location
                     (pose ?grasping-base-pose)))))
        ; move the head to look at location
        (exe:perform
         (desig:an action
                   (type looking)
                   (target (desig:a location (pose ?grasping-look-pose)))))
        ; see obj
        (setf ?obj-desig
              (exe:perform
               (desig:an action
                         (type detecting)
                         (object (desig:an
                                  object
                                  (type ?type))))))
        
        (print  (desig:reference
                 (desig:an action
                           (type picking-up)
                           (arm ?arm)
                           (object ?obj-desig))))

        ;; pick up obj
        (exe:perform 
         (desig:an action
                   (type picking-up)
                   (arm ?arm)
                   (object ?obj-desig)))
        (cram-occasions-events:on-event
         (make-instance 'cpoe:object-attached
           :object-name (desig:desig-prop-value ?obj-desig :name)
           :arm ?arm))
        
        ; move to obj
        ; (pp-plans::park-arms)
        ))))



(defun place-object (?placing-base-pose ?placing-look-pose ?place-pose ?obj-desig)
  "A plan to place an object which is currently in one of the robots hands.
?PLACING-BASE-POSE: The pose the robot should stand at in order to place the
object. Usually the pose where the human was standing while placing the object
in Virtual Reality.
?PLACING-LOOK-POSE: The pose where the robot looks at while placing the object.
The same pose at which the human placed the object.
?PLACE-POSE: The pose at which the object was placed in Virtual Reality.
Relative to the Kitchen Island table.
?OBJ-DESIG: The object deignator of the the object which the robot currently
holds in his hand and which is to be placed."
  (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
    (cpl:top-level
      (let* ((?arm (get-hand)))
        ; move to obj
        (pp-plans::park-arms)         
        ; move the robot to location
        (exe:perform
         (desig:an action
                   (type going)
                   (target (desig:a location (pose ?placing-base-pose)))))
        ; move the head to look at location     
        (exe:perform
         (desig:an action
                   (type looking)
                   (target (desig:a
                            location
                            (pose ?placing-look-pose)))))
        ; place obj
        (cpl:sleep 1.0)
          
        (exe:perform
         (desig:an action
                   (type placing)
                   (arm ?arm)
                   (object ?obj-desig)
                   (target (desig:a location (pose ?place-pose)))))
        (pp-plans::park-arms)))))

(defun pick-and-place (?grasping-base-pose ?grasping-look-pose ?placing-base-pose ?placing-look-pose ?place-pose ?type)
  "Picks up and object and places it down based on Virtual Reality data.
?GRASPING-BASE-POSE: The pose the robot should stand at, in order to be able to
grasp the object.
?GRASPING-LOOK-POSE: The pose the robot is going to look at, in order to look
for the object to be picked up.
?PLACING-BASE-POSE: The pose where the robot should stand in order to be able
to place down the picked up object.
?PLACING-LOOK-POSE: THe pose the robot is looking at, at which he will place
the object.
?PLACE-POSE: The actual placing pose of the object.
?TYPE: The type of the object the robot should interact with."
  (let* ((?obj-desig nil)
         (?arm (get-hand)))
    (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
      (cpl:top-level
        ;; make sure the arms are not in the way
        (pp-plans::park-arms)
        ;; move the robot to location
        (exe:perform
         (desig:an action
                   (type going)
                   (target (desig:a location (pose ?grasping-base-pose)))))
        ;; move the head to look at location
        (exe:perform
         (desig:an action
                   (type looking)
                   (target (desig:a location (pose ?grasping-look-pose)))))
        ;; see obj
        (setf ?obj-desig
              (exe:perform
               (desig:an action
                         (type detecting)
                         (object (desig:an object (type ?type))))))
        (print  (desig:reference
                 (desig:an action
                           (type picking-up)
                           (arm ?arm)
                           (object ?obj-desig))))

        ;; pick up obj
        (exe:perform 
         (desig:an action
                   (type picking-up)
                   (arm ?arm)
                   (object ?obj-desig)))
        (cram-occasions-events:on-event
         (make-instance 'cpoe:object-attached
           :object-name (desig:desig-prop-value ?obj-desig :name)
           :arm ?arm))
      
        (print (desig:a location (pose ?place-pose)))

        ;; move to obj
        (pp-plans::park-arms)
        ;; move the robot to location
        (exe:perform (desig:an action
                               (type going)
                               (target (desig:a location (pose ?placing-base-pose)))))
        ;; move the head to look at location
          
          
        (exe:perform (desig:an action
                               (type looking)
                               (target (desig:a location (pose ?placing-look-pose)))))
        ;; place obj
        (cpl:sleep 1.0)  
        (exe:perform
         (desig:an action
                   (type placing)
                   (arm ?arm)
                   (object ?obj-desig)
                   (target (desig:a location (pose ?place-pose)))))))))

