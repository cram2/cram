(in-package :le)

;; variables to store object designators in, for having the robot pick and place two objects at the same time.
(defvar ?obj-desig1 nil)
(defvar ?obj-desig2 nil)


;;------------------- from tutorials ----------------------
;;example call: (move-to-object (set-grasp-base-pose (make-poses "?PoseCameraStart")) (set-grasp-look-pose (make-poses "?PoseObjStart")))
(defun move-to-object (?grasping-base-pose ?grasping-look-pose)
  "Moves the robot into the position which the human had when interacting with an object. The robot is placed at the spot where the human was standing and is looking at the spot where the object was in Virtual Reality.
?GRASPING-BASE-POSE: The position for the robot base. Aka, where the human feet were. This transform is being calculated by the set-grasp-base-pose function
?GRASPING-LOOK-POSE: The position which the object had in Virtual Reality, and where the robot should be looking at. This position is calculated by the grasp-look-pose function.
RETURNS: Errors or a successfull movement action of the robot."
  (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
  (cpl:top-level 
    (cpl:seq
      (pp-plans::park-arms)
      ;; move the robot to location
      (exe:perform (desig:an action
                            (type going)
                            (target (desig:a location (pose ?grasping-base-pose)))))
      ;; move the head to look at location
      (exe:perform (desig:an action
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


(defun pick-up-object (?grasping-base-pose ?grasping-look-pose ?type &optional (?second nil))
  "A plan to pick up an object.
?GRASPING-BASE-POSE: The pose at which the human stood to pick up the object.
?GRASPING-LOOK-POSE: The pose at which the object was standing when picked up, and at which the robot will look for it.
?TYPE: the type of the object the robot should look for and which to pick up.
OPTIONAL ?SECOND: If set to true, the object designator will be saved in the global ?obj-desig2 variable instead of ?obj-desig1. This is a work around, to get the robot to pick up two objects, one in each hand, and to later place them.
RETURNS: The object designator of the object that has been picked up in this plan."
   (let* ((?obj-desig nil)
         (?arm (get-hand)))
    (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
      (cpl:top-level
        ;; make sure the arms are not in the way
        (pp-plans::park-arms)
        ;; move the robot to location
        (exe:perform (desig:an action
                               (type going)
                               (target (desig:a location (pose ?grasping-base-pose)))))
        ;; move the head to look at location
        (exe:perform (desig:an action
                               (type looking)
                               (target (desig:a location (pose ?grasping-look-pose)))))
        ;; see obj
        (setf ?obj-desig
              (exe:perform (desig:an action
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
        
        ;; move to obj
        ;;(pp-plans::park-arms)
        )
      (if ?second
          (setf ?obj-desig2 ?obj-desig)
          (setf ?obj-desig1 ?obj-desig))

      ?obj-desig)))



(defun place-object (?placing-base-pose ?placing-look-pose ?place-pose ?obj-desig ?second)
  "A plan to place an object which is currently in one of the robots hands.
?PLACING-BASE-POSE: The pose the robot should stand at in order to place the object. Usually the pose where the human was standing while placing the object in Virtual Reality.
?PLACING-LOOK-POSE: The pose where the robot looks at while placing the object. The same pose at which the human placed the object.
?PLACE-POSE: The pose at which the object was placed in Virtual Reality. Relative to the Kitchen Island table.
?OBJ-DESIG: The object deignator of the the object which the robot currently holds in his hand and which is to be placed.
?SECOND: If set to true, the plan will read out the global ?obj-desig2 variable instead of ?obj-desig1. This is a hacky workaround to have the robot interact and be able to pick and place two objects with one trip from one position to another. "
    (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
      (cpl:top-level
        (let* ((?arm (get-hand)))
          ;; move to obj
        (pp-plans::park-arms)
          (if ?second
          (setf ?obj-desig ?obj-desig2)
          (setf ?obj-desig ?obj-desig1))
          
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
                   (target (desig:a location (pose ?place-pose)))))
          (pp-plans::park-arms)))))

(defun pick-and-place (?grasping-base-pose ?grasping-look-pose ?placing-base-pose ?placing-look-pose ?place-pose ?type)
  "Picks up and object and places it down based on Virtual Reality data.
?GRASPING-BASE-POSE: The pose the robot should stand at, in order to be able to grasp the object.
?GRASPING-LOOK-POSE: The pose the robot is going to look at, in order to look for the object to be picked up.
?PLACING-BASE-POSE: The pose where the robot should stand in order to be able to place down the picked up object.
?PLACING-LOOK-POSE: THe pose the robot is looking at, at which he will place the object.
?PLACE-POSE: The actual placing pose of the object.
?TYPE: The type of the object the robot should interact with."
  (let* ((?obj-desig nil)
         (?arm (get-hand)))
    (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
      (cpl:top-level
        ;; make sure the arms are not in the way
        (pp-plans::park-arms)
        ;; move the robot to location
        (exe:perform (desig:an action
                               (type going)
                               (target (desig:a location (pose ?grasping-base-pose)))))
        ;; move the head to look at location
        (exe:perform (desig:an action
                               (type looking)
                               (target (desig:a location (pose ?grasping-look-pose)))))
        ;; see obj
        (setf ?obj-desig
              (exe:perform (desig:an action
                                     (type detecting)
                                     (object (desig:an object (type ?type))))))
        (print  (desig:reference
                 (desig:an action
                           (type picking-up)
                           (arm ?arm)
                           (object ?obj-desig))))

        ;;remove later
        ;; (move-object (make-poses "?PoseHandStart") 'axess3)

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






;; the basis of the plans was taken from Gayane Kazhoyan's "mobile-pick-and-place-plans" and adapted to the needs of this thesis. (link to original: https://github.com/cram2/cram/blob/master/cram_common/cram_mobile_pick_place_plans/src/pick-place-plans.lisp)

;; from mobile pick place ---------------------------------------------------
(cpl:def-cram-function pick-up (?object-designator
                                ?arm ?gripper-opening  ?grip-effort ?grasp
                                ?left-reach-poses ?right-reach-poses
                                ?left-lift-poses ?right-lift-poses)
  (cpl:par
    (roslisp:ros-info (pick-place pick-up) "Opening gripper")
    (exe:perform
     (desig:an action
               (type setting-gripper)
               (gripper ?arm)
               (position ?gripper-opening)))
    (roslisp:ros-info (pick-place pick-up) "Reaching")
    (cpl:with-failure-handling
        ((common-fail:manipulation-low-level-failure (e)
           (roslisp:ros-warn (pp-plans pick-up)
                             "Manipulation messed up: ~a~%Ignoring."
                             e)
           (return)))
      (exe:perform
       (desig:an action
                 (type reaching)
                 (left-poses ?left-reach-poses)
                 (right-poses ?right-reach-poses)))))
  (roslisp:ros-info (pick-place pick-up) "Gripping")
  (exe:perform
   (desig:an action
             (type gripping)
             (gripper ?arm)
             (effort ?grip-effort)
             (object ?object-designator)))
  (roslisp:ros-info (pick-place pick-up) "Assert grasp into knowledge base")
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-attached
     :object-name (desig:desig-prop-value ?object-designator :name)
     :arm ?arm))
  (roslisp:ros-info (pick-place pick-up) "Lifting")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pick-up)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         (return)))
    (exe:perform
     (desig:an action
               (type lifting)
               (left-poses ?left-lift-poses)
               (right-poses ?right-lift-poses)))))


; PLACING ----------------------------------------------

(cpl:def-cram-function place (?object-designator
                              ?arm
                              ?left-reach-poses ?right-reach-poses
                              ?left-put-poses ?right-put-poses
                              ?left-retract-poses ?right-retract-poses)
  (roslisp:ros-info (pick-place place) "Reaching")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pick-up)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         (return)))
    (exe:perform
     (desig:an action
               (type reaching)
               (left-poses ?left-reach-poses)
               (right-poses ?right-reach-poses))))
  (roslisp:ros-info (pick-place place) "Putting")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pick-up)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         (return)))
    (exe:perform
     (desig:an action
               (type putting)
               (left-poses ?left-put-poses)
               (right-poses ?right-put-poses))))
  (roslisp:ros-info (pick-place place) "Opening gripper")
  (exe:perform
   (desig:an action
             (type releasing)
             (object ?object-designator)
             (gripper ?arm)))
  (roslisp:ros-info (pick-place place) "Retract grasp in knowledge base")
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-detached
     :arm ?arm
     :object-name (desig:desig-prop-value ?object-designator :name)))
  (roslisp:ros-info (pick-place place) "Retracting")
  (cpl:with-failure-handling
      ((common-fail:manipulation-low-level-failure (e)
         (roslisp:ros-warn (pp-plans pick-up)
                           "Manipulation messed up: ~a~%Ignoring."
                           e)
         (return)))
    (exe:perform
     (desig:an action
               (type retracting)
               (left-poses ?left-retract-poses)
               (right-poses ?right-retract-poses)))))

(cpl:def-cram-function release (?left-or-right)
  (cpl:with-failure-handling
      ((common-fail:low-level-failure (e) ; ignore failures
         (roslisp:ros-warn (pick-and-place release) "~a" e)
         (return)))
    (exe:perform
     (desig:a motion
              (type opening)
              (gripper ?left-or-right)))
    (cram-occasions-events:on-event
     (make-instance 'cram-plan-occasions-events:robot-state-changed))))






