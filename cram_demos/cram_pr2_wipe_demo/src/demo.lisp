(in-package :pr2-wipe)



(defparameter *base-pose*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -0.00d0 1.0d0 0.0d0)
   (cl-transforms:make-quaternion 0.0d0 0.0d0 1.0d0 0.0d0)))

(defparameter *base-pose-surface*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -0.10d0 2.0d0 0.0d0)
   (cl-transforms:make-quaternion 0.0d0 0.0d0 1.0d0 0.0d0)))

(defparameter *base-pose-surface-kitchen-handle*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector 0.6d0 1.0d0 0.0d0)
   (cl-transforms:axis-angle->quaternion
    (cl-tf:make-3d-vector 0 0 1)
    0.0)))

(defparameter *base-pose-surface-kitchen*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector 0.7d0 0.8d0 0.0d0)
   (cl-transforms:axis-angle->quaternion
    (cl-tf:make-3d-vector 0 0 1)
    0.0)))

(defparameter *base-pose-table*
    (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -2.4d0 0.30d0 0.0d0)
   (cl-transforms:make-quaternion 0.0d0 0.0d0 1.0d0 0.0d0)))

(defparameter *downward-look-coordinate*
  (cl-transforms-stamped:make-pose-stamped
   "base_footprint" 0.0
   (cl-transforms:make-3d-vector 0.75335d0 0.076d0 0.758d0)
   (cl-transforms:make-quaternion 0.0d0 0.0d0 0.8d0 0.0d0)))

(defparameter *downward-look-coordinate-vertical*
  (cl-transforms-stamped:make-pose-stamped
   "base_footprint" 0.0
   (cl-transforms:make-3d-vector 0.75335d0 0.076d0 0.758d0)
   (cl-transforms:make-quaternion 0.0d0 0.0d0 1.0d0 0.0d0)))

(defparameter *base-pose-fridge*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector 0.55d0 -1.0d0 0.0d0)
   (cl-transforms:axis-angle->quaternion
    (cl-tf:make-3d-vector 0 0 1)
    0.0)))

(defparameter *base-pose-handle*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector 0.55d0 1.6d0 0.0d0)
   (cl-transforms:axis-angle->quaternion
    (cl-tf:make-3d-vector 0 0 1)
    0.0)))

(defparameter *base-pose-plate*
  (cl-transforms-stamped:make-pose-stamped
   "map" 0.0
   (cl-transforms:make-3d-vector -2.1 0.75d0 0.00d0)
   (cl-transforms:axis-angle->quaternion
    (cl-tf:make-3d-vector 0 0 1)
    0.0)))



;;Plan -> Function -> Designator -> Trajectory 

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;PLAN;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun start-environment ()
  (roslisp-utilities:startup-ros)
  (spawn-sponge)
  (btr::add-vis-axis-object 'surface-4)
  )

(defun execute-demo ()
  (roslisp-utilities:startup-ros)
  (spawn-sponge)
  (wipe-demo)
  )


(defun spawn-sponge ()
  ;;Placeholder for sponge
  ;; (btr-utils:spawn-object 'bowl-1 :bowl
  ;;                                :pose (cl-transforms:make-pose
  ;;                                       (cl-tf:make-3d-vector -0.8 1.3 0.9)
  ;;                                       (cl-tf:make-identity-rotation)))
  (btr-utils:spawn-object 'plate-1 :plate
                                 :pose (cl-transforms:make-pose
                                        (cl-tf:make-3d-vector -1.4 0.8 0.86)
                                        (cl-tf:make-identity-rotation)))
  
  (btr-utils:spawn-object 'knife-1 :knife
                                 :pose (cl-transforms:make-pose
                                        (cl-tf:make-3d-vector -0.8 1 0.88)
                                        (cl-tf:axis-angle->quaternion
                                         (cl-tf:make-3d-vector 0 0 -1)
                                         -3.0)))
  




  


  
  (let* ((world btr:*current-bullet-world*))
         
         ;; (plane-shape (make-instance 'static-plane-shape
         ;;                             :normal (cl-transforms:make-3d-vector 0 0 1)
         ;;                             :constant 0))
         ;; (box-shape (make-instance 'box-shape
         ;;                           :half-extents (cl-transforms:make-3d-vector 0.1 0.3 0.1)))
         ;; (plane-body (make-instance 'rigid-body
         ;;                            :collision-shape plane-shape))
         
         ;; (box-body (make-instance 'rigid-body
         ;;                             :collision-shape box-shape
         ;;                             :mass 0.1
         ;;                             :pose (cl-transforms::make-pose
         ;;                                    (cl-transforms:make-3d-vector -0.9 2 0.78)
         ;;                                    (cl-transforms:axis-angle->quaternion
         ;;                                     (cl-transforms:make-3d-vector 0.5 0.5 1)
         ;;                                     0.0))))
         )
         
    ;;(add-rigid-body world plane-body))
    ;;(add-rigid-body world box-body))

    (btr::add-object btr:*current-bullet-world* :colored-box 'surface-1 (cl-transforms::make-pose
                                            (cl-transforms:make-3d-vector -0.9 2.1 0.87)
                                            (cl-transforms:axis-angle->quaternion
                                             (cl-transforms:make-3d-vector 0.5 0.5 1)
                                             0.0)) :mass 0.0001 :size '(0.1 0.3 0.01)  :color '(0 10 0 0.5))

(btr::add-object btr:*current-bullet-world* :colored-box 'surface-2 (cl-transforms::make-pose
                                            (cl-transforms:make-3d-vector 1.18 1.48 0.95)
                                            (cl-transforms:axis-angle->quaternion
                                             (cl-transforms:make-3d-vector 0.0 0.0 1)
                                             3.15)) :mass 0.0001 :size '(0.01 0.01 0.25)  :color '(0 10 0 0.5))

  (btr::add-object btr:*current-bullet-world* :colored-box 'surface-3 (cl-transforms::make-pose
                                            (cl-transforms:make-3d-vector 1.4 0.8 0.87)
                                            (cl-transforms:axis-angle->quaternion
                                             (cl-transforms:make-3d-vector -0.5 0.5 1)
                                             0.0)) :mass 0.0001 :size '(0.08 0.2 0.01)  :color '(0 10 0 0.5))


   (btr::add-object btr:*current-bullet-world* :colored-box 'surface-4 (cl-transforms::make-pose
                                            (cl-transforms:make-3d-vector 1.25 -1 0.95)
                                            (cl-transforms:axis-angle->quaternion
                                             (cl-transforms:make-3d-vector 0 0 1)
                                             3.15)) :mass 0.0001 :size '(0.01 0.2 0.25)  :color '(0 10 0 0.5))

  
     (btr::add-object btr:*current-bullet-world* :colored-box 'surface-5 (cl-transforms::make-pose
                                            (cl-transforms:make-3d-vector -3.20 0.3 0.75)
                                            (cl-transforms:axis-angle->quaternion
                                             (cl-transforms:make-3d-vector 0.5 0.5 1)
                                             0.0)) :mass 0.0001 :size '(0.15 0.4 0.01)  :color '(0 10 0 0.5))

      (btr::add-object btr:*current-bullet-world* :colored-box 'surface-6 (cl-transforms::make-pose
                                            (cl-transforms:make-3d-vector -1.4 0.8 0.88)
                                            (cl-transforms:axis-angle->quaternion
                                             (cl-transforms:make-3d-vector 0.5 0.5 1)
                                             0.0)) :mass 0.0001 :size '(0.05 0.075 0.01)  :color '(0 10 0 0.5))

    





  )
  
 


  
  

(defun wipe-demo()
    (urdf-proj:with-simulated-robot
      (cpl:par



        ;;Picking up Sponge
        (let ((?navigation-goal *base-pose*))
          (exe:perform (desig:an action
                                 (type going)
                                 (target (desig:a location 
                                                  (pose ?navigation-goal))))))
        
        (let ((?looking-direction *downward-look-coordinate*))
          (dotimes (i 3)
            (exe:perform (desig:an action 
                                   (type looking)
                                   (target (desig:a location 
                                                  (pose ?looking-direction)))))))

        (cpl:with-retry-counters ((looking-retry 3))
          (cpl:with-failure-handling
              ((common-fail:low-level-failure
                   (e)
                 (declare (ignore e))
                 (cpl:do-retry looking-retry
                   (cpl:retry))
                 (roslisp:ros-warn (wipe-demo looking-fail)
                                 "~%No more retries~%")))
        (let ((?grasping-arm (list :left))
              (?perceived-object (urdf-proj::detect (desig:an object (type :knife)))))
          

          ;; (exe:perform (desig:an action
          ;;                        (type picking-up)
          ;;                          (grasp :front)
          ;;                          (object ?perceived-object)))
          ))))

  



        





 


      

  
     
   
        ;;Sponge in hand, moving to the surface to wipe.
        (let ((?navigation-goal-surface *base-pose-surface*))
          (exe:perform (desig:an action
                                 (type going)
                                 (target (desig:a location 
                                                  (pose ?navigation-goal-surface))))))


    ;;Looking at the surface to wipe
     (let* ((?looking-direction *downward-look-coordinate*))
            (exe:perform (desig:an action 
                                   (type looking)
                                   (target (desig:a location 
                                                    (pose ?looking-direction))))))


                    ;;Wiping the surface

(cpl:with-retry-counters ((looking-retry 3))
          (cpl:with-failure-handling
              ((common-fail:low-level-failure
                   (e)
                 (declare (ignore e))
                 (cpl:do-retry looking-retry
                   (cpl:retry))
                 (roslisp:ros-warn (wipe-demo looking-fail)
                                   "~%No more retries~%")))

            (let* ((?surface-to-wipe (urdf-proj::detect (desig:an object (type :colored-box))))
                   (?arm-for-wiping :left)
                   (?collision-mode nil))
      
      (exe:perform
       (desig:an action 
                 (type wiping)
                 (grasp :countertop)
                 (arm ?arm-for-wiping)
                 (surface ?surface-to-wipe)
                 (collision-mode ?collision-mode)
                 )))))
   


 

    (pp-plans::park-arms :left-arm T)

      (let ((?navigation-goal *base-pose-surface-kitchen*
                              ))
          (exe:perform (desig:an action
                                 (type going)
                                 (target (desig:a location 
                                                  (pose ?navigation-goal))))))

           (let* ((?looking-direction *downward-look-coordinate*))
            (exe:perform (desig:an action 
                                   (type looking)
                                   (target (desig:a location 
                                                    (pose ?looking-direction))))))



    (cpl:with-retry-counters ((looking-retry 3))
          (cpl:with-failure-handling
              ((common-fail:low-level-failure
                   (e)
                 (declare (ignore e))
                 (cpl:do-retry looking-retry
                   (cpl:retry))
                 (roslisp:ros-warn (wipe-demo looking-fail)
                                   "~%No more retries~%")))

            (let* ((?surface-to-wipe (urdf-proj::detect (desig:an object (type :colored-box))))
                   (?arm-for-wiping :left)
                   (?collision-mode nil))
      
      (exe:perform
       (desig:an action 
                 (type wiping)
                 (grasp :countertop)
                 (arm ?arm-for-wiping)
                 (surface ?surface-to-wipe)
                 (collision-mode ?collision-mode)
                 )))))



    (pp-plans::park-arms :left-arm T)

      (let ((?navigation-goal *base-pose-table*
                              ))
          (exe:perform (desig:an action
                                 (type going)
                                 (target (desig:a location 
                                                  (pose ?navigation-goal))))))

           (let* ((?looking-direction *downward-look-coordinate*))
            (exe:perform (desig:an action 
                                   (type looking)
                                   (target (desig:a location 
                                                    (pose ?looking-direction))))))



    (cpl:with-retry-counters ((looking-retry 3))
          (cpl:with-failure-handling
              ((common-fail:low-level-failure
                   (e)
                 (declare (ignore e))
                 (cpl:do-retry looking-retry
                   (cpl:retry))
                 (roslisp:ros-warn (wipe-demo looking-fail)
                                   "~%No more retries~%")))

            (let* ((?surface-to-wipe (urdf-proj::detect (desig:an object (type :colored-box))))
                   (?arm-for-wiping :left)
                   (?collision-mode nil))
      
      (exe:perform
       (desig:an action 
                 (type wiping)
                 (grasp :countertop)
                 (arm ?arm-for-wiping)
                 (surface ?surface-to-wipe)
                 (collision-mode ?collision-mode)
                 )))))


    
    (pp-plans::park-arms :left-arm T)

      (let ((?navigation-goal *base-pose-fridge*
                              ))
          (exe:perform (desig:an action
                                 (type going)
                                 (target (desig:a location 
                                                  (pose ?navigation-goal))))))

           (let* ((?looking-direction *downward-look-coordinate*))
            (exe:perform (desig:an action 
                                   (type looking)
                                   (target (desig:a location 
                                                    (pose ?looking-direction))))))



    (cpl:with-retry-counters ((looking-retry 3))
          (cpl:with-failure-handling
              ((common-fail:low-level-failure
                   (e)
                 (declare (ignore e))
                 (cpl:do-retry looking-retry
                   (cpl:retry))
                 (roslisp:ros-warn (wipe-demo looking-fail)
                                   "~%No more retries~%")))

            (let* ((?surface-to-wipe (urdf-proj::detect (desig:an object (type :colored-box))))
                   (?arm-for-wiping :left)
                   (?collision-mode nil))
      
      (exe:perform
       (desig:an action 
                 (type wiping)
                 (grasp :vertical)
                 (arm ?arm-for-wiping)
                 (surface ?surface-to-wipe)
                 (collision-mode ?collision-mode)
                 )))))


      (pp-plans::park-arms :left-arm T)

      (let ((?navigation-goal *base-pose-handle*
                              ))
          (exe:perform (desig:an action
                                 (type going)
                                 (target (desig:a location 
                                                  (pose ?navigation-goal))))))

           (let* ((?looking-direction *downward-look-coordinate*))
            (exe:perform (desig:an action 
                                   (type looking)
                                   (target (desig:a location 
                                                    (pose ?looking-direction))))))



    (cpl:with-retry-counters ((looking-retry 3))
          (cpl:with-failure-handling
              ((common-fail:low-level-failure
                   (e)
                 (declare (ignore e))
                 (cpl:do-retry looking-retry
                   (cpl:retry))
                 (roslisp:ros-warn (wipe-demo looking-fail)
                                   "~%No more retries~%")))

            (let* ((?surface-to-wipe (urdf-proj::detect (desig:an object (type :colored-box))))
                   (?arm-for-wiping :left)
                   (?collision-mode nil))
      
      (exe:perform
       (desig:an action 
                 (type wiping)
                 (grasp :vertical)
                 (arm ?arm-for-wiping)
                 (surface ?surface-to-wipe)
                 (collision-mode ?collision-mode)
                 )))))





         (pp-plans::park-arms :left-arm T)

      (let ((?navigation-goal *base-pose*
                              ))
          (exe:perform (desig:an action
                                 (type going)
                                 (target (desig:a location 
                                                  (pose ?navigation-goal))))))

           (let* ((?looking-direction *downward-look-coordinate*))
            (exe:perform (desig:an action 
                                   (type looking)
                                   (target (desig:a location 
                                                    (pose ?looking-direction))))))


      (cpl:with-retry-counters ((looking-retry 3))
          (cpl:with-failure-handling
              ((common-fail:low-level-failure
                   (e)
                 (declare (ignore e))
                 (cpl:do-retry looking-retry
                   (cpl:retry))
                 (roslisp:ros-warn (wipe-demo looking-fail)
                                 "~%No more retries~%")))
        (let ((?grasping-arm (list :left))
              (?perceived-object (urdf-proj::detect (desig:an object (type :knife)))))
          

          (exe:perform (desig:an action
                                 (type picking-up)
                                 (arm ?grasping-arm)
                                   (grasp :top)
                                   (object ?perceived-object))))))



     (pp-plans::park-arms :left-arm T)

      (let ((?navigation-goal *base-pose-plate*
                              ))
          (exe:perform (desig:an action
                                 (type going)
                                 (target (desig:a location 
                                                  (pose ?navigation-goal))))))

           (let* ((?looking-direction *downward-look-coordinate*))
            (exe:perform (desig:an action 
                                   (type looking)
                                   (target (desig:a location 
                                                    (pose ?looking-direction))))))



         (cpl:with-retry-counters ((looking-retry 3))
          (cpl:with-failure-handling
              ((common-fail:low-level-failure
                   (e)
                 (declare (ignore e))
                 (cpl:do-retry looking-retry
                   (cpl:retry))
                 (roslisp:ros-warn (wipe-demo looking-fail)
                                   "~%No more retries~%")))

            (let* ((?surface-to-wipe (urdf-proj::detect (desig:an object (type :colored-box))))
                   (?arm-for-wiping :left)
                   (?collision-mode nil))
      
      (exe:perform
       (desig:an action 
                 (type wiping)
                 (grasp :knife)
                 (arm ?arm-for-wiping)
                 (surface ?surface-to-wipe)
                 (collision-mode ?collision-mode)
                 )))))





      



      ))

(defun start ()
  (roslisp:start-ros-node "bullet_world")
  (cram-occupancy-grid-costmap::init-occupancy-grid-costmap)
  (cram-bullet-reasoning-belief-state::ros-time-init)
  (cram-location-costmap::location-costmap-vis-init)
  (cram-tf::init-tf)
  
  (prolog:prolog '(btr:bullet-world ?world))
  
  (prolog:prolog '(and (btr:bullet-world ?world)
                   (btr:debug-window ?world)))
  
  (prolog:prolog '(and (btr:bullet-world ?world)
                              (assert (btr:object ?world :static-plane :floor ((0 0 0) (0 0 0 1))
                                                                       :normal (0 0 1) :constant 0))))
  
  (let ((robot-urdf
          (cl-urdf:parse-urdf
           (roslisp:get-param "robot_description"))))
    (prolog:prolog
     `(and (btr:bullet-world ?world)
           (cram-robot-interfaces:robot ?robot)
           (assert (btr:object ?world :urdf ?robot ((0 0 0) (0 0 0 1)) :urdf ,robot-urdf))
           (-> (rob-int:robot-joint-states ?robot :arm :left :park ?left-joint-states)
               (assert (btr:joint-state ?world ?robot ?left-joint-states))
               (true))
           (-> (rob-int:robot-joint-states ?robot :arm :right :park ?right-joint-states)
               (assert (btr:joint-state ?world ?robot ?right-joint-states))
               (true)))))

  
  (let ((kitchen-urdf 
          (cl-urdf:parse-urdf 
           (roslisp:get-param "kitchen_description"))))
    (prolog:prolog
     `(and (btr:bullet-world ?world)
           (assert (btr:object ?world :urdf :kitchen ((0 0 0) (0 0 0 1))
                                            :urdf ,kitchen-urdf
                                            :collision-group :static-filter
                                            :collision-mask (:default-filter :character-filter))))))



  )




