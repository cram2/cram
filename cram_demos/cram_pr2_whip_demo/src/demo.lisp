;;;
;;; Copyright (c) 2022, Tina Van <van@uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors
;;;       may be used to endorse or promote products derived from this software
;;;       without specific prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :pr2-w-demo)


  ;;###############################################################################
  ;;                     mix/whip (for now STATIC) plan
  ;;###############################################################################

;me writting stuff
(defun test-container()
  (initialize)
	(urdf-proj:with-simulated-robot
	
          (move-pr2)
        ;  (spawn-whisk)
       ; (spawn-wglas)                            
        (spawn-rbowl)
       ; (spawn-saucepan)
       ;(spawn-rbowl)
        
    (let (
         ;utensil: fork,spoon,whisky or sth - for now spawn a fork
        (?object-utensil nil)
        (?pose-utensil nil)
        (?object-desig-source (desig:an object (type :bowl-round)))
        )
                
                ;getting utensil pose and saving it                        
      (setf ?pose-utensil (cl-tf:pose->pose-stamped
                             cram-tf:*fixed-frame*
                             0 
                             (btr:object-pose 'container-1)))
            (cram-executive:perform
           (desig:an action
                     (type looking)
                     (target (desig:a location (pose ?pose-utensil)))))  
     
 ;;           ;CAN SEE THE FORK?
       (setf ?object-utensil (urdf-proj::detect (desig:an object (type :bowl-round))))

            (exe:perform (desig:an action
                                   (type picking-up)
                                   (object ?object-utensil)
                                    (arm (:left)) 
                                    (grasp :handle-right))) )))                  
    

;;#############################whisp testing##############      
(defun test-tool-container ()
  (initialize)
	(urdf-proj:with-simulated-robot
	
          (move-pr2)
          ;(spawn-ladle)
         ; (spawn-fork)
	(spawn-whisk)

                                        
        (spawn-bigbowl)
       ; (spawn-saucepan)
       ;(spawn-rbowl)
        
    (let (
         ;utensil: fork,spoon,whisky or sth - for now spawn a fork
        (?object-utensil nil)
        (?pose-utensil nil)
        (?pose-container nil)
        (?object-container nil)
        (?object-desig-source (desig:an object (type :whisk))) ; :ladle))); :whisk)))
        )
                
                ;getting utensil pose and saving it                        
      (setf ?pose-utensil (cl-tf:pose->pose-stamped
                             cram-tf:*fixed-frame*
                             0 
                             (btr:object-pose 'tool-1)))
            (cram-executive:perform
           (desig:an action
                     (type looking)
                     (target (desig:a location (pose ?pose-utensil)))))  
     
 ;;           ;CAN SEE THE FORK?
       (setf ?object-utensil (urdf-proj::detect (desig:an object (type :whisk))))
                                    
      
      
 ;; ))) 

                                     
;;        ;;;; Picking-up the utensil object 
;; ;grabbing utensil
            (exe:perform (desig:an action
                                   (type picking-up)
                                   (object ?object-utensil)
                                    (arm (:right)) 
                                    (grasp :top)))
   
        (start-mix ?object-desig-source)
      ;(start-eclipse ?object-desig-source)
      )))

(defun start-mix (?object-desig-source)
    
  (let* ((?pose-container (cl-tf:pose->pose-stamped
			  cram-tf:*fixed-frame*
			  0 
			  (btr:object-pose 'container-1)))
			  
          (?object-container(urdf-proj::detect

                             (desig:an object (type big-bowl))))
                            ; (desig:an object (type saucepan))))
                           ; (desig:an object (type bowl-round))))
          (?reso 6))

;(btr::add-vis-axis-object 'saucepan-1) 
 
      
      (exe:perform (desig:an action
                             (type whisking)
			      (context :mix)
                             (reso ?reso)

                             ;; (sides ?reso)
                             (arm (:right))
                             (reso ?reso)
                             (object ?object-container)
                             (grasp :top)
                             (source ?object-desig-source)
                             ))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun test-mix-types ()
  (initialize)
	(urdf-proj:with-simulated-robot
	
	(move-pr2)
	(spawn-whisk)
        (spawn-bigbowl)
        
    (let* (
         ;utensil: fork,spoon,whisky or sth - for now spawn a fork
        (?object-utensil nil)
        (?pose-utensil nil)
        (?pose-container nil)
        (?object-container nil)
        (?object-desig-source (desig:an object (type :whisk)))
        )
                
                ;getting utensil pose and saving it                        
      (setf ?pose-utensil (cl-tf:pose->pose-stamped
                             cram-tf:*fixed-frame*
                             0 
                             (btr:object-pose 'tool-1)))
            (cram-executive:perform
           (desig:an action
                     (type looking)
                     (target (desig:a location (pose ?pose-utensil)))))  
     
 ;;           ;CAN SEE THE FORK?
       (setf ?object-utensil (urdf-proj::detect (desig:an object (type whisk))))
                                    
      
      
 ;; ))) 

                                     
;;        ;;;; Picking-up the utensil object 
;; ;grabbing utensil
            (exe:perform (desig:an action
                                   (type picking-up)
                                   (object ?object-utensil)
                                    (arm (:right)) 
                                    (grasp :top)))
   
      (start-mix ?object-desig-source)
     ; (start-eclipse ?object-desig-source)
     ;(start-orbit ?object-desig-source)
      )))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun testaxis()
  (testdesig)
  	(move-pr2)
	(spawn-whisk)
        (spawn-bigbowl)
  (spawn-saucepan)
  (btr::add-vis-axis-object 'container-1 ) 
  )

(defun testdesig()
(urdf-proj::with-simulated-robot 
	(exe:perform (desig:an action 
               (type my-action-designator)))
      ))
                                     
  ;;###############################################################################
  ;;                                    spawn
  ;;###############################################################################

(defun move-pr2()
      (btr-utils::move-robot '((-0.3 0.9 0) (0 0 1 0))))
                                  
(defun spawn-fork()
      (btr-utils:spawn-object 'tool-1 :fork
                              :pose (cl-transforms:make-pose
                                     (cl-tf:make-3d-vector -0.85 1 0.95)
                                     (cl-tf:make-identity-rotation))))

(defun spawn-spoon()
      (btr-utils:spawn-object 'tool-1 :spoon
                              :pose (cl-transforms:make-pose
                                     (cl-tf:make-3d-vector -0.85 1 0.95)
                                     (cl-tf:make-identity-rotation))))
                                     
(defun spawn-whisk()
      (btr-utils:spawn-object 'tool-1 :whisk
                              :pose (cl-transforms:make-pose
                                     (cl-tf:make-3d-vector -0.85 1 0.89)
                                     (cl-tf:make-identity-rotation))))
                                     
(defun spawn-ladle()
      (btr-utils:spawn-object 'tool-1 :ladle
                              :pose (cl-transforms:make-pose
                                     (cl-tf:make-3d-vector -0.85 1 0.92)
                                     (cl-tf:make-identity-rotation))))

(defun spawn-tspoon()
      (btr-utils:spawn-object 'tool-1 :tea-spoon
                              :pose (cl-transforms:make-pose
                                     (cl-tf:make-3d-vector -0.85 1 0.92)
                                     (cl-tf:make-identity-rotation))))

(defun spawn-bigbowl()
      (btr-utils:spawn-object 'container-1  :big-bowl
                              :pose (cl-transforms:make-pose
                                     (cl-tf:make-3d-vector -0.85 0.55 0.93)
                                     (cl-tf:make-identity-rotation))))

(defun spawn-saucepan()
      (btr-utils:spawn-object 'container-1  :saucepan
                              :pose (cl-transforms:make-pose
                                     (cl-tf:make-3d-vector -0.85 0.65 0.9)
                                     (cl-tf:make-identity-rotation))))
                                                                        
(defun spawn-rbowl()
      (btr-utils:spawn-object 'container-1 :bowl-round
                              :pose (cl-transforms:make-pose
                                     (cl-tf:make-3d-vector -0.85 0.6 0.9)
                                     (cl-tf:make-identity-rotation)))) 
                                     
(defun spawn-wglas()
      (btr-utils:spawn-object 'container-1 :wine-glas
                              :pose (cl-transforms:make-pose
                                     (cl-tf:make-3d-vector -0.85 0.6 0.95)
                                     (cl-tf:make-identity-rotation)))) 

  ;;###############################################################################
  ;;                                    atomic tasks
  ;;###############################################################################

(defun setup-whisk-and-bowl ()

	(move-pr2)
	(spawn-bigbowl)
	(spawn-whisk)
)

(defun start-mix-bbowl (?object-desig-source)
    
  (let ((?pose-container (cl-tf:pose->pose-stamped
			  cram-tf:*fixed-frame*
			  0 
			  (btr:object-pose 'big-bowl-1)))
			  )
    
    (cram-executive:perform
     (desig:an action
	       (type looking)
	       (target (desig:a location (pose ?pose-container)))))
    
    (let ((?object-container(urdf-proj::detect

                            (desig:an object (type big-bowl))))
          (?reso 6))

      ;; (exe:perform (desig:an action
      ;;                        (type ) - how to look up types again
      ;;                        (arm (:left))
      ;;                        (object ?object-container)
      ;;                        (grasp :left-top)))
(btr::add-vis-axis-object 'big-bowl-1) 
 
      
      (exe:perform (desig:an action
                             (type whisking)
			      (context :mix)
                             (reso ?reso)

                             ;; (sides ?reso)
                             (arm (:right))
                             (reso ?reso)
                             (object ?object-container)
                             (grasp :top)
                             (source ?object-desig-source)
                             )))))

(defun start-eclipse (?object-desig-source)
    
  (let ((?pose-container (cl-tf:pose->pose-stamped
			  cram-tf:*fixed-frame*
			  0 
			  (btr:object-pose 'big-bowl-1)))
			  )
    
    (cram-executive:perform
     (desig:an action
	       (type looking)
	       (target (desig:a location (pose ?pose-container)))))
    
    (let ((?object-container(urdf-proj::detect

                            (desig:an object (type big-bowl))))
          (?reso 6))

      ;; (exe:perform (desig:an action
      ;;                        (type ) - how to look up types again
      ;;                        (arm (:left))
      ;;                        (object ?object-container)
      ;;                        (grasp :left-top)))
(btr::add-vis-axis-object 'big-bowl-1) 
 
      
      (exe:perform (desig:an action
                             (type whisking)
			      (context :mix-eclipse)
                             (reso ?reso)

                             ;; (sides ?reso)
                             (arm (:right))
                             (reso ?reso)
                             (object ?object-container)
                             (grasp :top)
                             (source ?object-desig-source)
                             )))))

(defun start-orbit (?object-desig-source)

    
  (let ((?pose-container (cl-tf:pose->pose-stamped
			  cram-tf:*fixed-frame*
			  0 
			  (btr:object-pose 'big-bowl-1)))
			  )
    
    (cram-executive:perform
     (desig:an action
	       (type looking)
	       (target (desig:a location (pose ?pose-container)))))
    
    (let ((?object-container(urdf-proj::detect

                            (desig:an object (type big-bowl))))
          (?reso 6))

      ;; (exe:perform (desig:an action
      ;;                        (type ) - how to look up types again
      ;;                        (arm (:left))
      ;;                        (object ?object-container)
      ;;                        (grasp :left-top)))
(btr::add-vis-axis-object 'big-bowl-1) 
 
      
      (exe:perform (desig:an action
                             (type whisking)
			      (context :mix-orbit)
                             (reso ?reso)

                             ;; (sides ?reso)
                             (arm (:right))
                             (reso ?reso)
                             (object ?object-container)
                             (grasp :top)
                             (source ?object-desig-source)
                             )))))

(defun initialize ()
  (sb-ext:gc :full t)

  ;; (setf proj-reasoning::*projection-checks-enabled* t)

  (btr:detach-all-objects (btr:get-robot-object))
  (btr:detach-all-objects (btr:get-environment-object))
  (btr-utils:kill-all-objects)
  (setf (btr:joint-state (btr:get-environment-object)
                         "sink_area_left_upper_drawer_main_joint")
        0.0
        (btr:joint-state (btr:get-environment-object)
                         "sink_area_left_middle_drawer_main_joint")
        0.0
        (btr:joint-state (btr:get-environment-object)
                         "sink_area_left_bottom_drawer_main_joint")
        0.0
        (btr:joint-state (btr:get-environment-object)
                         "iai_fridge_door_joint")
        0.0
        (btr:joint-state (btr:get-environment-object)
                         "sink_area_dish_washer_door_joint")
        0.0
        (btr:joint-state (btr:get-environment-object)
                         "sink_area_dish_washer_tray_main")
        0.0
        (btr:joint-state (btr:get-environment-object)
                         "oven_area_area_right_drawer_main_joint")
        0.0
        (btr:joint-state (btr:get-environment-object)
                         "sink_area_trash_drawer_main_joint")
        0.0
        (btr:joint-state (btr:get-environment-object)
                         "kitchen_island_left_upper_drawer_main_joint")
        0.0)
  (btr-belief::publish-environment-joint-state
   (btr:joint-states (btr:get-environment-object)))

  (setf desig::*designators* (tg:make-weak-hash-table :weakness :key))

  (coe:clear-belief)

  (btr:clear-costmap-vis-object))






;; ======================== NEW STUFF van van ======================

(defparameter *sink-nav-goal*
  (cl-transforms-stamped:make-pose-stamped
   "map"
   0.0
   (cl-transforms:make-3d-vector 0.75d0 0.70d0 0.0)
   (cl-transforms:make-identity-rotation)))
(defparameter *island-nav-goal*
  (cl-transforms-stamped:make-pose-stamped
   "map"
   0.0
   (cl-transforms:make-3d-vector -0.2d0 2d0 0.0)
   (cl-transforms:make-quaternion 0 0 1 0)))
(defparameter *island-left-nav-goal*
  (cl-transforms-stamped:make-pose-stamped
   "map"
   0.0
   (cl-transforms:make-3d-vector -0.2d0 0.8 0.0)
   (cl-transforms:make-quaternion 0 0 1 0)))
(defparameter *look-goal*
  (cl-transforms-stamped:make-pose-stamped
   "base_footprint"
   0.0
   (cl-transforms:make-3d-vector 0.5d0 0.0d0 1.0d0)
   (cl-transforms:make-identity-rotation)))

(defun go-to-sink-or-island (&optional (sink-or-island :sink))
  (let ((?navigation-goal (ecase sink-or-island
                            (:sink *sink-nav-goal*)
                            (:island *island-nav-goal*)
                            (:island-left *island-left-nav-goal*)))
        (?ptu-goal *look-goal*))
    (cpl:par
      (exe:perform (desig:an action
                             (type parking-arms)))
      (exe:perform (desig:a motion
                            (type going)
                            (pose ?navigation-goal))))
    (exe:perform (desig:a motion
                          (type looking)
                          (pose ?ptu-goal)))))

(defun pick-object (&optional (?object-type :breakfast-cereal) (?arm :right) (?location :sink))
  (go-to-sink-or-island ?location)
  (let* ((?object-desig
           (desig:an object (type ?object-type)))
         (?perceived-object-desig
           (exe:perform (desig:an action
                                  (type detecting)
                                  (object ?object-desig)))))
    (dotimes (i 3)
      (exe:perform (desig:an action
                             (type looking)
                             (object ?perceived-object-desig))))
      (exe:perform (desig:an action
                             (type picking-up)
                             (arm (?arm))
			     (grasp :front)
                             (object ?perceived-object-desig)))))

(defun detect-object (&optional (?object-type :breakfast-cereal) (?arm :right) (?location :sink))
  (go-to-sink-or-island ?location)
  (let* ((?object-desig
           (desig:an object (type ?object-type)))
         (?perceived-object-desig
           (exe:perform (desig:an action
                                  (type detecting)
                                  (object ?object-desig)))))
    ?perceived-object-desig))

(defun flip-on-object (&optional (?object-type :pancake-maker)
			 (?arm :right)
			 (?grasp :front)
			 (?location :island-left))
  (go-to-sink-or-island ?location)
  (let* ((?object-desig
           (desig:an object (type ?object-type)))
         (?perceived-object-desig
           (exe:perform (desig:an action
                                  (type detecting)
                                  (object ?object-desig)))))
    (dotimes (i 3)
      (exe:perform (desig:an action
                             (type looking)
                             (object ?perceived-object-desig))))

    ;;TODO this is how i call the  desig you need to change nothing here but
    ;;spawn your bowl and call the right function in prolog.lisp
      (exe:perform (desig:an action
                             (type flipping)
                             (arm (?arm))
                             (object ?perceived-object-desig)
                             (grasp ?grasp)
                             ))))


