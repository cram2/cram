(in-package :cram-pr2-shopping-demo)

(defun spawn-shelve ()
  (let ((shelve-urdf
                   (cl-urdf:parse-urdf
                    (roslisp:get-param "shelve_description"))))
             (prolog:prolog
              `(and (btr:bullet-world ?world)
                    (assert (btr:object ?world :urdf :kitchen ((0 0 0) (0 0 0 1)) :urdf ,shelve-urdf))))))


(defun spawn-robot ()
  (setf cram-robot-interfaces:*robot-urdf*
                   (cl-urdf:parse-urdf
                    (roslisp:get-param "robot_description")))
             (prolog:prolog
              `(and (btr:bullet-world ?world)
                    (cram-robot-interfaces:robot ?robot)
                    (assert (btr:object ?world :urdf ?robot ((0 0 0) (0 0 0 1)) :urdf ,cram-robot-interfaces:*robot-urdf*))
                    (assert (btr:joint-state ?world ?robot (("torso_lift_joint" 0.15d0)))))))


(defun spawn-and-place-objects ()
  (spawn-objects)
  (place-objects)
  (btr:simulate btr:*current-bullet-world* 2))

(defun spawn-objects()
  (btr-utils:spawn-object 'dove :dove)
  (btr-utils:spawn-object 'somat :somat)
  (btr-utils:spawn-object 'heitmann :heitmann)
  (btr-utils:spawn-object 'denkmit :denkmit))

(defun place-objects()
  (btr-utils:move-object 'dove '((-1 -1.1 0.7) (0 0 0 1)))
  (btr-utils:move-object 'somat '((-0.5 -1.1 0.4) (0 0 0 1)))
  (btr-utils:move-object 'heitmann '((-1.3 -1.1 1) (0 0 0 1)))
  (btr-utils:move-object 'denkmit '((-2 -1.1 1.3) (0 0 0 1))))

(defun replace-denkmit()
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-detached-robot
                  :arm ':right
                  :object-name 'denkmit))
  (btr-utils:move-object 'denkmit '((-2 -1.1 1.3) (0 0 0 1))))

(defun replace-dove()
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-detached-robot
                  :arm ':left
                  :object-name 'dove))
  (btr-utils:move-object 'dove '((-1 -1.1 0.7) (0 0 0 1))))

(defun replace-heitmann()
 (cram-occasions-events:on-event
  (make-instance 'cpoe:object-detached-robot
                 :arm ':left
                 :object-name 'heitmann))
 (btr-utils:move-object 'heitmann '((-1.3 -1.1 1) (0 0 0 1))))

(defun replace-somat()
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-detached-robot
                  :arm ':left
                  :object-name 'somat))
  (btr-utils:move-object 'somat '((-0.5 -1.1 0.3) (0 0 0 1)))) 


                    (assert (btr:joint-state ?world ?robot (("torso_lift_joint" 0.15d0)))))))


(defun spawn-and-place-objects ()
  (spawn-objects)
  (place-objects)
  (btr:simulate btr:*current-bullet-world* 2))

(defun spawn-objects()
  (btr-utils:spawn-object 'dove :dove)
  (btr-utils:spawn-object 'somat :somat)
  (btr-utils:spawn-object 'heitmann :heitmann)
  (btr-utils:spawn-object 'denkmit :denkmit))

(defun place-objects()
  (btr-utils:move-object 'dove '((-1 -1.1 0.7) (0 0 0 1)))
  (btr-utils:move-object 'somat '((-0.5 -1.1 0.4) (0 0 0 1)))
  (btr-utils:move-object 'heitmann '((-1.3 -1.1 1) (0 0 0 1)))
  (btr-utils:move-object 'denkmit '((-2 -1.1 1.3) (0 0 0 1))))

(defun replace-denkmit()
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-detached-robot
                  :arm ':right
                  :object-name 'denkmit))
  (btr-utils:move-object 'denkmit '((-2 -1.1 1.3) (0 0 0 1))))

(defun replace-dove()
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-detached-robot
                  :arm ':left
                  :object-name 'dove))
  (btr-utils:move-object 'dove '((-1 -1.1 0.7) (0 0 0 1))))

(defun replace-heitmann()
 (cram-occasions-events:on-event
  (make-instance 'cpoe:object-detached-robot
                 :arm ':left
                 :object-name 'heitmann))
 (btr-utils:move-object 'heitmann '((-1.3 -1.1 1) (0 0 0 1))))

(defun replace-somat()
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-detached-robot
                  :arm ':left
                  :object-name 'somat))
  (btr-utils:move-object 'somat '((-0.5 -1.1 0.3) (0 0 0 1)))) 


                    (assert (btr:joint-state ?world ?robot (("torso_lift_joint" 0.15d0)))))))


(defun spawn-and-place-objects ()
  (spawn-objects)
  (place-objects)
  (btr:simulate btr:*current-bullet-world* 2))

(defun spawn-objects()
  (btr-utils:spawn-object 'dove :dove)
  (btr-utils:spawn-object 'somat :somat)
  (btr-utils:spawn-object 'heitmann :heitmann)
  (btr-utils:spawn-object 'denkmit :denkmit))

(defun place-objects()
  (btr-utils:move-object 'dove '((-1 -1.1 0.7) (0 0 0 1)))
  (btr-utils:move-object 'somat '((-0.5 -1.1 0.4) (0 0 0 1)))
  (btr-utils:move-object 'heitmann '((-1.3 -1.1 1) (0 0 0 1)))
  (btr-utils:move-object 'denkmit '((-2 -1.1 1.3) (0 0 0 1))))

(defun replace-denkmit()
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-detached-robot
                  :arm ':right
                  :object-name 'denkmit))
  (btr-utils:move-object 'denkmit '((-2 -1.1 1.3) (0 0 0 1))))

(defun replace-dove()
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-detached-robot
                  :arm ':left
                  :object-name 'dove))
  (btr-utils:move-object 'dove '((-1 -1.1 0.7) (0 0 0 1))))

(defun replace-heitmann()
 (cram-occasions-events:on-event
  (make-instance 'cpoe:object-detached-robot
                 :arm ':left
                 :object-name 'heitmann))
 (btr-utils:move-object 'heitmann '((-1.3 -1.1 1) (0 0 0 1))))

(defun replace-somat()
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-detached-robot
                  :arm ':left
                  :object-name 'somat))
  (btr-utils:move-object 'somat '((-0.5 -1.1 0.3) (0 0 0 1)))) 


                    (assert (btr:joint-state ?world ?robot (("torso_lift_joint" 0.15d0)))))))


(defun spawn-and-place-objects ()
  (spawn-objects)
  (place-objects)
  (btr:simulate btr:*current-bullet-world* 2))

(defun spawn-objects()
  (btr-utils:spawn-object 'dove :dove)
  (btr-utils:spawn-object 'somat :somat)
  (btr-utils:spawn-object 'heitmann :heitmann)
  (btr-utils:spawn-object 'denkmit :denkmit))

(defun place-objects()
  (btr-utils:move-object 'dove '((-1 -1.1 0.7) (0 0 0 1)))
  (btr-utils:move-object 'somat '((-0.5 -1.1 0.4) (0 0 0 1)))
  (btr-utils:move-object 'heitmann '((-1.3 -1.1 1) (0 0 0 1)))
  (btr-utils:move-object 'denkmit '((-2 -1.1 1.3) (0 0 0 1))))


;; Methods to place the objects back in the shelf after they were picked up but didnt placed 
(defun replace-denkmit()
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-detached-robot
                  :arm ':right
                  :object-name 'denkmit))
  (btr-utils:move-object 'denkmit '((-2 -1.1 1.3) (0 0 0 1))))

(defun replace-dove()
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-detached-robot
                  :arm ':left
                  :object-name 'dove))
  (btr-utils:move-object 'dove '((-1 -1.1 0.7) (0 0 0 1))))

(defun replace-heitmann()
 (cram-occasions-events:on-event
  (make-instance 'cpoe:object-detached-robot
                 :arm ':left
                 :object-name 'heitmann))
 (btr-utils:move-object 'heitmann '((-1.3 -1.1 1) (0 0 0 1))))

(defun replace-somat()
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-detached-robot
                  :arm ':left
                  :object-name 'somat))
  (btr-utils:move-object 'somat '((-0.5 -1.1 0.3) (0 0 0 1)))) 


(defun init ()
  (roslisp:start-ros-node "shopping_demo")
  
  (cram-bullet-reasoning-belief-state::ros-time-init)
  (cram-location-costmap::location-costmap-vis-init)
  (cram-tf::init-tf)

  (prolog:prolog '(btr:bullet-world ?world))
  
  (prolog:prolog '(and (btr:bullet-world ?world)
                   (btr:debug-window ?world)))
  
  (prolog:prolog '(and (btr:bullet-world ?world)
                              (assert (btr:object ?world :static-plane :floor ((0 0 0) (0 0 0 1))
                                                                       :normal (0 0 1) :constant 0))))
  (btr:add-objects-to-mesh-list "cram_pr2_shopping_demo")
  
  (spawn-robot)
  (spawn-shelve)
  (spawn-and-place-objects))

