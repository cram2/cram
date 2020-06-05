;;;
;;; Copyright (c) 2019, Jonas Dech <jdech[at]uni-bremen.de
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

(in-package :cram-pr2-shopping-demo)

(let ((y -0.38)
      (off -0.07))
  (defparameter *shelf-1-poses*
    `(((-0.36 ,y 0.1)
       (-0.28 ,y 0.1)
       (-0.19 ,y 0.1)
       (-0.03 ,y 0.1)
       (0.14 ,y 0.1)
       (0.23 ,y 0.1)
       (0.32 ,y 0.11))

      ((-0.43 ,y 0.11)
       (-0.26 ,y 0.11)
       (0.17 ,y 0.13)
       (0.37 ,y 0.16))

      ((-0.4 ,y 0.1)
       (-0.26 ,y 0.1)
       (-0.02 ,y 0.1)
       (0.21 ,y 0.1)
       (0.4 ,y 0.1))

      ((-0.4 ,y 0.13)
       (-0.217 ,y 0.1)
       (-0.033 ,y 0.12)
       (0.137 ,y 0.1)
       (0.43 ,y 0.1))

      ((-0.31 ,y 0.08)
       (-0.25 ,y 0.08)
       (-0.175 ,y 0.09)
       (-0.05 ,y 0.13)
       (0.082 ,y 0.15)
       (0.203 ,y 0.14))

      ((-0.43 ,(+ y off) 0.15)
       (-0.30 ,(+ y off) 0.15)
       (-0.14 ,(+ y off) 0.14)
       (0.01 ,(+ y off) 0.15)
       (0.15 ,(+ y off) 0.15)
       (0.29 ,(+ y off) 0.145)
       (0.42 ,(+ y off) 0.13)))))


(defun convert-shelf-poses (shelf poses)
  (let (res
        tmp
        (levels (if (equalp shelf "shelf-2")
                    '(:|ENVIRONMENT.shelf_2_level_3_link|
                      :|ENVIRONMENT.shelf_2_level_2_link|
                      :|ENVIRONMENT.shelf_2_level_1_link|
                      :|ENVIRONMENT.shelf_2_level_0_link|)
                    '(:|ENVIRONMENT.shelf_1_level_5_link|
                      :|ENVIRONMENT.shelf_1_level_4_link|
                      :|ENVIRONMENT.shelf_1_level_3_link|
                      :|ENVIRONMENT.shelf_1_level_2_link|
                      :|ENVIRONMENT.shelf_1_level_1_link|
                      :|ENVIRONMENT.shelf_1_level_0_link|))))
     ;; Loop for every level of the shelf
    (loop for i from 0 to (- (length levels) 1)
          do (let* ((level-poses (nth i poses))
                    (world-T-shelf (cl-transforms:pose->transform
                                    (btr:pose
                                     (btr:rigid-body
                                      (btr:object btr:*current-bullet-world* :environment)
                                      (nth i levels))))))
                ;; Loop for the objects
               (loop for j from 0 to (- (length level-poses) 1)
                     do (let* ((shelf-T-object
                                 (cram-tf:list->transform `(,(nth j level-poses) (0 0 1 0))))
                               (world-T-object
                                 (cl-transforms:transform* world-T-shelf shelf-T-object)))
                          (push (cl-transforms:transform->pose world-T-object) tmp)))
               (push (reverse tmp) res)
               (setf tmp '())))
    (reverse res)))


(defun spawn-objects-new ()
  (let ((poses (convert-shelf-poses "shelf-1" *shelf-1-poses*))
        (object-types '((:denkmit-edelstahl-reiniger-spray
                         :denkmit-edelstahl-reiniger
                         :denkmit-glaskeramik-reiniger
                         :denkmit-maschienen-entkalker
                         :denkmit-entkalker
                         :kuehne-essig-essenz
                         :heitmann-citronensaeure)
                        (:finish-deo
                         :finish-spuelmaschienen-tabs-quantum
                         :finish-spuelmaschienen-tabs-classic
                         :finish-spuelmaschienen-tabs-classic-vorratspack)
                        (:finish-spuelmaschienen-pulver
                         :finish-spuelmaschienen-protector
                         :somat-spuelmaschienen-tabs-extra
                         :somat-spuelmaschienen-tabs-classic
                         :somat-spuelmaschienen-pulver)
                        (:denkmit-spezialsalz
                         :denkmit-spuelmaschienen-tabs-power
                         :denkmit-spuelmaschienen-tabs-revolution
                         :denkmit-spuelmaschienen-tabs-classic
                         :denkmit-spuelmaschienen-tabs-nature)
                        (:denkmit-maschienenpfleger
                         :denkmit-maschienenpfleger
                         :finish-maschienenpfleger
                         :finish-spezialsalz
                         :finish-klarspueler
                         :denkmit-spuelmaschienen-tabs-allinone)
                        (:domestos-allzweckreiniger
                         :sagrotan-allzweckreiniger
                         :denkmit-allzweckreiniger
                         :denkmit-allzweckreiniger-frueling
                         :meister-proper-allzweckreiniger
                         :denkmit-allzweckreiniger-limette
                         :der-general-allzweckreiniger))))
    ;; Loop for shelf level
    (loop for i from 0 to (- (length poses) 1)
       ;; loop for objects in the level
      do (loop for j from 0 to (- (length (nth i poses)) 1)
               do (spawn-object-n-times (nth j (nth i object-types))
                                        (cram-tf:pose->list (nth j(nth i poses)))
                                        (+ (random 3) 1)
                                        `(,(float (/ (random 10) 10))
                                          ,(float (/ (random 10) 10))
                                          ,(float (/ (random 10) 10))))))))

(defun spawn-object-n-times (type pose times color)
  (loop for i from 0 to times
        do (let ((name (intern (string-upcase (format nil "~a-~a" type i)) :keyword)))
             (setf (second (first pose)) (+ (second (first pose)) 0.13))
             (when (btr:object btr:*current-bullet-world* name)
               (setf i (+ i times)
                     name (intern (string-upcase (format nil "~a-~a" type i)) :keyword)))
             (btr-utils:spawn-object name type :pose pose :color color))))

(defun spawn-shelf ()
  (let ((shelve-urdf
          (cl-urdf:parse-urdf
           (roslisp:get-param "shelf_description"))))
    (prolog:prolog
     `(and (btr:bullet-world ?world)
           (man-int:environment-name ?environment-name)
           (assert (btr:object ?world :urdf ?environment-name ((0 0 0) (0 0 0 1))
                               :urdf ,shelve-urdf
                               :collision-group :static-filter
                               :collision-mask (:default-filter
                                                :character-filter)))))))

(defun spawn-robot ()
  (setf rob-int:*robot-urdf*
        (cl-urdf:parse-urdf
         (roslisp:get-param "robot_description")))
  (prolog:prolog
   `(and (btr:bullet-world ?world)
         (rob-int:robot ?robot)
         (assert (btr:object ?world :urdf ?robot ((0 0 0) (0 0 0 1))
                             :urdf ,rob-int::*robot-urdf*))
         (assert (btr:joint-state ?world ?robot (("torso_lift_joint" 0.15d0)))))))

(defun spawn-basket ()
  (btr:add-object btr:*current-bullet-world* :basket
                  :b
                  (cl-transforms:make-pose
                   (cl-transforms:make-3d-vector 0.8 0.2 0.75)
                   (cl-transforms:make-quaternion 0 0 0 1))
                  :mass 1
                  :length 0.5 :width 0.3 :height 0.18 :handle-height 0.09)
  (let ((basket-desig (desig:an object (type basket) (name b))))
    (coe:on-event
     (make-instance 'cpoe:object-attached-robot
       :link "l_wrist_roll_link"
       :not-loose t
       :grasp :front
       :arm :left
       :object-name :b
       :object-designator basket-desig))))

(defun spawn-objects ()
  (let ((i 1))
    ;;Spawn objects in shelf 1
    (loop for j from -1.3 to -0.6 by 0.16
          do (let ((denkmit
                     (concatenate 'string
                                  "denkmit" "-" (write-to-string j) "-" (write-to-string i)))
                   (heitmamn
                     (concatenate 'string
                                  "heitmann" "-" (write-to-string j) "-" (write-to-string i)))
                   (dove
                     (concatenate 'string
                                  "dove" "-" (write-to-string j) "-" (write-to-string i))))
               (btr-utils:spawn-object
                (intern denkmit) :denkmit :pose `((,j 0.9 0.5) (0 0 1 0)))
               (btr-utils:spawn-object
                (intern heitmamn) :heitmann :pose `((,j 0.9 0.96) (0 0 1 0)))
               (btr-utils:spawn-object
                (intern dove) :dove :pose `((,j 0.9 0.73) (0 0 1 0)))))
    (incf i)
    ;; Spawn objects in shelf 2
    (loop for k from 0.6 to 1.4 by 0.12
          do (let ((denkmit
                     (concatenate 'string
                                  "denkmit" "-" (write-to-string k) "-" (write-to-string i)))
                   (heitmamn
                     (concatenate 'string
                                  "heitmann" "-" (write-to-string k) "-" (write-to-string i)))
                   (dove
                     (concatenate 'string
                                  "dove" "-" (write-to-string k) "-" (write-to-string i))))
               ;; (btr-utils:spawn-object
               ;;  (intern denkmit) :denkmit :pose `((,k 0.7 0.68) (0 0 1 0)) :color '(1 0 0 1))
               (btr-utils:spawn-object
                (intern heitmamn) :heitmann :pose `((,k 0.75 1.05) (0 0 1 0)))
               (btr-utils:spawn-object
                (intern dove) :dove :pose `((,k 0.75 1.39) (0 0 1 0)))))
    (incf i)
    ;; Spawn objects on the table
    (loop for k from -3.7 to -3.3 by 0.12
          do (let ((denkmit
                     (concatenate 'string
                                  "denkmit" "-" (write-to-string k) "-" (write-to-string i)))
                   (heitmamn
                     (concatenate 'string
                                  "heitmann" "-" (write-to-string k) "-" (write-to-string i)))
                   (dove
                     (concatenate 'string
                                  "dove" "-" (write-to-string k) "-" (write-to-string i))))
               (btr-utils:spawn-object
                (intern denkmit) :denkmit :pose `((,k 0.1 0.7) (0 0 1 0)) :color '(1 0 0 1))))))


(defun init ()
  ;; (roslisp:start-ros-node "shopping_demo")

  (cram-bullet-reasoning-belief-state::ros-time-init)
  (cram-location-costmap::location-costmap-vis-init)
  (cram-tf::init-tf)

  (prolog:prolog '(and
                   (btr:bullet-world ?world)
                   (btr:debug-window ?world)))

  (prolog:prolog '(and
                   (btr:bullet-world ?world)
                   (assert (btr:object ?world :static-plane
                            :floor
                            ((0 0 0) (0 0 0 1))
                            :normal (0 0 1) :constant 0
                            :collision-mask (:default-filter)))))
  (btr:add-objects-to-mesh-list "cram_pr2_shopping_demo"))


(roslisp-utilities:register-ros-init-function init)
(roslisp-utilities:register-ros-init-function spawn-robot)
(roslisp-utilities:register-ros-init-function spawn-shelf)
(roslisp-utilities:register-ros-init-function spawn-objects)
;; (roslisp-utilities:register-ros-init-function spawn-objects-new)
(roslisp-utilities:register-ros-init-function spawn-basket)

