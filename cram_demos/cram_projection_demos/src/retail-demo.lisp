;;;
;;; Copyright (c) 2018, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :demos)

(defparameter *y* -0.38)
(defparameter *off* -0.07)
(defparameter *shelf-1-poses*
  `(((-0.36 ,*y* 0.1)
     (-0.28 ,*y* 0.1)
     (-0.19 ,*y* 0.1)
     (-0.03 ,*y* 0.1)
     (0.14 ,*y* 0.1)
     (0.23 ,*y* 0.1)
     (0.32 ,*y* 0.11))

    ((-0.43 ,*y* 0.11)
     (-0.26 ,*y* 0.11)
     (0.17 ,*y* 0.13)
     (0.37 ,*y* 0.16))

    ((-0.4 ,*y* 0.1)
     (-0.26 ,*y* 0.1)
     (-0.02 ,*y* 0.1)
     (0.21 ,*y* 0.1)
     (0.4 ,*y* 0.1))

    ((-0.4 ,*y* 0.13)
     (-0.217 ,*y* 0.1)
     (-0.033 ,*y* 0.12)
     (0.137 ,*y* 0.1)
     (0.43 ,*y* 0.1))

    ((-0.31 ,*y* 0.08)
     (-0.25 ,*y* 0.08)
     (-0.175 ,*y* 0.09)
     (-0.05 ,*y* 0.13)
     (0.082 ,*y* 0.15)
     (0.203 ,*y* 0.14))

    ((-0.43 ,(+ *y* *off*) 0.15)
     (-0.30 ,(+ *y* *off*) 0.15)
     (-0.14 ,(+ *y* *off*) 0.14)
     (0.01 ,(+ *y* *off*) 0.15)
     (0.15 ,(+ *y* *off*) 0.15)
     (0.29 ,(+ *y* *off*) 0.145)
     (0.42 ,(+ *y* *off*) 0.13))))

(defparameter *basket-in-pr2-wrist*
  (cl-transforms:make-transform
   (cl-transforms:make-3d-vector 0.36 0 -0.15)
   (cl-transforms:make-quaternion 0.0d0 -0.7071067811865475d0
                                  0.0d0 0.7071067811865475d0)))

(defparameter *basket-in-boxy-wrist*
  (cl-transforms:make-transform
   (cl-transforms:make-3d-vector 0 -0.15 0.48)
   (cl-transforms:make-quaternion 0.5 0.5 0 0)))

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
                                      (btr:get-environment-object)
                                      (nth i levels))))))
               ;; Loop for the objects
               (loop for j from 0 to (- (length level-poses) 1)
                     do (let* ((shelf-T-object
                                 (cram-tf:list->transform
                                  `(,(nth j level-poses) (0 0 1 0))))
                               (world-T-object
                                 (cl-transforms:transform*
                                  world-T-shelf shelf-T-object)))
                          (push (cl-transforms:transform->pose world-T-object) tmp)))
               (push (reverse tmp) res)
               (setf tmp '())))
    (reverse res)))

(defun spawn-object-n-times (type pose times color)
  (loop for i from 0 to times
        do (let ((name (intern (string-upcase (format nil "~a-~a" type i))
                               :keyword)))
             (setf (second (first pose)) (+ (second (first pose)) 0.13))
             (when (btr:object btr:*current-bullet-world* name)
               (setf i (+ i times)
                     name (intern (string-upcase (format nil "~a-~a" type i))
                                  :keyword)))
             (btr-utils:spawn-object name type :pose pose :color color))))

(defun spawn-retail-objects-new ()
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

(defun spawn-basket ()
  (let* ((left-ee-frame
           (cut:var-value
            '?end-effector-link
            (car
             (prolog:prolog
              `(and (rob-int:robot ?robot)
                    (rob-int:end-effector-link ?robot :left ?end-effector-link))))))
         (map-T-wrist
           (btr:link-pose (btr:get-robot-object) left-ee-frame))
         (wrist-T-basket
           (case (rob-int:get-robot-name)
             (:pr2 *basket-in-pr2-wrist*)
             (t *basket-in-boxy-wrist*)))
         (map-T-basket
           (cl-transforms:transform*
            (cl-transforms:pose->transform map-T-wrist)
            wrist-T-basket))
         (basket-pose
           (cl-transforms:transform->pose map-T-basket))
         (basket-desig
           (desig:an object (type basket) (name b))))
    (btr:add-object btr:*current-bullet-world* :basket
                    :b
                    basket-pose
                    :mass 1
                    :length 0.5 :width 0.3 :height 0.18 :handle-height 0.09)
    (coe:on-event
     (make-instance 'cpoe:object-attached-robot
       :link left-ee-frame
       :not-loose t
       :grasp :top
       :arm :left
       :object-name :b
       :object-designator basket-desig))))

(defun spawn-retail-objects ()
  (sb-ext:gc :full t)
  (setf desig::*designators* (tg:make-weak-hash-table :weakness :key))
  (btr:clear-costmap-vis-object)
  (btr-utils:kill-all-objects)
  (btr:detach-all-objects (btr:get-robot-object))
  (btr:detach-all-objects (btr:get-environment-object))

  (let ((i 1))
    ;;Spawn objects in shelf 1
    (loop for j from -1.3 to -0.6 by 0.16
          do (let ((denkmit
                     (concatenate 'string
                                  "denkmit"
                                  "-" (write-to-string j)
                                  "-" (write-to-string i)))
                   (heitmamn
                     (concatenate 'string
                                  "heitmann"
                                  "-" (write-to-string j)
                                  "-" (write-to-string i)))
                   (dove
                     (concatenate 'string
                                  "dove"
                                  "-" (write-to-string j)
                                  "-" (write-to-string i))))
               (btr-utils:spawn-object
                (intern denkmit) :denkmit :pose `((,j 0.9 0.5) (0 0 1 0)))
               (btr-utils:spawn-object
                (intern heitmamn) :heitmann :pose `((,j 0.9 0.96) (0 0 1 0)))
               (btr-utils:spawn-object
                (intern dove) :dove :pose `((,j 0.9 0.73) (0 0 1 0)))))
    (incf i)
    ;; Spawn objects in shelf 2
    (loop for k from 0.6 to 1.4 by 0.12
          do (let (;; (denkmit
                   ;;   (concatenate 'string
                   ;;                "denkmit"
                   ;;                "-" (write-to-string k)
                   ;;                "-" (write-to-string i)))
                   (heitmamn
                     (concatenate 'string
                                  "heitmann"
                                  "-" (write-to-string k)
                                  "-" (write-to-string i)))
                   (dove
                     (concatenate 'string
                                  "dove"
                                  "-" (write-to-string k)
                                  "-" (write-to-string i))))
               ;; (btr-utils:spawn-object
               ;;  (intern denkmit) :denkmit :pose `((,k 0.7 0.68) (0 0 1 0))
               ;;                            :color '(1 0 0 1))
               (btr-utils:spawn-object
                (intern heitmamn) :heitmann :pose `((,k 0.75 1.05) (0 0 1 0)))
               (btr-utils:spawn-object
                (intern dove) :dove :pose `((,k 0.75 1.39) (0 0 1 0)))))
    (incf i)
    ;; Spawn objects on the table
    (loop for k from -3.7 to -3.3 by 0.12
          do (let ((denkmit
                     (concatenate 'string
                                  "denkmit"
                                  "-" (write-to-string k)
                                  "-" (write-to-string i)))
                   ;; (heitmamn
                   ;;   (concatenate 'string
                   ;;                "heitmann"
                   ;;                "-" (write-to-string k)
                   ;;                "-" (write-to-string i)))
                   ;; (dove
                   ;;   (concatenate 'string
                   ;;                "dove"
                   ;;                "-" (write-to-string k)
                   ;;                "-" (write-to-string i)))
                   )
               (btr-utils:spawn-object
                (intern denkmit) :denkmit :pose `((,k 0.1 0.7) (0 0 1 0))
                                          :color '(1 0 0 1))))))





(defun spawn-objects-on-small-shelf (&optional (spawn? t))
  (sb-ext:gc :full t)
  (setf desig::*designators* (tg:make-weak-hash-table :weakness :key))
  (btr:clear-costmap-vis-object)
  (btr-utils:kill-all-objects)
  (btr:detach-all-objects (btr:get-robot-object))
  (btr:detach-all-objects (btr:get-environment-object))
  #+only-for-the-real-robot-so-commenting-out-for-now
  (unless cram-projection:*projection-environment*
    (giskard::call-giskard-environment-service
     :remove-all)
    (when (btr:get-environment-object)
      (giskard::call-giskard-environment-service
       :add-environment
       :name (roslisp-utilities:rosify-underscores-lisp-name
              (rob-int:get-environment-name))
       :pose (cl-transforms-stamped:pose->pose-stamped
              cram-tf:*fixed-frame* 0.0 (btr:pose (btr:get-environment-object)))
       :joint-state-topic "kitchen/joint_states")))

  (when (and spawn? cram-projection:*projection-environment*)
    (btr-utils:spawn-object :balea-bottle-1 :balea-bottle :pose
                            '((1.9 -1.42 1.05) (0 0 0.7 0.7))
                            :color '(1 1 1))
    (btr:add-object btr:*current-bullet-world* :box-item
                    :denkmitgeschirrreinigernature-1
                    '((1.75 -1.45 1.06) (0 0 0.7 0.7))
                    :mass 0.2
                    :color '(0 1 0 1.0)
                    :size '(0.057 0.018 0.074)
                    :item-type :dish-washer-tabs)
    ;; (btr:simulate btr:*current-bullet-world* 50)
    (btr-utils:move-robot '((1.0 0 0) (0 0 0 1)))))








(defun retail-demo-donbot ()
  (spawn-objects-on-small-shelf)

  (let* ((?environment-name
           (rob-int:get-environment-name))
         (?search-location
           (desig:a location
                    (on (desig:an object
                                  (type shelf)
                                  (urdf-name shelf-2-base)
                                  (owl-name "shelf_system_verhuetung")
                                  (part-of ?environment-name)
                                  (level 4)))
                    (side left)
                    (range 0.2)))
         (?object
           (desig:an object
                     (type dish-washer-tabs)
                     (location ?search-location)))
         (?target-location-shelf
           (desig:a location
                    (on (desig:an object
                                  (type environment)
                                  (name ?environment-name)
                                  (part-of ?environment-name)
                                  (urdf-name shelf-1-level-2-link)))
                    (for ?object)
                    (attachments (donbot-shelf-1-front donbot-shelf-1-back))))
         (?robot-name (rob-int:get-robot-name))
         (?intermediate-locaiton-robot
           (desig:a location
                    (on (desig:an object
                                  (type robot)
                                  (name ?robot-name)
                                  (part-of ?environment-name)
                                  ;; (owl-name "donbot_tray")
                                  (urdf-name plate)))
                    (for ?object)
                    (attachments (donbot-tray-front donbot-tray-back)))))

    (exe:perform
     (desig:an action
               (type transporting)
               (object ?object)
               (target ?intermediate-locaiton-robot)
               ;; (target ?target-location-shelf)
               ))

    (exe:perform
     (desig:an action
               (type transporting)
               (object ?object)
               (target ?target-location-shelf)))

    ;; (setf ?object
    ;;       (desig:copy-designator
    ;;        (perform (a motion
    ;;                    (type :world-state-detecting)
    ;;                    (object (an object
    ;;                                (name denkmitgeschirrreinigernature-1)))))
    ;;        :new-description
    ;;        `((:location ,(a location
    ;;                         (on (an object
    ;;                                 (type robot)
    ;;                                 (name ?robot-name)
    ;;                                 (urdf-name plate)
    ;;                                 (owl-name "donbot_tray"))))))))

    ;; look at separators
    ;; (exe:perform
    ;;  (desig:an action
    ;;            (type looking)
    ;;            (direction right-separators)))
    ;; (cpl:sleep 5.0)
    ))




(defun retail-demo-not-donbot ()
  (flet ((grasp-object-from-shelf (?object shelf)
           (let* ((?environment-name
                    (rob-int:get-environment-name))
                  (?search-location
                    (if (eql shelf 1)
                        (desig:a location
                                 (on (desig:an object
                                               (type shelf)
                                               (urdf-name shelf-2-footprint)
                                               (part-of ?environment-name)
                                               (level 4)))
                                 (side right))
                        (desig:a location
                                 (on (desig:an object
                                               (type shelf)
                                               (urdf-name shelf-1-footprint)
                                               (part-of ?environment-name)
                                               (level 4)))
                                 (side right))))
                  (?object-desig
                    (desig:an object
                              (type ?object)
                              (location ?search-location))))
             (exe:perform
              (desig:an action
                        (type transporting)
                        (object ?object-desig)
                        (target (desig:a location
                                         (on (desig:an object
                                                       (type basket)
                                                       (name b)))
                                         (for ?object-desig)
                                         (attachment in-basket)))))))

         (place-object-in-shelf (?object-type &rest ?target-poses)
           (declare (type symbol ?object-type)
                    (type list ?target-poses))
           (let ((?env (rob-int:get-environment-name)))
             (exe:perform
              (desig:an action
                        (type transporting)
                        (object (desig:an object
                                          (type ?object-type)
                                          (location
                                           (desig:a location
                                                    (on (desig:an object
                                                                  (type counter-top)
                                                                  (urdf-name top)
                                                                  (part-of ?env)))
                                                    (side right)))))
                        (target (desig:a location
                                         (poses  ?target-poses))))))))

    (spawn-retail-objects)
    (spawn-basket)

    (place-object-in-shelf
     :denkmit
     (cl-transforms-stamped:make-pose-stamped
      "map" 0
      (cl-transforms:make-3d-vector 0.7 0.7 0.68)
      (cl-transforms:make-quaternion 0 0 1 0))
     (cl-transforms-stamped:make-pose-stamped
      "map" 0
      (cl-transforms:make-3d-vector 0.7 0.7 0.68)
      (cl-transforms:make-quaternion 0 0 1 1))
     (cl-transforms-stamped:make-pose-stamped
      "map" 0
      (cl-transforms:make-3d-vector 0.7 0.7 0.68)
      (cl-transforms:make-quaternion 0 0 -1 0))
     (cl-transforms-stamped:make-pose-stamped
      "map" 0
      (cl-transforms:make-3d-vector 0.7 0.7 0.68)
      (cl-transforms:make-quaternion 0 0 -1 1)))
    (grasp-object-from-shelf :heitmann 2)
    (grasp-object-from-shelf :dove 1)))


(defun retail-demo ()
  (urdf-proj:with-simulated-robot

    (case (rob-int:get-robot-name)
      (:iai-donbot (retail-demo-donbot))
      (t (retail-demo-not-donbot)))))















#+stuff-to-test-real-robot-interfaces
(defun stuff-that-works ()
  (cram-process-modules:with-process-modules-running
      (giskard:giskard-pm)
    (cpl-impl::named-top-level (:name :top-level)
      (exe:perform
       (let ((?pose (cl-transforms-stamped:make-pose-stamped
                     "base_footprint" 0.0
                     (cl-transforms:make-3d-vector
                      -0.27012088894844055d0
                      0.5643729567527771d0
                      1.25943687558174133d0)
                     (cl-transforms:make-quaternion
                      -0.4310053586959839d0
                      0.24723316729068756d0
                      0.752766489982605d0
                      0.4318017065525055d0 ))))
         (desig:a motion
                  (type moving-tcp)
                  (left-pose ?pose)
                  (collision-mode :allow-all)))))))
