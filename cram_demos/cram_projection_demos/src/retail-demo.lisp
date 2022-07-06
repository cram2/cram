;;;
;;; Copyright (c) 2018, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;                     Jonas Dech <jdech@uni-bremen.de>
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
(defparameter *big-shelf-poses*
  `(("level_5_link"
     (:denkmit-edelstahl-reiniger-spray . ((-0.36 ,*y* 0.1) (0 0 0 1)))
     (:denkmit-edelstahl-reiniger . ((-0.28 ,*y* 0.1) (0 0 0 1)))
     (:denkmit-glaskeramik-reiniger . ((-0.19 ,*y* 0.1) (0 0 0 1)))
     (:denkmit-maschienen-entkalker . ((-0.03 ,*y* 0.1) (0 0 0 1)))
     (:denkmit-entkalker . ((0.14 ,*y* 0.1) (0 0 0 1)))
     (:kuehne-essig-essenz . ((0.23 ,*y* 0.1) (0 0 0 1)))
     (:heitmann-citronensaeure . ((0.32 ,*y* 0.11) (0 0 0 1))))

    ("level_4_link"
     (:finish-deo . ((-0.43 ,*y* 0.11) (0 0 0 1)))
     (:finish-spuelmaschienen-tabs-quantum . ((-0.26 ,*y* 0.11) (0 0 0 1)))
     (:finish-spuelmaschienen-tabs-classic . ((0.17 ,*y* 0.13) (0 0 0 1)))
     (:finish-spuelmaschienen-tabs-classic-vorratspack . ((0.38 ,*y* 0.16)
                                                          (0 0 0 1))))

    ("level_3_link"
     (:finish-spuelmaschienen-pulver . ((-0.4 ,*y* 0.1) (0 0 0 1)))
     (:finish-spuelmaschienen-protector . ((-0.26 ,*y* 0.1) (0 0 0 1)))
     (:somat-spuelmaschienen-tabs-extra . ((-0.02 ,*y* 0.1) (0 0 0 1)))
     (:somat-spuelmaschienen-tabs-classic . ((0.21 ,*y* 0.1) (0 0 0 1)))
     (:somat-spuelmaschienen-pulver . ((0.4 ,*y* 0.1) (0 0 0 1))))

    ("level_2_link"
     (:denkmit-spezialsalz . ((-0.4 ,*y* 0.13) (0 0 0 1)))
     (:denkmit-spuelmaschienen-tabs-power . ((-0.212 ,*y* 0.1) (0 0 0 1)))
     (:denkmit-spuelmaschienen-tabs-revolution . ((-0.039 ,*y* 0.12) (0 0 0 1)))
     (:denkmit-spuelmaschienen-tabs-classic . ((0.137 ,*y* 0.1) (0 0 0 1)))
     ;; (:denkmit-spuelmaschienen-tabs-nature . ((0.43 ,*y* 0.1) (0 0 0 1)))
     )

    ("level_1_link"
     (:denkmit-maschienenpfleger . ((-0.31 ,*y* 0.08) (0 0 0 1)))
     (:denkmit-maschienenpfleger . ((-0.25 ,*y* 0.08) (0 0 0 1)))
     (:finish-maschienenpfleger . ((-0.175 ,*y* 0.09) (0 0 0 1)))
     (:finish-spezialsalz . ((-0.05 ,*y* 0.13) (0 0 0 1)))
     (:finish-klarspueler . ((0.082 ,*y* 0.15) (0 0 0 1)))
     (:denkmit-spuelmaschienen-tabs-allinone . ((0.203 ,*y* 0.14) (0 0 0 1))))

    ("level_0_link"
     (:domestos-allzweckreiniger . ((-0.43 ,(+ *y* *off*) 0.15) (0 0 0 1)))
     (:sagrotan-allzweckreiniger . ((-0.30 ,(+ *y* *off*) 0.15) (0 0 0 1)))
     (:denkmit-allzweckreiniger . ((-0.14 ,(+ *y* *off*) 0.14) (0 0 0 1)))
     (:denkmit-allzweckreiniger-frueling . ((0.01 ,(+ *y* *off*) 0.15) (0 0 0 1)))
     (:meister-proper-allzweckreiniger . ((0.15 ,(+ *y* *off*) 0.15) (0 0 0 1)))
     (:denkmit-allzweckreiniger-limette . ((0.29 ,(+ *y* *off*) 0.145) (0 0 0 1)))
     (:der-general-allzweckreiniger . ((0.42 ,(+ *y* *off*) 0.13) (0 0 0 1))))))

(defparameter *small-shelf-poses*
  '("shelf_2_shelf_2_level_3_link"
    (:balea-bottle . ((-0.15 -0.14 0.1) (0 0 0.5 0.5)))
    (:dish-washer-tabs . ((0 -0.13 0.11) (0 0 0.5 0.5)))
    ;; (:box-item . ((-0.075 -0.135 0.1) (0 0 0 1)))
    ))

(defparameter *real-random-small-shelf-poses*
  '("DMFloorT4W100_NLBMVEGQ"
    (:balea-bottle . ((-0.15 -0.14 0.16) (0 0 0.5 0.5)))
    (:dish-washer-tabs . ((0.1 -0.13 0.16) (0 0 0.5 0.5)))
    (:breakfast-cereal . ((-0.35 -0.135 0.2) (0 0 0.5 0.5)))
    (:breakfast-cereal . ((-0.35 0.135 0.2) (0 0 0.5 0.5)))
    (:breakfast-cereal . ((-0.35 0 0.2) (0 0 0.5 0.5)))))

(defparameter *real-small-shelf-poses*
  '("DMFloorT4W100_ZMIYURLQ"
    (:balea-bottle . ((-0.01 -0.14 0.16) (0 0 0.5 0.5)))
    (:dish-washer-tabs . ((0.12 -0.13 0.16) (0 0 0.5 0.5)))
    (:breakfast-cereal . ((-0.37 -0.135 0.19) (0 0 0.5 0.5)))
    (:breakfast-cereal . ((-0.37 0.135 0.19) (0 0 0.5 0.5)))
    (:breakfast-cereal . ((-0.37 0 0.19) (0 0 0.5 0.5)))))

(defparameter *basket-in-pr2-wrist*
  (cl-transforms:make-transform
   (cl-transforms:make-3d-vector 0.36 0 -0.15)
   (cl-transforms:make-quaternion 0.0d0 -0.7071067811865475d0
                                  0.0d0 0.7071067811865475d0)))

(defparameter *basket-in-boxy-wrist*
  (cl-transforms:make-transform
   (cl-transforms:make-3d-vector 0 -0.15 0.48)
   (cl-transforms:make-quaternion 0.5 0.5 0 0)))

;; Calculated based on the basket grasp pose
#+first-set-the-basket-correctly-in-hand-then-take-the-relative-pose
(let* ((obj-T-std-gripper
               (man-int:get-object-type-to-gripper-transform
                :basket :b :left :top))
             (map-T-std-gripper
               (cl-transforms:transform*
                (cl-transforms:reference-transform
                 (btr:link-pose (btr:get-robot-object) "gripper_left_grasping_frame"))
                (cl-transforms:make-transform
                 (cl-transforms:make-3d-vector (- 0.24 0.150575d0) 0 0 )
                 (cl-transforms:make-identity-rotation))
                (cl-transforms:transform-inv
                 (cl-transforms:make-transform
                  (cl-transforms:make-identity-vector)
                  (cl-transforms:matrix->quaternion
                   #2A((0 1 0)
                       (0 0 1)
                       (1 0 0)))))))
             (map-T-obj
               (cl-transforms:transform*
                map-T-std-gripper
                (cl-transforms:transform-inv
                 obj-T-std-gripper))))
         (setf (btr:pose (btr:object btr:*current-bullet-world* :b))
               (cl-transforms:transform->pose map-T-obj)))
(defparameter *basket-in-tiago-wrist*
  (cl-transforms:make-transform
   (cl-transforms:make-3d-vector 0.42 0.15 0)
   (cl-transforms:make-quaternion -0.5 0.5 0.5 -0.5))
  "In end-effector frame.")

(defun spawn-objects-on-big-shelf (&optional (minimal-color 0.0))
  (let ((type-and-pose-list
          (make-poses-relative-multiple *big-shelf-poses* "shelf_1_")))
    (mapcar (lambda (type-and-pose)
              (destructuring-bind (type . pose) type-and-pose
                (spawn-object-n-times
                 type
                 pose
                 (+ (random 3) 1)
                 (list (cut:random-with-minimum 1.0 minimal-color)
                       (cut:random-with-minimum 1.0 minimal-color)
                       (cut:random-with-minimum 1.0 minimal-color))
                :x
                0.13)))
            type-and-pose-list)))

(defun spawn-random-box-objects-on-shelf (base-link-name
                                          &key
                                            list-of-level-link-names
                                            (maximal-box-size-in-level-frame
                                             '(0.15 0.1 0.2))
                                            (minimal-box-size-one-dimension
                                             0.03)
                                            (weight
                                             0.2)
                                            (space-between-objects
                                             0.05)
                                            (start-x-offset
                                             0.0)
                                            (minimal-color
                                             0.0))
  (setf list-of-level-link-names
        (or list-of-level-link-names
            (mapcar #'cl-urdf:name
                    (btr:find-levels-under-link
                     (gethash base-link-name
                              (cl-urdf:links
                               (btr:urdf
                                (btr:get-environment-object))))))))

  ;; for each shelf level
  (loop for link-name in list-of-level-link-names
        for level-transform = (cram-tf:pose->transform-stamped
                               cram-tf:*fixed-frame*
                               link-name
                               0.0
                               (btr:link-pose
                                (btr:get-environment-object)
                                link-name))
        for level-bb = (btr:calculate-bb-dims
                        (gethash link-name
                                 (btr:links (btr:get-environment-object))))
        for end-x-in-link-frame = (- (/ (cl-transforms:x level-bb) 2.0)
                                     space-between-objects)
        for start-x-in-link-frame = (+ (- end-x-in-link-frame)
                                       start-x-offset)
        for start-z-in-link-frame = (/ (cl-transforms:z level-bb) 2.0)
        ;; we assume that the face of the level is the -Y axis
        for end-y-in-link-frame = (- (/ (cl-transforms:y level-bb) 2.0)
                                     space-between-objects)
        for start-y-in-link-frame = (- end-y-in-link-frame)
        ;; for each row of items
        do (loop for color = (list (cut:random-with-minimum 1.0 minimal-color)
                                   (cut:random-with-minimum 1.0 minimal-color)
                                   (cut:random-with-minimum 1.0 minimal-color))
                 for box-size = (mapcar (lambda (dimension)
                                          (cut:random-with-minimum
                                           dimension
                                           minimal-box-size-one-dimension))
                                        maximal-box-size-in-level-frame)
                 for box-size/2 = (mapcar (alexandria:rcurry #'/ 2.0) box-size)
                 for z-in-link-frame = (+ start-z-in-link-frame
                                          (third box-size/2))
                 for x-in-link-frame = (+ start-x-in-link-frame
                                          (first box-size/2))
                   then (+ next-start-x-in-link-frame
                           (first box-size/2))
                 for next-start-x-in-link-frame = (+ x-in-link-frame
                                                     (first box-size/2)
                                                     space-between-objects)
                 while (< next-start-x-in-link-frame end-x-in-link-frame)
                 ;; go into the depth of the shelf
                 do (loop for next-start-y-in-link-frame = start-y-in-link-frame
                            then (+ y-in-link-frame
                                    (second box-size/2)
                                    space-between-objects)
                          for y-in-link-frame = (+ next-start-y-in-link-frame
                                                   (second box-size/2))
                          while (< next-start-y-in-link-frame end-y-in-link-frame)
                          do (let* ((name
                                      (intern (format nil "BOX-~a-~a-~a"
                                                      link-name
                                                      x-in-link-frame
                                                      y-in-link-frame)))
                                    (link-frame-t-box
                                      (cl-transforms-stamped:make-transform-stamped
                                       link-name "box" 0.0
                                       (cl-transforms:make-3d-vector
                                        x-in-link-frame
                                        y-in-link-frame
                                        z-in-link-frame)
                                       (cl-transforms:make-identity-rotation)))
                                    (pose-in-fixed-frame
                                      (cram-tf:apply-transform
                                       level-transform
                                       link-frame-t-box
                                       :result-as-pose-or-transform :pose))
                                    (btr-object
                                      (btr:add-object
                                       btr:*current-bullet-world*
                                       :box-item
                                       name
                                       pose-in-fixed-frame
                                       :mass weight
                                       :size box-size/2
                                       :color color
                                       :item-type :retail-item)))
                               (when (some (alexandria:rcurry #'typep 'btr:item)
                                           (btr:find-objects-in-contact
                                            btr:*current-bullet-world* btr-object))
                                 (btr:remove-object
                                  btr:*current-bullet-world* name)))))))

(defun spawn-objects-on-small-shelf (&optional (minimal-color 0.0))
  (sb-ext:gc :full t)
  (setf desig::*designators* (tg:make-weak-hash-table :weakness :key))
  ;; (btr-utils:kill-all-objects)
  (btr:detach-all-objects (btr:get-robot-object))
  (btr:detach-all-objects (btr:get-environment-object))
  (btr-utils:move-robot '((1.0 0 0) (0 0 0 1)))

  (let ((poses (make-poses-relative *small-shelf-poses*)))
    (mapcar (lambda (type-and-pose)
              (destructuring-bind (type . pose) type-and-pose
                (spawn-object-n-times type pose 1
                                      (case type
                                        (:dish-washer-tabs '(0 1 0))
                                        (:balea-bottle '(1 1 0))
                                        (t '(1.0 1.0 0.9))))))
            poses)
    (btr:add-object btr:*current-bullet-world*
                    :box-item
                    :small-shelf-dish-washer-tabs-collision-box
                    (cdr (find :dish-washer-tabs poses :key #'car))
                    :mass 0.0
                    :size '(0.15 0.1 0.1)
                    :color '(0 0 0)
                    :item-type :collision-thingy)
    (btr:add-object btr:*current-bullet-world*
                    :box-item
                    :small-shelf-balea-bottle-collision-box
                    (cdr (find :balea-bottle poses :key #'car))
                    :mass 0.0
                    :size '(0.15 0.1 0.1)
                    :color '(0 0 0)
                    :item-type :collision-thingy))

  (spawn-random-box-objects-on-shelf "shelf_2_base"
                                     :start-x-offset 0.05
                                     :minimal-color minimal-color)

  (btr:remove-object btr:*current-bullet-world* :small-shelf-dish-washer-tabs-collision-box)
  (btr:remove-object btr:*current-bullet-world* :small-shelf-balea-bottle-collision-box)
  ;; (btr:simulate btr:*current-bullet-world* 50)
  )

(defun spawn-objects-on-real-small-shelf ()
  (let ((poses (make-poses-relative *real-small-shelf-poses*)))
    (mapcar (lambda (type-and-pose)
              (destructuring-bind (type . pose) type-and-pose
                (spawn-object-n-times type pose 1
                                      (case type
                                        (:dish-washer-tabs '(0 1 0))
                                        (:balea-bottle '(1 0.5 0))
                                        (t '(1.0 1.0 0.9))))))
            poses)))

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
             (:boxy *basket-in-boxy-wrist*)
             (:tiago-dual *basket-in-tiago-wrist*)
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



(defun retail-demo ()
  ;; (setf cram-tf:*tf-broadcasting-enabled* t)
  ;; (roslisp-utilities:startup-ros)
  (urdf-proj:with-simulated-robot
    (if (eql (rob-int:get-robot-name) :kmr-iiwa)
        (setf btr:*visibility-threshold* 0.7)
        (setf btr:*visibility-threshold* 0.5))
    (kill-and-detach-all)
    (let ((?pose (cl-transforms-stamped:make-pose-stamped
                  "map" 0.0
                  (cl-transforms-stamped:make-3d-vector 2 0 0.0d0)
                  (cl-transforms:make-quaternion 0 0 1 0))))
      (exe:perform
       (desig:a motion
                (type going)
                (pose ?pose))))
    (if (eql (rob-int:get-environment-name) :store)
        (spawn-objects-on-real-small-shelf)
        (progn
          (spawn-objects-on-small-shelf 0.6)
          (spawn-objects-on-big-shelf 0.6)))
    (unless (member (rob-int:get-robot-name) '(:iai-donbot :kmr-iiwa))
      (spawn-basket))

    (let* ((?source-shelf-base-urdf-name
             (if (eql (rob-int:get-environment-name) :store)
                 :|DMShelfW100_EVZDYXFU|
                 :shelf-2-base))
           (?source-shelf-base-level
             (if (eql (rob-int:get-environment-name) :store)
                 4 ;3
                 4))
           (?target-shelf-level-urdf-name
             (if (eql (rob-int:get-environment-name) :store)
                 :|DMFloorT6W100_YVLKGJSB| ; :|DMFloorT6W100_KYINFGDM|
                 :shelf-1-level-2-link))
           (?target-shelf-dishwasher-attachments
             (if (eql (rob-int:get-environment-name) :store)
                 '(;; :dish-washer-tabs-real-shelf-1-front
                   :dish-washer-tabs-real-shelf-1-back)
                 '(:dish-washer-tabs-shelf-1-front
                   :dish-washer-tabs-shelf-1-back)))
           (?target-shelf-balea-attachments
             (if (eql (rob-int:get-environment-name) :store)
                 '(;; :balea-bottle-real-shelf-1-front
                   :balea-bottle-real-shelf-1-back)
                 '(:balea-bottle-shelf-1-front
                   :balea-bottle-shelf-1-back)))
           (?environment-name
             (rob-int:get-environment-name))
           (?robot-name
             (rob-int:get-robot-name))
           (?search-location
             (desig:a location
                      (on (desig:an object
                                    (type shelf)
                                    (urdf-name ?source-shelf-base-urdf-name)
                                    (part-of ?environment-name)
                                    (level ?source-shelf-base-level)))
                      (side left)
                      (range 0.2)))

           (?dish-washer-tabs-desig
             (desig:an object
                       (type dish-washer-tabs)
                       (location ?search-location)))
           (?balea-bottle-desig
             (desig:an object
                       (type balea-bottle)
                       (location ?search-location)))
           (?target-location-shelf-dish-washer-tabs
             (desig:a location
                      (on (desig:an object
                                    (type environment)
                                    (name ?environment-name)
                                    (part-of ?environment-name)
                                    (urdf-name ?target-shelf-level-urdf-name)))
                      (for ?dish-washer-tabs-desig)
                      (attachments ?target-shelf-dishwasher-attachments)))
           (?target-location-shelf-balea-bottle
             (desig:a location
                      (on (desig:an object
                                    (type environment)
                                    (name ?environment-name)
                                    (part-of ?environment-name)
                                    (urdf-name ?target-shelf-level-urdf-name)))
                      (for ?balea-bottle-desig)
                      (attachments ?target-shelf-balea-attachments)))
           (?target-location-donbot-tray-dish-washer-tabs
             (desig:a location
                      (on (desig:an object
                                    (type robot)
                                    (name ?robot-name)
                                    (part-of ?robot-name)
                                    (owl-name "donbot_tray")
                                    (urdf-name plate)))
                      (for ?dish-washer-tabs-desig)
                      (attachments (donbot-tray-back donbot-tray-front))))
           (?target-location-kukabot-tray-dish-washer-tabs
             (desig:a location
                      (on (desig:an object
                                    (type robot)
                                    (name ?robot-name)
                                    (part-of ?robot-name)
                                    (owl-name "kukabot_tray")
                                    (urdf-name base-link)))
                      (for ?dish-washer-tabs-desig)
                      (attachments (kukabot-tray-back kukabot-tray-front))))
           (?target-location-basket-dish-washer-tabs
             (desig:a location
                      (on (desig:an object
                                    (type basket)
                                    (name b)))
                      (for ?dish-washer-tabs-desig)
                      (attachments (in-basket-back
                                    in-basket-front
                                    in-basket-other-back
                                    in-basket-other-front))))
           (?target-location-robot-dish-washer-tabs
             (case ?robot-name
               (:iai-donbot
                ?target-location-donbot-tray-dish-washer-tabs)
               (:kmr-iiwa
                ?target-location-kukabot-tray-dish-washer-tabs)
               (t
                ?target-location-basket-dish-washer-tabs))))

      (cpl:with-failure-handling
          ((cpl:simple-plan-failure (e)
             (declare (ignore e))
             (return)))
        (exe:perform
         (desig:an action
                   (type transporting)
                   (object ?dish-washer-tabs-desig)
                   (target ?target-location-robot-dish-washer-tabs)
                   ;; (grasps (back))
                   )))
      (cpl:with-failure-handling
          ((cpl:simple-plan-failure (e)
             (declare (ignore e))
             (return)))
        (exe:perform
         (desig:an action
                   (type transporting)
                   (object ?balea-bottle-desig)
                   (target ?target-location-shelf-balea-bottle)
                   ;; (grasps (back))
                   )))
      (cpl:with-failure-handling
          ((cpl:simple-plan-failure (e)
             (declare (ignore e))
             (return)))
        (exe:perform
         (desig:an action
                   (type transporting)
                   (object ?dish-washer-tabs-desig)
                   (target ?target-location-shelf-dish-washer-tabs)
                   ;; vvv donbot tries to grasp through itself otherwise
                   (grasps (back)))))

      ;; look at separators
      ;; (exe:perform
      ;;  (desig:an action
      ;;            (type looking)
      ;;            (direction right-separators)))
      ;; (cpl:sleep 5.0)
      )))


(defmethod cram-occasions-events:on-event
    publish-object 3 ((event cram-plan-occasions-events:robot-state-changed))

  (let ((items (remove-if-not (lambda (object)
                                (typep object 'btr:item))
                              (btr:objects btr:*current-bullet-world*))))
    (dolist (item items)
      (let* ((name
               (btr:name item))
             (ros-name
               (roslisp-utilities:rosify-underscores-lisp-name name))
             (pose
               (btr:pose item))
             (mesh-path
               (second (assoc (car (btr:item-types item)) btr::*mesh-files*)))
             (color
               (cl-bullet-vis:collision-shape-color
                (cl-bullet:collision-shape
                 (btr:rigid-body item name)))))
        (cram-tf:visualize-marker pose
                                  ;; :topic "cram_items"
                                  :namespace ros-name
                                  :marker-type :mesh_resource
                                  :scale-list '(1 1 1)
                                  :r-g-b-list color
                                  :mesh-path mesh-path)))))












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
