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
    (:balea-bottle . ((-0.15 -0.1 0.1) (0 0 0.5 0.5)))
    (:dish-washer-tabs . ((0 -0.13 0.11) (0 0 0.5 0.5)))))

(defparameter *basket-in-pr2-wrist*
  (cl-transforms:make-transform
   (cl-transforms:make-3d-vector 0.36 0 -0.15)
   (cl-transforms:make-quaternion 0.0d0 -0.7071067811865475d0
                                  0.0d0 0.7071067811865475d0)))

(defparameter *basket-in-boxy-wrist*
  (cl-transforms:make-transform
   (cl-transforms:make-3d-vector 0 -0.15 0.48)
   (cl-transforms:make-quaternion 0.5 0.5 0 0)))

(defun spawn-object-n-times (type pose times color &optional offset-axis offset)
  "`offset-axis' can be one of :x, :y or :z."
  (loop for i from 0 to (1- times)
        for name = (intern (string-upcase (format nil "~a-~a" type i)) :keyword)
        do (when offset-axis
             (setf pose (cram-tf:translate-pose pose offset-axis offset)))
           ;; if object with name NAME already exists, create a new unique name
           (when (btr:object btr:*current-bullet-world* name)
             (setf i (+ i times)
                   name (intern (string-upcase (format nil "~a-~a" type i))
                                :keyword)))
           (btr:add-object btr:*current-bullet-world* :mesh name pose :mesh type
                           :mass 0.0
                           :color color)))

(defun spawn-objects-on-big-shelf ()
  (let ((type-and-pose-list
          (make-poses-relative-multiple *big-shelf-poses* "shelf_1_")))
    (mapcar (lambda (type-and-pose)
              (destructuring-bind (type . pose) type-and-pose
                (spawn-object-n-times
                 type
                 pose
                 (+ (random 3) 1) ; times
                `(,(float (/ (random 10) 10)) ; color
                  ,(float (/ (random 10) 10))
                  ,(float (/ (random 10) 10)))
                :x
                0.13)))
            type-and-pose-list)))

(defun spawn-objects-on-small-shelf ()
  (sb-ext:gc :full t)
  (setf desig::*designators* (tg:make-weak-hash-table :weakness :key))
  (btr:clear-costmap-vis-object)
  ;; (btr-utils:kill-all-objects)
  (btr:detach-all-objects (btr:get-robot-object))
  (btr:detach-all-objects (btr:get-environment-object))
  (mapcar (lambda (type-and-pose)
            (destructuring-bind (type . pose) type-and-pose
              (spawn-object-n-times type pose 1
                                    (case type
                                      (:dish-washer-tabs '(0 1 0))
                                      (t '(1.0 1.0 0.9))))))
          (make-poses-relative *small-shelf-poses*))
  ;; (btr:simulate btr:*current-bullet-world* 50)
  (btr-utils:move-robot '((1.0 0 0) (0 0 0 1))))

(defun spawn-random-box-objects-on-shelf (base-link-name &optional list-of-level-link-names maximal-box-size)
  (let* ((maximal-size (if (equalp maximal-box-size NIL)
                               (cl-transforms:make-3d-vector 0.3 0.4 0.3)
                               maximal-box-size))
         (max-boxes (/ 0.9 (cl-transforms:y maximal-size)))
         ;;(offset-between-object (/ (- 0.85 (* max-boxes y)) (+ max-boxes 2)))
         (list-of-level-link-names (if (eql list-of-level-link-names NIL)
                                       (btr-spatial-cm::find-levels-under-link base-link-name)
                                       list-of-level-link-names))
         (level-poses (mapcar (lambda (level-name)
                                (cl-transforms:origin
                                 (btr:link-pose (btr:get-environment-object) level-name)))
                              list-of-level-link-names)))

    (print level-poses)
    (loop for pose in level-poses
          do (let ((prev-x 0.05)
                   (pos-x (- (cl-transforms:x pose) 0.45)))
               (loop for i from 0 to max-boxes
                     do  (let* ((box-size-rand `(,(random (cl-transforms:x maximal-size))
                                                 ,(random (cl-transforms:y maximal-size))
                                                 ,(random (cl-transforms:z maximal-size))))
                                (x (first box-size-rand))
                                (offset-between-object (+ (/ (+ x prev-x) 2) 0.05))
                                (spawn-pose `((,(setf pos-x (+ pos-x offset-between-object))
                                               ,(cl-transforms:y pose)
                                               ,(+ (cl-transforms:z pose) (/ (third box-size-rand) 2)))
                                              (0 0 0 1)))
                                (color-rand `(,(float (/ (random 10) 10))
                                              ,(float (/ (random 10) 10))
                                              ,(float (/ (random 10) 10)))))
                           (setf prev-x (first box-size-rand))
                           (print spawn-pose)
                           (btr:add-object btr:*current-bullet-world*
                                           :colored-box (intern (concatenate 'string "box-" (write-to-string i)))
                                           (cram-tf:list->pose spawn-pose) :mass 1 :size box-size-rand
                                                                           :color color-rand)))))))
         
                                      

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



(defun retail-demo ()
  (urdf-proj:with-simulated-robot

    (setf btr:*visibility-threshold* 0.5)
    (btr-utils:kill-all-objects)
    (spawn-objects-on-small-shelf)
    (spawn-objects-on-big-shelf)
    (unless (eql (rob-int:get-robot-name) :iai-donbot)
      (spawn-basket))

    (let* ((?environment-name
             (rob-int:get-environment-name))
           (?robot-name
             (rob-int:get-robot-name))
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
                                    (urdf-name shelf-1-level-2-link)))
                      (for ?dish-washer-tabs-desig)
                      (attachments (dish-washer-tabs-shelf-1-front
                                    dish-washer-tabs-shelf-1-back))))
           (?target-location-shelf-balea-bottle
             (desig:a location
                      (on (desig:an object
                                    (type environment)
                                    (name ?environment-name)
                                    (part-of ?environment-name)
                                    (urdf-name shelf-1-level-2-link)))
                      (for ?balea-bottle-desig)
                      (attachments (balea-bottle-shelf-1-front
                                    balea-bottle-shelf-1-back))))
           (?target-location-tray-dish-washer-tabs
             (desig:a location
                      (on (desig:an object
                                    (type robot)
                                    (name ?robot-name)
                                    (part-of ?environment-name)
                                    (owl-name "donbot_tray")
                                    (urdf-name plate)))
                      (for ?dish-washer-tabs-desig)
                      (attachments (donbot-tray-front donbot-tray-back))))
           (?target-location-basket-dish-washer-tabs
             (desig:a location
                      (on (desig:an object
                                    (type basket)
                                    (name b)))
                      (for ?dish-washer-tabs-desig)
                      (attachments (; in-basket-front
                                    in-basket-back))))
           (?target-location-robot-dish-washer-tabs
             (case ?robot-name
               (:iai-donbot ?target-location-tray-dish-washer-tabs)
               (t ?target-location-basket-dish-washer-tabs))))

      (exe:perform
       (desig:an action
                 (type transporting)
                 (object ?dish-washer-tabs-desig)
                 (target ?target-location-robot-dish-washer-tabs)))
      (exe:perform
       (desig:an action
                 (type transporting)
                 (object ?balea-bottle-desig)
                 (target ?target-location-shelf-balea-bottle)))
      (exe:perform
       (desig:an action
                 (type transporting)
                 (object ?dish-washer-tabs-desig)
                 (target ?target-location-shelf-dish-washer-tabs)))

      ;; look at separators
      ;; (exe:perform
      ;;  (desig:an action
      ;;            (type looking)
      ;;            (direction right-separators)))
      ;; (cpl:sleep 5.0)
      )))















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
