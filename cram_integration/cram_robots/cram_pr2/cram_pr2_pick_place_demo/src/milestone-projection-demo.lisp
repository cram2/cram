;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;                     Amar Fayaz <amar@uni-bremen.de>
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

(in-package :demo)

(defparameter *demo-object-spawning-poses*
  '((:bowl
     "sink_area_left_middle_drawer_main"
     ((0.10573 -0.1505 -0.062256) (0 0 -1 0)))
    (:cup
     "sink_area_left_middle_drawer_main"
     ((0.1275 0.12 -0.0547167) (0 0 -1 0)))
    (:spoon
     ;; "oven_area_area_middle_upper_drawer_main"
     "sink_area_left_upper_drawer_main"
     ((0.125 0 -0.0136) (0 -0.053938 -0.998538 -0.003418)))
    ;; So far only this orientation works
    (:breakfast-cereal
     "oven_area_area_right_drawer_board_3_link"
     ((0.123 -0.03 0.11) (0.0087786 0.005395 -0.838767 -0.544393)))
    ;; ((:breakfast-cereal . ((1.398 1.490 1.2558) (0 0 0.7071 0.7071)))
    ;; (:breakfast-cereal . ((1.1 1.49 1.25) (0 0 0.7071 0.7071)))
    (:milk
     ;; "iai_fridge_main_middle_level"
     ;; ((0.10355 0.022 0.094) (0.00939 -0.00636 -0.96978 -0.2437))
     "iai_fridge_door_shelf1_bottom"
     ((-0.01 -0.05 0.094) (0 0 0 1)))))


(defparameter *delivery-poses*
  `((:bowl . ((-0.8399440765380859d0 1.2002920786539713d0 0.8932199478149414d0)
              (0.0 0.0 0.33465 0.94234)))
    (:cup . ((-0.8908212025960287d0 1.4991984049479166d0 0.9027448018391927d0)
             (0.0 0.0 0.33465 0.94234)))
    (:spoon . ((-0.8409400304158529d0 1.38009208679199219d0 0.8673268000284831d0)
               (0.0 0.0 1 0)))
    (:milk . ((-0.8495257695515951d0 1.6991498311360678d0 0.9483174006144206d0)
              (0.0 0.0 -0.9 0.7)))
    (:breakfast-cereal . ((-0.78 0.8 0.95) (0 0 0.6 0.4)))))

(defparameter *cleaning-deliver-poses*
  `((:bowl . ((1.45 -0.4 1.0) (0 0 0 1)))
    (:cup . ((1.45 -0.4 1.0) (0 0 0 1)))
    (:spoon . ((1.45 -0.4 1.0) (0 0 0 1)))
    (:milk . ((1.2 -0.5 0.8) (0 0 1 0)))
    (:breakfast-cereal . ((1.15 -0.5 0.8) (0 0 1 0)))))


(defun attach-object-to-the-world (object-type)
  (when *demo-object-spawning-poses*
    (btr:attach-object (btr:get-environment-object)
                       (btr:object btr:*current-bullet-world*
                                   (intern (format nil "~a-1" object-type) :keyword))
                       :link (second (find object-type
                                           *demo-object-spawning-poses*
                                           :key #'car)))))

(defun make-poses-list-relative (spawning-poses-list)
  "Gets a list of ((:type \"frame\" cords-list) ...)
Converts these coordinates into CRAM-TF:*FIXED-FRAME* frame and returns a list in form
 ((TYPE . POSE) ...)."
  (when spawning-poses-list
    (mapcar (lambda (type-and-frame-and-pose-list)
              (destructuring-bind (type frame pose-list)
                  type-and-frame-and-pose-list
                (let* ((map-T-surface
                         (cl-transforms:pose->transform
                          (btr:link-pose (btr:get-environment-object) frame)))
                       (surface-T-object
                         (cl-transforms:pose->transform
                          (cram-tf:list->pose pose-list)))
                       (map-T-object
                         (cl-transforms:transform* map-T-surface surface-T-object))
                       (map-P-object
                         (cl-transforms:transform->pose map-T-object)))
                  `(,type . ,map-P-object))))
            spawning-poses-list)))

(defun spawn-objects-on-fixed-spots (&key
                                       (spawning-poses-relative
                                        *demo-object-spawning-poses*)
                                       (object-types
                                        '(:breakfast-cereal :cup :bowl :spoon :milk)))
  (btr-utils:kill-all-objects)
  (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo")
  (btr:detach-all-objects (btr:get-robot-object))
  ;; spawn objects at default poses
  (let* ((spawning-poses-absolute
           (make-poses-list-relative spawning-poses-relative))
         (objects (mapcar (lambda (object-type)
                            (btr-utils:spawn-object
                             (intern (format nil "~a-1" object-type) :keyword)
                             object-type
                             :pose (cdr (assoc object-type
                                               ;; *demo-object-spawning-poses*
                                               spawning-poses-absolute
                                               ))))
                          object-types)))
    ;; stabilize world
    ;; (btr:simulate btr:*current-bullet-world* 100)
    objects)

  (mapcar #'attach-object-to-the-world object-types))



(defun setup-for-demo (object-list)
  (initialize)
  (when cram-projection:*projection-environment*
    (spawn-objects-on-fixed-spots :object-types object-list
                                  :spawning-poses-relative *delivery-poses*))
  ;; (park-robot)
  )

(defun projection-init ()
  (setf btr:*visibility-threshold* 0.7)
  (setf cram-urdf-projection:*debug-short-sleep-duration* 0.0)
  (setf cram-urdf-projection:*debug-long-sleep-duration* 0.0)
  (setf cram-urdf-projection-reasoning::*projection-checks-enabled* t)
  (spawn-objects-on-fixed-spots))

(defun setting-demo (&optional (object-list '(:milk :breakfast-cereal
                                              :bowl :spoon :cup)))
  (initialize)
  (when cram-projection:*projection-environment*
    (spawn-objects-on-fixed-spots))
  ;; (park-robot)
  ;; (setup-for-demo object-list)

  (dolist (?object-type object-list)
    (let* ((?deliver-pose (cram-tf:ensure-pose-in-frame
                           (btr:ensure-pose
                            (cdr (assoc ?object-type *delivery-poses*)))
                           cram-tf:*fixed-frame*))
           (?deliver-location (a location (pose ?deliver-pose)))
           (?color (cdr (assoc ?object-type *object-colors*)))
           ;; (?grasp (cdr (assoc ?object-type *object-grasps*)))
           (?object (an object
                        (type ?object-type)
                        ;; (location ?fetch-location)
                        (desig:when ?color
                          (color ?color)))))
      (exe:perform
       (an action
           (type transporting)
           (object ?object)
           (context :table-setting)
           ;; (grasps (:back :top :front))
           ;; (arms (left right))
           ;; (desig:when ?grasp
           ;;   (grasp ?grasp))
           (target ?deliver-location))))))

(defun cleaning-demo (&optional (object-list '(:milk :breakfast-cereal
                                                :bowl :spoon :cup)))
  "Cleans up object to the designated locations by iterating over
`object-list' "
  ;; (setup-for-demo object-list)

  (let* ((object-cleanup-locations '((:milk :trash)
                                     (:bowl :sink)
                                     (:spoon :sink)
                                     (:cup :sink)
                                     (:breakfast-cereal :vertical-drawer)))
         (?fetch-table-location (a location
                                   (on (an object
                                           (type counter-top)
                                           (urdf-name kitchen-island-surface)
                                           (part-of iai-kitchen)))
                                   (side back)
                                   (side right)))
         (deliver-location-lambdas
           `((:sink ,(lambda (?object-type)
                       (a location
                          (above (an object
                                     (type sink)
                                     (urdf-name sink-area-sink)
                                     (part-of iai-kitchen)))
                          (side right)
                          ;; the "for" condition for spoon adds a height
                          ;; that is too high for pr2 to reach
                          (desig:when (not (eq ?object-type :spoon))
                            (for (an object (type ?object-type)))))))
             (:trash ,(lambda (?object-type)
                        (a location
                           (above (an object
                                      (type drawer)
                                      (urdf-name sink-area-trash-drawer-main)
                                      (part-of iai-kitchen)))
                           (z-offset 0.1)
                           (side front)
                           (side right)
                           (range 0.2)
                           (for (an object (type ?object-type))))))
             (:vertical-drawer
              ,(lambda (?object-type)
                 (a location
                    (above (an object
                               (type drawer)
                               (urdf-name oven-area-area-right-drawer-main)
                               (part-of iai-kitchen)
                               (level topmost)))
                    (for (an object (type ?object-type))))))))
         (get-delivery-location-designator
           (lambda (object-type)
             (let ((del-location-key
                     (second (assoc object-type object-cleanup-locations))))
               (funcall (second
                         (assoc del-location-key deliver-location-lambdas))
                        object-type)))))

    (loop for ?obj in object-list
          do (let ((?delivery-location
                     (funcall get-delivery-location-designator ?obj)))
               (perform (an action
                          (type transporting)
                          (object (an object
                                      (type ?obj)
                                      (location ?fetch-table-location)))
                          (location ?fetch-table-location)
                          (target ?delivery-location)))))))
