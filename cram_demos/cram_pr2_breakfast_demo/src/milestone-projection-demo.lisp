;;;
;;; Copyright (c) 2022, Vanessa Hassouna <hassouna@cs.uni-bremen.de>
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

;; (setf cram-tf:*tf-broadcasting-enabled* t)

(defparameter *demo-object-spawning-poses*
  '((:bowl
     "sink_area_left_middle_drawer_main"
     ((0.10 -0.0505 -0.062256) (0 0 -1 0)))
    ;; (:cup
    ;;  "sink_area_left_bottom_drawer_main"
    ;;  ((0.11 0.12 -0.0547167) (0 0 -1 0)))
    (:plate
     "kitchen_island_left_upper_drawer_main"
     ((0.11 0.08 -0.026367) (0 0 -1 0)))
    (:spatula
     ;; "oven_area_area_middle_upper_drawer_main"
     "sink_area_left_upper_drawer_main"
     ((0.125 0 -0.0136) (0 -0.053938 -0.998538 -0.003418)))
    ;; So far only this orientation works
    (:mondamin
     "oven_area_area_right_drawer_board_3_link"
     ((0.123 -0.03 0.11) (0.0087786 0.005395 -0.838767 -0.544393)))
    ;; ((:breakfast-cereal . ((1.398 1.490 1.2558) (0 0 0.7071 0.7071)))
    ;; (:breakfast-cereal . ((1.1 1.49 1.25) (0 0 0.7071 0.7071)))
    (:milk
     ;; "iai_fridge_main_middle_level"
     ;; ((0.10355 0.022 0.094) (0.00939 -0.00636 -0.96978 -0.2437))
     "iai_fridge_door_shelf1_bottom"
     ((-0.01 -0.05 0.094) (0 0 0 1)))
    (:pancake-maker
     "kitchen_island_surface"
     ((0.0 0.0 0.01) (0 0 0 1)))
    ))

(defun attach-object-to-the-world (object-type spawning-poses-relative)
  (when spawning-poses-relative
    (btr:attach-object (btr:get-environment-object)
                       (btr:object btr:*current-bullet-world*
                                   (intern (format nil "~a-1" object-type) :keyword))
                       :link (second (find object-type
                                           spawning-poses-relative
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
                                        '(:mondamin :plate :bowl :milk :spatula)))
  (btr-utils:kill-all-objects)
  (btr:add-objects-to-mesh-list "cram_pr2_breakfast_demo")
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

  (mapcar (alexandria:rcurry #'attach-object-to-the-world spawning-poses-relative)
          object-types)
  (btr-utils:spawn-object 'green-dot
                              :pancake-maker
                              :color '(0 1 0 0.5)
                              :pose '((-0.95 0.9 0.9) (0 0 0 1)))
  )



(defun setting-demo (&optional (object-list '(:bowl :spoon :cup
                                              :milk :breakfast-cereal)))
  (initialize)
  (setf btr:*visibility-threshold* 0.7)
  (when (or cram-projection:*projection-environment*
            ;; dont want to add dependency on sim PMs, thus this stupid hack
            (roslisp:get-param "/base_simulator/sim_frequency" nil))
    (spawn-objects-on-fixed-spots
     :object-types object-list
     :spawning-poses-relative *demo-object-spawning-poses*))
  ;; (park-robot)

  (dolist (?object-type object-list)
    (let* ((?deliver-pose (cram-tf:ensure-pose-in-frame
                           (btr:ensure-pose
                            (cdr (assoc ?object-type
                                        *delivery-poses-island-absolute*)))
                           cram-tf:*fixed-frame*))
           (?deliver-location (a location (pose ?deliver-pose)))
           (?color (cdr (assoc ?object-type *object-colors*)))
           (?arm (cdr (assoc ?object-type *object-arms*)))
           (?material (cdr (assoc ?object-type *object-materials*)))
           (?grasp (cdr (assoc ?object-type *object-grasps*)))
           (?object (an object
                        (type ?object-type)
                        ;; (location ?fetch-location)
                        (desig:when ?color
                          (color ?color))
                        (desig:when ?material
                          (material ?material)))))
      
      (exe:perform
       (an action
           (type transporting)
           (object ?object)
           (context :table-setting)
           ;;(grasps (:top ))
           (desig:when ?arm
               (arms (?arm)))
           ;(:?right-grasp :top)
           (desig:when ?grasp
             (grasp ?grasp))
           (target ?deliver-location)
           )))))

(defun cleaning-demo (&optional (object-list '(:breakfast-cereal :milk
                                               :spoon :cup :bowl)))
  "Cleans up object to the designated locations by iterating over
`object-list' "
  ;; (setup-for-demo object-list)
  (initialize)
  (when (or cram-projection:*projection-environment*
            ;; dont want to add dependency on sim PMs, thus this stupid hack
            (roslisp:get-param "/base_simulator/sim_frequency" nil))
    (spawn-objects-on-fixed-spots
     :object-types object-list
     :spawning-poses-relative *delivery-poses-dining-table-relative*))

  (dolist (?object-type object-list)
    (let ((?grasps (when (eq ?object-type :cup)
                     '(:front :back :left-side :right-side))))
      (exe:perform
       (desig:an action
                 (type transporting)
                 (object (desig:an object (type ?object-type)))
                 (context table-cleaning)
                 (desig:when ?grasps
                   (grasps ?grasps)))))))

(defun reset-world ()
  (cl-bullet-vis:show-window cram-bullet-reasoning:*debug-window*))


(defparameter *delivery-poses-dining-island-relative*
  `((:milk
     "kitchen_island_surface"
     ((-0.012 -0.031 0.4862768868605296d0 ;; 0.8362768809000651d0
              )
      (-0.00932157 0.00720728 0.965580536 0.25983724)))))


(defparameter *delivery-poses-island-absolute*
  `((:milk . ((-0.9 1.5 0.98) (0.0 1 1 1)))
    (:bowl . ((-0.9 1.9 0.95) (0 0 0 1)))))


;; (defparameter *delivery-poses*
;;   `((:bowl . ((-0.8399440765380859d0 1.2002920786539713d0 0.8932199478149414d0)
;;               (0.0 0.0 0.33465 0.94234)))
;;     (:cup . ((-0.8908212025960287d0 1.4991984049479166d0 0.9027448018391927d0)
;;              (0.0 0.0 0.33465 0.94234)))
;;     (:spoon . ((-0.8409400304158529d0 1.38009208679199219d0 0.8673268000284831d0)
;;                (0.0 0.0 1 0)))
;;     (:milk . ((-0.8495257695515951d0 1.6991498311360678d0 0.9483174006144206d0)
;;               (0.0 0.0 -0.9 0.7)))
;;     (:breakfast-cereal . ((-0.78 0.8 0.95) (0 0 0.6 0.4)))))

;; (defparameter *cleaning-deliver-poses*
;;   `((:bowl . ((1.45 -0.4 1.0) (0 0 0 1)))
;;     (:cup . ((1.45 -0.4 1.0) (0 0 0 1)))
;;     (:spoon . ((1.45 -0.4 1.0) (0 0 0 1)))
;;     (:milk . ((1.2 -0.5 0.8) (0 0 1 0)))
;;     (:breakfast-cereal . ((1.15 -0.5 0.8) (0 0 1 0)))))


