;;;
;;; Copyright (c) 2019, Amar Fayaz <amar@uni-bremen.de>
;;;                     Arthur Niedzwiecki <aniedz@cs.uni-bremen.de>
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
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
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

(in-package :btr-belief-tests)

(defparameter *kitchen-changed* t)

(defun init-env ()
  "Resets the world. Only resets kitchen if it changed."
  (when *kitchen-changed*
    (let ((kitchen-urdf
            (cl-urdf:parse-urdf
             (roslisp:get-param "kitchen_description"))))
      (prolog:prolog
       `(and (btr:bullet-world ?world)
             (assert (btr:object ?world
                                 :urdf
                                 :iai-kitchen ((0 0 0) (0 0 0 1))
                                 :urdf ,kitchen-urdf
                                 :collision-group :static-filter
                                 :collision-mask (:default-filter
                                                  :character-filter)
                                 :compound T)))))
    (setf *kitchen-changed* nil))
  (btr:prolog-?w
    `(btr:item-type ?w ?obj ?type)
    `(btr:retract (btr:object ?w ?obj)))
  (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo"))


(define-test sink-counter-stable-position-test
  (let* ((obj-name 'bowl-1)
         (spawn-pose (cl-transforms-stamped:make-pose-stamped
                      "map" 0.0
                      (cl-transforms:make-3d-vector 1.4 0.8 0.9)
                      (cl-transforms:make-identity-rotation))))
    (init-env)
    (prolog:prolog `(and (btr:bullet-world ?world)
                         (assert (btr:object ?world :mesh ,obj-name
                                             ,(cram-tf:pose->list spawn-pose)
                                             :mass 0.2 :color (1 0 0) :mesh :bowl))))
    (btr:simulate btr:*current-bullet-world* 100)
    (assert-true (< (cl-transforms:v-dist
                     (cl-transforms:origin spawn-pose)
                     (cl-transforms:origin
                      (btr:pose (btr:object btr:*current-bullet-world* obj-name))))
                    0.1))))

(define-test sink-counter-unstable-position-test
  (let* ((obj-name 'bowl-1)
         (spawn-pose (cl-transforms-stamped:make-pose-stamped
                      "map" 0.0
                      (cl-transforms:make-3d-vector 1.4 0.8 1.9)
                      (cl-transforms:make-identity-rotation))))
    (init-env)
    (prolog:prolog `(and (btr:bullet-world ?world)
                         (assert (btr:object ?world :mesh ,obj-name
                                             ,(cram-tf:pose->list spawn-pose)
                                             :mass 0.2 :color (1 0 0) :mesh :bowl))))
    (btr:simulate btr:*current-bullet-world* 100)
    (assert-true (> (cl-transforms:v-dist
                     (cl-transforms:origin spawn-pose)
                     (cl-transforms:origin
                      (btr:pose (btr:object btr:*current-bullet-world* obj-name))))
                    0.1))))

(define-test a-case-that-our-approach-cannot-correct-but-would-still-be-fine-test
  (let* ((obj-name 'bowl-1)
         (spawn-pose (cl-transforms-stamped:make-pose-stamped
                      "map" 0.0
                      (cl-transforms:make-3d-vector 1.53 1.25 0.9)
                      (cl-transforms:make-identity-rotation))))
    (init-env)
    (prolog:prolog `(and (btr:bullet-world ?world)
                         (assert (btr:object ?world :mesh ,obj-name
                                             ,(cram-tf:pose->list spawn-pose)
                                             :mass 0.2 :color (1 0 0) :mesh :bowl))))
    ;; assert that although the object is unstable,
    ;; the distance is not high enough to correct
    (assert-false (btr:stable-p (btr:object btr:*current-bullet-world* obj-name)))
    (btr:simulate btr:*current-bullet-world* 100)
    (assert-true (< (cl-transforms:v-dist
                     (cl-transforms:origin spawn-pose)
                     (cl-transforms:origin (btr:pose (btr:object
                                                      btr:*current-bullet-world*
                                                      obj-name))))
                    btr-belief:*perception-instability-threshold*))
    (init-env)
    (prolog:prolog `(and (btr:bullet-world ?world)
                         (assert (btr:object ?world :mesh ,obj-name
                                             ,(cram-tf:pose->list spawn-pose)
                                             :mass 0.2 :color (1 0 0) :mesh :bowl))))
    (btr-belief::stabilize-perceived-object-pose
     btr:*current-bullet-world* obj-name spawn-pose)
    (assert-true (< (cl-transforms:v-dist
                     (cl-transforms:origin spawn-pose)
                     (cl-transforms:origin (btr:pose (btr:object
                                                      btr:*current-bullet-world*
                                                      obj-name))))
                    btr-belief:*perception-instability-threshold*))))


(define-test bowl-on-edge-falling-stabilize-test
  "Spawns bowl on the edge of the sink area surface, about to fall.
Tests if the bowl is stable after correction."
  (let* ((?obj-name 'bowl-1)
         (spawn-pose (cl-transforms-stamped:make-pose-stamped
                      "map" 0.0
                      (cl-transforms:make-3d-vector 1.27 0.8 0.9)
                      (cl-transforms:make-identity-rotation)))
         (bowl-desig (desig:an object (type bowl) (name ?obj-name))))
    ;; Prepare the object designator to be fired in the perceive event.
    (setf (slot-value bowl-desig 'desig:data)
          (make-instance 'desig:object-designator-data
            :object-identifier ?obj-name
            :pose spawn-pose))
    (init-env)
    (prolog:prolog `(and (btr:bullet-world ?world)
                         (assert (btr:object ?world :mesh ,?obj-name
                                             ,(cram-tf:pose->list spawn-pose)
                                             :mass 0.2 :color (1 0 0) :mesh :bowl))))
    ;; Verify that object is unstable.
    (btr:simulate btr:*current-bullet-world* 10)
    (assert-false (btr:stable-p (btr:object btr:*current-bullet-world* ?obj-name)))
    ;; reset object
    (init-env)
    (prolog:prolog `(and (btr:bullet-world ?world)
                         (assert (btr:object ?world :mesh ,?obj-name
                                             ,(cram-tf:pose->list spawn-pose)
                                             :mass 0.2 :color (1 0 0) :mesh :bowl))))
    ;; Fire event
    (cram-occasions-events:on-event
     (make-instance 'cram-plan-occasions-events:object-perceived-event
       :object-designator bowl-desig
       :perception-source :whatever))
    ;; Verify object is stable.
    (btr:simulate btr:*current-bullet-world* 10)
    (assert-true (btr:stable-p (btr:object btr:*current-bullet-world* ?obj-name)))))


(define-test bowl-on-other-side-falling-stabilize-test
  "Spawns bowl on the edge of the sink area surface, about to fall.
Tests if the bowl is stable after correction."
  (let* ((?obj-name 'bowl-1)
         (spawn-pose (cl-transforms-stamped:make-pose-stamped
                      "map" 0.0
                      (cl-transforms:make-3d-vector -0.6607 1.2022 0.89)
                      (cl-transforms:make-identity-rotation)))
         (bowl-desig (desig:an object (type bowl) (name ?obj-name))))
    ;; Prepare the object designator to be fired in the perceive event.
    (setf (slot-value bowl-desig 'desig:data)
          (make-instance 'desig:object-designator-data
            :object-identifier ?obj-name
            :pose spawn-pose))
    (init-env)
    (prolog:prolog `(and (btr:bullet-world ?world)
                         (assert (btr:object ?world :mesh ,?obj-name
                                             ,(cram-tf:pose->list spawn-pose)
                                             :mass 0.2 :color (1 0 0) :mesh :bowl))))
    ;; Verify that object is unstable.
    (btr:simulate btr:*current-bullet-world* 10)
    (assert-false (btr:stable-p (btr:object btr:*current-bullet-world* ?obj-name)))
    ;; reset object
    (init-env)
    (prolog:prolog `(and (btr:bullet-world ?world)
                         (assert (btr:object ?world :mesh ,?obj-name
                                             ,(cram-tf:pose->list spawn-pose)
                                             :mass 0.2 :color (1 0 0) :mesh :bowl))))
    ;; Fire event
    (cram-occasions-events:on-event
     (make-instance 'cram-plan-occasions-events:object-perceived-event
       :object-designator bowl-desig
       :perception-source :whatever))
    ;; Verify object is stable.
    (btr:simulate btr:*current-bullet-world* 10)
    (assert-true (btr:stable-p (btr:object btr:*current-bullet-world* ?obj-name)))))


(define-test spoon-falling-through-drawer-test
  (unwind-protect
       (let* ((?obj-name 'spoon-1)
              (?obj-type :spoon)
              (spawn-pose (cl-transforms-stamped:make-pose-stamped
                           "map" 0.0
                           (cl-transforms:make-3d-vector 1.05 0.87 0.71)
                           (cl-transforms:make-identity-rotation)))
              (desig (desig:an object (type ?obj-type) (name ?obj-name))))
         (setf (slot-value desig 'desig:data)
               (make-instance 'desig:object-designator-data
                 :object-identifier ?obj-name
                 :pose spawn-pose))

         (init-env)
         (setf (btr:joint-state (btr:get-environment-object)
                                "sink_area_left_upper_drawer_main_joint") 0.4)
         (setf *kitchen-changed* t)
         (prolog:prolog `(and (btr:bullet-world ?world)
                              (assert (btr:object ?world :mesh ,?obj-name
                                                  ,(cram-tf:pose->list spawn-pose)
                                                  :mass 0.2 :color (1 0 0) :mesh ,?obj-type))))

         (cram-occasions-events:on-event
          (make-instance 'cram-plan-occasions-events:object-perceived-event
            :object-designator desig
            :perception-source :whatever))

         (btr:simulate btr:*current-bullet-world* 10)
         (assert-true (> (cl-transforms:z
                          (cl-transforms:origin
                           (btr:pose (btr:object btr:*current-bullet-world* ?obj-name))))
                         0.5)))
    (setf (btr:joint-state (btr:get-environment-object)
                           "sink_area_left_upper_drawer_main_joint") 0.0)))


(define-test milk-in-fridge-door-test
  (unwind-protect
       (let* ((?obj-name 'milk-1)
              (?obj-type :milk)
              (spawn-pose (cl-transforms-stamped:make-pose-stamped
                           "map" 0.0
                           (cl-transforms:make-3d-vector 0.95 -1.14 0.66)
                           (cl-transforms:make-identity-rotation)))
              (desig (desig:an object (type ?obj-type) (name ?obj-name))))
         (setf (slot-value desig 'desig:data)
               (make-instance 'desig:object-designator-data
                 :object-identifier ?obj-name
                 :pose spawn-pose))

         (init-env)
         (setf (btr:joint-state (btr:get-environment-object)
                                "iai_fridge_door_joint") 1.57)
         (setf *kitchen-changed* t)
         (prolog:prolog `(and (btr:bullet-world ?world)
                              (assert (btr:object ?world :mesh ,?obj-name
                                                  ,(cram-tf:pose->list spawn-pose)
                                                  :mass 0.2 :color (1 0 0) :mesh ,?obj-type))))

         (cram-occasions-events:on-event
          (make-instance 'cram-plan-occasions-events:object-perceived-event
            :object-designator desig
            :perception-source :whatever))

         (btr:simulate btr:*current-bullet-world* 10)
         (assert-true (> (cl-transforms:z
                          (cl-transforms:origin
                           (btr:pose (btr:object btr:*current-bullet-world* ?obj-name))))
                         0.5)))
    (setf (btr:joint-state (btr:get-environment-object)
                           "iai_fridge_door_joint") 0.0)))


#+This-test-has-some-bug-that-needs-to-be-fixed-Commenting-until-it-is-checked
(define-test kitchen-island-counted-edge-unstable-with-recovery-test
  (let* ((obj-name :bowl-1)
         (spawn-pose (cl-transforms-stamped:make-pose-stamped
                      "map" 0.0
                      (cl-transforms:make-3d-vector -0.6607 1.2022 0.822)
                      (cl-transforms:make-identity-rotation))))
         (let ((kitchen-urdf
                 (cl-urdf:parse-urdf
                  (roslisp:get-param "kitchen_description"))))
           (prolog:prolog
            `(and (btr:bullet-world ?world)
                  (assert (btr:object ?world
                                      :urdf
                                      :kitchen ((0 0 0) (0 0 0 1))
                                      :urdf ,kitchen-urdf
                                      :collision-group :static-filter
                                      :collision-mask (:default-filter
                                                       :character-filter)
                                      :compound T)))))
    (btr:prolog-?w
      `(btr:item-type ?w ?obj ?type)
      `(btr:retract (btr:object ?w ?obj)))
    (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo")
    (prolog:prolog '(and (btr:bullet-world ?world)
                     (assert (btr:object ?world :mesh :bowl-1 ((-0.6607 1.2022 0.822) (0 0 0 1))
                              :mass 0.2 :color (1 0 0) :mesh :bowl))))
    (btr:simulate btr:*current-bullet-world* 100)
    (assert-true (> (cl-transforms:v-dist
                     (cl-transforms:origin spawn-pose)
                     (cl-transforms:origin (btr:pose (btr:object
                                                      btr:*current-bullet-world*
                                                      obj-name))))
                    btr-belief::*perception-instability-threshold*))
    (btr-belief::check-and-correct-perception-instability :bowl-1 spawn-pose)
    (assert-true (< (cl-transforms:v-dist
                     (cl-transforms:origin spawn-pose)
                     (cl-transforms:origin (btr:pose (btr:object
                                                      btr:*current-bullet-world*
                                                      obj-name))))
                    btr-belief::*perception-instability-threshold*))))
