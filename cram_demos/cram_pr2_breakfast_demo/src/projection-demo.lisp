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

(defparameter *object-spawning-poses*
  '("sink_area_surface"
    ((:breakfast-cereal . ((0.2 -0.15 0.1) (0 0 0 1)))
     (:cup . ((0.2 -0.35 0.1) (0 0 0 1)))
     (:bowl . ((0.18 -0.55 0.1) (0 0 0 1)))
     (:spoon . ((0.15 -0.4 -0.05) (0 0 0 1)))
     (:milk . ((0.07 -0.35 0.1) (0 0 0 1)))))
  "Relative poses on sink area")

(defparameter *object-placing-poses*
  '((:breakfast-cereal . ((-0.78 0.9 0.95) (0 0 1 0)))
    (:cup . ((-0.79 1.35 0.9) (0 0 0.7071 0.7071)))
    (:bowl . ((-0.76 1.19 0.88) (0 0 0.7071 0.7071)))
    (:spoon . ((-0.78 1.5 0.86) (0 0 0 1)))
    (:milk . ((-0.75 1.7 0.95) (0 0 0.7071 0.7071))))
  "Absolute poses on kitchen_island.")

(defparameter *object-grasping-arms*
  '(;; (:breakfast-cereal . :right)
    ;; (:cup . :left)
    ;; (:bowl . :right)
    ;; (:spoon . :right)
    ;; (:milk . :right)
    ))


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
(defparameter *look-goal*
  (cl-transforms-stamped:make-pose-stamped
   "base_footprint"
   0.0
   (cl-transforms:make-3d-vector 0.5d0 0.0d0 1.0d0)
   (cl-transforms:make-identity-rotation)))


(defun make-poses-relative (spawning-poses)
  "Gets an associative list in a form of (FRAME ((TYPE . COORDINATES-LIST) ...)),
where coordinates-list is defined in the FRAME coordinate frame.
Converts these coordinates into CRAM-TF:*FIXED-FRAME* frame and returns a list in form
 ((TYPE . POSE) ...)."
  (when spawning-poses
    (let* ((map-T-surface (cl-transforms:pose->transform
                           (btr:link-pose (btr:get-environment-object)
                                          (first spawning-poses)))))
      (mapcar (lambda (type-and-pose-list)
                (destructuring-bind (type . pose-list)
                    type-and-pose-list
                  (let* ((surface-T-object
                           (cl-transforms:pose->transform (cram-tf:list->pose pose-list)))
                         (map-T-object
                           (cl-transforms:transform* map-T-surface surface-T-object))
                         (map-P-object
                           (cl-transforms:transform->pose map-T-object)))
                    `(,type . ,map-P-object))))
              (second spawning-poses)))))


(defun spawn-objects-on-sink-counter (&key
                                        (object-types '(:breakfast-cereal
                                                        :cup
                                                        :bowl
                                                        :spoon
                                                        :milk))
                                        (spawning-poses-relative *object-spawning-poses*)
                                        (random NIL))
  ;; make sure mesh paths are known, kill old objects and destroy all attachments
  (btr:add-objects-to-mesh-list "cram_pr2_pick_place_demo")
  (btr-utils:kill-all-objects)
  (btr:detach-all-objects (btr:get-robot-object))
  (btr:detach-all-objects (btr:get-environment-object))

  ;; spawn objects
  (let* ((spawning-poses-absolute
           (make-poses-relative spawning-poses-relative))
         (objects
           (mapcar (lambda (object-type)
                     (let* (;; generate object name of form :TYPE-1
                            (object-name
                              (intern (format nil "~a-1" object-type) :keyword))
                            ;; spawn object in Bullet World at default pose
                            (object
                              (btr-utils:spawn-object object-name object-type))
                            ;; calculate new pose: either random or from input list
                            (object-pose
                              (if random
                                  ;; generate a pose on a surface
                                  (let* ((aabb-z
                                           (cl-transforms:z
                                            (cl-bullet:bounding-box-dimensions
                                             (btr:aabb object)))))
                                    (cram-tf:translate-pose
                                     (desig:reference
                                      (if (eq object-type :spoon)
                                          (desig:a location
                                                   (in (desig:an object
                                                                 (type drawer)
                                                                 (urdf-name
                                                                  sink-area-left-upper-drawer-main)
                                                                 (part-of iai-kitchen)))
                                                   (side front)
                                                   (range 0.2)
                                                   (range-invert 0.12))
                                          (desig:a location
                                                   (on (desig:an object
                                                                 (type counter-top)
                                                                 (urdf-name sink-area-surface)
                                                                 (part-of iai-kitchen)))
                                                   ;; below only works for knowrob sem-map
                                                   ;; (centered-with-padding 0.1)
                                                   (side left)
                                                   (side front))))
                                     :z (/ aabb-z 2.0)))
                                  ;; take the pose from the function input list
                                  (cdr (assoc object-type spawning-poses-absolute))))
                            ;; rotate new pose randomly around Z
                            (rotated-object-pose
                              (cram-tf:rotate-pose object-pose
                                                   :z (/ (* 2 pi) (random 10.0)))))
                       ;; move object to calculated pose on surface
                       (btr-utils:move-object object-name rotated-object-pose)
                       ;; return object
                       object))
                   object-types)))

    ;; make sure generated poses are stable, especially important for random ones
    ;; TDOO: if unstable, call itself

     ;; attach spoon to the drawer
    (when (btr:object btr:*current-bullet-world* :spoon-1)
      (btr:attach-object (btr:get-environment-object)
                         (btr:object btr:*current-bullet-world* :spoon-1)
                         :link "sink_area_left_upper_drawer_main"))

    ;; stabilize world
    (btr:simulate btr:*current-bullet-world* 100)

    ;; return list of BTR objects
    objects))



(defun go-to-sink-or-island (&optional (sink-or-island :sink))
  (let ((?navigation-goal (ecase sink-or-island
                            (:sink *sink-nav-goal*)
                            (:island *island-nav-goal*)))
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
    (cpl:par
      (exe:perform (desig:an action
                             (type looking)
                             (object ?perceived-object-desig)))
      (exe:perform (desig:an action
                             (type picking-up)
                             (arm (?arm))
                             (object ?perceived-object-desig))))))

(defun place-object (?target-pose &optional (?arm :right))
  (go-to-sink-or-island :island)
  (cpl:par
    (exe:perform (desig:a motion
                          (type looking)
                          (pose ?target-pose)))
    (exe:perform (desig:an action
                           (type placing)
                           (arm ?arm)
                           (target (desig:a location
                                            (pose ?target-pose)))))))




(defun pour-into (?object-type-to-pour-into ?arms ?from-which-side-pouring)
  (let ((?object-to-pour-into (get-object-designator ?object-type-to-pour-into)))
    (cpl:seq
      (exe:perform (desig:an action     
                             (type looking)
                             (object ?object-to-pour-into)))
      (exe:perform
       (desig:an action
                 (type pouring)
                 (object ?object-to-pour-into)
                 (arms (?arms))
                 (grasp ?from-which-side-pouring))))))


(defun get-object-designator (?object-type &optional ?pose-in-map)
  (let* ((?object-name (btr:name (get-item ?object-type)))
         (?pose (if ?pose-in-map
                    (ensure-pose-stamped ?pose-in-map)
                    (cl-tf:pose->pose-stamped
                     cram-tf:*fixed-frame*
                     0.0
                     (btr:pose (get-item ?object-type)))))
         (?transform-in-base (pp-plans::pose->transform-stamped-in-base ?pose ?object-name))
         (?pose-in-base (cl-tf:ensure-pose-stamped ?transform-in-base))
         (?transform (cram-tf:pose-stamped->transform-stamped ?pose ?object-name)))
    (desig:an object
              (type ?object-type)
              (name ?object-name)
              (pose ((pose ?pose-in-base)
                     (transform ?transform-in-base)
                     (transform-in-map ?transform)
                     (pose-in-map ?pose))))))
(defun get-item (?object-type)
  (find ?object-type
        (remove-if-not
         (lambda (x) 
           (equalp
            'cram-bullet-reasoning::item x))
         (btr:objects
          btr:*current-bullet-world*)
         :key #'type-of)
        :key (alexandria:compose #'car #'btr:item-types)))


(defun ensure-pose-stamped (pose)
  (when pose
    (case (type-of pose)
      ('cl-transforms-stamped:pose-stamped pose)
      ('cons (make-pose-absolute pose))
      (T (error "cannot translate pose of type ~a in cl-transforms-stamped" (type-of pose))))))

(defun make-pose-absolute (frame-and-pose)
  (when frame-and-pose
    (let* ((btr-object (btr:object btr:*current-bullet-world* (first frame-and-pose)))
           (link-pose (btr:link-pose (btr:get-environment-object) (first frame-and-pose)))
           (map-T-surface (cl-transforms:pose->transform
                           (if btr-object
                               (btr:pose btr-object)
                               link-pose))))
        (let* ((input-pose (cdr frame-and-pose))
               (pose (case (type-of input-pose)
                       ('cl-transforms-stamped:pose-stamped input-pose)
                       ('cons (cram-tf:list->pose input-pose))
                       (T (error "Unknown pose data type: only lists ~
                                  and pose-stampeds are allow as input."))))
               (surface-T-object
                 (cl-transforms:pose->transform pose))
               (map-T-object
                 (cl-transforms:transform* map-T-surface surface-T-object)))
          (cl-tf:pose->pose-stamped
           cram-tf:*fixed-frame*
           0.0
           (cl-tf:transform->pose map-T-object))))))

(defun park-arms ()
  (exe:perform
   (desig:an action
             (type positioning-arm)
             (left-configuration park)
             (right-configuration park))))



(defun scoop (&key
                    ((:arm ?arm))
                    ((:frame ?knob-frame))
                    ((:pose ?knob-pose))
                    ((:configuration ?on-or-off))
                  &allow-other-keys)

 ;; Turn knob
  (roslisp:ros-info (popcorn turn-knob) "Turning knob")
  (let* ((offset 10)
         (offset-rotation (cond
                            ((eq ?on-or-off :on) offset)
                            ((eq ?on-or-off :off) (* -1 offset))
                            (t 10)))
         (knob-joint (concatenate 'string ?knob-frame "_joint"))
         (?knob-rotate-pose-in-knob-frame (get-rotate-pose-in-knob-frame
                                             (* -1 
                                                (cram-math:degrees->radians
                                                 offset-rotation))
                                             ?knob-frame)))
    (roslisp:ros-info (popcorn turn-knob) "?knob-rotate-pose-in-knob-frame")
      (loop with ?knob-rotate-pose = nil
            with start = 0
            with end = 90
            for degree from start to end by offset do
              ;; Calculate the pose of the knob
	      (roslisp:ros-info (popcorn turn-knob) "make-pose-absolut")
	       (format nil "~a-1" ?knob-rotate-pose-in-knob-frame)
              (setf ?knob-rotate-pose
                    (make-pose-absolute (cons ?knob-frame
					      ?knob-rotate-pose-in-knob-frame)))
	       (roslisp:ros-info (popcorn turn-knob) "moving the robots arm")
              ;; Moving the robots arm
              (exe:perform
               (desig:a motion
                        (type moving-tcp)
                        (desig:when (eq ?arm :right)
                          (right-pose ?knob-rotate-pose))
                        (desig:when (eq ?arm :left)
                          (left-pose ?knob-rotate-pose))
                        (collision-mode :allow-hand)))
              ;; Setting the joint state of the knob accordingly
              (let ((joint-state (cond
                                   ((eq ?on-or-off :on) degree)
                                   ((eq ?on-or-off :off) (- 90 degree))
                                   (t degree))))
                (setf (btr:joint-state (btr:get-environment-object)
                                       knob-joint)
                      (* -1 
                         (cram-math:degrees->radians joint-state))))))
  (roslisp:ros-info (popcorn turn-knob) "Turned knob")

  ;; Park arms
  (roslisp:ros-info (popcorn turn-knob) "Parking arms")
  (exe:perform
   (desig:an action
             (type positioning-arm)
             (left-configuration park)
             (right-configuration park))))

(defun get-rotate-pose-in-knob-frame (z-rotation 
                                      &optional (knob-frame "bowl_1"))
             
  (cl-tf:make-pose-stamped
   knob-frame
   0.0
   (cl-tf:make-3d-vector 0.0 0.0 0.064)
   (cl-tf:euler->quaternion :ax (/ 3.14 2) 
                            :ay (/ 3.14 2)
                            :az z-rotation)))


(def-fact-group popcorn-actions (desig:action-grounding)
 
  ;;type turning-knob
  (<- (desig:action-grounding ?action-designator (scoop ?resolved-action-designator))
    (spec:property ?action-designator (:type :turning-knob))

    (spec:property ?action-designator (:arm ?arm))
    (man-int:robot-free-hand ?_ ?arm)
    
    (spec:property ?action-designator (:frame ?knob-frame))

    (lisp-fun get-pose ?knob-frame ?knob-pose)
    (not (equal ?knob-pose nil))

    (spec:property ?action-designator (:configuration ?on-or-off))
    (member ?on-or-off (:on :off))
    
    (desig:designator :action ((:type :turning-knob)
                               (:arm ?arm)
                               (:frame ?knob-frame)
                               (:pose ?knob-pose)
                               (:configuration ?on-or-off))
                      ?resolved-action-designator)))



(defun get-pose (?frame)
  (cram-tf:transform->pose-stamped 
   cram-tf:*fixed-frame*
   0.0
   (cram-tf:apply-transform 
    (cl-tf:lookup-transform cram-tf:*transformer* cram-tf:*fixed-frame* ?frame)
    (cl-tf:transform->transform-stamped 
     ?frame
     ?frame
     0.0
     (cl-tf:pose->transform
      (cl-tf:pose-stamped->pose
       (get-rotate-pose-in-knob-frame 0 ?frame)))))))
    
