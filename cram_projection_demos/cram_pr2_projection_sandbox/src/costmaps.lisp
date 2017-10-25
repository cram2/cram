;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :proj-sand)

(defun make-side-costmap-generator (obj axis sign)
  "Returns a lambda function which for each (x y) gives 1.0 if it is on the sign side of the axis. "
  (when obj
    (let* ((bb-center (cl-transforms:origin (sem-map-utils:pose obj)))
           (bb-x (cl-transforms:x bb-center))
           (bb-y (cl-transforms:y bb-center)))
      (lambda (x y)
        (if (case axis
              (:x (funcall sign x bb-x))
              (:y (funcall sign y bb-y)))
            1.0
            0.0)))))

(defmethod location-costmap:costmap-generator-name->score ((name (eql 'side-generator))) 5)

(def-fact-group demo-costmap (location-costmap:desig-costmap)
  (<- (location-costmap:desig-costmap ?designator ?costmap)
    (desig:desig-prop ?designator (:side :left))
    (desig:desig-prop ?designator (:on ?_))
    (desig:desig-prop ?designator (:name ?supp-obj-name))
    (lisp-fun sem-map-desig:designator->semantic-map-objects
              ?designator ?supp-objects)
    (member ?supp-object ?supp-objects)
    (location-costmap:costmap ?costmap)
    (location-costmap:costmap-add-function
     side-generator
     (make-side-costmap-generator ?supp-object :y >)
     ?costmap)))

(defun spawn-objects-on-sink-counter ()
  (kill-all-objects)
  (add-objects-to-mesh-list)
  (let ((object-types '(:cereal :cup :bowl :spoon :milk)))
    ;; spawn at default location
    (let ((objects (mapcar (lambda (object-type)
                             (btr-utils:spawn-object
                              (intern (format nil "~a-1" object-type) :keyword)
                              object-type))
                           object-types)))
      ;; move on top of counter tops
      (mapcar (lambda (btr-object)
                (let* ((aabb-z (cl-transforms:z
                                (cl-bullet:bounding-box-dimensions (btr:aabb btr-object))))
                       (new-pose (cram-tf:rotate-pose
                                  (cram-tf:translate-pose
                                   (desig:reference
                                    (desig:a location
                                             (on "CounterTop")
                                             (name "iai_kitchen_sink_area_counter_top")
                                             (side left)
                                             (centered-with-padding 0.1)))
                                   :z-offset (/ aabb-z 2.0))
                                  :z (/ pi (random 10.0)))))
                  (btr-utils:move-object (btr:name btr-object) new-pose)))
              objects)))
  ;; stabilize world
  (btr:simulate btr:*current-bullet-world* 100))

(defparameter *sink-nav-goal*
  (cl-transforms-stamped:make-pose-stamped
   "map"
   0.0
   (cl-transforms:make-3d-vector 0.75d0 0.70d0 0.0)
   (cl-transforms:make-identity-rotation)))
(defparameter *sink-look-goal*
  (cl-transforms-stamped:make-pose-stamped
   "base_footprint"
   0.0
   (cl-transforms:make-3d-vector 0.5d0 0.0d0 1.0d0)
   (cl-transforms:make-identity-rotation)))

(defun go-to-sink ()
  (with-simulated-robot
    (let ((?navigation-goal *sink-nav-goal*)
          (?ptu-goal *sink-look-goal*))
      (cpl:par
        (pp-plans::park-arms)
        (exe:perform (desig:a motion
                              (type going)
                              (target (desig:a location (pose ?navigation-goal)))))
        (exe:perform (desig:a motion
                              (type moving-torso)
                              (joint-angle 0.3))))
      (exe:perform (desig:a motion
                            (type looking)
                            (target (desig:a location (pose ?ptu-goal))))))))

(defun pick-object (&optional (?object-type :milk) (?arm :right))
  (with-simulated-robot
    (pp-plans:park-arms)

    (let ((?bottle-desig (desig:an object (type ?object-type))))
      (let ((?perceived-bottle-desig (pp-plans::perceive ?bottle-desig)))
        (cpl:par
          (exe:perform (desig:an action
                                 (type looking)
                                 (object ?perceived-bottle-desig)))
          (exe:perform (desig:an action
                                 (type picking-up)
                                 (arm ?arm)
                                 (object ?perceived-bottle-desig))))))

    (cpl:sleep 1.0)

    (exe:perform (desig:an action
                           (type placing)
                           (arm ?arm)))))
