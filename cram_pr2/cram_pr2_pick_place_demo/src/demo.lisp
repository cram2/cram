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

(in-package :demo)

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
   (cl-transforms:make-3d-vector -0.2d0 1.5d0 0.0)
   (cl-transforms:make-quaternion 0 0 1 0)))
(defparameter *look-goal*
  (cl-transforms-stamped:make-pose-stamped
   "base_footprint"
   0.0
   (cl-transforms:make-3d-vector 0.5d0 0.0d0 1.0d0)
   (cl-transforms:make-identity-rotation)))

(defparameter *object-spawning-poses*
  '((:cereal . ((1.4 0.4 0.95) (0 0 0 1)))
    (:cup . ((1.3 0.6 0.9) (0 0 0 1)))
    (:bowl . ((1.4 0.8 0.87) (0 0 0 1)))
    (:spoon . ((1.4 1.0 0.86) (0 0 0 1)))
    (:milk . ((1.5 0.6 0.95) (0 0 0 1)))))
(defparameter *object-placing-poses*
  '((:cereal . ((-0.85 1.8 0.95) (0 0 0 1)))
    (:cup . ((-0.9 1.4 0.9) (0 0 0.7071 0.7071)))
    (:bowl . ((-0.7 1.3 0.89) (0 0 0 1)))
    (:spoon . ((-0.78 1.5 0.86) (0 0 1 0)))
    (:milk . ((-0.79 1.2 0.95) (0 0 0.7071 0.7071)))))

(defparameter *object-grasping-arms*
  '((:cereal . :right)
    (:cup . :right)
    (:bowl . :left)
    (:spoon . :left)
    (:milk . :left)))

(defmacro with-simulated-robot (&body body)
  `(let ((results
           (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
             (cpl:top-level
               ,@body))))
     (car (cram-projection::projection-environment-result-result results))))

;; (defmacro with-real-robot (&body body)
;;   `(cram-process-modules:with-process-modules-running
;;        (pr2-pms::pr2-perception-pm pr2-pms::pr2-base-pm pr2-pms::pr2-arms-pm
;;                                    pr2-pms::pr2-grippers-pm pr2-pms::pr2-ptu-pm)
;;      (cpl:top-level
;;        ,@body)))

(defun spawn-objects-on-sink-counter (&optional (spawning-poses *object-spawning-poses*))
  (btr-utils:kill-all-objects)
  (add-objects-to-mesh-list)
  (btr:detach-all-objects (btr:get-robot-object))
  (let ((object-types '(:cereal :cup :bowl :spoon ;; :milk
                        )))
    ;; spawn objects at default poses
    (let ((objects (mapcar (lambda (object-type)
                             (btr-utils:spawn-object
                              (intern (format nil "~a-1" object-type) :keyword)
                              object-type
                              :pose (cdr (assoc object-type spawning-poses))))
                           object-types)))
      ;; stabilize world
      (btr:simulate btr:*current-bullet-world* 100)
      objects)))

(defun go-to-sink-or-island (&optional (sink-or-island :sink))
  (let ((?navigation-goal (ecase sink-or-island
                            (:sink *sink-nav-goal*)
                            (:island *island-nav-goal*)))
        (?ptu-goal *look-goal*))
    (cpl:par
      (pp-plans::park-arms)
      (exe:perform (desig:a motion
                            (type going)
                            (target (desig:a location (pose ?navigation-goal))))))
    (exe:perform (desig:a motion
                          (type looking)
                          (target (desig:a location (pose ?ptu-goal)))))))

(defun pick-object (&optional (?object-type :cereal) (?arm :right))
  (pp-plans:park-arms)
  (go-to-sink-or-island :sink)
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
                             (arm ?arm)
                             (object ?perceived-object-desig))))))

(defun place-object (?target-pose &optional (?arm :right))
  (format t "IN PLACE OJB:~%~%~%~%~a~%" (prolog:prolog `(cpoe:object-in-hand ?obj ?arm)))
  (pp-plans:park-arms)
  (go-to-sink-or-island :island)
  (cpl:par
    (exe:perform (desig:a motion
                          (type looking)
                          (target (desig:a location
                                           (pose ?target-pose)))))
    (exe:perform (desig:an action
                           (type placing)
                           (arm ?arm)
                           (target (desig:a location
                                            (pose ?target-pose)))))))

(defun collisions-without-attached ()
  (let ((colliding-object-names
          (mapcar #'btr:name
                  (btr:find-objects-in-contact
                   btr:*current-bullet-world*
                   (btr:get-robot-object))))
        (attached-object-names
          (mapcar #'car
                  (btr:attached-objects (btr:get-robot-object)))))
    (set-difference colliding-object-names attached-object-names)))

(defun go-without-collisions (?navigation-location &optional (retries 21))
  (declare (type desig:location-designator ?navigation-location))

  (pp-plans:park-arms)

  ;; Store current world state and in the current world try to go to different
  ;; poses that satisfy `?navigation-location'.
  ;; If chosen pose results in collisions, choose another pose.
  ;; Repeat `reachable-location-retires' + 1 times.
  ;; Store found pose into designator or throw error if good pose not found.
  (let* ((world btr:*current-bullet-world*)
         (world-state (btr::get-state world)))
    (unwind-protect
         (cpl:with-retry-counters ((reachable-location-retries retries))
           ;; If a navigation-pose-in-collisions failure happens, retry N times
           ;; with the next solution of `?navigation-location'.
           (cpl:with-failure-handling
               ((common-fail:navigation-pose-in-collision (e)
                  (roslisp:ros-warn (pp-plans fetch) "Failure happened: ~a" e)
                  (cpl:do-retry reachable-location-retries
                    (setf ?navigation-location (desig:next-solution ?navigation-location))
                    (if ?navigation-location
                        (progn
                          (roslisp:ros-warn (pp-plans check-nav-collisions) "Retrying...~%")
                          (cpl:retry))
                        (roslisp:ros-warn (pp-plans check-nav-collisions)
                                          "No more samples left to try :'(.")))
                  (roslisp:ros-warn (pp-plans go-without-collisions)
                                    "Couldn't find a nav pose for~%~a.~%Propagating up."
                                    ?navigation-location)))

             ;; Pick one pose, store it in `pose-at-navigation-location'
             ;; In projected world, drive to picked pose
             ;; If robot is in collision with any object in the world, throw a failure.
             ;; Otherwise, the pose was found, so return location designator,
             ;; which is currently referenced to the found pose.
             (handler-case
                 (let ((pose-at-navigation-location (desig:reference ?navigation-location)))
                   (pr2-proj::drive pose-at-navigation-location)
                   (when (collisions-without-attached)
                     (roslisp:ros-warn (pp-plans fetch) "Pose was in collision.")
                     (cpl:sleep 0.1)
                     (cpl:fail 'common-fail:navigation-pose-in-collision
                               :pose-stamped pose-at-navigation-location))
                   (roslisp:ros-info (pp-plans fetch) "Found reachable pose.")
                   ?navigation-location)
               (desig:designator-error (e)
                 (roslisp:ros-warn (pp-plans check-nav-collisions)
                                   "Desig ~a couldn't be resolved: ~a.~%Cannot navigate."
                                   ?navigation-location e)
                 (cpl:fail 'common-fail:high-level-failure)))))

      ;; After playing around and messing up the world, restore the original state.
      (btr::restore-world-state world-state world)))

  (exe:perform (desig:an action
                         (type going)
                         (target ?navigation-location))))


(defun search-for-object (?object-designator ?search-location &optional (retries 5))
  (cpl:with-retry-counters ((search-location-retries retries))
    (cpl:with-failure-handling
        (((or common-fail:perception-object-not-found
              common-fail:navigation-pose-in-collision) (e)
           (roslisp:ros-warn (pp-plans search-for-object) "Failure happened: ~a" e)
           (cpl:do-retry search-location-retries
             (setf ?search-location (desig:next-solution ?search-location))
             (if ?search-location
                 (progn
                   (roslisp:ros-warn (pp-plans search-for-object) "Retrying...~%")
                   (cpl:retry))
                 (progn
                   (roslisp:ros-warn (pp-plans search-for-object) "No samples left :'(~%")
                   (cpl:fail 'common-fail:object-nowhere-to-be-found))))
           (roslisp:ros-warn (pp-plans search-for-object) "No retries left :'(~%")
           (cpl:fail 'common-fail:object-nowhere-to-be-found)))
      (let* ((?pose-at-search-location (desig:reference ?search-location))

             (?nav-location (desig:a location
                                     (visible-for pr2)
                                     (location (desig:a location
                                                        (pose ?pose-at-search-location))))))
        (go-without-collisions ?nav-location)

        (exe:perform (desig:an action
                               (type looking)
                               (target (desig:a location
                                                (pose ?pose-at-search-location))))))
      (pp-plans::perceive ?object-designator))))


(defun equalize-two-list-lengths (first-list second-list)
  (let* ((first-length (length first-list))
         (second-length (length second-list))
         (max-length (max first-length second-length)))
    (values
     (if (> max-length first-length)
        (append first-list (make-list (- max-length first-length)))
        first-list)
     (if (> max-length second-length)
        (append second-list (make-list (- max-length second-length)))
        second-list))))

(defun equalize-lists-of-lists-lengths (first-list-of-lists second-list-of-lists)
  (let ((max-length (max (length first-list-of-lists)
                         (length second-list-of-lists)))
        first-result-l-of-ls second-result-l-of-ls)

   (loop for i from 0 to (1- max-length)
         do (let ((first-list (nth i first-list-of-lists))
                  (second-list (nth i second-list-of-lists)))
              (multiple-value-bind (first-equalized second-equalized)
                  (equalize-two-list-lengths first-list second-list)
                (setf first-result-l-of-ls
                      (append first-result-l-of-ls first-equalized)
                      second-result-l-of-ls
                      (append second-result-l-of-ls second-equalized)))))

   (values first-result-l-of-ls
           second-result-l-of-ls)))

(defun check-picking-up-collisions (pick-up-action-desig &optional (retries 16))
  (let* ((world btr:*current-bullet-world*)
         (world-state (btr::get-state world)))

    (unwind-protect
         (cpl:with-retry-counters ((pick-up-configuration-retries retries))
           (cpl:with-failure-handling
               (((or common-fail:manipulation-pose-unreachable
                     common-fail:manipulation-pose-in-collision) (e)
                  (roslisp:ros-warn (pp-plans pick-object) "Manipulation failure happened: ~a" e)
                  (cpl:do-retry pick-up-configuration-retries
                    (setf pick-up-action-desig (next-solution pick-up-action-desig))
                    (cond
                      (pick-up-action-desig
                       (roslisp:ros-info (pp-plans pick-object) "Retrying...")
                       (cpl:retry))
                      (t
                       (roslisp:ros-warn (pp-plans pick-object) "No more samples to try :'(")
                       (cpl:fail 'common-fail:object-unreachable))))
                  (roslisp:ros-warn (pp-plans pick-object) "No more retries left :'(")
                  (cpl:fail 'common-fail:object-unreachable)))

             (let ((pick-up-action-referenced (reference pick-up-action-desig)))
               (destructuring-bind (_action object-designator arm gripper-opening _effort _grasp
                                    left-reach-poses right-reach-poses
                                    left-lift-poses right-lift-poses)
                   pick-up-action-referenced
                 (declare (ignore _action _effort))
                 (let ((object-name
                         (desig:desig-prop-value object-designator :name)))
                   (roslisp:ros-info (pp-plans manipulation)
                                     "Trying grasp ~a on object ~a with arm ~a~%"
                                     _grasp object-name arm)
                   (let ((left-poses-list-of-lists (list left-reach-poses left-lift-poses))
                         (right-poses-list-of-lists (list right-reach-poses right-lift-poses)))
                     (multiple-value-bind (left-poses right-poses)
                         (equalize-lists-of-lists-lengths left-poses-list-of-lists
                                                          right-poses-list-of-lists)
                       (mapcar (lambda (left-pose right-pose)
                                 (pr2-proj::gripper-action gripper-opening arm)
                                 (pr2-proj::move-tcp left-pose right-pose)
                                 (sleep 0.1)
                                 (when (remove object-name
                                               (btr:find-objects-in-contact
                                                btr:*current-bullet-world*
                                                (btr:get-robot-object))
                                               :key #'btr:name)
                                   (btr::restore-world-state world-state world)
                                   (cpl:fail 'common-fail:manipulation-pose-in-collision)))
                               left-poses
                               right-poses))))))))
      (btr::restore-world-state world-state world)
      (format t "CLEANING UP PICKING UP COLLISION CHECK~%"))))



(defvar *obj* nil)

(defun fetch (?object-designator ?search-location)
  (let* ((?perceived-object-desig
           (search-for-object ?object-designator ?search-location))
         (?perceived-object-pose-in-base
           (desig:reference (desig:a location (of ?perceived-object-desig))))
         (?perceived-object-pose-in-map
           (cram-tf:ensure-pose-in-frame
            ?perceived-object-pose-in-base
            cram-tf:*fixed-frame*
            :use-zero-time t)))
    (roslisp:ros-info (pp-plans fetch) "Found object ~a" ?perceived-object-desig)

    (cpl:with-failure-handling
        ((common-fail:navigation-pose-in-collision (e)
           (declare (ignore e))
           (roslisp:ros-warn (pp-plans fetch) "Object ~a is unfetchable." ?object-designator)
           (cpl:fail 'common-fail:object-unfetchable :object ?object-designator)))

      (let ((?pick-up-location
              (desig:a location
                       (reachable-for pr2)
                       (location (desig:a location
                                          (pose ?perceived-object-pose-in-map))))))

        (cpl:with-retry-counters ((relocation-for-ik-retries 10))
          (cpl:with-failure-handling
              (((or common-fail:object-unreachable
                    common-fail:perception-object-not-found) (e)
                 (roslisp:ros-warn (pp-plans fetch) "Object is unreachable: ~a" e)
                 (cpl:do-retry relocation-for-ik-retries
                   (setf ?pick-up-location (next-solution ?pick-up-location))
                   (if ?pick-up-location
                       (progn
                         (roslisp:ros-info (pp-plans fetch) "Relocating...")
                         (cpl:retry))
                       (progn
                         (roslisp:ros-warn (pp-plans fetch) "No more samples to try :'(")
                         (cpl:fail 'common-fail:object-unfetchable)))
                   (roslisp:ros-warn (pp-plans fetch) "No more retries left :'(")
                   (cpl:fail 'common-fail:object-unfetchable))))

            (go-without-collisions ?pick-up-location)
            (setf ?pick-up-location (desig:current-desig ?pick-up-location))

            (exe:perform (desig:an action
                                   (type looking)
                                   (target (desig:a location
                                                    (pose ?perceived-object-pose-in-map)))))

            (let ((?more-precise-perceived-object-desig
                    (pp-plans::perceive ?object-designator)))
              (let ((pick-up-action
                      (desig:an action
                                (type picking-up)
                                (object ?more-precise-perceived-object-desig))))
                (check-picking-up-collisions pick-up-action)
                (setf pick-up-action (desig:current-desig pick-up-action))
                (exe:perform pick-up-action)
                (setf *obj* ?more-precise-perceived-object-desig)))))))

    (pp-plans:park-arms)
    (desig:current-desig ?object-designator)))

(defun check-placing-collisions (placing-action-desig)
  (let* ((world btr:*current-bullet-world*)
         (world-state (btr::get-state world)))

    (unwind-protect
         (cpl:with-failure-handling
             ((common-fail:manipulation-pose-unreachable (e)
                (roslisp:ros-warn (pp-plans deliver)
                                  "Object is unreachable: ~a.~%Propagating up."
                                  e)
                (cpl:fail 'common-fail:object-unreachable)))

           (let ((placing-action-referenced (reference placing-action-desig)))
             (destructuring-bind (_action object-designator arm
                                  left-reach-poses right-reach-poses
                                  left-put-poses right-put-poses
                                  left-retract-poses right-retract-poses)
                 placing-action-referenced
               (declare (ignore _action))
               (let ((object-name
                       (desig:desig-prop-value object-designator :name)))
                 (roslisp:ros-info (pp-plans manipulation)
                                   "Trying to place object ~a with arm ~a~%"
                                   object-name arm)
                (let ((left-poses-list-of-lists
                        (list left-reach-poses left-put-poses left-retract-poses))
                      (right-poses-list-of-lists
                        (list right-reach-poses right-put-poses right-retract-poses)))
                  (multiple-value-bind (left-poses right-poses)
                      (equalize-lists-of-lists-lengths left-poses-list-of-lists
                                                       right-poses-list-of-lists)
                    (mapcar (lambda (left-pose right-pose)
                              (pr2-proj::gripper-action :open arm)
                              (pr2-proj::move-tcp left-pose right-pose)
                              (sleep 0.1)
                              (when (or
                                     (remove object-name
                                             (btr:find-objects-in-contact
                                              btr:*current-bullet-world*
                                              (btr:get-robot-object))
                                             :key #'btr:name)
                                     (remove (btr:name
                                              (find-if (lambda (x)
                                                         (typep x 'btr:semantic-map-object))
                                                       (btr:objects btr:*current-bullet-world*)))
                                             (remove (btr:get-robot-name)
                                                     (btr:find-objects-in-contact
                                                      btr:*current-bullet-world*
                                                      (btr:object
                                                       btr:*current-bullet-world*
                                                       object-name))
                                                     :key #'btr:name)
                                             :key #'btr:name))
                                (btr::restore-world-state world-state world)
                                (cpl:fail 'common-fail:manipulation-pose-in-collision)))
                            left-poses
                            right-poses)))))))
      (btr::restore-world-state world-state world)
      (format t "CLEANING UP PLACING COLLISION CHECK~%"))))

(defun deliver (?object-designator ?target-location)
  (cpl:with-retry-counters ((target-location-retries 5))
    (cpl:with-failure-handling
        (((or common-fail:object-unreachable
              common-fail:navigation-pose-in-collision) (e)
           (roslisp:ros-warn (pp-plans deliver) "Failure happened: ~a" e)
           (cpl:do-retry target-location-retries
             (setf ?target-location (desig:next-solution ?target-location))
             (if ?target-location
                 (progn
                   (roslisp:ros-warn (pp-plans deliver) "Retrying...~%")
                   (cpl:retry))
                 (progn
                   (roslisp:ros-warn (pp-plans deliver) "No samples left :'(~%")
                   (cpl:fail 'common-fail:object-undeliverable))))
           (roslisp:ros-warn (pp-plans deliver) "No target-location-retries left :'(~%")
           (cpl:fail 'common-fail:object-undeliverable)))

      (let* ((?pose-at-target-location (desig:reference ?target-location))

             (?nav-location (desig:a location
                                     (reachable-for pr2)
                                     (location (desig:a location
                                                        (pose ?pose-at-target-location))))))


        (cpl:with-retry-counters ((relocation-for-ik-retries 10))
          (cpl:with-failure-handling
              (((or common-fail:object-unreachable
                    common-fail:manipulation-pose-in-collision) (e)
                 (roslisp:ros-warn (pp-plans deliver) "Object is unreachable: ~a" e)
                 (cpl:do-retry relocation-for-ik-retries
                   (setf ?nav-location (next-solution ?nav-location))
                   (if ?nav-location
                       (progn
                         (roslisp:ros-info (pp-plans deliver) "Relocating...")
                         (cpl:retry))
                       (progn
                         (roslisp:ros-warn (pp-plans deliver) "No more samples to try :'(")
                         (cpl:fail 'common-fail:object-undeliverable))))
                 (return)))

            (go-without-collisions ?nav-location)
            (setf ?nav-location (desig:current-desig ?nav-location))

            (exe:perform (desig:an action
                                   (type looking)
                                   (target (desig:a location
                                                    (pose ?pose-at-target-location)))))

            (let ((placing-action
                    (desig:an action
                              (type placing)
                              (object ?object-designator)
                              (target (desig:a location
                                               (pose ?pose-at-target-location))))))
              (check-placing-collisions placing-action)
              (setf placing-action (desig:current-desig placing-action))
              (exe:perform placing-action)
              (return-from deliver))))

        (roslisp:ros-warn (pp-plans deliver) "No relocation-for-ik-retries left :'(")
        (cpl:fail 'common-fail:object-undeliverable)))))

(defun demo-hard-coded ()
  (spawn-objects-on-sink-counter)

  (with-simulated-robot

    (dolist (object-type '(:cereal :cup :bowl :spoon ;; :milk
                           ))

      (let ((placing-target
              (cl-transforms-stamped:pose->pose-stamped
               "map" 0.0
               (cram-bullet-reasoning:ensure-pose
                (cdr (assoc object-type *object-placing-poses*)))))
            (arm-to-use
              (cdr (assoc object-type *object-grasping-arms*))))

        (pick-object object-type arm-to-use)
        (format t "NOW OBJECT IN HAND? ~a~%" (prolog:prolog `(cpoe:object-in-hand ?obj ?arm)))
        (place-object placing-target arm-to-use)))))

(defun demo-random ()
  (let ((?pose (cl-transforms-stamped:make-pose-stamped
                cram-tf:*fixed-frame*
                0.0
                (cl-transforms:make-identity-vector)
                (cl-transforms:make-identity-rotation))))
   (exe:perform
    (desig:an action
              (type going)
              (target (desig:a location
                               (pose ?pose))))))

  (when (eql cram-projection:*projection-environment*
             'cram-pr2-projection::pr2-bullet-projection-environment)
    (btr:detach-all-objects (btr:get-robot-object))
    (spawn-objects-on-sink-counter-randomly))

  (setf cram-robot-pose-guassian-costmap::*orientation-samples* 3)

  (let ((list-of-objects '(:cup :spoon :cereal :bowl :milk)))
    (let* ((short-list-of-objects (remove (nth (random (length list-of-objects))
                                               list-of-objects)
                                          list-of-objects)))
      (setf short-list-of-objects (remove (nth (random (length short-list-of-objects))
                                               short-list-of-objects)
                                          short-list-of-objects))

      (dolist (?object-type list-of-objects)
        (let ((?placing-target-pose
                (cl-transforms-stamped:pose->pose-stamped
                 "map" 0.0
                 (cram-bullet-reasoning:ensure-pose
                  (cdr (assoc ?object-type *object-placing-poses*))))))

          (cpl:with-failure-handling
              ((common-fail:high-level-failure (e)
                 (roslisp:ros-warn (pp-plans demo) "Failure happened: ~a~%Skipping..." e)
                 (return)))
            (let* ((?object
                     (fetch (desig:an object
                                      (type ?object-type))
                            (desig:a location
                                     (on "CounterTop")
                                     (name "iai_kitchen_sink_area_counter_top")))))

              (let* ((object-pose-in-base
                       (desig:reference (desig:a location (of ?object))))
                     (?object-pose-in-map
                       (cram-tf:ensure-pose-in-frame
                        object-pose-in-base cram-tf:*fixed-frame* :use-zero-time t)))

                (cpl:with-failure-handling
                    ((common-fail:high-level-failure (e)
                       (declare (ignore e))
                       (let ((?map-in-front-of-sink-pose
                               (cl-transforms-stamped:make-pose-stamped
                                cram-tf:*fixed-frame*
                                0.0
                                (cl-transforms:make-3d-vector 0.6 -0.5 0)
                                (cl-transforms:make-identity-rotation)))
                             (?placing-pose
                               (cl-transforms-stamped:make-pose-stamped
                                cram-tf:*robot-base-frame*
                                0.0
                                (cl-transforms:make-3d-vector 0.7 0 1.3)
                                (cl-transforms:make-identity-rotation))))
                         (exe:perform
                          (desig:an action
                                    (type going)
                                    (target (desig:a location
                                                     (pose ?map-in-front-of-sink-pose)))))
                         (exe:perform
                          (desig:an action
                                    (type placing)
                                    (target (desig:a location
                                                     (pose ?placing-pose)))))
                         (break))))
                  (deliver ?object
                           (desig:a location
                                    (pose ?placing-target-pose))))))))))))
