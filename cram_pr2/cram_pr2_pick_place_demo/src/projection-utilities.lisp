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

(defmacro with-simulated-robot (&body body)
  `(let ((results
           (proj:with-projection-environment pr2-proj::pr2-bullet-projection-environment
             (cpl:top-level
               ,@body))))
     (car (cram-projection::projection-environment-result-result results))))

(defun add-objects-to-mesh-list (&optional (ros-package "cram_pr2_pick_place_demo"))
  (mapcar (lambda (object-filename-and-object-extension)
            (declare (type list object-filename-and-object-extension))
            (destructuring-bind (object-filename object-extension)
                object-filename-and-object-extension
              (let ((lisp-name (roslisp-utilities:lispify-ros-underscore-name
                                object-filename :keyword)))
                (push (list lisp-name
                            (format nil "package://~a/resource/~a.~a"
                                    ros-package object-filename object-extension)
                            nil)
                      btr::*mesh-files*)
                (remove-duplicates btr::*mesh-files* :key #'car)
                lisp-name)))
          (mapcar (lambda (pathname)
                    (list (pathname-name pathname) (pathname-type pathname)))
                  (directory (physics-utils:parse-uri
                              (format nil "package://~a/resource/*.*" ros-package))))))

(defun robot-colliding-objects-without-attached ()
  (let ((colliding-object-names
          (mapcar #'btr:name
                  (btr:find-objects-in-contact
                   btr:*current-bullet-world*
                   (btr:get-robot-object))))
        (attached-object-names
          (mapcar #'car
                  (btr:attached-objects (btr:get-robot-object)))))
    (set-difference colliding-object-names attached-object-names)))

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

(defun check-navigating-collisions (navigation-location-desig &optional (samples-to-try 10))
  (declare (type desig:location-designator navigation-location-desig))
  "Store current world state and in the current world try to go to different
poses that satisfy `navigation-location-desig'.
If chosen pose results in collisions, choose another pose.
Repeat `navigation-location-samples' + 1 times.
Store found pose into designator or throw error if good pose not found."
  (let* ((world btr:*current-bullet-world*)
         (world-state (btr::get-state world)))

    (unwind-protect
         (cpl:with-retry-counters ((navigation-location-samples samples-to-try))
           ;; If a navigation-pose-in-collisions failure happens, retry N times
           ;; with the next solution of `navigation-location-desig'.
           (cpl:with-failure-handling
               ((common-fail:navigation-pose-in-collision (e)
                  (roslisp:ros-warn (pp-plans coll-check) "Failure happened: ~a" e)
                  (cpl:do-retry navigation-location-samples
                    (handler-case
                        (setf navigation-location-desig
                              (desig:next-solution navigation-location-desig))
                      (desig:designator-error ()
                        (roslisp:ros-warn (pp-plans coll-check)
                                          "Designator cannot be resolved: ~a. Propagating up." e)
                        (cpl:fail 'common-fail:navigation-pose-in-collision)))
                    (if navigation-location-desig
                        (progn
                          (roslisp:ros-warn (pp-plans coll-check) "Retrying...~%")
                          (cpl:retry))
                        (progn
                          (roslisp:ros-warn (pp-plans coll-check)
                                            "No other samples in designator.")
                          (cpl:fail 'common-fail:navigation-pose-in-collision))))
                  (roslisp:ros-warn (pp-plans coll-check)
                                    "Couldn't find a nav pose for~%~a.~%Propagating up."
                                    navigation-location-desig)
                  (cpl:fail 'common-fail:navigation-pose-in-collision)))

             ;; Pick one pose, store it in `pose-at-navigation-location'
             ;; In projected world, drive to picked pose
             ;; If robot is in collision with any object in the world, throw a failure.
             ;; Otherwise, the pose was found, so return location designator,
             ;; which is currently referenced to the found pose.
             (handler-case
                 (let ((pose-at-navigation-location (desig:reference navigation-location-desig)))
                   (pr2-proj::drive pose-at-navigation-location)
                   (when (robot-colliding-objects-without-attached)
                     (roslisp:ros-warn (pp-plans coll-check) "Pose was in collision.")
                     (cpl:sleep 0.1)
                     (cpl:fail 'common-fail:navigation-pose-in-collision
                               :pose-stamped pose-at-navigation-location))
                   (roslisp:ros-info (pp-plans coll-check) "Found navigation pose.")
                   navigation-location-desig)
               (desig:designator-error (e)
                 (roslisp:ros-warn (pp-plans coll-check)
                                   "Desig ~a couldn't be resolved: ~a.~%Cannot navigate."
                                   navigation-location-desig e)
                 (cpl:fail 'common-fail:navigation-pose-in-collision)))))

      ;; After playing around and messing up the world, restore the original state.
      (btr::restore-world-state world-state world))))


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
                    (handler-case
                        (setf pick-up-action-desig (next-solution pick-up-action-desig))
                      (desig:designator-error ()
                        (roslisp:ros-warn (pp-plans coll-check)
                                          "Designator cannot be resolved: ~a. Propagating up." e)
                        (cpl:fail 'common-fail:object-unreachable)))
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
      (btr::restore-world-state world-state world))))


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
      (btr::restore-world-state world-state world))))

