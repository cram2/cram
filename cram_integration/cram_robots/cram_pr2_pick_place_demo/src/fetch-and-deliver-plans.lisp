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

(cpl:def-cram-function go-without-collisions (?navigation-location &optional (retries 10))
  ;; (declare (type desig:location-designator ?navigation-location))

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
                    (handler-case
                        (setf ?navigation-location (desig:next-solution ?navigation-location))
                      (desig:designator-error ()
                        (cpl:fail 'common-fail:navigation-pose-in-collision)))
                    (if ?navigation-location
                        (progn
                          (roslisp:ros-warn (pp-plans check-nav-collisions) "Retrying...~%")
                          (cpl:retry))
                        (progn
                          (roslisp:ros-warn (pp-plans check-nav-collisions)
                                            "No more samples left to try :'(.")
                          (cpl:fail 'common-fail:navigation-pose-in-collision))))
                  (roslisp:ros-warn (pp-plans go-without-collisions)
                                    "Couldn't find a nav pose for~%~a.~%Propagating up."
                                    ?navigation-location)
                  (cpl:fail 'common-fail:navigation-pose-in-collision)))

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
                 (cpl:fail 'common-fail:navigation-pose-in-collision)))))

      ;; After playing around and messing up the world, restore the original state.
      (btr::restore-world-state world-state world)))

  (cpl:with-failure-handling
      (((or common-fail:navigation-low-level-failure
            common-fail:actionlib-action-timed-out) (e)
         (roslisp:ros-warn (pp-plans go-without-coll)
                           "Navigation failed: ~a~%.Assuming pose in collision." e)
         (cpl:fail 'common-fail:navigation-pose-in-collision)))
    (exe:perform (desig:an action
                           (type going)
                           (target ?navigation-location)))))



(cpl:def-cram-function search-for-object (?object-designator ?search-location
                                                             &optional (retries 2))

  (cpl:with-retry-counters ((search-location-retries retries))
    (cpl:with-failure-handling
        (((or common-fail:perception-low-level-failure
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
        (exe:perform (desig:an action
                               (type navigating)
                               (location ?nav-location)))
        (exe:perform (desig:an action
                               (type looking)
                               (target (desig:a location
                                                (pose ?pose-at-search-location))))))
      (exe:perform (desig:an action
                             (type detecting)
                             (object ?object-designator))))))



(defvar *obj* nil)
(cpl:def-cram-function fetch (?object-designator ?search-location)
  (let* ((object-designator-properties
           (desig:properties ?object-designator))
         (?perceived-object-desig
           (exe:perform (desig:an action
                                  (type searching)
                                  (object ?object-designator)
                                  (location ?search-location))))
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

        (cpl:with-retry-counters ((relocation-for-ik-retries 3))
          (cpl:with-failure-handling
              (((or common-fail:object-unreachable
                    common-fail:perception-low-level-failure
                    common-fail:gripping-failed) (e)
                 (roslisp:ros-warn (pp-plans fetch) "Object is unreachable: ~a" e)
                 (cpl:do-retry relocation-for-ik-retries
                   (setf ?pick-up-location (next-solution ?pick-up-location))
                   (if ?pick-up-location
                       (progn
                         (roslisp:ros-info (pp-plans fetch) "Relocating...")
                         (cpl:retry))
                       (progn
                         (roslisp:ros-warn (pp-plans fetch) "No more samples to try :'(")
                         (cpl:fail 'common-fail:object-unfetchable))))
                 (roslisp:ros-warn (pp-plans fetch) "No more retries left :'(")
                 (cpl:fail 'common-fail:object-unfetchable)))

            (flet ((reperceive (copy-of-object-designator-properties)
                     (let* ((?copy-of-object-designator
                              (desig:make-designator :object copy-of-object-designator-properties))
                            (?more-precise-perceived-object-desig
                              (exe:perform (desig:an action
                                                     (type detecting)
                                                     (object ?copy-of-object-designator)))))
                       ;; (desig:equate ?object-designator ?more-precise-perceived-object-desig)
                       (let ((pick-up-action
                               (desig:an action
                                         (type picking-up)
                                         (object ?more-precise-perceived-object-desig))))
                         (check-picking-up-collisions pick-up-action)
                         (setf pick-up-action (desig:current-desig pick-up-action))
                         (exe:perform pick-up-action)
                         (setf *obj* ?more-precise-perceived-object-desig)))))

              (exe:perform (desig:an action
                                     (type navigating)
                                     (location ?pick-up-location)))
              (setf ?pick-up-location (desig:current-desig ?pick-up-location))

              (exe:perform (desig:an action
                                     (type looking)
                                     (target (desig:a location
                                                      (pose ?perceived-object-pose-in-map)))))

              (reperceive object-designator-properties))))))

    (pp-plans:park-arms)
    (desig:current-desig ?object-designator)))



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

            (exe:perform (desig:an action
                                   (type navigating)
                                   (location ?nav-location)))
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


(defun drop-at-sink ()
  (let ((?map-in-front-of-sink-pose
          (cl-transforms-stamped:make-pose-stamped
           cram-tf:*fixed-frame*
           0.0
           (cl-transforms:make-3d-vector 0.7 -0.4 0)
           (cl-transforms:make-identity-rotation)))
        (?placing-pose
          (cl-transforms-stamped:make-pose-stamped
           cram-tf:*robot-base-frame*
           0.0
           (cl-transforms:make-3d-vector 0.7 0 1.2)
           (cl-transforms:make-identity-rotation))))
    (cpl:with-failure-handling
        ((common-fail:navigation-low-level-failure (e)
           (declare (ignore e))
           (return)))
      (exe:perform
       (desig:an action
                 (type going)
                 (target (desig:a location
                                  (pose ?map-in-front-of-sink-pose))))))
    (cpl:with-failure-handling
        ((cpl:plan-failure (e)
           (declare (ignore e))
           (return)))
      (exe:perform
       (desig:an action
                 (type placing)
                 (target (desig:a location
                                  (pose ?placing-pose))))))))

(cpl:def-cram-function fetch-and-deliver (?object-designator
                                          ?fetching-location ?delivering-location)
  (let ((?fetched-object
          (exe:perform (desig:an action
                                 (type fetching)
                                 (object ?object-designator)
                                 (location ?fetching-location)))))
    (cpl:with-failure-handling
        ((common-fail:high-level-failure (e)
           (declare (ignore e))
           (drop-at-sink)))
      (exe:perform (desig:an action
                             (type delivering)
                             (object ?fetched-object)
                             (target ?delivering-location))))))
