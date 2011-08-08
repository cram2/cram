;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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
;;;

(in-package :plan-lib)

(def-goal (achieve (object-in-hand ?obj ?side))
  (ros-info (achieve plan-lib) "(achieve (object-in-hand))")
  (let ((retry-count 0)
        (alternative-poses-cnt 0)
        (obstacles nil))
    (with-failure-handling
        ((object-lost (f)
           (ros-warn (achieve plan-lib) "Object lost.")
           (assert-occasion
            `(object-in-hand-failure object-lost ,?obj ,?side ,f))
           (achieve `(arms-at ,(make-designator
                                'action `((type trajectory) (pose parked) (side ,?side)))))
           (when (< (incf retry-count) 3)
             (retry)))
         ;; (manipulation-failure (f)
         ;;   (assert-occasion
         ;;    `(object-in-hand-failure manipulation-failed ,?obj ,?side ,f))
         ;;   (ros-warn (achieve plan-lib) "Manipulation action failed. ~a" f)
         ;;   (setf alternative-poses-cnt 0)
         ;;   (when (< (incf retry-count) 3)
         ;;     (achieve `(arms-at ,(make-designator
         ;;                          'action `((type trajectory) (pose parked) (side ,?side)))))
         ;;     (retract-occasion `(loc Robot ?_))
         ;;     (retry)))
         )
      (ros-info (achieve plan-lib) "Calling perceive")
      (setf ?obj (perceive ?obj))
      (ros-info (achieve plan-lib) "Perceive done")
      (ros-info (achieve plan-lib) "Searching for obstacles")
      (with-designators ((pick-up-see-loc (location `((to see) (obj ,?obj))))
                         (obj-loc (location `((of ,?obj)))))
        (at-location (pick-up-see-loc)
          (setf obstacles (remove-if
                           (lambda (o)
                             (or
                              (< (cl-transforms:v-dist
                                  (cl-transforms:origin (designator-pose o))
                                  (cl-transforms:origin (designator-pose ?obj)))
                                 0.1)
                              (< (cl-transforms:x
                                  (cl-transforms:origin
                                   (tf:transform-pose
                                    *tf* :target-frame "/base_footprint"
                                    :pose (designator-pose o))))
                                 0.35)))
                           (achieve `(obstacles-found ,obj-loc))))))
      (with-designators ((pick-up-loc (location `((to reach) (obj ,?obj))))
                         (grasp-trajectory (action `((type trajectory) (to grasp) (obj ,?obj) (side ,?side)
                                                     ,@(mapcar (lambda (o) `(obstacle ,o))
                                                               obstacles))))
                         (lift-trajectory (action `((type trajectory) (to lift) (obj ,?obj) (side ,?side)
                                                    ,@(mapcar (lambda (o) `(obstacle ,o))
                                                              obstacles))))
                         (carry-trajectory (action `((type trajectory) (to carry) (obj ,?obj) (side ,?side)
                                                     ,@(mapcar (lambda (o) `(obstacle ,o))
                                                               obstacles)))))
        (with-failure-handling
            ((manipulation-pose-unreachable (f)
               (assert-occasion
                `(object-in-hand-failure manipulation-pose-unreachable ,?obj ,?side ,f))
               (ros-warn (achieve plan-lib) "Got unreachable grasp pose. Trying alternatives")
               (when (< alternative-poses-cnt 1)
                 (incf alternative-poses-cnt)
                 (setf pick-up-loc (next-solution pick-up-loc))
                 (retract-occasion `(loc Robot ?_))
                 (achieve `(arms-at ,(make-designator
                                      'action `((type trajectory) (pose parked) (side ,?side)))))
                 (retry))))
          (ros-info (achieve plan-lib) "Grasping")
          (at-location (pick-up-loc)
            (achieve `(looking-at ,(reference (make-designator 'location `((of ,?obj))))))
            (achieve `(arms-at ,grasp-trajectory))))
        (with-failure-handling
            ((manipulation-pose-unreachable (f)
               (declare (ignore f))
               (ros-warn (achieve plan-lib) "Carry pose failed")
               (return)))
          (achieve `(arms-at ,lift-trajectory))
          (achieve `(arms-at ,carry-trajectory))))))
  ?obj)

(def-goal (achieve (object-placed-at ?obj ?loc))
  (ros-info (achieve plan-lib) "(achieve (object-placed-at))")
  (setf ?obj (current-desig ?obj))
  (let ((object-in-hand-bdgs (holds `(object-in-hand ,?obj ?side)))
        (alternative-poses-cnt 0)
        (obstacles nil))
    (assert object-in-hand-bdgs ()
            "The object `~a ~a' needs to be in the hand before being able to place it."
            ?obj (description ?obj))
    (let ((side (var-value '?side (car object-in-hand-bdgs)))
          (obj (current-desig ?obj)))
      (with-designators ((pick-up-see-loc (location `((to see) (location ,?loc)))))
        (at-location (pick-up-see-loc)
          (setf obstacles (remove-if
                           (lambda (o)
                             (or
                              (< (cl-transforms:v-dist
                                  (cl-transforms:origin (designator-pose o))
                                  (cl-transforms:origin (obj-desig-location ?obj)))
                                 0.1)
                              (< (cl-transforms:x
                                  (cl-transforms:origin
                                   (tf:transform-pose
                                    *tf* :target-frame "/base_footprint"
                                    :pose (designator-pose o))))
                                 ;; Evil magic constant! This
                                 ;; filtering should better be done in
                                 ;; the process module
                                 0.35)))
                           (achieve `(obstacles-found ,?loc))))))
      (with-designators ((put-down-loc (location `((to reach) (location ,?loc))))
                         (put-down-trajectory (action `((type trajectory) (to put-down)
                                                        (obj ,obj) (at ,?loc) (side ,side)
                                                        ,@(mapcar (lambda (o) `(obstacle ,o))
                                                                  obstacles))))
                         (park-trajectory (action `((type trajectory) (pose parked) (side ,side)
                                                    ,@(mapcar (lambda (o) `(obstacle ,o))
                                                              obstacles)))))
        (at-location (put-down-loc)
          (with-failure-handling
            ((manipulation-failure (f)
               (assert-occasion
                `(object-in-hand-failure manipulation-pose-unreachable ,?obj ,side ,f))
               (ros-warn (achieve plan-lib) "Got unreachable grasp pose. Trying alternatives")
               (when (< alternative-poses-cnt 1)
                 (incf alternative-poses-cnt)
                 (setf put-down-loc (next-solution put-down-loc))
                 (retract-occasion `(loc Robot ?_))
                 (retry))))
            (achieve `(looking-at ,(reference ?loc)))
            (achieve `(arms-at ,put-down-trajectory)))
          (achieve `(arms-at ,park-trajectory)))
        (assert-occasion `(object-placed-at ,obj ,?loc))))))

(def-goal (achieve (arm-parked ?side))
  (flet ((park-both-arms ()
           (with-designators ((parking (action `((type trajectory) (pose parked) (side :both)))))
             (achieve `(arms-at ,parking))
             (assert-occasion `(arm-parked :left))
             (assert-occasion `(arm-parked :right))))
         (park-one-arm (side)
           (unless (holds `(arm-parked ,side))
             (with-designators ((parking (action `((type trajectory) (pose parked) (side ,side)))))
               (achieve `(arms-at ,parking)))
             (assert-occasion `(arm-parked ,side)))))
    (ros-info (achieve plan-lib) "(achieve (arm-parked ~a))" ?side)
    (let ((parked-arms (remove nil (list (holds `(arm-parked :left))
                                         (holds `(arm-parked :right)))))
          (carried-objs (holds `(object-in-hand ?_ ?_))))
      (ros-info (achieve plan-lib) "Already parked arms ~a~%" parked-arms)
      (case ?side
        (:both (cond ((or parked-arms carried-objs)
                      (park-one-arm :left)
                      (park-one-arm :right))
                     (t
                      (park-both-arms))))
        ((:left :right) (park-one-arm ?side))))))

(def-goal (achieve (arms-at ?traj))
  (let ((side (desig-prop-value ?traj 'side)))
    (retract-occasion `(arms-at ?_))    
    (pm-execute :manipulation ?traj)
    (when (member side '(:both :left))
      (retract-occasion `(arm-parked :left)))
    (when (member side '(:both :right))
      (retract-occasion `(arm-parked :right)))
    (retract-occasion `(arms-at ?_))
    (assert-occasion `(arms-at ,?traj))))
