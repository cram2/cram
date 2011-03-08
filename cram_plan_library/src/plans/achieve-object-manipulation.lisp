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
        (alternative-poses-cnt 0))
    (with-failure-handling
        ((object-lost (f)
           (declare (ignore f))
           (ros-warn (achieve plan-lib) "Object lost.")
           (when (< (incf retry-count) 3)
             (retry)))
         (manipulation-failed (f)
           (assert-occasion
            `(object-in-hand-failure manipulation-failed ,?obj ,?side ,f))
           (ros-warn (achieve plan-lib) "Manipulation action failed. ~a" f)
           (setf alternative-poses-cnt 0)
           (achieve `(arms-at ,(make-designator 'action `((type trajectory) (pose open) (side ,?side)))))
           (when (< (incf retry-count) 3)
             (retry))))
      (ros-info (achieve plan-lib) "Calling perceive")
      (setf ?obj (perceive ?obj))
      (ros-info (achieve plan-lib) "Perceive done")
      (with-designators ((pick-up-loc (location `((to reach) (obj ,?obj))))
                         (open-trajectory (action `((type trajectory) (pose open) (side ,?side))))
                         (grasp-trajectory (action `((type trajectory) (to grasp) (obj ,?obj) (side ,?side))))
                         (lift-trajectory (action `((type trajectory) (to lift) (obj ,?obj) (side ,?side))))
                         (carry-trajectory (action `((type trajectory) (to carry) (obj ,?obj) (side ,?side)))))
        (with-failure-handling
            ((manipulation-pose-unreachable (f)
               (assert-occasion
                `(object-in-hand-failure manipulation-pose-unreachable ,?obj ,?side ,f))
               (ros-warn (achieve plan-lib) "Got unreachable grasp pose. Trying alternatives")
               (when (< alternative-poses-cnt 3)
                 (incf alternative-poses-cnt)
                 (setf pick-up-loc (next-solution pick-up-loc))
                 (retract-occasion `(loc Robot ?_))
                 (retry))))

          ;; It is not good to open the arm and then drive as long as
          ;; driving might tuck the arms again. Right now, we open
          ;; them twice just in case that happens, but this is a
          ;; pretty bad solution. The right solution would be to pass
          ;; some semantic information to at-location, e.g. that we
          ;; navigate to go to a good grasp position and that we do
          ;; not want to move the arms in such a case.
          (ros-info (achieve plan-lib) "Grasping")
          (achieve `(arms-at ,open-trajectory))
          (at-location (pick-up-loc)
            (achieve `(arms-at ,open-trajectory))
            (achieve `(looking-at ,(reference (make-designator 'location `((of ,?obj))))))
            (achieve `(arms-at ,grasp-trajectory))
            (assert-occasion `(object-in-hand ,?obj ,?side))
            (achieve `(arms-at ,lift-trajectory))
            (achieve `(arms-at ,carry-trajectory)))))))
  ?obj)

(def-plan get-free-location (location-desig &optional (threshold 0.2))
  (flet ((check-distance (obj pose)
           (typecase obj
             (perception-pm:cop-perceived-object
                (let* ((jlo (jlo:frame-query (jlo:make-jlo :name "/base_link")
                                            (perception-pm:object-jlo obj)))
                       (cov-max (max (jlo:cov jlo 0 0)
                                     (jlo:cov jlo 1 1))))
                  (> (cl-transforms:v-dist (cl-transforms:origin pose)
                                           (cl-transforms:origin
                                            (perception-pm:object-pose obj)))
                     (+ threshold cov-max))))
             (t (> (cl-transforms:v-dist (cl-transforms:origin pose)
                                         (cl-transforms:origin
                                          (perception-pm:object-pose obj)))
                   threshold)))))
    (when location-desig
      (with-designators ((cluster (object `((type cluster) (at ,location-desig)))))
        (achieve `(looking-at ,location-desig))
        (with-failure-handling ((object-not-found (f)
                                  (declare (ignore f))
                                  (return location-desig)))
          (let ((clusters (achieve `(object-detected ,cluster))))
            (if (some (lambda (obj)
                        (check-distance (reference obj) (reference location-desig)))
                      clusters)
                location-desig
                (get-free-location (next-solution location-desig) threshold))))))))

(def-goal (achieve (object-placed-at ?obj ?loc))
  (ros-info (achieve plan-lib) "(achieve (object-placed-at))")
  (setf ?obj (current-desig ?obj))
  (let ((object-in-hand-bdgs (holds `(object-in-hand ,?obj ?side)))
        (alternative-poses-cnt 0)
        (retry-count 0))
    (assert object-in-hand-bdgs ()
            "The object `~a ~a' needs to be in the hand before being able to place it."
            ?obj (description ?obj))
    (let ((side (var-value '?side (car object-in-hand-bdgs)))
          (obj (current-desig ?obj)))
      (with-failure-handling
          ((manipulation-failed (f)
             (declare (ignore f))
             (ros-warn (achieve plan-lib) "Manipulation action failed.")
             (with-designators ((open-trajectory (action `((type trajectory) (pose open) (side ,side)))))
               (achieve `(arms-at ,open-trajectory)))
             (when (< (incf retry-count) 3)
               (retry))))
        (ros-info (achieve-object-placed-at plan-lib) "searching for free locations")
        (setf ?loc (get-free-location ?loc))
        (ros-info (achieve-object-placed-at plan-lib) "found free location ~a" (reference ?loc))        
        (with-designators ((put-down-loc (location `((to reach) (location ,?loc))))
                           (put-down-trajectory (action `((type trajectory) (to put-down)
                                                          (obj ,obj) (at ,?loc) (side ,side))))
                           (open-trajectory (action `((type trajectory) (pose open) (side ,side))))
                           (hand-open-trajectory (action `((type trajectory) (to open) (gripper ,side))))
                           (unhand-trajectory (action `((type trajectory) (to lift) (side ,side)))))
          (with-failure-handling
              ((manipulation-pose-unreachable (f)
                 (assert-occasion
                  `(object-in-hand-failure manipulation-pose-unreachable ,?obj ,side ,f))
                 (ros-warn (achieve plan-lib) "Got unreachable grasp pose. Trying alternatives")
                 (when (< alternative-poses-cnt 3)
                   (incf alternative-poses-cnt)
                   (setf put-down-loc (next-solution put-down-loc))
                   (retract-occasion `(loc Robot ?_))
                   (retry))))
            (at-location (put-down-loc)
              (achieve `(looking-at ,(reference ?loc)))
              (achieve `(arms-at ,open-trajectory))              
              (achieve `(arms-at ,put-down-trajectory))
              (achieve `(arms-at ,hand-open-trajectory))
              (achieve `(arms-at ,unhand-trajectory))))
          (retract-occasion `(object-in-hand ,obj ?_))
          (assert-occasion `(object-placed-at ,obj ,?loc)))))))

(def-goal (achieve (arm-parked ?side))
  (flet ((park-both-arms ()
           (with-designators ((open (action `((type trajectory) (pose open) (side :both))))
                              (parking (action `((type trajectory) (pose parked) (side :both))))
                              (hand-open (action `((type trajectory) (to open) (gripper :both))))
                              (hand-closed (action `((type trajectory) (to close) (gripper :both)))))
             (achieve `(arms-at ,hand-open))
             (achieve `(arms-at ,open))
             (achieve `(arms-at ,hand-closed))
             (achieve `(arms-at ,parking))
             (assert-occasion `(arm-parked :left))
             (assert-occasion `(arm-parked :right))))
         (park-one-arm (side)
           (unless (holds `(arm-parked ,side))
             (if (holds `(object-in-hand ?_ ,side))
                 (with-designators ((parking (action `((type trajectory) (to carry) (side ,side)))))
                   (ros-info (achieve plan-lib) "going to left carry pose")
                   (achieve `(arms-at ,parking)))
                 (with-designators ((hand-open (action `((type trajectory) (to open) (gripper ,side))))
                                    (hand-closed (action `((type trajectory) (to close) (gripper ,side))))
                                    (open (action `((type trajectory) (pose open) (side ,side))))
                                    (parking (action `((type trajectory) (pose parked) (side ,side)))))
                   (achieve `(arms-at ,hand-open))
                   (achieve `(arms-at ,open))
                   (achieve `(arms-at ,hand-closed))
                   (achieve `(arms-at ,parking))))
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
