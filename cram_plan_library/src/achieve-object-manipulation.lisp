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

(in-package :plan-lib)

(def-goal (achieve (object-picked ?obj))
  (ros-info (achieve plan-lib) "(achieve (object-picked))")
  (with-retry-counters ((carry-retry-count 1)
                        (lift-retry-count 1)
                        (misgrasp-retry-count 1)
                        (near-reperceive-retry-count 3))
    (with-designators ((grasp-action
                        (action `((type trajectory) (to grasp) (obj ,?obj))))
                       (lift-action
                        (action `((type trajectory) (to lift) (obj ,?obj))))
                       (carry-action
                        (action `((type trajectory) (to carry) (obj ,?obj)))))
      (let ((obj-orig ?obj)
            (obj-look-location (make-designator 'location `((of ,?obj)))))
        (with-failure-handling
            ((object-not-found (f)
               (declare (ignore f))
               (ros-warn
                (achieve plan-lib) "Failed to reperceive object from near.")
               (do-retry near-reperceive-retry-count
                 (setf obj-look-location (next-different-location-solution
                                          obj-look-location))
                 (when obj-look-location
                   (ros-warn
                    (achieve plan-lib) "Retrying with new look location.")
                   (retry))))
             (manipulation-pose-unreachable (f)
               (declare (ignore f))
               (ros-warn (achieve plan-lib) "Failed to grasp object.")
               (do-retry misgrasp-retry-count
                 (ros-warn (achieve plan-lib) "Retrying.")
                 (retry)))
             (object-lost (f)
               (declare (ignore f))
               (cpl:fail 'manipulation-pose-unreachable)))
          (ros-info () "Looking at object: ~a~%" obj-look-location)
          (achieve `(looking-at ,(reference obj-look-location)))
          (setf ?obj (first (perceive-object 'currently-visible ?obj)))
          (when (not ?obj)
            (setf ?obj obj-orig)
            (cpl:fail 'object-not-found))
          (achieve `(looking-at ,(reference
                                  (make-designator 'location `((of ,?obj))))))
          (ros-warn (achieve plan-lib) "Found object.")
          (with-failure-handling
              ((object-lost (f)
                 (declare (ignore f))
                 (ros-warn (achieve plan-lib) "Grasp failed.")
                 (do-retry misgrasp-retry-count
                   (ros-warn (achieve plan-lib) "Reperceiving and retrying.")
                   (setf ?obj (first (perceive-object 'currently-visible ?obj)))
                   (when (not ?obj)
                     (setf ?obj obj-orig)
                     (cpl:fail 'object-not-found))
                   (achieve `(looking-at
                              ,(reference
                                (make-designator 'location `((of ,?obj))))))
                   (retry))))
            (when (not (desig-equal obj-orig ?obj))
              (equate obj-orig ?obj))
            (perform grasp-action)
            (monitor-action grasp-action))))
      (ros-warn (achieve plan-lib) "Grasped object.")
      (with-failure-handling
          ((manipulation-pose-unreachable (f)
             (declare (ignore f))
             (ros-warn (achieve plan-lib) "Lift failed.")
             (do-retry lift-retry-count
               (ros-warn (achieve plan-lib) "Retrying.")
               (retry))
             (return)))
        (perform lift-action)
        (monitor-action lift-action))
      (ros-warn (achieve plan-lib) "Lifted object.")
      (with-failure-handling
          ((manipulation-pose-unreachable (f)
             (declare (ignore f))
             (ros-warn (achieve plan-lib) "Carry failed.")
             (do-retry carry-retry-count
               (ros-warn (achieve plan-lib) "Retrying.")
               (retry))
             (return)))
        (perform carry-action)
        (monitor-action carry-action))
      (ros-warn (achieve plan-lib) "Went into carry pose.")))
  ?obj)

(def-goal (achieve (object-in-hand ?obj))
  (ros-info (achieve plan-lib) "(achieve (object-in-hand))")
  (with-retry-counters ((alt-perception-poses-cnt 3)
                        (alt-grasp-poses-cnt 3)
                        (initial-perception-retry-count 3))
    (with-designators ((pick-up-loc
                        (location `((to reach) (obj ,?obj)))))
      (with-failure-handling
          ((manipulation-pose-unreachable (f)
             (declare (ignore f))
             (ros-warn
              (achieve plan-lib)
              "Got unreachable grasp pose.")
             (do-retry alt-perception-poses-cnt
               (ros-warn
                (achieve plan-lib)
                "Re-perceiving object.")
               (retry))))
        (with-failure-handling
            ((object-not-found (f)
               (declare (ignore f))
               (ros-warn
                (achiebe plan-lib)
                "Failed to perceive object in the first place.")
               (do-retry initial-perception-retry-count
                 (ros-warn (achiebe plan-lib) "Retrying.")
                 (retry))))
          (ros-info (achieve plan-lib) "Perceiving object")
          (setf ?obj (perceive-object 'a ?obj)))
        (ros-info (achieve plan-lib) "Perceive done")
        (with-failure-handling
            ((manipulation-pose-unreachable (f)
               (declare (ignore f))
               (ros-warn
                (achieve plan-lib)
                "Couldn't reach object.")
               (do-retry alt-grasp-poses-cnt
		 (ros-warn
		  (achieve plan-lib)
		  "Trying from different pose.")
                 (retry-with-updated-location
                  pick-up-loc (next-different-location-solution
                               pick-up-loc)))))
          (ros-info (achieve plan-lib) "Grasping")
          (at-location (pick-up-loc)
            (achieve `(object-picked ,?obj)))))))
  ?obj)

(def-goal (achieve (object-put ?obj ?loc))
  (ros-info (achieve plan-lib) "(achieve (object-put))")
  (let ((obj (current-desig ?obj)))
    (assert
     (holds `(object-in-hand ,obj)) ()
     "The object `~a' needs to be in the hand before being able to place it."
     obj)
    (with-designators ((put-down-action
                        (action `((type trajectory) (to put-down)
                                  (obj ,obj) (at ,?loc))))
                       (park-action
                        (action `((type trajectory) (to park) (obj ,obj)))))
      (with-failure-handling
          ((manipulation-failure (f)
             (declare (ignore f))
             (ros-warn
              (achieve plan-lib)
              "Got unreachable putdown pose.")
             (cpl:fail 'manipulation-pose-unreachable)))
        (achieve `(looking-at ,(reference ?loc)))
        (perform put-down-action)
        (monitor-action put-down-action))
      (perform park-action)
      (monitor-action park-action))))

(def-goal (achieve (object-placed-at ?obj ?loc))
  (ros-info (achieve plan-lib) "(achieve (object-placed-at))")
  (let ((obj (current-desig ?obj)))
    (assert
     (holds `(object-in-hand ,obj)) ()
     "The object `~a' needs to be in the hand before being able to place it."
     obj)
    (with-retry-counters ((goal-pose-retries 3)
                          (manipulation-retries 3))
      (with-failure-handling
          ((designator-error (condition)
             (when (eq (desig-prop-value (designator condition) 'to) 'execute)
               ;; When we couldn't resolve `put-down-loc' the
               ;; destination pose is probably not reachable. In that
               ;; case, we try to find a new solution for `?loc' and
               ;; retry.
               (ros-warn
                (achieve plan-lib)
                "Unable to resolve put-down location designator.")
               (do-retry goal-pose-retries
                 (retry-with-updated-location ?loc (next-solution ?loc)))))
           (manipulation-failure (f)
             (declare (ignore f))
             (ros-warn
              (achieve plan-lib)
              "Got unreachable putdown pose. Trying different put-down
              location")
             (do-retry goal-pose-retries
               (retry-with-updated-location ?loc (next-solution ?loc)))))
        (with-designators ((put-down-loc (location `((to reach)
                                                     (location ,?loc)))))
          (at-location (put-down-loc) 
            (with-failure-handling
                ((manipulation-failure (f)
                   (declare (ignore f))
                   (ros-warn
                    (achieve plan-lib)
                    "Got unreachable putdown pose. Trying alternatives")
                   (do-retry manipulation-retries
                     (retry-with-updated-location
                      put-down-loc
                      (next-different-location-solution put-down-loc)))))
              (achieve `(object-put ,?obj ,put-down-loc)))))))))

;; (def-goal (achieve (object-placed-at ?obj ?loc))
;;   (ros-info (achieve plan-lib) "(achieve (object-placed-at))")
;;   (let ((obj (current-desig ?obj)))
;;     (assert
;;      (holds `(object-in-hand ,obj)) ()
;;      "The object `~a' needs to be in the hand before being able to place it."
;;      obj)
;;     (with-retry-counters ((goal-pose-retries 3)
;;                           (manipulation-retries 3))
;;       (with-failure-handling
;;           ((designator-error (condition)
;;              (when (eq (desig-prop-value (designator condition) 'to) 'execute)
;;                ;; When we couldn't resolve `put-down-loc' the
;;                ;; destination pose is probably not reachable. In that
;;                ;; case, we try to find a new solution for `?loc' and
;;                ;; retry.
;;                (ros-warn
;;                 (achieve plan-lib)
;;                 "Unable to resolve put-down location designator.")
;;                (do-retry goal-pose-retries
;;                  (retry-with-updated-location ?loc (next-solution ?loc)))))
;;            (manipulation-failure (f)
;;              (declare (ignore f))
;;              (ros-warn
;;               (achieve plan-lib)
;;               "Got unreachable putdown pose. Trying different put-down location")
;;              (do-retry goal-pose-retries
;;                (retry-with-updated-location ?loc (next-solution ?loc)))))
;;         (with-designators ((put-down-trajectory
;;                             (action `((type trajectory) (to put-down)
;;                                       (obj ,obj) (at ,?loc))))
;;                            (park-trajectory
;;                             (action `((type trajectory) (to park) (obj ,obj))))
;;                            (put-down-loc
;;                             (location `((to reach)
;;                                         (location ,?loc)))))
;;           (with-failure-handling
;;               ((manipulation-failure (f)
;;                  (declare (ignore f))
;;                  (ros-warn
;;                   (achieve plan-lib)
;;                   "Got unreachable putdown pose. Trying alternatives")
;;                  (do-retry manipulation-retries
;;                    (retry-with-updated-location
;;                     put-down-loc
;;                     (next-different-location-solution put-down-loc)))))
;;             (at-location (put-down-loc)
;;               (achieve `(looking-at ,(reference ?loc)))
;;               (perform put-down-trajectory)
;;               (monitor-action put-down-trajectory)))
;;           (perform park-trajectory)
;;           (monitor-action park-trajectory))))))

(def-goal (achieve (arms-parked))
  (with-designators ((parking (action `((type trajectory) (to park)))))
    (perform parking)
    (monitor-action parking)))
