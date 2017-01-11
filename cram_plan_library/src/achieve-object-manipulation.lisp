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

(define-hook cram-language::on-performing-object-grasp (object))
(define-hook cram-language::on-performing-object-putdown (object pose))

(defun try-reference-location (loc)
  (with-retry-counters ((designator-resolution-retry-counter 10))
    (with-failure-handling
        ((designator-error (e)
           (declare (ignore e))
           (do-retry designator-resolution-retry-counter
             (retry))))
      (reference loc))))

(def-goal (achieve (object-picked ?obj))
  (ros-info (achieve plan-lib) "(achieve (object-picked))")
  (with-retry-counters ((lift-retry-count 2)
                        (misgrasp-retry-count 0)
                        (near-reperceive-retry-count 1)
                        (carry-retry-count 20))
    (with-designators ((obj-loc
                        :location `((:of ,?obj)))
                       (grasp-action
                        :action `((:type :trajectory) (:to :grasp) (:obj ,?obj)))
                       (lift-action
                        :action `((:type :trajectory) (:to :lift) (:obj ,?obj)))
                       (carry-action
                        :action `((:type :trajectory) (:to :carry) (:obj ,?obj))))
      (with-failure-handling
          ((object-not-found (f)
             (declare (ignore f))
             (ros-warn
              (achieve plan-lib) "Failed to perceive object from near.")
             (do-retry near-reperceive-retry-count
               (ros-warn (achieve plan-lib) "Retrying.")
               (retry))))
        (ros-info (achieve plan-lib) "Looking at object location: ~a~%" obj-loc)
        (let ((perceived-object
                (progn
                  (achieve `(looking-at ,(reference obj-loc)))
                  (let ((perceived-object
                          (first (perceive-object :currently-visible
                                                  ?obj))))
                    (unless perceived-object
                      (ros-info (achieve plan-lib) "Didn't find the object.")
                      (fail 'object-not-found))
                    perceived-object))))
          (ros-info (achieve plan-lib) "Found the object.")
          (when (not (desig-equal ?obj perceived-object))
            (equate ?obj perceived-object))
          (with-failure-handling
              ((manipulation-failure (f)
                 (declare (ignore f))
                 (ros-warn (achieve plan-lib) "Failed to grasp object.")
                 (do-retry misgrasp-retry-count
                   (ros-warn (achieve plan-lib) "Retrying.")
                   (retry))
                 (fail 'manipulation-pose-unreachable)))
            (achieve `(looking-at ,(reference
                                    (make-designator
                                     :location
                                     `((:of ,perceived-object))))))
            (cram-language::on-performing-object-grasp perceived-object)
            (perform grasp-action)
            (monitor-action grasp-action))))
      (ros-info (achieve plan-lib) "Grasped object.")
      (with-failure-handling
          (((or manipulation-failure
                manipulation-pose-unreachable) (f)
             (declare (ignore f))
             (ros-warn (achieve plan-lib) "Lift failed.")
             (do-retry lift-retry-count
               (ros-warn (achieve plan-lib) "Retrying.")
               (retry))
             ;; NOTE(winkler): When lift fails, an unhand action must
             ;; be performed. This is not done yet, and this failure
             ;; (although it almost never ever comes up, but might)
             ;; will lead to a different pose from which the robot
             ;; will go into the carry pose. This might turn out to be
             ;; problematic.
             (with-designators
                 ((emergency-put-location
                   :location `((:of ,?obj)))
                  (emergency-put-action
                   :action `((:type :trajectory) (:to :put-down) (:obj ,?obj)
                             (:at ,emergency-put-location))))
               (try-reference-location emergency-put-location)
               (perform emergency-put-action)
               (monitor-action emergency-put-action)
               (achieve `(arms-parked))
               (fail 'manipulation-pose-unreachable))))
        (perform lift-action)
        (monitor-action lift-action))
      (ros-info (achieve plan-lib) "Lifted object.")
      (reset-counter carry-retry-count)
      (with-failure-handling
          ((manipulation-failure (f)
             (declare (ignore f))
             (ros-warn (achieve plan-lib) "Carry failed.")
             (do-retry carry-retry-count
               (ros-warn (achieve plan-lib) "Retrying.")
               (retry))
             (fail 'manipulation-pose-unreachable)))
        (perform carry-action)
        (monitor-action carry-action))
      (ros-info (achieve plan-lib) "Went into carry pose.")))
  (current-desig ?obj))

(def-goal (achieve (object-in-hand ?obj))
  (ros-info (achieve plan-lib) "(achieve (object-in-hand))")
  (with-retry-counters ((alt-perception-poses-cnt 3)
                        (initial-perception-retry-count 3)
                        (alt-grasp-poses-cnt 6))
    (with-designators ((pick-up-loc :location `((:to :reach) (:obj ,?obj))))
      (with-failure-handling
          ((manipulation-pose-unreachable (f)
             (declare (ignore f))
             (ros-warn (achieve plan-lib) "Got unreachable grasp pose.")
             (do-retry alt-perception-poses-cnt
               (ros-warn (achieve plan-lib) "Re-perceiving object.")
               (retry))))
        (reset-counter initial-perception-retry-count)
        (let ((perceived-object
                (with-failure-handling
                    ((object-not-found (f)
                       (declare (ignore f))
                       (ros-warn
                        (achieve plan-lib)
                        "Failed to perceive object in the first place.")
                       (do-retry initial-perception-retry-count
                         (ros-warn (achieve plan-lib) "Retrying.")
                         (retry))))
                  (ros-info (achieve plan-lib) "Perceiving object")
                  (perceive-object :a ?obj))))
          (ros-info (achieve plan-lib) "Perceive done")
          (reset-counter alt-grasp-poses-cnt)
          (with-failure-handling
              ((manipulation-pose-unreachable (f)
                 (declare (ignore f))
                 (ros-warn
                  (achieve plan-lib)
                  "Couldn't reach object.")
                 (do-retry alt-grasp-poses-cnt
                   (ros-warn (achieve plan-lib) "Trying from different pose.")
                   (retry-with-updated-location
                    pick-up-loc (next-different-location-solution pick-up-loc)))))
            (ros-info (achieve plan-lib) "Grasping")
            (try-reference-location pick-up-loc)
            (at-location (pick-up-loc)
              (achieve `(object-picked ,perceived-object)))))))))

(def-goal (achieve (object-put ?obj ?loc))
  (ros-info (achieve plan-lib) "(achieve (object-put))")
  (let ((obj (current-desig ?obj)))
    (assert
     (holds `(object-in-hand ,obj)) ()
     "The object `~a' needs to be in the hand before being able to place it."
     obj)
    (with-designators ((put-down-action
                        :action `((:type :trajectory) (:to :put-down)
                                  (:obj ,obj) (:at ,?loc)))
                       (park-action
                        :action `((:type :trajectory) (:to :park) (:obj ,obj))))
      (with-failure-handling
          (((or manipulation-failed manipulation-pose-unreachable) (f)
             (declare (ignore f))
             ;; Park arm first
             (perform park-action)
             (monitor-action park-action)
             (ros-warn
              (achieve plan-lib)
              "Got unreachable putdown pose.")
             (fail 'manipulation-pose-unreachable)))
        (try-reference-location ?loc)
        (cram-language::on-performing-object-putdown
         obj (reference ?loc))
        (achieve `(looking-at ,(reference ?loc)))
        (perform put-down-action)
        (monitor-action put-down-action))
      (with-retry-counters ((park-retry-counter 5))
        (with-failure-handling
            ((manipulation-failure (f)
               (declare (ignore f))
               (ros-warn (achieve plan-lib) "Unable to park.")
               (do-retry park-retry-counter
                 (ros-warn (achieve plan-lib) "Retrying.")
                 (retry))
               (fail 'manipulation-pose-unreachable)))
          (perform park-action)
          (monitor-action park-action)
          (current-desig ?obj))))))

(def-goal (achieve (object-placed-at ?obj ?loc))
  (ros-info (achieve plan-lib) "(achieve (object-placed-at))")
  (let ((obj (current-desig ?obj)))
    (assert
     (holds `(object-in-hand ,obj)) ()
     "The object `~a' needs to be in the hand before being able to place it."
     obj)
    (with-retry-counters ((goal-pose-retries 3)
                          (manipulation-retries 6))
      (with-failure-handling
          ((manipulation-failure (f)
             (declare (ignore f))
             (ros-warn
              (achieve plan-lib)
              "Got unreachable putdown pose. Trying different put-down location")
             (do-retry goal-pose-retries
               (retry-with-updated-location
                ?loc (next-different-location-solution ?loc)))))
        (with-designators ((put-down-loc :location `((:to :reach) (:location ,?loc))))
          (reset-counter manipulation-retries)
          (with-failure-handling
              ((manipulation-failure (f)
                 (declare (ignore f))
                 (ros-warn
                  (achieve plan-lib)
                  "Got unreachable putdown pose. Trying alternatives.")
                 (do-retry manipulation-retries
                   (retry-with-updated-location
                    put-down-loc
                    (next-different-location-solution put-down-loc)))))
            (try-reference-location put-down-loc)
            (at-location (put-down-loc)
              (achieve `(object-put ,?obj ,?loc)))))))))

(def-goal (achieve (arms-parked))
  (with-designators ((parking :action `((:type :trajectory) (:to :park))))
    (perform parking)
    (monitor-action parking)))

;; (def-goal (achieve (object-flipped (?obj ?tool-1 ?tool-2 ?flipping-parameters)))
;;   (with-retry-counters ((find-object-retry-count 3))
;;     (achieve `(object-in-hand ,?tool-1))
;;     (achieve `(object-in-hand ,?tool-2))
;;     (with-failure-handling
;;         ((object-not-found (f)
;;            (declare (ignore f))
;;            (ros-warn
;;             (achieve plan-lib)
;;             "Failed to perceive the object to flip.")
;;            (do-retry find-object-retry-count
;;              (ros-warn (achieve plan-lib) "Retrying.")
;;              (retry))))
;;       (ros-info (achieve plan-lib) "Perceiving object")
;;       (setf ?obj (perceive-object :a ?obj)))
;;     (with-designators
;;         ((reach-object-location (:location `((:to :reach)
;;                                              (:obj ,?obj)
;;                                             ;; TODO(winkler): Replace
;;                                             ;; these fixed side
;;                                             ;; prameters by the
;;                                             ;; prolog-resolved sides
;;                                             ;; in which ?tool-1 and
;;                                             ;; ?tool-2 are held.
;;                                              (:sides (:left :right)))))
;;          (obj-look-location (:location `((:of ,?obj)))))
;;       (at-location (reach-object-location)
;;         ;; TODO(winkler): Add a variant of `looking-at' that accepts
;;         ;; an object-designator as a parameter
;;         (achieve `(looking-at ,(reference obj-look-location)))
;;         (let* ((parameterization (description ?flipping-parameters))
;;                (new-description `((:to :flip)
;;                                   (:obj ,?obj)
;;                                   (:tools (,?tool-1 ,?tool-2))))
;;                (appended-description
;;                  (append (loop for param in parameterization
;;                                when (not (find param new-description
;;                                                :test (lambda (x y)
;;                                                        (eql (car x)
;;                                                             (car y)))))
;;                                  collect param)
;;                          new-description)))
;;           (with-designators
;;               ((flip-action (:action appended-description)))
;;             (perform flip-action)))))))
