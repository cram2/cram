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

(def-cram-function navigate (goal)
  (ros-info (achieve plan-lib) "Driving to location ~a." goal)
  (unless (reference goal)
    (fail "Location designator invalid."))
  (when (> (distance-to-drive goal) 1.5)
    (ros-info (achieve plan-lib) "Distance to drive: ~a, parking arms.~%"
              (distance-to-drive goal))
    (achieve `(looking-at :forward))
    (achieve `(arms-parked)))
  (with-designators ((navigation-action :action `((:type :navigation) (:goal ,goal))))
    (perform navigation-action)
    (monitor-action navigation-action)))

(def-goal (achieve (loc Robot ?loc))
  (navigate ?loc))

(def-goal (achieve (loc ?obj ?loc))
  (ros-info (achieve plan-lib) "(achieve (loc ?obj ?loc)")
  (ros-info (achieve plan-lib) "?obj `~a' ?loc `~a'" (description ?obj) (description ?loc))
  (unless (perceive-state `(loc ,?obj ,?loc))
    (let ((retry-count 0))
      (with-failure-handling
          ((object-lost (f)
             (declare (ignore f))
             (when (< (incf retry-count) 3)
               (retry))))
        (with-failure-handling
            ((manipulation-pose-unreachable (f)
               (fail 'manipulation-pickup-failed :result (result f))))
          (achieve `(object-in-hand ,?obj)))
        (with-failure-handling
            ((manipulation-pose-unreachable (f)
               (fail 'manipulation-pose-occupied :result (result f))))
          (achieve `(object-placed-at ,?obj ,?loc)))))))
