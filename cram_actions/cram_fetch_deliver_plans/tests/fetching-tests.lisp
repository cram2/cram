;;;
;;; Copyright (c) 2020, Amar Fayaz <amar@uni-bremen.de>
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

(in-package :fd-plans-tests)

(defparameter *fetch-tests* '(object-can-be-fetched-no-errors-fetch-test
                              object-cannot-be-reached-but-seen-fetch-test
                              object-cannot-be-seen-fetch-test
                              object-reached-on-second-robot-location-fetch-test
                              object-unreachable-with-first-arm-fetch-test
                              object-unreachable-with-only-unreachable-arm-fetch-test
                              object-seen-on-second-search-location-fetch-test
                              navigation-pose-invalid-before-perception-fetch-test
                              perception-pose-valid-fetch-pose-invalid-fetch-test))


(define-test object-can-be-fetched-no-errors-fetch-test
  (init-projection)
  (spawn-object *valid-location-on-island* :bowl)
  (urdf-proj:with-simulated-robot
    (perform (an action
                 (type fetching)
                 (location (a location
                              (poses (*valid-location-on-island*
                                      *valid-location-on-sink-area-surface*))))
                 (object (an object
                             (type bowl)
                             (location (a location
                                          (poses (*valid-location-on-island*
                                                  *valid-location-on-sink-area-surface*))))))
                 (robot-location (a location (poses (*valid-robot-pose-towards-island*)))))))
  (assert-nil (get-total-error-count)))


(define-test object-cannot-be-reached-but-seen-fetch-test
  (init-projection)
  (spawn-object *valid-location-on-island* :bowl)
  (assert-error
   'common-fail:fetching-failed
   (urdf-proj:with-simulated-robot
     (perform (an action
                  (type fetching)
                  (location (a location
                               (poses (*valid-location-on-island*
                                       *valid-location-on-sink-area-surface*))))
                  (object (an object
                              (type bowl)
                              (location (a location
                                           (poses (*valid-location-on-island*
                                                   *valid-location-on-sink-area-surface*))))))
                  (robot-location (a location
                                     (poses (*valid-robot-pose-towards-island-near-wall*))))))))
  (assert-equal 1 (get-error-count-for-error 'common-fail:fetching-failed))
  (assert-equal 3 (get-error-count-for-error 'common-fail:object-unreachable))
  (assert-equal 8 (get-error-count-for-error 'common-fail:manipulation-low-level-failure)))


(define-test object-cannot-be-seen-fetch-test
  (init-projection)
  (spawn-object *valid-location-on-sink-area-surface-near-oven* :bowl)
  (assert-error
   'common-fail:fetching-failed
   (urdf-proj:with-simulated-robot
     (perform (an action
                  (type fetching)
                  (location (a location
                               (poses (*valid-location-on-island*
                                       *valid-location-on-sink-area-surface*))))
                  (object (an object
                              (type bowl)
                              (location (a location
                                           (poses (*valid-location-on-island*
                                                   *valid-location-on-sink-area-surface*))))))
                  (robot-location (a location
                                     (poses (*valid-robot-pose-towards-island-near-wall*))))))))
  (assert-equal 1 (get-error-count-for-error 'common-fail:fetching-failed))
  (assert-equal 2 (get-error-count-for-error 'common-fail:perception-object-not-found)))


(define-test object-reached-on-second-robot-location-fetch-test
  (init-projection)
  (spawn-object *valid-location-on-island* :bowl)
  (urdf-proj:with-simulated-robot
    (perform (an action
                 (type fetching)
                 (location (a location
                              (poses (*valid-location-on-island*
                                      *valid-location-on-sink-area-surface*))))
                 (object (an object
                             (type bowl)
                             (location (a location
                                          (poses (*valid-location-on-island*
                                                  *valid-location-on-sink-area-surface*))))))
                 (robot-location (a location
                                    (poses (*valid-robot-pose-towards-island-near-wall*
                                            *valid-robot-pose-towards-island*)))))))
  (assert-equal 0 (get-error-count-for-error 'common-fail:fetching-failed))
  (assert-equal 3 (get-error-count-for-error 'common-fail:object-unreachable))
  (assert-equal 8 (get-error-count-for-error 'common-fail:manipulation-low-level-failure)))


(define-test object-unreachable-with-first-arm-fetch-test
  (init-projection)
  (spawn-object *valid-location-on-sink-area-surface-near-oven* :bowl)
  (urdf-proj:with-simulated-robot
    (perform (an action
                 (type fetching)
                 (arms (right left))
                 (location (a location
                              (poses (*valid-location-on-sink-area-surface*))))
                 (object (an object
                             (type bowl)
                             (location (a location
                                          (poses (*valid-location-on-sink-area-surface*))))))
                 (robot-location (a location
                                    (poses (*valid-robot-pose-towards-sink-area-surface*)))))))
  (assert-equal 0 (get-error-count-for-error 'common-fail:fetching-failed))
  (assert-equal 1 (get-error-count-for-error 'common-fail:object-unreachable))
  (assert-equal 5 (get-error-count-for-error 'common-fail:manipulation-goal-in-collision)))


(define-test object-unreachable-with-only-unreachable-arm-fetch-test
  (init-projection)
  (spawn-object *valid-location-on-sink-area-surface-near-oven* :bowl)
  (assert-error
   'common-fail:fetching-failed
   (urdf-proj:with-simulated-robot
     (perform (an action
                  (type fetching)
                  (arms (right))
                  (location (a location
                               (poses (*valid-location-on-sink-area-surface*))))
                  (object (an object
                              (type bowl)
                              (location (a location
                                           (poses (*valid-location-on-sink-area-surface*))))))
                  (robot-location (a location
                                     (poses (*valid-robot-pose-towards-sink-area-surface*))))))))
  (assert-equal 1 (get-error-count-for-error 'common-fail:fetching-failed))
  (assert-equal 2 (get-error-count-for-error 'common-fail:object-unreachable))
  (assert-equal 4 (get-error-count-for-error 'common-fail:manipulation-goal-in-collision)))


(define-test object-seen-on-second-search-location-fetch-test
  (init-projection)
  (spawn-object *valid-location-on-sink-area-surface-near-oven* :bowl)
  (urdf-proj:with-simulated-robot
    (perform (an action
                 (type fetching)
                 (arms (left))
                 (location (a location
                              (poses (*valid-location-on-sink-area-surface*
                                      *valid-location-on-island*))))
                 (object (an object
                             (type bowl)
                             (location (a location
                                          (poses (*valid-location-on-sink-area-surface*
                                                  *valid-location-on-island*))))))
                 (robot-location (a location
                                    (poses (*valid-robot-pose-towards-island-near-wall*
                                            *valid-robot-pose-towards-sink-area-surface*)))))))
  ;; Can't find in the first location
  (assert-equal 2 (get-error-count-for-error 'common-fail:perception-object-not-found)))


(define-test navigation-pose-invalid-before-perception-fetch-test
  (init-projection)
  (spawn-object *valid-location-on-sink-area-surface* :bowl)
  (assert-error
   'common-fail:fetching-failed
   (urdf-proj:with-simulated-robot
     (perform (an action
                  (type fetching)
                  (arms (left))
                  (location (a location
                               (poses (*valid-location-on-sink-area-surface*
                                       *valid-location-on-island*))))
                  (object (an object
                              (type bowl)
                              (location (a location
                                           (poses (*valid-location-on-sink-area-surface*
                                                   *valid-location-on-island*))))))
                  (robot-location (a location
                                     (poses (*invalid-robot-pose-towards-sink-area-surface*))))))))
  (assert-equal 1 (get-error-count-for-error 'common-fail:fetching-failed))
  (assert-equal 1 (get-error-count-for-error 'common-fail:navigation-goal-in-collision))
  (assert-equal 1 (get-error-count-for-error 'common-fail:navigation-pose-unreachable)))


(define-test perception-pose-valid-fetch-pose-invalid-fetch-test
  (init-projection)
  (spawn-object *valid-location-on-sink-area-surface* :bowl)
  (assert-error
   'common-fail:fetching-failed
   (urdf-proj:with-simulated-robot
     (perform (an action
                  (type fetching)
                  (arms (left))
                  (location (a location
                               (poses (*valid-location-on-sink-area-surface*
                                       *valid-location-on-island*))))
                  (object (an object
                              (type bowl)
                              (location (a location
                                           (poses (*valid-location-on-sink-area-surface*
                                                   *valid-location-on-island*))))))
                  (robot-location (a location
                                     (poses (*valid-robot-pose-towards-island*
                                             *invalid-robot-pose-towards-sink-area-surface*))))))))
  (assert-equal 1 (get-error-count-for-error 'common-fail:fetching-failed))
  (assert-equal 4 (get-error-count-for-error 'common-fail:manipulation-low-level-failure))
  (assert-equal 2 (get-error-count-for-error 'common-fail:object-unreachable))
  (assert-equal 50 (get-error-count-for-error 'common-fail:navigation-goal-in-collision))
  (assert-equal 50 (get-error-count-for-error 'common-fail:navigation-pose-unreachable)))
