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

(defparameter *delivering-tests* '(object-can-be-delivered-no-errors-deliver-test
                                   object-delivery-location-unreachable-deliver-test
                                   delivery-attempt-with-wrong-arm-deliver-test
                                   object-delivery-location-unreachable-from-2-robot-locations-deliver-test
                                   object-reachable-on-2nd-robot-location-deliver-test
                                   object-reachable-on-2nd-target-location-deliver-test
                                   first-deliver-location-designator-error-deliver-test))

(define-test object-can-be-delivered-no-errors-deliver-test
  (init-projection)
  (spawn-object *valid-location-on-island* :bowl)
  ;; Deterministic Fetch
  (let ((?fetched-object))
    (urdf-proj:with-simulated-robot
      (setf ?fetched-object
            (perform (an action
                              (type fetching)
                              (location (a location
                                           (poses (*valid-location-on-island*))))
                              (object (an object
                                          (type bowl)
                                          (location (a location
                                                       (poses
                                                        (*valid-location-on-island*))))))
                              (robot-location (a location
                                                 (poses
                                                  (*valid-robot-pose-towards-island*))))))))
    (sleep 0.5)
    (urdf-proj:with-simulated-robot
      (perform (an action
                   (type delivering)
                   (target (a location
                              (poses (*valid-location-on-sink-area-surface*))))
                   (object ?fetched-object)
                   (robot-location (a location
                                      (poses
                                       (*valid-robot-pose-towards-sink-area-surface*))))))))
  (assert-nil (get-total-error-count)))

(define-test object-delivery-location-unreachable-deliver-test
  (init-projection)
  (spawn-object *valid-location-on-island* :bowl)
  ;; Deterministic Fetch
  (let ((?fetched-object))
    (urdf-proj:with-simulated-robot
      (setf ?fetched-object
            (perform (an action
                              (type fetching)
                              (location (a location
                                           (poses (*valid-location-on-island*))))
                              (object (an object
                                          (type bowl)
                                          (location (a location
                                                       (poses
                                                        (*valid-location-on-island*))))))
                              (robot-location (a location
                                                 (poses
                                                  (*valid-robot-pose-towards-island*))))))))
    (sleep 0.5)
    (assert-nil (get-total-error-count))
    (assert-error
     'common-fail:delivering-failed
     (urdf-proj:with-simulated-robot
      (perform (an action
                   (type delivering)
                   (target (a location
                              (poses (*valid-location-on-sink-area-surface*))))
                   (object ?fetched-object)
                   (robot-location (a location
                                      (poses
                                       (*valid-robot-pose-towards-island*)))))))))
  (assert-equal 1 (get-error-count-for-error 'common-fail:delivering-failed))
  (assert-equal 2 (get-error-count-for-error 'common-fail:object-undeliverable))
  (assert-equal 1 (get-error-count-for-error 'common-fail:manipulation-low-level-failure))
  (assert-equal 1 (get-error-count-for-error 'common-fail:object-unreachable)))


(define-test delivery-attempt-with-wrong-arm-deliver-test
  (init-projection)
  (spawn-object *valid-location-on-island* :bowl)
  ;; Deterministic Fetch
  (let ((?fetched-object))
    (urdf-proj:with-simulated-robot
      (setf ?fetched-object
            (perform (an action
                              (type fetching)
                              (location (a location
                                           (poses (*valid-location-on-island*))))
                              (arm (right))
                              (object (an object
                                          (type bowl)
                                          (location (a location
                                                       (poses
                                                        (*valid-location-on-island*))))))
                              (robot-location (a location
                                                 (poses
                                                  (*valid-robot-pose-towards-island*))))))))
    (sleep 0.5)
    (assert-error
     'common-fail:delivering-failed
     (urdf-proj:with-simulated-robot
      (perform (an action
                   (type delivering)
                   (target (a location
                              (poses (*valid-location-on-sink-area-surface*))))
                   (arm (left))
                   (object ?fetched-object)
                   (robot-location (a location
                                      (poses
                                       (*valid-robot-pose-towards-sink-area-surface*)))))))))
  (assert-equal 1 (get-error-count-for-error 'common-fail:delivering-failed))
  (assert-equal 2 (get-error-count-for-error 'common-fail:object-undeliverable))
  (assert-equal 1 (get-error-count-for-error 'common-fail:object-unreachable)))


(define-test object-delivery-location-unreachable-from-2-robot-locations-deliver-test
  (init-projection)
  (spawn-object *valid-location-on-island* :bowl)
  ;; Deterministic Fetch
  (let ((?fetched-object))
    (urdf-proj:with-simulated-robot
      (setf ?fetched-object
            (perform (an action
                              (type fetching)
                              (location (a location
                                           (poses (*valid-location-on-island*))))
                              (object (an object
                                          (type bowl)
                                          (location (a location
                                                       (poses
                                                        (*valid-location-on-island*))))))
                              (robot-location (a location
                                                 (poses
                                                  (*valid-robot-pose-towards-island*))))))))
    (sleep 0.5)
    (assert-error
     'common-fail:delivering-failed
     (urdf-proj:with-simulated-robot
      (perform (an action
                   (type delivering)
                   (target (a location
                              (poses (*valid-location-on-sink-area-surface*))))
                   (object ?fetched-object)
                   (robot-location (a location
                                      (poses
                                       (*valid-robot-pose-towards-island*
                                        *valid-robot-pose-towards-island-near-wall*)))))))))
  (assert-equal 1 (get-error-count-for-error 'common-fail:delivering-failed))
  (assert-equal 12 (get-error-count-for-error 'common-fail:object-undeliverable))
  (assert-equal 21 (get-error-count-for-error 'common-fail:manipulation-low-level-failure))
  (assert-equal 21 (get-error-count-for-error 'common-fail:object-unreachable)))


(define-test object-reachable-on-2nd-robot-location-deliver-test
  (init-projection)
  (spawn-object *valid-location-on-island* :bowl)
  ;; Deterministic Fetch
  (let ((?fetched-object))
    (urdf-proj:with-simulated-robot
      (setf ?fetched-object
            (perform (an action
                              (type fetching)
                              (location (a location
                                           (poses (*valid-location-on-island*))))
                              (object (an object
                                          (type bowl)
                                          (location (a location
                                                       (poses
                                                        (*valid-location-on-island*))))))
                              (robot-location (a location
                                                 (poses
                                                  (*valid-robot-pose-towards-island*))))))))
    (sleep 0.5)
    (urdf-proj:with-simulated-robot
      (perform (an action
                   (type delivering)
                   (target (a location
                              (poses (*valid-location-on-sink-area-surface*))))
                   (object ?fetched-object)
                   (robot-location (a location
                                      (poses
                                       (*valid-robot-pose-towards-island*
                                        *valid-robot-pose-towards-sink-area-surface*))))))))
  (assert-equal 0 (get-error-count-for-error 'common-fail:delivering-failed))
  (assert-equal 1 (get-error-count-for-error 'common-fail:object-undeliverable))
  (assert-equal 1 (get-error-count-for-error 'common-fail:manipulation-low-level-failure))
  (assert-equal 1 (get-error-count-for-error 'common-fail:object-unreachable)))


(define-test object-reachable-on-2nd-target-location-deliver-test
  (init-projection)
  (spawn-object *valid-location-on-island* :bowl)
  ;; Deterministic Fetch
  (let ((?fetched-object))
    (urdf-proj:with-simulated-robot
      (setf ?fetched-object
            (perform (an action
                              (type fetching)
                              (location (a location
                                           (poses (*valid-location-on-island*))))
                              (object (an object
                                          (type bowl)
                                          (location (a location
                                                       (poses
                                                        (*valid-location-on-island*))))))
                              (robot-location (a location
                                                 (poses
                                                  (*valid-robot-pose-towards-island*))))))))
    (sleep 0.5)
    (urdf-proj:with-simulated-robot
      (perform (an action
                   (type delivering)
                   (target (a location
                              (poses (*valid-location-on-sink-area-surface*
                                      *valid-location-on-island*))))
                   (object ?fetched-object)
                   (robot-location (a location
                                      (poses
                                       (*valid-robot-pose-towards-island*
                                        ))))))))
  (assert-equal 0 (get-error-count-for-error 'common-fail:delivering-failed))
  (assert-equal 0 (get-error-count-for-error 'common-fail:object-undeliverable))
  (assert-equal 1 (get-error-count-for-error 'common-fail:manipulation-low-level-failure))
  (assert-equal 0 (get-error-count-for-error 'common-fail:object-unreachable)))

(define-test first-deliver-location-designator-error-deliver-test
  (init-projection)
  (spawn-object *valid-location-on-island* :bowl)
  ;; Deterministic Fetch
  (let ((?fetched-object))
    (urdf-proj:with-simulated-robot
      (setf ?fetched-object
            (perform (an action
                         (type fetching)
                         (location (a location
                                      (poses (*valid-location-on-island*))))
                         (object (an object
                                     (type bowl)
                                     (location (a location
                                                  (poses
                                                   (*valid-location-on-island*))))))
                         (robot-location (a location
                                            (poses
                                             (*valid-robot-pose-towards-island*))))))))
    (sleep 0.5)
    (assert-nil (get-total-error-count))
    (urdf-proj:with-simulated-robot
      (perform (an action
                   (type delivering)
                   (target (a location
                              (poses (*invalid-location-outside-map*
                                      *valid-location-on-island*))))
                   (object ?fetched-object)))))
  (assert-equal 0 (get-error-count-for-error 'common-fail:delivering-failed))
  (assert-equal 0 (get-error-count-for-error 'common-fail:object-undeliverable))
  (assert-equal 0 (get-error-count-for-error 'common-fail:manipulation-low-level-failure))
  (assert-equal 0 (get-error-count-for-error 'common-fail:object-unreachable)))
