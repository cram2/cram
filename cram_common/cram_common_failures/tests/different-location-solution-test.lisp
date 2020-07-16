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
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
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

(in-package :cram-common-failures-tests)

(defparameter *target-location* (cl-transforms-stamped:make-pose-stamped
                                 "map"
                                 0.0
                                 (cl-transforms:make-3d-vector 0 1 0)
                                 (cl-transforms:make-quaternion 0 0 0 1)))

(defparameter *test-location* (let ((?target *target-location*))
                                (desig:a location
                                         (reachable-for pr2)
                                         (location (desig:a location
                                                            (pose ?target))))))

(defun setup-world ()
  (setf rob-int:*robot-urdf*
        (cl-urdf:parse-urdf (roslisp:get-param "robot_description")))
  (prolog:prolog
   `(and (btr:bullet-world ?world)
         (rob-int:robot ?robot)
         (assert (btr:object ?world :urdf ?robot ((0 0 0) (0 0 0 1))
                             :urdf ,rob-int:*robot-urdf*))))
  (btr:detach-all-objects (btr:get-robot-object)))


;; roslaunch cram_pr2_pick_place_demo sandbox.launch

(define-test next-different-location-solution-test
  (unless (eq roslisp::*node-status* :RUNNING)
    (roslisp-utilities:startup-ros))
  (setup-world)
  (let ((first-location (desig:reference *test-location*))
        (new-location (desig:reference
                       (common-fail:next-different-location-solution
                        *test-location*))))
    (assert-true (< cram-common-failures::*default-distance-threshold*
                    (cl-transforms:v-dist
                     (cl-transforms:origin first-location)
                     (cl-transforms:origin new-location))))))


(define-test retry-with-diff-loc-designator-test
  (unless (eq roslisp::*node-status* :RUNNING)
    (roslisp-utilities:startup-ros))
  (setup-world)
  (let ((location-output))
    (assert-error 'common-fail:object-nowhere-to-be-found
                  (cpl:with-retry-counters ((robot-location-retries 8))
                    (cpl:with-failure-handling
                        (((or common-fail:navigation-goal-in-collision
                              common-fail:looking-high-level-failure
                              common-fail:perception-low-level-failure) (e)
                           (common-fail:retry-with-loc-designator-solutions
                               *test-location*
                               robot-location-retries
                               (:error-object-or-string e
                                :warning-namespace
                                (fd-plans search-for-object)
                                :rethrow-failure
                                'common-fail:object-nowhere-to-be-found)
                             (push
                              (desig:reference *test-location*)
                              location-output))))
                      (cpl:fail 'common-fail:looking-high-level-failure))))

    (assert-true  (eq 8 (length location-output)))
    (maplist (lambda (pose-list)
               (when (>= (length pose-list) 2)
                 (assert-true
                  (< cram-common-failures::*default-distance-threshold*
                     (cl-transforms:v-dist
                      (cl-transforms:origin (first pose-list))
                      (cl-transforms:origin (second pose-list)))))))
             location-output)))
