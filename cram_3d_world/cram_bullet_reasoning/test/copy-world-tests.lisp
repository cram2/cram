;;; Copyright (c) 2014, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :bullet-reasoning-tests)

(define-test test-copy-world-for-objects-not-in-collision
  (handler-case
      (let ((urdf (cl-urdf:parse-urdf (roslisp:get-param "robot_description")))
            (first-world (copy-world *current-bullet-world*)))
        (with-current-bullet-world first-world
          (assert-false
           (cut:lazy-car
            (prolog:prolog `(and
                          (clear-bullet-world)
                          (bullet-world ?w)
                          (assert (object ?w static-plane floor ((0 0 0) (0 0 0 1))
                                          :normal (0 0 1) :constant 0))
                          (robot ?robot)
                          (assert (object ?w urdf ?robot ((0 0 -0.5) (0 0 0 1)) :urdf ,urdf))
                          ;; the next predicate should fail: robot collides with floor
                          (robot-not-in-collision-with-environment ?w ?robot)))))
          (let ((second-world (copy-world *current-bullet-world*)))
            (with-current-bullet-world second-world
              (assert-true
               (cut:lazy-car
                (prolog:prolog `(and
                              (clear-bullet-world)
                              (bullet-world ?w)
                              (assert (object ?w static-plane floor
                                              ((0 0 0) (0 0 0 1))
                                              :normal (0 0 1) :constant 0
                                              :no-robot-collision t))
                              (robot ?robot)
                              (assert (object ?w urdf ?robot ((0 0 -0.5) (0 0 0 1))
                                              :urdf ,urdf))
                              ;; the next predicate should not fail anymore
                              (robot-not-in-collision-with-environment ?w ?robot)))))))
          (assert-false
           (cut:lazy-car
            (prolog:prolog `(and
                          (bullet-world ?w)
                          (robot ?robot)
                          ;; the next predicate should fail again
                          (robot-not-in-collision-with-environment ?w ?robot)))))))
    (error () (format t "~%You probably don't have the robot_description_lowres
parameter on your ROS parameter server~%~%"))))
