;;;
;;; Copyright (c) 2021, Amar Fayaz <amar@uni-bremen.de>
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

(in-package :urdf-proj-tests)

;; run (roslisp-utilities:startup-ros)

(define-test navigation-prospection-test
  (fd-plans-tests::init-projection)
  (let ((initial-robot-pose)
        (robot-pose-after-fetching)
        (robot-pose-after-prospection))
    
    (urdf-proj:with-simulated-robot
      (setf initial-robot-pose (btr:pose (btr:object btr:*current-bullet-world* :PR2)))
      (urdf-proj:with-prospection
        (perform (an action
                     (type navigating)
                     (location
                      (a location
                         (poses (fd-plans-tests::*valid-robot-pose-towards-sink-area-surface*))))))

        (setf robot-pose-after-fetching (btr:pose (btr:object btr:*current-bullet-world* :PR2))))
      
      (setf robot-pose-after-prospection
            (btr:pose (btr:object btr:*current-bullet-world* :PR2))))

    ;; Pose changed after navigating
    (assert-false (btr-tests::pose-equal initial-robot-pose robot-pose-after-fetching))

    ;; Pose returned to initial values after prospection
    (assert-true (btr-tests::pose-equal initial-robot-pose robot-pose-after-prospection))))


(define-test navigation-prospection-error-test
  (fd-plans-tests::init-projection)
  (let ((initial-robot-pose)
        (robot-pose-after-prospection))

    ;; Error should be bubbled up from the prospection
    (assert-error 'common-fail:high-level-failure 
     (urdf-proj:with-simulated-robot
      (setf initial-robot-pose (btr:pose (btr:object btr:*current-bullet-world* :PR2)))
      (urdf-proj:with-prospection
        (perform (an action
                     (type navigating)
                     (location
                      (a location
                         (poses
                          (fd-plans-tests::*invalid-robot-pose-towards-sink-area-surface*)))))))))

      
    (setf robot-pose-after-prospection (btr:pose (btr:object btr:*current-bullet-world* :PR2)))

    ;; Pose returned to initial values after prospection
    (assert-true (btr-tests::pose-equal initial-robot-pose robot-pose-after-prospection))))


(define-test fetching-prospection-test
  (fd-plans-tests::init-projection)
  (fd-plans-tests::spawn-object fd-plans-tests::*valid-location-on-island* :bowl)
  (let ((initial-bowl-pose)
        (initial-robot-pose)
        (bowl-pose-after-fetching)
        (robot-pose-after-fetching)
        (bowl-pose-after-prospection)
        (robot-pose-after-prospection))
    
    (urdf-proj:with-simulated-robot
      (setf initial-bowl-pose (btr:pose (btr:object btr:*current-bullet-world* :bowl-1)))
      (setf initial-robot-pose (btr:pose (btr:object btr:*current-bullet-world* :PR2)))
      (urdf-proj:with-prospection
        (perform (an action
                     (type fetching)
                     (location
                      (a location
                         (poses (fd-plans-tests::*valid-location-on-island*))))
                     (object
                      (an object
                          (type bowl)
                          (location (a location
                                       (poses (fd-plans-tests::*valid-location-on-island*))))))
                     (robot-location
                      (a location
                         (poses (fd-plans-tests::*valid-robot-pose-towards-island*))))))

        (setf bowl-pose-after-fetching (btr:pose (btr:object btr:*current-bullet-world* :bowl-1))) 
        (setf robot-pose-after-fetching (btr:pose (btr:object btr:*current-bullet-world* :PR2))))
      
      (setf bowl-pose-after-prospection
            (btr:pose (btr:object btr:*current-bullet-world* :bowl-1))) 
      (setf robot-pose-after-prospection
            (btr:pose (btr:object btr:*current-bullet-world* :PR2))))

    ;; Pose changed after fetching
    (assert-false (btr-tests::pose-equal initial-bowl-pose bowl-pose-after-fetching))
    (assert-false (btr-tests::pose-equal initial-robot-pose robot-pose-after-fetching))

    ;; Pose returned to initial values after prospection
    (assert-true (btr-tests::pose-equal initial-bowl-pose bowl-pose-after-prospection))
    (assert-true (btr-tests::pose-equal initial-robot-pose robot-pose-after-prospection))))


