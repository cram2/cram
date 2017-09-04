;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :pr2-cloud)

(defun test-circle ()
  (move-in-projection-to-fridge)
  (cram-process-modules:with-process-modules-running
      (pr2-pms::pr2-perception-pm pr2-pms::pr2-base-pm pr2-pms::pr2-arms-pm
                                  pr2-pms::pr2-grippers-pm pr2-pms::pr2-ptu-pm)
    (cpl:top-level
      (mapcar (lambda (transform)
                (let ((?pose (strip-transform-stamped transform)))
                 (exe:perform
                  (desig:a motion
                           (type moving-tcp)
                           (right-target (desig:a location (pose ?pose)))))))
              (local-gripper-trajectory-in-base-from-radius)))))

(defun test-gripper ()
  (cram-process-modules:with-process-modules-running
      (pr2-pms::pr2-perception-pm pr2-pms::pr2-base-pm pr2-pms::pr2-arms-pm
                                  pr2-pms::pr2-grippers-pm pr2-pms::pr2-ptu-pm)
    (cpl:top-level
      (exe:perform
       (desig:a motion (type opening) (gripper left))))))

(defun test-navigation ()
  (cram-process-modules:with-process-modules-running
      (pr2-pms::pr2-perception-pm pr2-pms::pr2-base-pm pr2-pms::pr2-arms-pm
                                  pr2-pms::pr2-grippers-pm pr2-pms::pr2-ptu-pm)
    (cpl:top-level
      (exe:perform
       (let ((?pose (cl-transforms-stamped:make-pose-stamped
                     "map" 0.0
                     (cl-transforms:make-3d-vector 0.5 -0.8 0)
                     (cl-transforms:make-identity-rotation))))
         (desig:a motion (type going) (target (desig:a location (pose ?pose)))))))))

(defun test-manipulation ()
  (cram-process-modules:with-process-modules-running
      (pr2-pms::pr2-perception-pm pr2-pms::pr2-base-pm pr2-pms::pr2-arms-pm
                                  pr2-pms::pr2-grippers-pm pr2-pms::pr2-ptu-pm)
    (cpl:top-level
      (exe:perform
       (let ((?pose (strip-transform-stamped
                     (car (subseq (local-gripper-trajectory-in-base "MoveFridgeHandle") 23)))))
         (desig:a motion (type moving-tcp) (left-target (desig:a location (pose ?pose)))))))))

(defun execute-trajectory-in-real-world ()
  (cram-process-modules:with-process-modules-running
      (pr2-pms::pr2-perception-pm pr2-pms::pr2-base-pm pr2-pms::pr2-arms-pm
                                  pr2-pms::pr2-grippers-pm pr2-pms::pr2-ptu-pm)
    (cpl:top-level
      (exe:perform
       (desig:an action (type closing) (gripper right)))

      (mapcar (lambda (transform)
                (cpl:with-failure-handling
                    ((common-fail:actionlib-action-timed-out (e)
                       (format t "Action timed out: ~a~%Ignoring...~%" e)
                       (return)))
                  (exe:perform
                   (let ((?pose (strip-transform-stamped transform)))
                     (desig:a motion
                              (type moving-tcp)
                              (right-target (desig:a location (pose ?pose))))))))
              (filter-trajectory-of-big-rotations
               (subseq (local-gripper-trajectory-in-base "MoveFridgeHandle") 23)
               0.1)))))

;;; arms down
;; (exe:perform (desig:an action
;;                             (to move-arm-motion)
;;                             (left ((0.09611d0 0.68d0 0.35466d0)
;;                                    (-0.45742778331019085d0
;;                                     0.3060123951483878d0
;;                                     0.3788581151804847d0
;;                                     0.744031427853262d0)))
;;                             (right ((0.0848d0 -0.712d0 0.35541d0)
;;                                     (-0.061062529688043946d0
;;                                      -0.6133522138254498d0
;;                                      0.197733462359113d0
;;                                      -0.7622151317882601d0)))))
