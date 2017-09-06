;;;
;;; Copyright (c) 2016, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :pr2-pp-plans)

(cpl:def-cram-function move-arms-in-sequence (left-poses right-poses)
  "Make `?left-poses' and `?right-poses' to lists if they are not already"

  (flet ((fill-in-with-nils (some-list desired-length)
           (let ((current-length (length some-list)))
             (if (> desired-length current-length)
                 (append some-list (make-list (- desired-length current-length)))
                 some-list))))

    (unless (listp left-poses)
      (setf left-poses (list left-poses)))
    (unless (listp right-poses)
      (setf right-poses (list right-poses)))

    ;; Move arms through all but last poses of `?left-poses' and `?right-poses'
    ;; while ignoring failures: accuracy is not so important in intermediate poses.
    (let ((max-length (max (length left-poses) (length right-poses))))
      (mapc (lambda (?left-pose ?right-pose)

              (cpl:with-failure-handling
                  ((common-fail:manipulation-low-level-failure (e) ; ignore failures
                     (roslisp:ros-warn (boxy-plans move-arms-in-sequence) "~a~%Ignoring." e)
                     (return)))

                (exe:perform
                 (desig:a motion
                          (type moving-tcp)
                          (desig:when ?left-pose
                            (left-target (desig:a location (pose ?left-pose))))
                          (desig:when ?right-pose
                            (right-target (desig:a location (pose ?right-pose))))))))

            (fill-in-with-nils (butlast left-poses) max-length)
            (fill-in-with-nils (butlast right-poses) max-length)))

    ;; Move arm to the last pose of `?left-poses' and `?right-poses'.
    (let ((?left-pose (car (last left-poses)))
          (?right-pose (car (last right-poses))))

      (cpl:with-failure-handling
          ((common-fail:manipulation-low-level-failure (e)
             ;; propagate failures up
             ;; (roslisp:ros-error (boxy-plans move-arms-in-sequence) "~a~%Failing." e)
             (roslisp:ros-warn (pick-and-place reach) "~a~%Ignoring." e)
             (return)
             ))

        (exe:perform
         (desig:a motion
                  (type moving-tcp)
                  (desig:when ?left-pose
                    (left-target (desig:a location (pose ?left-pose))))
                  (desig:when ?right-pose
                    (right-target (desig:a location (pose ?right-pose))))))))))


(cpl:def-cram-function release (?left-or-right)
  (cpl:with-failure-handling
      ((common-fail:low-level-failure (e) ; ignore failures
         (roslisp:ros-warn (pick-and-place open-gripper) "~a" e)
         (return)))
    (exe:perform
     (desig:a motion
              (type opening)
              (gripper ?left-or-right)))))


(cpl:def-cram-function grip (?left-or-right ?effort)
  (cpl:with-retry-counters ((grasping-retries 1))
    (cpl:with-failure-handling
        ((common-fail:low-level-failure (e) ; regrasp once then propagate up
           (cpl:do-retry grasping-retries
             (roslisp:ros-warn (pick-and-place grip) "~a" e)
             (cpl:retry))
           (cpl:fail 'common-fail:low-level-failure)))
      (exe:perform
         (desig:a motion
                  (type gripping)
                  (gripper ?left-or-right)
                  (effort ?effort))))))

(cpl:def-cram-function close-gripper (?left-or-right)
  (cpl:with-failure-handling
      ((common-fail:low-level-failure (e) ; ignore failures
         (roslisp:ros-warn (pick-and-place grip) "~a" e)
         (return)))
    (exe:perform
     (desig:a motion
              (type closing)
              (gripper ?left-or-right)))))

(cpl:def-cram-function set-gripper-to-position (?left-or-right ?position)
  (cpl:with-failure-handling
      ((common-fail:low-level-failure (e) ; ignore failures
         (roslisp:ros-warn (pick-and-place grip) "~a" e)
         (return)))
    (exe:perform
     (desig:a motion
              (type moving-gripper-joint)
              (gripper ?left-or-right)
              (joint-angle ?position)))))

(cpl:def-cram-function look-at (object-designator)
  (let ((?pose (cram-object-interfaces:get-object-pose object-designator)))
    (exe:perform
     (desig:a motion
              (type looking)
              (target (desig:a location (pose ?pose)))))))

(cpl:def-cram-function navigate (?location-designator)
  (cpl:with-retry-counters ((nav-retries 2))
    (cpl:with-failure-handling
        ((common-fail:navigation-low-level-failure (e)
           (roslisp:ros-warn (pick-and-place go)
                             "Some low-level failure happened: ~a"
                             e)
           (cpl:do-retry nav-retries
             (roslisp:ros-warn (pick-and-place go) "Retrying...")
             (cpl:retry))))
      (exe:perform
       (desig:a motion (type going) (target ?location-designator))))))
