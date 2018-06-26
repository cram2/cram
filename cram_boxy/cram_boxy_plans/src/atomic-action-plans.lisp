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

(in-package :boxy-plans)

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
      (format t "~%~%GOT POSES: ~a~%" left-poses)
      (format t "~%BUT LAST: ~a~%" (fill-in-with-nils (butlast left-poses) max-length))
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


(cpl:def-cram-function wiggle (left-poses right-poses)
  (let (?arm ?target-pose)
    (if (car left-poses)
        (setf ?arm :left
              ?target-pose (car left-poses))
        (if (car right-poses)
            (setf ?arm :right
                  ?target-pose (car right-poses))
            (error "pushing action needs a goal for at least one arm.")))

    (cpl:with-failure-handling
        ((common-fail:low-level-failure (e) ; ignore failures
           (roslisp:ros-warn (boxy-plans wiggle) "~a" e)
           (return)))

      (exe:perform
       (desig:a motion
                (type wiggling-tcp)
                (arm ?arm)
                (target (desig:a location (pose ?target-pose))))))))


(cpl:def-cram-function release (?left-or-right)
  (cpl:with-failure-handling
      ((common-fail:low-level-failure (e) ; ignore failures
         (roslisp:ros-warn (boxy-plans open-gripper) "~a" e)
         (return)))
    (exe:perform
     (desig:a motion
              (type opening)
              (gripper ?left-or-right)))))


(cpl:def-cram-function grip (?left-or-right &optional ?effort)
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
                  (desig:when ?effort (effort ?effort)))))))


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

(defun perceive (?detecting-or-inspecting ?object-designator)
  "DETECT is an operator that stands on the same level as PERFORM. It's not a standard action."
  (cpl:with-retry-counters ((perceive-retries 5))
    (cpl:with-failure-handling
        ((common-fail:perception-object-not-found (e)
           (cpl:do-retry perceive-retries
             (roslisp:ros-warn (boxy-plans perceive) "~a" e)
             (cpl:sleep 1.0)
             (cpl:retry))))
      (exe:perform
       (desig:a motion
                (type ?detecting-or-inspecting)
                (object ?object-designator)))
      ;; (cram-robosherlock:perceive detect-or-inspect object-designator)
      )))
