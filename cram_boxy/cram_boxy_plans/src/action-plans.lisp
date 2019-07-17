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

;; (cpl:def-cram-function wiggle (left-poses right-poses)
;;   (let (?arm ?target-pose)
;;     (if (car left-poses)
;;         (setf ?arm :left
;;               ?target-pose (car left-poses))
;;         (if (car right-poses)
;;             (setf ?arm :right
;;                   ?target-pose (car right-poses))
;;             (error "pushing action needs a goal for at least one arm.")))

;;     (cpl:with-failure-handling
;;         ((common-fail:low-level-failure (e) ; ignore failures
;;            (roslisp:ros-warn (boxy-plans wiggle) "~a" e)
;;            (return)))

;;       (exe:perform
;;        (desig:a motion
;;                 (type wiggling-tcp)
;;                 (arm ?arm)
;;                 (pose ?target-pose))))))


(cpl:def-cram-function cram-inspect (?object-designator)
  (cpl:with-retry-counters ((perceive-retries 4))
    (cpl:with-failure-handling
        ((common-fail:perception-object-not-found (e)
           (cpl:do-retry perceive-retries
             (roslisp:ros-warn (boxy-plans perceive) "~a" e)
             (cpl:sleep 1.0)
             (cpl:retry))))
      (exe:perform
       (desig:a motion
                (type inspecting)
                (object ?object-designator)))
      ;; (cram-robosherlock:perceive detect-or-inspect object-designator)
      )))

(cpl:def-cram-function look (?left-goal-pose ?right-goal-pose)
  (roslisp:ros-info (boxy-plans look) "Looking with wrist camera")
  (exe:perform (desig:an action
                         (type reaching)
                         (left-poses (?left-goal-pose))
                         (right-poses (?right-goal-pose))))
  (cpl:sleep 1.0)
  (print "slept 1")
  (cpl:sleep 1.0)
  (print "slept 2"))
