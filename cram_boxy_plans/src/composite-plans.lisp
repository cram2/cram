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

(defun perform-phases-in-sequence (action-designator)
  (declare (type desig:action-designator action-designator))
  (let ((phases (desig:desig-prop-value action-designator :phases)))
    (mapc (lambda (phase)
            (format t "Executing phase: ~%~a~%~%" phase)
            (exe:perform phase))
          phases)))


(cpl:def-cram-function pick-up (action-designator object arm grasp)
  (perform-phases-in-sequence action-designator)
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-gripped :object object :arm arm :grasp grasp)))


(cpl:def-cram-function place (action-designator arm)
  (perform-phases-in-sequence action-designator)
  (cram-occasions-events:on-event
   (make-instance 'cpoe:object-released :arm arm)))


(defun perceive (?object-designator
                 &key
                   (quantifier :a)
                   (object-chosing-function #'identity))
  (cpl:with-retry-counters ((perceive-retries 5))
    (cpl:with-failure-handling
        ((common-fail:perception-object-not-found (e)
           (cpl:do-retry perceive-retries
             (roslisp:ros-warn (boxy-plans perceive) "~a" e)
             (cpl:retry))))
      (let* ((resulting-designators
               (case quantifier
                 (:all
                  (cram-robosherlock:call-robosherlock-service
                   `((type detecting)
                     (objects ,?object-designator))
                   :quantifier :all)
                  ;; (exe:perform
                  ;;  (desig:a motion
                  ;;           (type detecting)
                  ;;           (objects ?object-designator)))
                  )
                 (t
                  (cram-robosherlock:call-robosherlock-service
                   `((type detecting)
                     (objects ,?object-designator))
                   :quantifier :a)
                  ;; (exe:perform
                  ;;  (desig:a motion
                  ;;           (type detecting)
                  ;;           (object ?object-designator)))
                  )))
             (resulting-designator
               (funcall object-chosing-function resulting-designators)))
        resulting-designator))))
