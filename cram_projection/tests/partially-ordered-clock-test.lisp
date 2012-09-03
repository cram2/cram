;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :cram-projection-tests)

(define-test partially-ordered-clock.clock-wait-increments-correctly
  (let ((clock (make-instance 'partially-ordered-clock))
        (duration 10))
    (let ((start-time (clock-time clock)))
      (clock-wait clock duration)
      (assert-equality #'< start-time (clock-time clock))
      (assert-equality #'>= (clock-time clock) (+ start-time duration)))))

(define-test partially-ordered-clock.correct-parallel-order
  ;; Tests if parallel waits are really performed in parallel. Please
  ;; not that the parameter :INCREMENT-DELAY highly influences the
  ;; outcome of this test. It is set to 0.05 which is pretty
  ;; optimistic and if the test fails, we should increase it.
  (let ((clock (make-instance 'partially-ordered-clock
                 :initial-time 0 :increment-delay 0.05))
        (duration-1 5)
        (duration-2 10)
        (termination-time-1 nil)
        (termination-time-2 nil))
    (cpl:top-level
      (cpl:par
        (cpl:seq
          (clock-wait clock duration-1)
          (setf termination-time-1 (clock-time clock)))
        (cpl:seq
          (clock-wait clock duration-2)
          (setf termination-time-2 (clock-time clock)))))
    (assert-equality #'<= duration-1 termination-time-1)
    (assert-equality #'<= duration-2 termination-time-2)))

(define-test partially-ordered-clock.with-clock-disabled
  (let ((clock (make-instance 'partially-ordered-clock
                 :initial-time 0 :increment-delay 0.05))
        (duration-1 5)
        (duration-2 10)
        (termination-time-1 nil)
        (termination-time-2 nil)
        (termination-real-time-1 nil)
        (termination-real-time-2 nil)
        (sleep-terminated-time nil))
    (cpl:top-level
      (cpl:par
        (cpl:seq
          (clock-wait clock duration-1)
          (setf termination-time-1 (clock-time clock))
          (setf termination-real-time-1 (get-internal-real-time)))
        (cpl:seq
          (clock-wait clock duration-2)
          (setf termination-time-2 (clock-time clock))
          (setf termination-real-time-2 (get-internal-real-time)))
        (with-partially-ordered-clock-disabled clock
          (cpl:sleep 0.1)
          (setf sleep-terminated-time (get-internal-real-time)))))
    (assert-equality #'<= duration-1 termination-time-1)
    (assert-equality #'<= duration-2 termination-time-2)
    (assert-equality #'>= termination-real-time-1 sleep-terminated-time)
    (assert-equality #'>= termination-real-time-2 sleep-terminated-time)))
