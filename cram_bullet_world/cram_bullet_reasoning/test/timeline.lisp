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

(in-package :bullet-reasoning-tests)

(define-test timeline-correct-last-element
  (let ((timeline (make-instance 'timeline)))
    (timeline-advance timeline (make-instance 'event :timestamp 0.0))
    (assert-eq (btr::events timeline) (btr::last-event timeline))
    (timeline-advance timeline (make-instance 'event :timestamp 1.0))
    (assert-false (eq (btr::events timeline) (btr::last-event timeline)))
    (assert-eq (last (btr::events timeline)) (btr::last-event timeline))))

(define-test timeline-correct-front-insert
  (let ((timeline (make-instance 'timeline))
        (event-1 (make-instance 'event :timestamp 1.0))
        (event-2 (make-instance 'event :timestamp 0.0)))
    (timeline-advance timeline event-1)
    (timeline-advance timeline event-2)
    (assert-eq (first (btr::events timeline)) event-2)
    (assert-eq (second (btr::events timeline)) event-1)))

(define-test timeline-correct-order
  (let* ((timeline (make-instance 'timeline))
         (unordered-timestamps '(0.0 1.0 2.0 1.5 3.0 2.5))
         (ordered-timestamps (sort (mapcar #'identity unordered-timestamps) #'<)))
    (dolist (stamp unordered-timestamps)
      (timeline-advance timeline (make-instance 'event :timestamp stamp)))
    (loop for event in (btr::events timeline)
          for stamp in ordered-timestamps
          do (assert-eq stamp (btr::timestamp event)))))