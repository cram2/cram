;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;

(in-package :btr)

(defclass timeline ()
  ((events :initform nil :reader events
           :documentation "The (temporally ordered) sequence of events
           that belong to this timeline")
   (last-event :initform nil :reader last-event
               :documentation "A reference to the last event on the
               timeline. It represents the current state of the
               world.")
   (lock :initform (sb-thread:make-mutex) :reader timeline-lock)))

(defgeneric timeline-init (world &optional time)
  (:documentation "Creates and initializes a new timeline with a
  specific world state")
  (:method ((world bt-reasoning-world) &optional (time 0.0))
    (let ((timeline (make-instance 'timeline)))
      (timeline-advance
       timeline
       (make-instance 'event
                      :event 'start
                      :world-state (get-state world)
                      :timestamp time)))))

(defgeneric timeline-advance (timeline event)
  (:documentation "Advances the `timeline', i.e. adds the event `event' to its end")
  (:method ((timeline timeline) (event event))
    (with-slots (events last-event lock) timeline
      (sb-thread:with-mutex (lock)
        (cond ((and last-event (> (timestamp (car last-event)) (timestamp event)))
               (setf events (insert-before-if
                             event
                             (lambda (timeline-event)
                               (> (timestamp timeline-event)
                                  (timestamp event)))
                             events :count 1)))
              (t (let ((new-entry (cons event nil)))
                   (if last-event
                       (setf (cdr last-event) new-entry)
                       (setf events new-entry))
                   (setf last-event new-entry))))))
    timeline))

(defgeneric timeline-current-world-state (timeline)
  (:documentation "Returns the world state object of the last event on the timeline")
  (:method ((timeline timeline))
    (with-slots (last-event lock) timeline
      (sb-thread:with-mutex (lock)
        (when (car last-event)
          (event-world-state (car last-event)))))))

(defgeneric timeline-lookup (timeline stamp)
  (:documentation "Returns the world state that correspinds to `stamp'
  in `timeline'")
  (:method ((timeline timeline) (stamp number))
    (with-slots (events lock) timeline
      (sb-thread:with-mutex (lock)
        (reduce (lambda (prev curr)
                  (when (> (timestamp curr) stamp)
                    (return-from timeline-lookup prev))
                  (event-world-state curr))
                (cdr events)
                :initial-value (event-world-state (car events)))))))

(defvar *current-timeline* (timeline-init *current-bullet-world*))
