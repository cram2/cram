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

(defclass event ()
  ((event :initarg :event :reader event
          :documentation "The event that is represented by this
          class")
   (world-state :initarg :world-state :reader event-world-state
                :documentation "The world state after the event occurred")
   (timestamp :initarg :timestamp :reader timestamp
              :documentation "The time stamp indicating when the event
              occurred")))

(defclass action ()
  ((action :initarg :action :reader action)))

(defgeneric execute-action (world action &rest action-arguments)
  (:documentation "Retunrs the sequence of events that are generated
  by exectuing `action' in `world'"))

(defclass timeline ()
  ((events :initform nil :reader events
           :documentation "The (temporally ordered) sequence of events
           that belong to this timeline")
   (last-event :initform nil :reader last-event
               :documentation "A reference to the last event on the
               timeline. It represents the current state of the
               world.")))

(defgeneric timeline-advance (timeline event)
  (:documentation "Advances the `timeline', i.e. adds the event `event' to its end")
  (:method ((timeline timeline) (event event))
    (flet ((copy-and-advance-event (event time)
             (with-slots (event world-state timestamp) event
               (make-instance
                'event
                :event event
                :world-state world-state
                :timestamp (+ time timestamp)))))
      (with-slots (events last-event) timeline
        (let ((new-entry (cons (copy-and-advance-event
                                event (timestamp (car last-event)))
                               nil)))
          (if last-event
              (setf (cdr last-event) new-entry)
              (setf events new-entry))
          (setf last-event new-entry))))))

(defgeneric timeline-lookup (timeline stamp)
  (:documentation "Returns the world state that correspinds to `stamp'
  in `timeline'")
  (:method ((timeline timeline) (stamp number))
    (reduce (lambda (prev curr)
              (when (< stamp (timestamp curr))
                (return-from timeline-lookup prev))
              curr)
            (events timeline))))

(defgeneric timeline-execute-action (timeline action)
  (:documentation "Executes an action on `timeline', i.e. advances the
  timeline with `action'")
  (:method ((timeline timeline) (action action))
    (map 'nil (curry #'timeline-advance timeline)
         (execute-action action))))
