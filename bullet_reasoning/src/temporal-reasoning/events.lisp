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

(defgeneric apply-event (world event-pattern &optional timestamp)
  (:documentation "Executes the event that matches `event-pattern'
  `world' and returns the new world instance.")
  (:method ((world bt-reasoning-world) event-pattern
            &optional (timestamp (cut:current-timestamp)))
    (unless (prolog `(event ,event-pattern))
      (error 'simple-error :format-control "Failed to apply event `~a'"
                           :format-arguments (list event-pattern)))
    (make-instance 'event
      :event event-pattern :world-state (get-state *current-bullet-world*)
      :timestamp timestamp)))

(defun make-event-name-from-pat (event-pattern)
  "Returns a symbol representing the `event-pattern'. It constructs a
  symbol by concatenating all symbols in `event-pattern' and
  separating them by a dash."
  (etypecase event-pattern
    (symbol event-pattern)
    (list
       (assert (every #'symbolp event-pattern) ()
               "event-pattern ~a invalid. It must contain only symbols"
               event-pattern)
       (intern
        (reduce (lambda (prev curr)
                  (concatenate 'string prev "-" (symbol-name curr)))
                (cdr event-pattern) :initial-value (symbol-name (car event-pattern)))))))

(defmacro def-event (event-pattern &body body)
  (let ((fact-group-name (intern
                          (concatenate
                           'string "EVENT-"
                           (symbol-name (make-event-name-from-pat event-pattern))))))
    `(def-fact-group ,fact-group-name (event)
       (<- (event ,event-pattern)
         ,@body))))
