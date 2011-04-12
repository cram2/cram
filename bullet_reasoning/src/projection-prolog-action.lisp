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

(defvar *prolog-actions* nil
  "List of currently defined prolog actions")

(defclass prolog-action (action)
  ((action-arg-pattern :initarg :action-arg-pattern :reader action-arg-pattern
                       :documentation "The arguments patter of the
                       action. It is required to bind variables for
                       prolog when executing the action.")
   (prolog-event-codes :initform
                       nil :reader prolog-event-codes
                       :documentation "Temporally ordered sequence of
                       events that belong to this action.")))

(defgeneric prolog-action-add-event (action event-pat time)
  (:documentation "States that `prolog-code' is to be exectuted at
  `time' when the action is executed")
  (:method ((prolog-action action) event-pat time)
    (with-slots (prolog-event-codes) prolog-action
      (push (cons event-pat time) prolog-event-codes))))

(defmethod execute-action ((world bt-reasoning-world) (action prolog-action)
                           &rest action-arguments)
  (apply #'execute-action (get-state world) action action-arguments))

(defmethod execute-action ((world-state bt-reasoning-world-state) (action prolog-action)
                           &rest action-arguments)
  (with-slots (prolog-event-codes) action
    (let ((ordered-actions (sort (map 'list #'identity prolog-event-codes)
                                 #'<))
          (world (restore-world-state world-state)))
      (mapcar (lambda (action-def)
                (destructuring-bind (event-pat time)
                    action-def
                  (let ((bdgs
                         (lazy-car
                          (prolog `(event ,event-pat ,world)
                                  (mapcar #'cons
                                          (action-arg-pattern action)
                                          action-arguments)))))
                    (assert bdgs ()
                            "Execution of action ~a failed while executing prolog code for event ~a"
                            (action action) event-pat)
                    (make-instance 'event
                                   :event event-pat
                                   :world-state (get-state world)
                                   :timestamp time))))
              ordered-actions))))

(defmacro def-prolog-projection-rule (name args &body body)
  "Defines a new projection rule. A projection rule consists of a
sequence of events and the corresponding time points when they should
occurr. The events used in these projection rules must be defined via
the EVENT predicate. It has the form: 

  (event ?event-pat ?world)

The body of the projection rule is a sequence with elements of the
form (event ?event-pat ?time) where ?event-pat is a pattern for which
the event predicate is defined and ?time is a form evaluating to the
time after the action has been started when the event occurrs. An
example for a simple projection for the robot base moving is:

 (def-prolog-projection-rule robot-at (?robot ?pose)
   (event (robot-at ?robot ?pose) (calculate-required-time ?robot ?pose)))

"
  (flet ((parse-body-form (form)
           (with-vars-bound (?event ?time)
               (unify form '(event ?event ?time))
             (assert (and ?event ?time) ()
                     "Invalid event pattern ~a in projection rule ~a"
                     form name)
             (values ?event ?time))))
    (assert (every #'is-var args) ()
            "Arguments pattern ~a invalid. Arguments of projection rule ~a must all be variables of the form ?var."
            args name)
    (with-gensyms (action)
      `(eval-when (:load-toplevel :execute)
         (let ((,action (make-instance 'prolog-action
                                       :action ,name
                                       :action-arg-pattern ',args)))
           ,@(mapcar (lambda (form)
                       (multiple-value-bind (?event ?time)
                           (parse-body-form form)
                         `(prolog-action-add-event ,action ',?event ,?time)))
                     body)
           (when (find action *prolog-actions* :key #'action)
              (style-warn "Redefining projection rule for action ~a" name))
           (push ,action (remove ,action *prolog-actions* :key #'action)))))))
