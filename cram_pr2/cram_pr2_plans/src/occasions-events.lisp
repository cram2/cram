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

(in-package :pr2-plans)

(defvar *objects-in-hand* nil
  "An assoc list of objects in hand of structure (arm object-designator grasp). E.g.:
 ((:left (an object (type bottle) (pose ((pose blabla)))) :side)
  (:right (an object (type cup)) :front))")

(defun reset-objects-in-hand ()
  (setf *objects-in-hand* '((:left) (:right))))

(defun reset-beliefstate ()
  (reset-objects-in-hand))

(roslisp-utilities:register-ros-init-function reset-beliefstate)

(defun get-object-in-hand (arm)
  "Returns two values: object designator and grasp."
  (let ((grasp-and-object (cdr (assoc arm *objects-in-hand*))))
    (when grasp-and-object
      (values (first grasp-and-object)
              (second grasp-and-object)))))

(defun set-object-in-hand (arm grasp object-designator)
  (unless (member arm *objects-in-hand* :key #'car)
    (error "arm can only be left or right"))
  (let ((already-in-hand (get-object-in-hand arm)))
    (when already-in-hand
      (warn "object ~a is already in ~a hand" already-in-hand arm))
    (setf (cdr (assoc arm *objects-in-hand*))
          (list object-designator grasp))))

(defun delete-object-in-hand (arm)
  (setf (cdr (assoc arm *objects-in-hand*)) nil))

(def-fact-group pr2-pick-and-place-occasions (cram-plan-occasions-events:object-in-hand)

  (<- (cram-plan-occasions-events:object-in-hand ?object ?side)
    ;; TODO: add ?ROBOT argument to this predicate!
    (bound ?side)
    (not (bound ?object))
    (lisp-fun get-object-in-hand ?side ?object)
    (lisp-pred identity ?object))

  (<- (cram-plan-occasions-events:object-in-hand ?object ?side)
    ;; TODO: add ?ROBOT argument to this predicate!
    (not (bound ?side))
    (not (bound ?object))
    (member ?side (:left :right))
    (lisp-fun get-object-in-hand ?side ?object)
    (lisp-pred identity ?object))

  (<- (cram-plan-occasions-events:object-in-hand ?object ?side)
    ;; TODO: add ?ROBOT argument to this predicate!
    (not (bound ?side))
    (bound ?object)
    (format "WARNING: asking if object-in-hand with bound object designator")
    (fail)))

(defclass object-gripped (cram-occasions-events:event)
  ((arm :initarg :arm
        :reader event-arm
        :initform (error 'simple-error
                         :format-control "OBJECT-GRIPPED event requires ARM."))
   (object :initarg :object
           :reader event-object
           :initform (error 'simple-error
                            :format-control "OBJECT-GRIPPED event requires OBJECT."))
   (grasp :initarg :grasp
          :reader event-grasp
          :initform (error 'simple-error
                           :format-control "OBJECT-GRIPPED event requires GRASP.")))
  (:documentation "Event that is generated whenever the robot successfully
closed a gripper around an object."))

(defclass object-released (cram-occasions-events:event)
  ((arm :initarg :arm
        :reader event-arm
        :initform (error 'simple-error
                         :format-control "OBJECT-GRIPPED event requires OBJECT."))))

(defmethod cram-occasions-events:on-event object-in-hand ((event object-gripped))
  (set-object-in-hand (event-arm event) (event-grasp event) (event-object event))

  (let* ((robot (btr:get-robot-object))
         (link (cut:var-value
                '?ee-link
                (car (prolog:prolog
                      `(and (cram-robot-interfaces:robot ?robot)
                            (cram-robot-interfaces:end-effector-link
                             ?robot ,(event-arm event) ?ee-link))))))
         (object-name (desig:object-identifier (desig:reference (event-object event))))
         (btr-object (btr:object btr:*current-bullet-world* object-name)))
    (when btr-object
      (if (btr:object-attached robot btr-object)
          (btr:attach-object robot btr-object link :loose t)
          (btr:attach-object robot btr-object link :loose nil)))))

(defmethod cram-occasions-events:on-event object-in-hand ((event object-released))
  (delete-object-in-hand (event-arm event))

  (let ((robot (btr:get-robot-object))
        (link (cut:var-value
                '?ee-link
                (car (prolog:prolog
                      `(and (cram-robot-interfaces:robot ?robot)
                            (cram-robot-interfaces:end-effector-link
                             ?robot ,(event-arm event) ?ee-link))))))
        (object (cram-bullet-reasoning-belief-state:get-designator-object (event-object event))))
    (when object
      (btr:detach-object robot object link)
      (btr:simulate btr:*current-bullet-world* 10))))

;; (cram-occasions-events:on-event
;;            (make-instance 'object-gripped
;;              :object ?carried-object
;;              :arm :left-or-right
;;              :grasp :top-of-side-or-front))

;; (prolog:prolog `(object-in-hand ?object :left))
