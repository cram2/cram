;;;
;;; Copyright (c) 2009 - 2010
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

(in-package #:cram-execution-trace)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Utilities for dealing with timestamps and durations
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;; TODO@demmeln: Comments and docstrings for this crazy stuff

(defparameter +equal-time-precision+ 1d-3)

(defun equal-time (t1 t2)
  (< (abs (- t1 t2)) +equal-time-precision+))

(defun count-equal-times (time insts)
  (if (or (endp insts) (not (equal-time time (timestamp (car insts)))))
      0
      (+ 1 (count-equal-times time (cdr insts)))))

(defun round-to-precision (d p)
  (declare (type timestamp d)
           (type integer p))
  (let ((f (float (expt 10 p) d)))
    (/ (round (* f d)) f)))

(defun order-of-magnitude (d)
  (declare (type timestamp d))
  (assert (< d 1))
  (ceiling (- (log d 10))))

(defun changes->durations (changes max-time)
  (maplist (lambda (x)
             (cons (caar x)
                   `(throughout ,(cdar x)
                                ,(if (cdr x)
                                     (cdadr x)
                                     max-time))))
           changes))

(defun calculate-max-time (execution-trace trace-list-accessor zero-time)
  (let ((max zero-time))
    (maphash-values (lambda (value)
                      (setf max (reduce #'max (funcall trace-list-accessor value)
                                        :key #'timestamp
                                        :initial-value max)))
                    execution-trace)
    ;; Make sure the max time is strictly greater than all fluent changes
    (round-to-precision (float (+ 0.1 max) max)
                        (order-of-magnitude +equal-time-precision+))))

(defun calculate-task-list (task-tree)
  (flatten-task-tree task-tree))

(defun calculate-goal-task-list (task-list)
  (remove-if-not #'goal-task-tree-node-p task-list))

(defun calc-delta (count)
  (if (< count 10)
      (* 0.1 +equal-time-precision+)
      (* 0.1 (calc-delta (truncate count 10)))))

(defun delta-precision (d)
  (+ 1 (order-of-magnitude d)))

(defun calculate-fluent-changes (traced-instances)
  ;; We trust the fluent traces are in cronological order.
  (let ((last-time -1)
        (delta 0)
        (precision (order-of-magnitude +equal-time-precision+))
        (current 0))
    (maplist (lambda (rest)
               (let ((value (traced-value (car rest)))
                     (time (timestamp (car rest))))
                 (if (equal-time last-time time)
                     (incf current delta)
                     (let ((count (1+ (count-equal-times time (cdr rest)))))
                       (if (= 0 count)
                           (setf last-time -1
                                 delta 0
                                 precision (order-of-magnitude +equal-time-precision+)
                                 current 0)
                           (setf last-time time
                                 delta (calc-delta count)
                                 precision (delta-precision delta)
                                 current 0))))
                 (cons value (round-to-precision (+ time current) precision))))
             traced-instances)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Object Identities
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;; NOTE 30.04.2010 @demmeln: Object Identities unused as of now. Maybe remove?

(defvar *next-identity* 1 "Keeps trac of next free identity for *object-id-hash-table*.")

(defvar *object-id-hash-table*
  (tg:make-weak-hash-table :weakness :key :test 'cl::eq)
  "Keeps trac of object identities (eq objects have the same identity).
   Its a weak hash-table so objects can be gc'd.")

(defun get-next-free-id ()
  "Get next useable identity for the *object-id-hash-table*."
  (prog1
      *next-identity*
    (incf *next-identity*)))

(defun get-eq-id (obj)
  "Returns an identity (integer) that is same for two objects if and only if they are eq."
  (multiple-value-bind (id found?)
      (gethash obj *object-id-hash-table*)
    (if found?
        id
        (setf (gethash obj *object-id-hash-table*)
              (get-next-free-id)))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Durable copying
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defgeneric durable-copy (obj)
  (:documentation "Returns a copy of itself which will not change in the
  future. This is used specifically for traceing the value of fluents (as a
  generic deep-copy is impossible). For mutable objects it should be a deep
  copy. The default is just returning obj and not copying anything."))

(defmethod durable-copy (obj)
  "Default behaviour is to assume immutability and just return the object as
   is."
  obj)

(defmethod durable-copy ((obj list))
  "For cons cells assume they construct a list and use copy list."
  (copy-list obj))
