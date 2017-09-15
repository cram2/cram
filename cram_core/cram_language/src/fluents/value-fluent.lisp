;;;
;;; Copyright (c) 2009, Lorenz Moesenlechner <moesenle@cs.tum.edu>,
;;;                     Nikolaus Demmel <demmeln@cs.tum.edu>
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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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

(in-package :cpl-impl)

(defclass value-fluent (fluent fl-cacheable-value-mixin fl-printable-mixin)
  ((value :initarg :value :initform nil
           :documentation "The value of the fluent")
   (test :initarg :test :initform #'eql
         :documentation "Test to check for value changes (used in setf value).")))

(defmethod print-object ((fluent value-fluent) stream)
  (print-unreadable-object (fluent stream :type nil :identity nil)
    (format stream "~@<FLUENT ~A ~:_[~S]~:>"
            (name fluent)
            (slot-value fluent 'value))))

(defgeneric (setf value) (new-value fluent)
  (:documentation "Setter method to set the new value of the fluent"))

(defmethod value ((fluent value-fluent))
  (with-fluent-locked fluent
    (slot-value fluent 'value)))

(defmethod (setf value) (new-value (fluent value-fluent))
  (let ((need-pulse? nil))
    (with-slots (test value) fluent
      (without-scheduling
        (with-fluent-locked fluent
          (unless (funcall test value new-value)
            (setf (slot-value fluent 'value) new-value)
            (setf need-pulse? t)))
        (when need-pulse?
          (pulse fluent)))))
  new-value)

(defun make-fluent (&rest args &key (class 'value-fluent) name value
                    (allow-tracing t) (max-tracing-freq nil) &allow-other-keys)
  "Use this to create fluents. Default class is VALUE-FLUENT.

   Pass :ALLOW-TRACING NIL to never trace this fluent (default T).

   Pass :MAX-TRACING-FREQ to restrict the frequency with which updates to this
   fluents are traced. The unit is '1/second'. Default is NIL (meaning no
   restriction). "
  (declare (ignore name value))
  (labels ((remove-key-arg (arg-name args)
             (when args
               (cond ((eq (car args) arg-name)
                      (remove-key-arg arg-name (cddr args)))
                     (t
                      (list* (car args) (cadr args)
                             (remove-key-arg arg-name (cddr args)))))))
           (remove-key-args (keys args)
             (reduce (lambda (args key)
                       (remove-key-arg key args))
                     keys
                     :initial-value args)))
    (let ((fluent (apply #'make-instance class (remove-key-args '(:class :allow-tracing
                                                                  :max-tracing-freq)
                                                                args))))
      (on-make-fluent-hook fluent allow-tracing max-tracing-freq)
      fluent)))

;; TODO @demmeln: Add note and testcase about value changes possibly not being
;; picked up by tracing. (lost values)

(define-hook on-make-fluent-hook (fluent allow-tracing max-tracing-freq)
  (:documentation "This is a generic function with hook method
  combination. All defined methods are executed after a fluent is created."))

(defmacro def-cpl-parameter (name &optional (val nil) (doc nil))
  "Defines a global variable (like DEFPARAMETER) that is implemented as a
   fluent. You can access it like a lisp variable with \"NAME\", and also setf
   it like a lisp variable \"(SETF NAME)\" - note there is no need for
   \"(VALUE NAME)\". However the variable being implemented as a fluent means
   that the access is implicitely synchronized with a lock, and all changes
   are traced if fluent tracing is enabled.

   You cannot directly access the implementing fluent which means that you
   cannot use it for building fluent nets or in cpl-constructs like
   WHENEVER. (Just use a normal variable containing a fluent for those
   purposes).

   You can use this for example for defining global datastructures that need
   not participate directly in cpl constructs, but should be traced in the
   execution trace and/or accessed by multiple tasks/threads simultaniously."
  (let ((glob-var (gensym (concatenate 'string "CPL-PARAMETER-"(symbol-name name) "-"))))
    `(progn
       (defparameter ,glob-var (make-fluent :name ',glob-var :value ,val) ,doc)
       (define-symbol-macro ,name (value ,glob-var)))))
