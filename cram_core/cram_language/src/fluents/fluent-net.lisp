;;;
;;; Copyright (c) 2009, Lorenz Moesenlechner <moesenle@cs.tum.edu>
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

(defclass fl-net-fluent (fluent)
  ((calculate-value-fun :initarg :calculate-value-fun
                        :documentation "The function object to
                        calculate the value of the fluent."))
  (:documentation "Class of fluent networks that directly depend on
  fluents that are not cachable."))

(defclass fl-net-cacheable-fluent (fluent fl-cacheable-value-mixin fl-printable-mixin)
  ((calculate-value-fun :initarg :calculate-value-fun)
   (value :initarg :value :documentation "The cached value of this
   fluent network.")))

(defmethod value ((fluent fl-net-fluent))
  ;; Invoke the function for every value request.
  (funcall (slot-value fluent 'calculate-value-fun)))

(defgeneric update-value (fluent)
  (:documentation "Re-calculates the value of the fluent. This method
  returns two values, the new value and T if the value actually
  changed.")
  (:method ((fluent fl-net-cacheable-fluent))
    (with-slots (value calculate-value-fun) fluent
      (let ((new-value (funcall calculate-value-fun)))
        (with-fluent-locked fluent
          (let ((value-changed (not (and (slot-boundp fluent 'value)
                                         (eql new-value value)))))
            (setf (slot-value fluent 'value) new-value)
            (values new-value value-changed)))))))

(defmethod value ((fluent fl-net-cacheable-fluent))
  (with-fluent-locked fluent
    (when (slot-boundp fluent 'value)
      (return-from value (slot-value fluent 'value))))
  (nth-value 0 (update-value fluent)))

(defmethod pulse :around ((fluent fl-net-cacheable-fluent))
  (when (nth-value 1 (update-value fluent))
    (call-next-method)))

(defmacro def-fluent-operator (&whole w name args &body body)
  "def-fluent-operator allows to define fluent operators. It creates a
   new function with the given name. When it is called with a fluent
   in its parameter list, a fluent network is returned, with the
   function body as its calculator. Otherwise, the body is called
   directly.

   We need one special case. Some operators need to always pulse the
   returned fluent net. This is solved by havin a second value. When
   the body returns a non-nil second value, the fluent net is always
   pulsed, otherwise only when the fluent net value has changed."
  (multiple-value-bind (body decls docstring)
      (parse-body body :documentation t :whole w)
    (with-gensyms (fl-args)
      `(defun ,name (&rest ,fl-args)
         ,docstring
         (labels ((,name ,args ,@decls ,@body))
           (let ((fluents (remove-if-not (of-type 'fluent) ,fl-args)))
             (if fluents
                 (call-as-fluent-operator #',name ,fl-args
                                          :fluents fluents
                                          :name ',name)
                 (apply #',name ,fl-args))))))))

(defun call-as-fluent-operator (function args
                                &key (fluents (required-argument))
                                     (name    (required-argument))
                                     (force-no-cache nil))
  "The meat of DEF-FLUENT-OPERATOR."
  (flet ((make-fluent-callback (weak-result-fluent fl-name)
           "Returns the update callback that is called whenever a
            dependecy changes its value."
           (lambda (value)
             (declare (ignore value))
             (without-scheduling
               (let ((result-fluent (tg:weak-pointer-value weak-result-fluent)))
                 (cond (result-fluent
                        (pulse result-fluent))
                       (t
                        ;; Do garbage-collection when reference to
                        ;; fluent is no longer valid.
                        (dolist (fluent fluents)
                          (remove-update-callback fluent fl-name)))))))))
    (let* ((fl-name (format-gensym "FN-~A" name))
           (cachable (and (not force-no-cache)
                          (every (rcurry #'typep 'fl-cacheable-value-mixin)
                                 fluents)))
           (result-fluent (make-fluent
                           :class (if cachable
                                      'fl-net-cacheable-fluent
                                      'fl-net-fluent)
                           :name fl-name
                           :calculate-value-fun (lambda ()
                                                  (apply function args))))
           (weak-result-fluent (tg:make-weak-pointer result-fluent)))
      (without-scheduling
        (dolist (fluent fluents)
          (register-update-callback
           fluent fl-name
           (make-fluent-callback weak-result-fluent fl-name))))
      (when cachable
        ;; calculate the cached value to get pulses correctly. The
        ;; initial calculation of the cached value should not trigger
        ;; a pulse.
        (update-value result-fluent))
      result-fluent)))

;;; FIXME: allow arbitrary arglist, use PARSE-ORDINARY-ARGLIST,
;;; and generate call template from that.
(defmacro define-fluent-net-wrapper (name wrapped-fn)
  `(def-fluent-operator ,name (&rest args)
     (apply #',wrapped-fn (mapcar #'value args))))

(define-fluent-net-wrapper fl< <)

(define-fluent-net-wrapper fl> >)

(define-fluent-net-wrapper fl= =)

(define-fluent-net-wrapper fl-eq eq)

(define-fluent-net-wrapper fl-eql eql)

(define-fluent-net-wrapper fl-member member)

(define-fluent-net-wrapper fl+ +)

(define-fluent-net-wrapper fl- -)

(define-fluent-net-wrapper fl* *)

(define-fluent-net-wrapper fl/ /)

(define-fluent-net-wrapper fl-not not)

;;; AND and OR cannot be implemented as macros for fluent. All
;;; previous operators return whether a fluent or the value, depending
;;; on whether at least one fluent is in the arguments or not. That
;;; means, variables must be expanded before deciding which return
;;; value to use, which requires expansion of values. Thus, AND and OR
;;; have to behave slightly different for fluents and we name them
;;; differently.

(def-fluent-operator fl-and (&rest args)
  "The and-operator for fluents. It is fundamentally different to the
   definition of common-lisp's and in that it is not implemented as a
   macro. That means, all args are evaluated when using fl-and."
  (cond ((null args)
         t)
        ((cdr args)
         (and (value (car args))
              (apply #'fl-and (cdr args))))
        (t
         (value (car args)))))

(def-fluent-operator fl-or (&rest args)
  "The or-operator for fluents. For more information on why it is a
   function please refere to the documentation of fl-and."
  (when args
    (or (value (car args))
        (apply #'fl-or (cdr args)))))

(defun fl-value-changed (fluent &key (test #'eql) (key #'identity))
  "Returns a fluent that gets pulsed and has the value T whenever
  `fluent' changes its value."
  (check-type fluent fluent)
  (let ((old-value (funcall key (value fluent))))
    (call-as-fluent-operator (lambda ()
                               (cond ((funcall test (funcall key (value fluent)) old-value)
                                      nil)
                                     (t
                                      (unless *peek-value*
                                        (setf old-value (value fluent)))
                                      t)))
                             ()
                             :fluents (list fluent)
                             :name 'fl-updated
                             :force-no-cache t)))

(def-fluent-operator fl-funcall (fun &rest args)
  "Generic fluent-operator. Applys args to function whenever a
   fluent in args changes."
  (apply fun (mapcar #'value args)))

;;; FIXME: implement this in a non-sucky way.

(defun fl-apply (fun arg &rest more-args)
  "FIXME"
  (let* ((args (cons arg more-args))
         (spread-list (lastcar more-args))
         (fluents (append (remove-if-not (of-type 'fluent) args)
                          (remove-if-not (of-type 'fluent) spread-list))))
    (call-as-fluent-operator #'(lambda (&rest args)
                                 (let ((xs (butlast args))
                                       (list (lastcar args)))
                                   (apply fun (append (mapcar #'value xs)
                                                      (list (mapcar #'value list))))))
                           args :fluents fluents :name 'fl-apply)))

