;;;
;;; Copyright (c) 2009, Lorenz Moesenlechner <moesenle@cs.tum.edu>
;;;                     Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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
;;;     * Neither the name of Technical University of Munich or
;;;       University of Bremen nor the names of its
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

(in-package :cut)

(define-method-combination hooks (&key (hook-combination #'list))
  ((methods *))
  "Every method should be qualified by a symbol. All matching methods
   are executed and the results are combined using `hook-combination'
   which returns the final result of the method. The
   `hook-combination' function gets every method result as a
   parameter."
  `(funcall ,hook-combination
            ,@(loop for m in methods
                    collecting `(call-method ,m))))

(define-method-combination ordered-hooks (&key (hook-combination #'list))
  ((methods *))
  "Every method should be qualified by a symbol or two symbols.
If there are two symbols, the second one has to be a positive integer,
according to which the function calls are ordered, lower value means earlier evaluation:
 (defmethod my-generic-function some-qualifier 1 (some-arg) ...)
will be called before
 (defmethod my-generic-function some-qualifier 2 (some-arg) ...)
If there is only one symbol or if second symbol is not a positive integer,
the order of function calls is assumed irrelevant.
If both numbered and not numbered methods are present,
first the numbered ones are executed in the corresponding order,
then the rest of the functions are being called.
The results are combined using `hook-combination', which returns the final result of the method.
The `hook-combination' function gets every method result as a parameter."
  (let* ((ordered-methods
           (sort
            (remove-if-not (lambda (method)
                             (let ((method-qualifiers (method-qualifiers method)))
                               (and (= (length method-qualifiers) 2)
                                    (typep (second method-qualifiers) '(integer 0 *)))))
                           methods)
            #'<
            :key (lambda (method)
                   (second (method-qualifiers method)))))
         (other-methods
           (set-difference methods ordered-methods)))
    `(funcall ,hook-combination
              ,@(loop for m in (append ordered-methods other-methods)
                      collecting `(call-method ,m)))))

(define-method-combination first-in-order-and-around ()
  ((methods *))
  "Every method should be qualified by two symbols or :around.
The second one has to be a positive integer. The function will lowest integer will be called:
 (defmethod my-generic-function some-qualifier 1 (some-arg) ...)
will be called and not
 (defmethod my-generic-function some-qualifier 2 (some-arg) ...)
If there are multiple methods with the same order number, the most specific one will be called.
Standard :around method combination with around methods is also supported."
  (let* ((ordered-methods
           (stable-sort
            (remove-if-not (lambda (method)
                             (let ((method-qualifiers (method-qualifiers method)))
                               (and (= (length method-qualifiers) 2)
                                    (typep (second method-qualifiers) '(integer 0 *)))))
                           methods)
            #'<
            :key (lambda (method)
                   (second (method-qualifiers method)))))
         (method-to-call
           (car ordered-methods))
         (around-methods
           (remove-if-not (lambda (method)
                            (let ((method-qualifiers (method-qualifiers method)))
                              (eq (car method-qualifiers) :around)))
                          methods)))
    (if around-methods
        `(call-method ,(first around-methods)
                      (,@(rest around-methods)
                       ,method-to-call))
        `(call-method ,method-to-call))))


(defmacro define-hook (fun-name lambda-list &body options)
  "Wrapper around DEFGENERIC. If `options' does not include a
   :METHOD-COMBINATION clause, the HOOKS method combination is used. Also if
   the defined function is called an no methods are applicable, NIL is
   returned instead of an error condition being signaled."
  (let ((method-combination-clause (or (assoc :method-combination options)
                                       '(:method-combination hooks)))
        (options (remove :method-combination options :key #'car)))
    `(progn (defgeneric ,fun-name ,lambda-list
              ,method-combination-clause
              ,@options)
            (defmethod no-applicable-method ((generic-function (EQL #',fun-name)) &rest args)
              (declare (ignore args generic-function))
              nil))))


(defgeneric copy-object (obj)
  (:documentation "Creates a (deep-) copy of OBJ.")
  (:method ((var t))
    var)
  (:method ((var structure-object))
    (copy-structure var))
  (:method ((obj standard-object))
    (let ((new (allocate-instance (class-of obj))))
      (mapcar #'(lambda (slot)
                  (let ((slot-name (sb-mop:slot-definition-name slot)))
                    (when (slot-boundp obj slot-name)
                      (setf (slot-value new slot-name)
                            (copy-object (slot-value obj slot-name))))))
              (sb-mop:class-slots (class-of obj)))
      new)))
