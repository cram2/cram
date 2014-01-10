;;; Copyright (c) 2013, Jan Winkler <winkler@cs.uni-bremen.de>
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
;;;       Universität Bremen nor the names of its contributors 
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

(in-package :cram-plan-library)

(defclass policy ()
  ((name :reader name :initarg :name)
   (parameters :reader parameters :initarg :parameters)
   (check :reader check :initarg :check)
   (recover :reader recover :initarg :recover)
   (clean-up :reader clean-up :initarg :clean-up)))

(defvar *policies* nil "List of defined policies")

(define-condition policy-not-found () ())

(defmacro make-policy (name parameters &rest properties)
  (let ((prop-check (rest (find :check properties
                                 :test (lambda (x y)
                                         (eql x (first y))))))
         (prop-clean-up (rest (find :clean-up properties
                                    :test (lambda (x y)
                                            (eql x (first y))))))
         (prop-recover (rest (find :recover properties
                                   :test (lambda (x y)
                                           (eql x (first y)))))))
     `(make-instance 'policy :name ',name
                             :parameters ',parameters
                             :check (when ',prop-check
                                      (lambda ,parameters ,@prop-check))
                             :recover (when ',prop-recover
                                        (lambda ,parameters ,@prop-recover))
                             :clean-up (when ',prop-clean-up
                                         (lambda ,parameters ,@prop-clean-up)))))

(defmacro define-policy (name parameters &rest properties)
  `(progn
     (setf *policies*
           (remove ',name *policies*
                   :test (lambda (x y)
                           (eql x (name y)))))
     (let ((new-policy (make-policy ,name ,parameters ,@properties)))
       (push new-policy *policies*)
       new-policy)))

(defun named-policy (policy-name)
  (let ((policy (find policy-name *policies*
                      :test (lambda (x y) (eql x (name y))))))
    (cond (policy policy)
          (t (fail 'policy-not-found)))))

(defmacro with-policy (policy policy-parameters &body body)
  (let ((parameters `(parameters ,policy))
        (check `(check ,policy))
        (clean-up `(clean-up ,policy))
        (recover `(recover ,policy)))
    `(unwind-protect
          (pursue
            (when ,check
              (let (,@(mapcar (lambda (var val)
                                `(,var ,val))
                              parameters
                              policy-parameters))
                (loop while (not (funcall ,check ',policy-parameters)))
                (when ,recover
                  (funcall ,recover ',policy-parameters))))
            (progn ,@body))
       (when ,clean-up
         (let (,@(mapcar (lambda (var val)
                           `(,var ,val))
                         parameters
                         policy-parameters))
           (funcall ,clean-up ',policy-parameters))))))
