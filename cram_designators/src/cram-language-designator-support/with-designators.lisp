;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :cram-language-designator-support)

(defvar *designator-create-functions* nil
  "List of function objects that return a valid designator object
given a name, a designator class and the designator properties.")

(defun default-designator-create-function (name class properties)
  (declare (ignore name))
  (make-designator class properties))

(defun create-designator (name class properties)
  (or
   (dolist (create-function *designator-create-functions*)
     (let ((new-designator (funcall create-function name class properties)))
       (when new-designator
         (return new-designator))))
   (default-designator-create-function name class properties)))

(cut:define-hook cram-language::on-with-designator (designator))

(defmacro with-designators (&whole sexp defs &body body)
  `(let ((log-params
           ',(mapcar (lambda (def)
                       (destructuring-bind (name type props) def
                         `(,name ,(write-to-string (list type props)))))
                     defs)))
     (with-task-tree-node (:path-part `(goal-context `(with-designators))
                           :name "WITH-DESIGNATORS"
                           :sexp ,sexp
                           :lambda-list nil
                           :parameters nil
                           :log-parameters log-params
                           :log-pattern
                           (list (cons 'definitions ',defs)))
       (let* ,(mapcar (lambda (def)
                        (destructuring-bind (name type props) def
                          `(,name
                            (let ((desig (create-designator
                                          ',name ',type ,props)))
                              (cram-language::on-with-designator desig)
                              desig))))
               defs)
         ,@body))))

(defun register-designator-create-function (function)
  "Registers a new designator create function. `function' needs to be
a function that takes three parameters, the variable name the new
designator will be bound to, the class of the designator and its
properties. The function returns either NIL or a new designator
object. WITH-DESIGNATORS tries all create functions in inverse
definition order until one returns a NON-nil value. Of all functions
return NIL, the designator is constructed using MAKE-DESIGNATOR."
  (setf *designator-create-functions*
        (cons function (remove function *designator-create-functions*))))
