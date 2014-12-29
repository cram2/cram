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

(defun matching-goal-fun (name args &optional (goal-funs (get name :goals)))
  (when goal-funs
    (multiple-value-bind (bdgs ok?) (cut:pat-match (caar goal-funs) args)
      (if ok?
          (values (car goal-funs) bdgs)
          (matching-goal-fun name args (cdr goal-funs))))))

(defun make-goal-fun (name pattern declarations body)
  (with-gensyms (bdgs-var block-name)
    (let* ((body `(with-tags ,@body))
           (goal-fun `(lambda (,bdgs-var)
                        (with-task-tree-node
                            (:path-part `(goal (,',name ,@',pattern))
                             :name ,(format nil "GOAL-~a" name)
                             :sexp (,name ,pattern ,body)
                             :log-parameters
                             (list (list 'name ',name)
                                   (list 'pattern
                                         ,(write-to-string
                                           (car pattern)))
                                   (list 'declarations
                                         ,(write-to-string declarations)))
                             :log-pattern
                             (list (cons 'name ',name)
                                   (cons 'pattern ',pattern)
                                   (cons 'declarations ',declarations)
                                   (cons 'body ',body)
                                   (cons
                                    'parameters
                                    (mapcar (lambda (var)
                                              (cons var (cut:var-value var ,bdgs-var)))
                                            ',(cut:vars-in pattern))))
                             :lambda-list ,(cut:vars-in pattern)
                             :parameters (mapcar (alexandria:rcurry
                                                  #'cut:var-value ,bdgs-var)
                                                 ',(cut:vars-in pattern)))
                          ,@declarations
                          (block ,block-name
                            (flet ((succeed (&rest result)
                                     (return-from ,block-name
                                       (apply #'values result))))
                              ,body))))))
      goal-fun)))

(defun register-goal (name pattern goal-fun &optional doc-string)
  (when (assoc pattern (get name :goals) :test #'equal)
    #+sbcl (sb-int:style-warn "Redefining goal `~a'" `(,name ,@pattern)))
  (cond ((not (get name :goals))
         (setf (get name :goals)
               (list (list pattern goal-fun doc-string))))
        (t
         (setf (get-alist pattern (get name :goals) :test #'equal)
               (list goal-fun doc-string)))))

(defun describe-goal (goal)
  (destructuring-bind (name &rest pattern) goal
    (third
     (find pattern (get name :goals)
           :key #'car :test #'patterns-eq))))

(defun call-goal (name args)
  (multiple-value-bind (def bdgs) (matching-goal-fun name args)
    (unless def
      (error 'simple-error
             :format-control "No goal found that matches arguments: ~a ~a"
             :format-arguments (list name args)))
    (funcall (second def) bdgs)))

(defmacro declare-goal (name lambda-list &body body)
  "Declares the goal `name'. Before matching goals defined with
`def-goal' are executed, `body' is executed with `pattern' lexically
bound to the parameter specified when the goal was called. `body' is
executed in an unnamed block that allows for suppressing the execution
of any goal. The idea behind that is that `body' can verify if the
goal already holds and suppress execution of any code."
  (multiple-value-bind (forms declarations doc-string)
      (parse-body body :documentation t)
    (with-gensyms (pattern)
      `(progn
         (setf (get ',name :goals) nil)
         (defun ,name (&rest ,pattern)
           ,doc-string
           (block nil
             (flet ((before ,lambda-list
                      ,@declarations
                      ,@forms))
               (apply #'before ,pattern))
             (call-goal ',name ,pattern)))))))

(defmacro def-goal ((name &rest pattern) &body body)
  "
Defines a new goal. Goals always have the form

\(<name> [pattern]*)

where patterns can be arbitrary expressions containing
variables (indicated by a ?-prefix) and symbols.
Example: (achieve (loc ?obj ?location)) When defining goals with
similar expressions (e.g. (loc Robot ?l) and (loc ?a ?b)) the most
specific expression must be defined first. Otherwise, the previously
defined expression will be overwritten.

In the lexical environment of `body', the function (SUCCEED [values])
is defined which performs a non-local exit, returning `values' as
return values.
"
  (multiple-value-bind (forms declarations doc-string)
      (parse-body body :documentation t)
    `(let ((goal-fun ,(make-goal-fun name pattern declarations forms)))
       (register-goal ',name ',pattern goal-fun ,doc-string))))
