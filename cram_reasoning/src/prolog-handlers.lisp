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

(in-package :crs)

(defvar *prolog-handlers* (make-hash-table :test 'eq))

(defmacro def-prolog-handler (name (bdgs &rest pattern) &body body)
  `(setf (gethash ',name *prolog-handlers*)
         (lambda (,bdgs ,@pattern)
           ,@body)))

(defun get-prolog-handler (name)
  (gethash name *prolog-handlers*))

(def-prolog-handler and (bdgs &rest pats)
  (labels ((do-and (clauses bdgs)
             (if clauses
                 (lazy-mapcan (lambda (goal-1-bdgs)
                                (do-and (cdr clauses) goal-1-bdgs))
                              (prolog (car clauses) bdgs))
                 (list bdgs))))
    (do-and pats bdgs)))

(def-prolog-handler or (bdgs &rest pats)
  (lazy-mapcan (lambda (pat)
                 (let ((result (prolog pat bdgs)))
                   result))
               pats))

(def-prolog-handler not (bdgs form)
  (unless (prolog form bdgs)
    (list bdgs)))

(def-prolog-handler -> (bdgs cond if &optional (else '(fail)))
  (let ((new-bdgs (prolog cond bdgs)))
    (if new-bdgs
        (lazy-mapcan (lambda (bdg) (prolog if bdg)) new-bdgs)
        (prolog else bdgs))))

(def-prolog-handler lisp-fun (bdgs function &rest args)
  (let ((arguments (butlast args))
        (result-pat (car (last args))))
    (handler-case
        (let ((result (apply (symbol-function function)
                             (mapcar (rcurry #'var-value bdgs) arguments))))
          ;; (format t "result: ~a ~a bdgs: ~a~%" result result-var bdgs)
          (multiple-value-bind (new-bdgs matched?) (unify result-pat result bdgs)
            (when matched?
              (list new-bdgs))))
      (error (e)
        (warn 'simple-warning
              :format-control "An error occurred while executing the lisp function `~a': `~a'"
              :format-arguments (list function e))
        nil))))

(def-prolog-handler lisp-pred (bdgs pred &rest args)
  (handler-case
      (when (apply (symbol-function pred)
                   (mapcar (rcurry #'var-value bdgs) args))
        (list bdgs))
    (error (e)
      (warn 'simple-warning
            :format-control "An error occurred while executing the lisp predicate `~a': `~a'"
            :format-arguments (list pred e))
      nil)))

(def-prolog-handler bound (bdgs pattern)
  (when (is-bound pattern bdgs)
    (list bdgs)))

(def-prolog-handler ground (bdgs pattern)
  (when (is-ground pattern bdgs)
    (list bdgs)))

(def-prolog-handler fail (bdgs)
  (declare (ignore bdgs))
  nil)

(def-prolog-handler once (bdgs pattern)
  (let ((result (prolog pattern bdgs)))
    (when result
      (list (lazy-car result)))))

(def-prolog-handler findall (bdgs var-pattern pattern result-pattern)
  (let ((result (prolog pattern bdgs)))
    (multiple-value-bind (new-bdgs matched?)
        (unify result-pattern (loop for binding in (force-ll result)
                                 for bound-vars = (substitute-vars var-pattern binding)
                                 when (is-ground bound-vars nil)
                                 collect bound-vars)
               bdgs)
      (when matched?
        (list new-bdgs)))))

(def-prolog-handler bagof (bdgs var-pattern pattern result-pattern)
  (let ((result (prolog pattern bdgs)))
    (when result
      (multiple-value-bind (new-bdgs matched?)
          (unify result-pattern (loop for binding in (force-ll result)
                                      for bound-vars = (substitute-vars var-pattern binding)
                                      when (is-ground bound-vars nil)
                                        collect bound-vars)
                 bdgs)
        (when matched?
          (list new-bdgs))))))

;; action must succeed for all bindins of cond
(def-prolog-handler forall (bdgs cond action)
  (let ((result (prolog cond bdgs)))
    (loop for binding in (force-ll result) ;; could be improved by doing this laszily
       when (not (prolog action binding))
       do (return nil)
       finally (return (list bdgs)))))

;;; Pass a form containing vars (in arbitrary nesting) as the first
;;; parameter. Then pass an arbitray number of goals (which will be
;;; "anded"). FILTER-BINDINGS will return only bindings to the variables
;;; contained in `vars'. This can be used e.g. for complicated interactive
;;; queries to restrict the output to only what it wanted.

;;; FIXME: Could it happen, that some vars are bound to other variables, or to
;;; values that are not ground? There might be some interesting conflicts if
;;; those are not all gensymed. Investigate. Example:
;;;
;;; (force-ll (prolog '(and (filter-bindings ?x
;;;                          (== ?x ?y))
;;;                     (== ?y 23)
;;;                     (== ?x 42))))
;;;
;;; Should this fail or succeed? At the moment it fails. For it to succeed, we
;;; would probably need to rename the variable ?y to a gensym, or in the
;;; general case rename all appearing variabels, that are not already gensyms.
(def-prolog-handler filter-bindings (bdgs vars &rest goals)
  (lazy-mapcar (rcurry (curry #'filter-bindings vars) nil)
               (prove-all goals bdgs)))
