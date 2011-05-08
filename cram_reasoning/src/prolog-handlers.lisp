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

(defvar *break-on-lisp-errors* nil)

(defmacro def-prolog-handler (name (bdgs &rest pattern) &body body)
  `(setf (gethash ',name *prolog-handlers*)
         (lambda (,bdgs ,@pattern)
           ,@body)))

(defun get-prolog-handler (name)
  (gethash name *prolog-handlers*))

(def-prolog-handler and (bdgs &rest pats)
  (prove-all pats bdgs))

(def-prolog-handler or (bdgs &rest pats)
  (lazy-mapcan (lambda (pat)
                 (let ((result (prolog pat bdgs)))
                   result))
               pats))

(def-prolog-handler not (bdgs form)
  (unless (prolog form bdgs)
    (list bdgs)))

(def-prolog-handler -> (bdgs cond if &optional (else '(fail)))
  (let ((new-bdgs (lazy-car (prolog cond bdgs))))
    (if new-bdgs
        (prolog if new-bdgs)
        (prolog else bdgs))))

(def-prolog-handler *-> (bdgs cond if &optional (else '(fail)))
  (let ((new-bdgs (prolog cond bdgs)))
    (if new-bdgs
        (lazy-mapcan (curry #'prolog if) new-bdgs)
        (prolog else bdgs))))

(def-prolog-handler cut (bdgs)
  (signal 'cut-signal)
  (list bdgs))

(def-prolog-handler lisp-fun (bdgs function &rest args)
  (let ((arguments (butlast args))
        (result-pat (car (last args))))
    (block nil
      (handler-bind
          ((error (lambda (e)
                    (unless *break-on-lisp-errors*
                      (warn 'simple-warning
                            :format-control "An error occurred while executing the lisp function `~a': `~a'"
                            :format-arguments (list function e))
                      (return nil)))))
        (let ((result (apply (symbol-function function)
                             (mapcar (rcurry #'var-value bdgs) arguments))))
          ;; (format t "result: ~a ~a bdgs: ~a~%" result result-var bdgs)
          (multiple-value-bind (new-bdgs matched?) (unify result-pat result bdgs)
            (when matched?
              (list new-bdgs))))))))

(def-prolog-handler lisp-pred (bdgs pred &rest args)
  (block nil
    (handler-bind
        ((error (lambda (e)
                  (unless *break-on-lisp-errors*
                    (warn 'simple-warning
                          :format-control "An error occurred while executing the lisp predicate `~a': `~a'"
                          :format-arguments (list pred e))
                    (return nil)))))
      (when (apply (symbol-function pred)
                   (mapcar (rcurry #'var-value bdgs) args))
        (list bdgs)))))

(def-prolog-handler bound (bdgs pattern)
  (when (is-bound pattern bdgs)
    (list bdgs)))

(def-prolog-handler ground (bdgs pattern)
  (when (is-ground pattern bdgs)
    (list bdgs)))

(def-prolog-handler true (bdgs)
  (list bdgs))

(def-prolog-handler fail (bdgs)
  (declare (ignore bdgs))
  nil)

(def-prolog-handler once (bdgs &rest pattern)
  (let ((result (prolog `(and ,@pattern) bdgs)))
    (when result
      (list (lazy-car result)))))

(def-prolog-handler call (bdgs goal)
  (prolog (substitute-vars goal bdgs) bdgs))

(def-prolog-handler findall (bdgs var-pattern pattern result-pattern)
  (let ((result (prolog pattern bdgs)))
    (multiple-value-bind (new-bdgs matched?)
        (unify result-pattern
               (lazy-mapcar (lambda (binding)
                              (substitute-vars var-pattern binding))
                            result)
               bdgs)
      (when matched?
        (list new-bdgs)))))

(def-prolog-handler bagof (bdgs var-pattern pattern result-pattern)
  (let ((result (prolog pattern bdgs)))
    (when result
      (multiple-value-bind (new-bdgs matched?)
          (unify result-pattern
                 (lazy-mapcar (lambda (binding)
                                (substitute-vars var-pattern binding))
                              result)
                 bdgs)
        (when matched?
          (list new-bdgs))))))

(def-prolog-handler setof (bdgs var-pattern pattern result-pattern)
  (let ((result (prolog pattern bdgs)))
    (when result
      (multiple-value-bind (new-bdgs matched?)
          (unify result-pattern
                 (remove-duplicates
                  (loop for binding in (force-ll result)
                        collecting (substitute-vars var-pattern binding))
                  :test #'equal)
                 bdgs)
        (when matched?
          (list new-bdgs))))))

(def-prolog-handler every (bdgs generator pattern)
  ;; proves PATTERN for all possible solutions of GENERATOR. In
  ;; contrast to FORALL and BAGOF, it only succeeds if all free
  ;; variables in PATTERN stay constant over all solutions of
  ;; GENERATOR.
  (labels ((remove-free-vars (free-vars bdgs)
             (remove-if (rcurry #'member free-vars)
                        bdgs :key #'car))
           (merge-bdgs (bdgs-1 bdgs-2)
             (reduce (lambda (bdgs curr-bdg)
                       (destructuring-bind (name . val) curr-bdg
                         (or (add-bdg name val bdgs)
                             (return-from merge-bdgs nil))))
                     bdgs-1 :initial-value bdgs-2))
           (prove-for-sequence (pat free-vars seq bdgs)
             (if seq
                 (let ((merged-bdgs (merge-bdgs (car seq) bdgs)))
                   (when merged-bdgs
                     (lazy-mapcan (curry #'prove-for-sequence
                                         pat free-vars (cdr seq))
                                  (lazy-mapcar (curry #'remove-free-vars
                                                      free-vars)
                                               (prolog pat merged-bdgs)))))
                 (list bdgs))))
    (let* ((generator-solutions (force-ll (prolog generator bdgs)))
           (free-vars (remove-duplicates
                       (mapcan (curry #'mapcar #'car) generator-solutions))))
      (prove-for-sequence pattern free-vars generator-solutions bdgs))))

;; action must succeed for all bindins of cond
(def-prolog-handler forall (bdgs cond action)
  (let ((result (prolog cond bdgs)))
    (block nil
      (force-ll
       (lazy-mapcar (lambda (binding)
                      (unless (prolog action binding)
                        (return nil)))
                    result))
      (list bdgs))))

(def-prolog-handler member (bdgs ?pat ?ll)
  (let ((?ll (var-value ?ll bdgs)))
    (when (lazy-list-p ?ll)
      (lazy-mapcar (lambda (s)
                     (unify ?pat s bdgs))
                   ?ll))))

(def-prolog-handler take (bdgs ?n ?l ?o)
  (let ((?n (var-value ?n bdgs))
        (?l (var-value ?l bdgs))
        (?o (var-value ?o bdgs)))
    (list (unify ?o (lazy-take ?n ?l) bdgs))))

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
