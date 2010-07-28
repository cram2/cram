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

(in-package :crs)

;;; NOTE: Unify cannot deal with infinite bindigns. For example evaluating
;;; (var-value '?x (unify '?x '(1 . ?x))) will cause a stack overflow. Now
;;; occurs check happens.

;;; NOTE: #demmeln: Unify cannot cope with some uses of segvars. Avoid using
;;; it in prolog/unify (it warns when segvars are used). Also some helper
;;; functions (filter-bindings, rename-vars, substitue-vars, var-value,
;;; add-bdg, with-vars-bound, with-pat-vars-bound, vars-in, is-bound,
;;; is-ground etc) don't implement segvars correctly at the moment.

(defun unify (lhs rhs &optional bdgs)
  "Unifiy two forms."
  (let ((lhs (substitute-vars lhs bdgs))
        (rhs (substitute-vars rhs bdgs)))
    (cond ((or (is-segvar lhs) (is-segvar rhs))
           (error "Found segvar that is not in car of a cons-cell while unifying ~a and ~a"
                  lhs rhs))
          ((or (and (atom lhs) (atom rhs)
                    (eql lhs rhs))
               (is-unnamed-var lhs)
               (is-unnamed-var rhs))
           (values bdgs t))
          ((is-var lhs)
           (add-bdg lhs rhs bdgs))
          ((is-var rhs)
           (add-bdg rhs lhs bdgs))
          ((is-segform lhs)
           (warn "Using segvars in unify... This is not implementet properly at the moment.")
           (match-segvar lhs rhs bdgs #'unify))
          ((is-segform rhs)
           (warn "Using segvars in unify... This is not implementet properly at the moment.")
           (match-segvar rhs lhs bdgs #'unify))
          ((and (consp lhs) (consp rhs))
           (multiple-value-bind (new-bdgs matched?)
               (unify (car lhs) (car rhs) bdgs)
             (if matched?
                 (unify (cdr lhs) (cdr rhs) new-bdgs)
                 (values nil nil))))
          (t (values nil nil)))))

(declaim (inline unify-p))
(defun unify-p (lhs rhs &optional bdgs)
  "Returns T if `lhs' and `rhs' unify."
  (nth-value 1 (unify lhs rhs bdgs)))

(defun prove-one (goal binds)
  (let ((handler (get-prolog-handler (car goal))))
    (if handler
        (apply handler binds (cdr goal))
        (lazy-mapcan (lambda (clause-match)
                       (prove-all (fact-clauses (car clause-match))
                                  (cdr clause-match)))
                     (get-matching-clauses goal binds)))))

(defun prove-all (goals binds)
  (cond ((null goals)
         (list binds))
        (t
         (lazy-mapcan (lambda (goal-1-binds)
                        (prove-all (cdr goals) goal-1-binds))
                      (prove-one (car goals) binds)))))

(defun get-matching-clauses (query binds)
  (let ((list-of-facts (get-predicate-facts (car query))))
    (unless list-of-facts
      (warn "Trying to proove goal ~a with undefined functor ~s." query (car query)))
    (loop for fact-def in list-of-facts
       if (unify-p (fact-head fact-def) query binds)
       collect (let* ((renamed-fact (rename-vars fact-def))
                      (new-binds (unify (fact-head renamed-fact) query binds)))
                 (cons renamed-fact new-binds)))))

(defun filter-bindings (form bdgs &optional initial-bdgs)
  "Assume that `bdgs' is a superset of `initial-bdgs'. Now extend
   `initial-bdgs' by those bindings from `bdgs', that appear in `form'. The
   effect of this is filtering out all new bindings of variables that are not
   in `form'. Nothing will be removed, that is already in `initial-bdgs'."
  ;; Filter every new binding except the ones of variables in form or of
  ;; variables that already appear in initial-bdgs. Including all vars
  ;; appearing in initial-bdgs is neccessary: Suppose a variable ?X is
  ;; appearing in form and ?X is bound to ?Y in initial-bdgs, with ?Y still
  ;; unbound in initial-bdgs. Now assume further, that ?Y gets bound during a
  ;; prolog query and is now bound in bdgs. This means that also ?X is
  ;; implicitly bound in bdgs. We are not interested in what ?Y is bound to,
  ;; but if we dont add this new binding of ?Y, ?X will also lose its new
  ;; binding, and that we are interested in.
  (%filter-bindings (vars-in (cons form initial-bdgs)) bdgs initial-bdgs))

(defun filter-add-bdg (var val bdgs)
  "This is to be used by %FILTER-BINDINGS. We trust there will be no conflicts
   between new and old bindings, and if a binding to `var' is already there we
   just leave it as is."
  (let ((old-bdg (assoc var bdgs)))
    (if old-bdg
        (values bdgs t) ; it is essential here, that we don't check if old and
                        ; new binding are compatible, since the whole thing
                        ; might be inconsistend while %FILTER-BINDINGS is not
                        ; fully done. In the end everything should be fine,
                        ; since the bindings we filter do come from a
                        ; successful prolog query.
        (values (cons `(,var . ,val) bdgs) t))))

(defun %filter-bindings (form extended-bdgs initial-bdgs)
  (cond ((is-var form)
         (let ((val (var-value form extended-bdgs)))
           (if (eq val form)
               initial-bdgs
               (filter-add-bdg form val initial-bdgs))))
        ((or (not form) (atom form))
         initial-bdgs)
        (t
         (%filter-bindings (cdr form) extended-bdgs
                           (%filter-bindings (car form) extended-bdgs initial-bdgs)))))

(defun prolog (query &optional (binds nil))
  (lazy-mapcar (rcurry (curry #'filter-bindings query) binds)
               (prove-one query binds)))
