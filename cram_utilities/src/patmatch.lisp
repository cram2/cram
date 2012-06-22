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

(in-package :cut)

(defun is-var (pat)
  "Predicate that returns a non-nil value if the pattern is a variable."
  (and (symbolp pat)
       (eql (elt (symbol-name pat) 0)
            #\?)))

(defun is-unnamed-var (pat)
  "Returns true if `pat' is the unnamed variable (?_)."
  (eq pat '?_))

(defun is-segvar (pat)
  "Predicate that returns a non-nil value if the `pat' is a segment-variable."
  (and (symbolp pat)
       (<= 2 (length (symbol-name pat)))
       (eql (elt (symbol-name pat) 0) #\!)
       (eql (elt (symbol-name pat) 1) #\?)))

(defun is-segform (form)
  "Predicates that returns a non-nil value if form is a segform, i.e. its car
   is a segvar."
  (and (consp form)
       (is-segvar (car form))))

(defun var-name (var)
  "Extracts the variable name from a segvar. If `var' is a segvar, VAR-NAME
   returns a variable in the same package as `var', but with a name without
   the preceeding '!'. Returns `var' unchanged if `var' is a variable. Throws
   an error otherwise."
  (cond ((is-var var)
         var)
        ((is-segvar var)
         (let ((pkg (symbol-package var))
               (name (subseq (symbol-name var) 1)))
           (if pkg (intern name pkg) (make-symbol name)))) 
        (t
         (error "No variable."))))

(defun gen-var (&optional base)
  "Returns a newly generated unique var. `base' is an optional string used as
   a prefix for the variable name, which must start with '?'."
  (let ((base (or base "?VAR-")))
    (assert (and (stringp base)
                 (<= 1 (length base))
                 (eql (elt base 0) #\?)))
    (gensym base)))

(defun is-genvar (var)
  "Returns true if the variable has been generated with gen-var."
  (assert (is-var var))
  (not (symbol-package var)))

(defun add-bdg (var val bdgs)
  "Adds a variable binding to the bindings list. The first return value is the
   updated bindings and the second return value is non-nil, if binding has
   been added successfully and nil, if a conflict with a previous binding of
   the same variable exists."
  (if (is-unnamed-var var)
      (values bdgs t)
      (let ((old-bdg (assoc var bdgs)))
        (if old-bdg
            (if (equal (var-value (cdr old-bdg) bdgs) val)
                (values bdgs t)
                (values nil nil))
            (values (cons `(,var . ,val) bdgs) t)))))

(defun substitute-vars (pat bdgs)
  "Substitues all variables in a pattern `pat' recursively with their
   bindings."
  (map-tree (rcurry #'var-value bdgs) pat))

(defun var-value (var binds)
  "Returns the value of a variable. Note: The variable is simplified,
  i.e. if its value is a variable the value of that variable is returned."
  (if (is-var var)
      (let ((value (assoc var binds)))
        (if (not value)
            var
            (substitute-vars (cdr value) binds)))
      var))

(defun match-segvar (pat seq bdgs continuation)
  "Matches a segvar. The continuation is a recursively called function that
   matches the forms after the segform. (TODO: explain better)"
  (labels ((collect-segval (&optional (seq seq) (content nil))
             (multiple-value-bind (new-bdgs matched?)
                 (funcall continuation (cdr pat) seq bdgs)
               (if (and (not matched?) seq)
                   (collect-segval (cdr seq) `(,@content ,(car seq)))
                   (values matched? new-bdgs content)))))
    (if (endp (cdr pat))
        (if (proper-list-p seq) ;; seg-var is guarantued to be a list if it matches
            (add-bdg (var-name (car pat)) seq bdgs)
            (values nil nil))
        (multiple-value-bind (success? new-bdgs content)
            (collect-segval)
          (if success?
              (add-bdg (var-name (car pat)) content new-bdgs)
              (values nil nil))))))

(defun pat-match (pat seq &optional (bdgs nil) &rest rest)
  "Match a pattern."
  (unless (listp bdgs)
    (push bdgs rest)
    (setf bdgs nil))
  (when (or (is-segvar seq) (is-var seq))
    (error "Found variable ~a on right hand side of pattern match." seq))
  (destructuring-bind (&key (test #'eql)) rest
    (cond ((is-var pat)
           (add-bdg (var-name pat) seq bdgs))
          ((is-segvar pat)
           (error "Segvar ~a must be car of a cons-cell." pat))
          ((and (atom pat) (atom seq))
           (if (funcall test pat seq)
               (values bdgs t)
               (values nil nil)))
          ((is-segform pat)
           (match-segvar pat seq bdgs (rcurry #'pat-match :test test)))
          ((and (consp pat) (consp seq))
           (multiple-value-bind (new-bdgs matched?)
               (pat-match (car pat) (car seq) bdgs :test test)
             (if matched?
                 (pat-match (cdr pat) (cdr seq) new-bdgs :test test)
                 (values nil nil))))
          (t (values nil nil)))))

(defun pat-match-p (pat seq &optional (bdgs nil) &rest rest)
  "Pattern matching predicate. Does not return bdgs but only T or nil,
  indicating if the pattern matches."
  (nth-value 1 (apply #'pat-match pat seq bdgs rest)))

;;; NOTE #demmeln: Some of the following functions do not handle segvars at
;;; all. Should they?

(defun vars-in (pat)
  "Returns a list of all variables in pattern (without duplicates)."
  (labels ((worker (&optional (pat pat) (vars nil))
             (cond ((consp pat)
                    (worker (cdr pat) (worker (car pat) vars)))
                   ((and (is-var pat)
                         (not (member pat vars)))
                    (cons pat vars))
                   (t
                    vars))))
    (worker)))

(defmacro with-vars-bound (vars bdg &body body)
  (with-gensyms (g-bdg)
    `(let ((,g-bdg ,bdg))
       (let ,(loop for var in vars
                collecting `(,var (var-value ',var ,g-bdg)))
         ,@body))))

(defmacro with-vars-strictly-bound (vars bdg &body body)
  "Like WITH-VARS-BOUND with the difference that it throws an error if
  a variable couldn't be bound."
  `(with-vars-bound ,vars ,bdg
     ,@(loop for var in vars collecting
             `(when (is-var ,var)
                (error 'simple-error
                       :format-control "Couldn't resolve variable ~a."
                       :format-arguments (list ,var))))
     ,@body))

(defmacro with-pat-vars-bound (pat seq &body body)
  "Evaluates `body' in a context where all the variables in `pat' are bound to
   their binding resulting from calling PAT-MATCH on `pat' and `seq'. Throws
   an error, if the pattern match fails. `pat' is not evaluated, whereas `seq'
   is evaluated."
  (with-gensyms (bdgs matched? g-seq)
    `(let ((,g-seq ,seq))
       (multiple-value-bind (,bdgs ,matched?) (pat-match ',pat ,g-seq)
         (unless ,matched?
           (error "Pattern ~a does not match ~a in with-pat-vars-bound."
                  ',pat ,g-seq))
         (unless ,bdgs
           (warn "Pattern ~a does not contain any variables in with-pat-vars-bound"
                 ',pat))
         (let ,(mapcar (lambda (var)
                         `(,var (var-value ',var ,bdgs)))
                       (vars-in pat))
           ,@body)))))

(defun is-bound (pat bdgs)
  "Returns NIL if pat is a variable that is either not bound or bound to an
   unbound variable, T otherwise."
  (not (and (is-var pat)
            (is-var (var-value pat bdgs)))))

(defun is-ground (pat bdgs)
  "Returns NIL if pat is a pattern with unbound parts, T otherwise. A pattern
   contains unbound parts, if it contains an unbound variable, or if it
   contains a variable bound to a pattern with unbound parts."
  (null (vars-in (substitute-vars pat bdgs))))

(defun rename-vars (pat &key (gensym-var-names t))
  "Renames all variables in `par' with unique symbols. If `gensym-var-names'
   is true, gensym is used to create the variables. If it is NIL, the symbol
   name of the newly create variables will be exactly the same."
  (sublis (mapcar (lambda (x) (cons x (if gensym-var-names
                                          (gen-var (format nil "~a-" x))
                                          (make-symbol (symbol-name x)))))
                  (remove '?_ (vars-in pat)))
          pat))

(defun patterns-eq (pattern-1 pattern-2)
  "Returns T if two patterns are equal. Equality means that all
non-variables are EQ and the variables occur at the same locations."
  (cond ((and (not pattern-1) (not pattern-2))
         t)
        ((eq (car pattern-1) (car pattern-2))
         (patterns-eq (cdr pattern-1) (cdr pattern-2)))
        ((and (is-var (car pattern-1)) (is-var (car pattern-2)))
         (patterns-eq (cdr pattern-1) (cdr pattern-2)))
        ((and (listp (car pattern-1)) (listp (car pattern-2)))
         (and (patterns-eq (car pattern-1) (car pattern-2))
              (patterns-eq (cdr pattern-1) (cdr pattern-2))))
        (t nil)))

(defun bindings-equal (lhs rhs &key (test #'equal))
  (loop for (variable . value) in lhs
        for rhs-value = (cdr (assoc variable rhs))
        unless (funcall test value rhs-value) do
          (return nil)
        finally (return t)))

