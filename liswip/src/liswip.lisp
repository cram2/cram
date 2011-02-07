;;;
;;; Copyright (c) 2009, Lars Kunze <kunzel@cs.tum.edu>
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


(in-package :liswip)

(defvar *pl-user-module* "user")

(defvar *processing-query* nil
  "When T, we are currently processing a query.")

(defvar *xsd-float* "http://www.w3.org/2001/XMLSchema#float")
(defvar *xsd-int* "http://www.w3.org/2001/XMLSchema#int")

(define-condition prolog-error (simple-error) ())

;;; We need to use a lexically bound variable for initialization marker
;;; since dynamic variables are thread local.

(let ((liswip-initialized nil))
  (defun liswip-initialized (&optional new-value)
    (when new-value
      (setf liswip-initialized new-value))
    liswip-initialized))

(defmacro with-prolog-engine (&body body)
  "Ensures that a prolog engine is running and evaluates `body'. When
  a new engine had to be created, destroys it after `body' finished."
  `(let ((engine-created nil))
     (assert (liswip-initialized) () "Not initialized yet. Call SWI-INIT first.")
     (unwind-protect
          (progn
            (when (< (pl-thread-self) 0)
              (setf engine-created t)
              (pl-thread-attach-engine (null-pointer)))
            ,@body)
       (when engine-created
         (pl-thread-destroy-engine)))))

(defun swi-init (&key
                 (prolog-binary "/usr/bin/swipl")
                 (options (list "--nosignals")))
  (unless (liswip-initialized)
    (let ((argv nil))
      (unwind-protect
          (progn
            (setf argv (foreign-alloc :string :initial-contents (cons prolog-binary options)))
            (pl-initialise (1+ (length options)) argv)
            (init-lisp-blobs)
            (liswip-initialized t)
            (register-swi-predicates))
        (foreign-free argv)))))

(defun swi-cleanup ()
  (pl-cleanup 0)
  (cleanup-lisp-blobs)
  (liswip-initialized nil))

(defun prologify (s)
  (flet ((contains-lower-case-char (symbol)
           (and 
            (find-if (lambda (ch)
                       (let ((lch (char-downcase ch)))
                         (and (find lch "abcdefghijklmnopqrstuvwxyz")
                              (eq lch ch))))
                     (symbol-name symbol))
            t)))
    (if (contains-lower-case-char s)
        (string s)
        (string-downcase (substitute #\_ #\- (string s))))))

(defun lispify (s)
  (string-upcase (string s)))

(defun get-term-value (term)
  (declare (special *lispify*))
  (with-foreign-object (val :pointer)
    (let ((term-type (pl-term-type term)))
      (cond ((eql term-type pl-integer) 
             (pl-get-integer term val)
             (mem-ref val :int))
            ((is-owl-integer term)
             (get-owl-type term))
            ((eql term-type pl-float) 
             (pl-get-float term val)
             (mem-ref val :double))
            ((is-owl-float term)
             (get-owl-type term))
            ((eql term-type pl-term) ;; compound term
             (cond ((not (eql (pl-is-list term) 0))
                    (get-terms-from-list term))
                   (t
                    (with-foreign-objects ((name 'atom_t)
                                           (arity :int))
                      (pl-get-name-arity term name arity)
                      (cons (intern (if *lispify*
                                        (lispify (pl-atom-chars (mem-ref name 'atom_t)))
                                        (pl-atom-chars (mem-ref name 'atom_t))))
                            (loop for i from 1 to (mem-ref arity :int)
                                  with args-term = (pl-new-term-ref)
                                  collecting (progn
                                               (pl-get-arg i term args-term)
                                               (get-term-value args-term))))))))
            ((eql term-type pl-string) 
             (pl-get-chars term val 127)
             (mem-ref val :string))
            ((lisp-object? term)
             (get-lisp-object term :gc (not *processing-query*)))
            ((eql term-type pl-atom)
             (pl-get-chars term val 127)
             (intern (if *lispify*
                         (lispify (mem-ref val :string))
                         (mem-ref val :string))))
            ((eql term-type pl-variable)
             (pl-get-chars term val 127)
             (intern (if *lispify*
                         (lispify (concatenate 'string "?" (mem-ref val :string)))
                         (concatenate 'string "?" (mem-ref val :string)))))
            (t
             (pl-get-chars term val 127)
             (mem-ref val :string))))))

(defun get-terms-from-list (termlist)
  (let ((head (pl-new-term-ref))
        (lst  (pl-copy-term-ref termlist)))
    (loop until (eql (pl-get-list lst head lst) 0)
         collecting  (get-term-value head))))

(defun get-owl-type (term)
   (let ((type (pl-new-term-ref))
         (val (pl-new-term-ref)))
     (pl-get-arg 1 term type)
     (pl-get-arg 2 type val)
     (read-from-string (string (get-term-value val)))))

(defun is-owl-integer (term)
  (is-owl-type *xsd-int* term))

(defun is-owl-float (term)
  (is-owl-type *xsd-float* term))

(defun is-owl-type (type term)
  (with-foreign-object (val :pointer)
    (if (eql (pl-term-type term) pl-term)
        (progn
          (pl-get-chars term val 127)
          (search (concatenate (quote string) "literal(type(" type) (mem-ref val :string)))
        nil)))

(defun put-term-value (term val &optional vars)
  (declare (special *prologify*))
  (cond ((not val)
         (pl-put-nil term))
        ((is-var val)
         (when (not (gethash val vars))
           (setf (gethash val vars) (pl-new-term-ref)))
         (pl-put-term term (gethash val vars)))
        ((and (listp val) (eq (car val) 'quote))
         (labels ((put-list (term list)
                    (cond ((consp list)
                           (let ((first (pl-new-term-ref))
                                 (rest (pl-new-term-ref)))
                             (put-list first (car list))
                             (put-list rest (cdr list))
                             (pl-cons-list term first rest)))
                          (t
                           (put-term-value term list)))))
           (put-list term (cadr val))))
        ((listp val)
         (let* ((functor-terms (pl-query val vars))
                (functor (car functor-terms))
                (terms (cdr functor-terms)))
           (pl-cons-functor-v term functor terms)))
        ((floatp val)
         (pl-put-float term (coerce val 'double-float)))
        ((integerp val)
         (pl-put-integer term val))
        ((stringp val)
         ;;(pl-put-atom-chars term val))
         (pl-put-string-chars term val))
        ;; needed for string handling,
        ;; but causes errors when rdf_has predicate is
        ;; called with string instead of an atom
        ((symbolp val)
         (pl-put-atom-chars term (if *prologify*
                                     (prologify val)
                                     (string val))))
        (t
         (put-lisp-object term val))))

;; takes lisp expr, builds prolog expr, calls prolog, and retrieves answer
(defun swi-prolog (expr &key (prologify t) (lispify t))
  "Evaluates `expr' in SWI Prolog. If `prologify' is true, converts
   all lispy symbols into prolog. - is replaced by _ and the name is in
   lower case."
  (assert (not *processing-query*) ()
          "Cannot call prolog when already inside a query.")
  (with-prolog-engine
    (let ((fid nil)
          (qid nil)
          (*prologify* prologify)
          (*lispify* lispify))
      (declare (special *prologify* *lispify*))
      (unwind-protect
           (progn
             (setf fid (pl-open-foreign-frame))
             (let* ((vars (make-hash-table))
                    (functor-terms (pl-query expr vars))
                    (functor (car functor-terms))
                    (terms (cdr functor-terms))
                    (pred (pl-pred functor (pl-new-module (pl-new-atom *pl-user-module*)))))
               (setf qid (pl-open-query (null-pointer) PL-Q-CATCH_EXCEPTION pred terms))
               (let ((exception (pl-exception qid)))
                 (unless (eql exception 0)
                   (error 'prolog-error
                          :format-control "Prolog query failed with error ~a."
                          :format-arguments (get-term-value exception))))
               (let* ((success nil)
                      (result (loop until (eql (let ((*processing-query* t))
                                                 (pl-next-solution qid))
                                               0)
                                 collecting (loop for k being the hash-keys of vars using (hash-value v)
                                               collecting (cons k (get-term-value v)))
                                 do (setf success t))))
                 (values result success))))
        (when qid
          (pl-close-query qid))
        (when fid
          (pl-close-foreign-frame fid))))))
  
;; parses lisp expression expr and constructs query by using the C interface
(defun pl-query (expr vars)
  (case (car expr)
    (and (pl-query-and (cdr expr) vars))
    (or (pl-query-or (cdr expr) vars))
    (not (pl-query-not (cdr expr) vars))
    (t (pl-query-simple (car expr) (cdr expr) vars))))

;; handle and expressions
(defun pl-query-and (clauses vars)
  (let* ((num (length clauses)))
    (cond ((>= num 2)
           (let* ((and-functor (pl-new-functor (pl-new-atom ",") 2))
                  (and-terms (pl-new-term-refs 2))
                  (functor-terms1 (pl-query (car clauses) vars))
                  (functor1 (car functor-terms1))
                  (terms1 (cdr functor-terms1))
                  (functor-terms2 (if (eql num 2)
                                      (pl-query (cadr clauses) vars)
                                      (pl-query-and (cdr clauses) vars)))
                  (functor2 (car functor-terms2))
                  (terms2 (cdr functor-terms2)))
             (pl-cons-functor-v and-terms  functor1 terms1)
             (pl-cons-functor-v (+ and-terms 1) functor2 terms2)
             (cons and-functor and-terms)))
          (t
           (pl-query (car clauses) vars)))))

;; handle or expressions
(defun pl-query-or (clauses vars)
  (let* ((num (length clauses)))
    (cond ((>= num 2)
           (let* ((or-functor (pl-new-functor (pl-new-atom ";") 2))
                  (or-terms (pl-new-term-refs 2))
                  (functor-terms1 (pl-query (nth 0 clauses) vars))
                  (functor1 (car functor-terms1))
                  (terms1 (cdr functor-terms1))
                  (functor-terms2 (if (eql num 2)
                                      (pl-query (nth 1 clauses) vars)
                                      (pl-query-or (cdr clauses) vars)))
                  (functor2 (car functor-terms2))
                  (terms2 (cdr functor-terms2)))
             (pl-cons-functor-v or-terms  functor1 terms1)
             (pl-cons-functor-v (+ or-terms 1) functor2 terms2)
             (cons or-functor or-terms)))
          (t
           (pl-query (car clauses) vars)))))

;; handle not expressions
(defun pl-query-not (clauses vars)
  (assert (eql (length clauses) 1) () "Not takes exactly one clause.")
  (let* ((not-functor (pl-new-functor (pl-new-atom "not") 1))
         (not-term (pl-new-term-ref))
         (functor-terms1 (pl-query (car clauses) vars))
         (functor1 (car functor-terms1))
         (terms1 (cdr functor-terms1)))
    (pl-cons-functor-v not-term functor1 terms1)
    (cons not-functor not-term)))
            
;; handle simple expressions, i.e. prolog predicates  
(defun pl-query-simple (pred args vars)
  (declare (special *prologify*))
  (let* ((arity (length args))
         (terms (pl-new-term-refs arity))
         (functor (pl-new-functor (pl-new-atom (if *prologify*
                                                   (prologify pred)
                                                   (string pred))) arity)))
    (loop for i from 0 below arity
       do (put-term-value (+ terms i) (nth i args) vars))
    (cons functor terms)))

(defun decode-term (term arity)
  "Returns the list of term values. Term must have arity `arity'."
  (loop for i from 0 below arity
     collecting (get-term-value (+ term i))))

(defun swi-unify (term value)
  "Uses SWI-Prolog to unify `term' with `value'. This function has side effects!"
  (flet ((unify-list (term value)
           (assert (listp value))
           (let ((first (pl-new-term-ref))
                 (rest (pl-new-term-ref)))
             (pl-unify-list term first rest)
             (swi-unify first (car value))
             (swi-unify rest (cdr value)))))
    (typecase value
      (integer (pl-unify-integer term value))
      (real (pl-unify-float term value))
      (symbol (pl-unify-atom-chars term (symbol-name value)))
      (string (pl-unify-string-chars term value))
      (list (unify-list term value))
      (t (unify-lisp-object term value)))))
