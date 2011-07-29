(in-package :cram-utilities)

(defmacro string-case (var &body cases)
  (once-only (var)
    `(cond ,@(mapcar (lambda (case-expr)
                       (destructuring-bind (pat &rest body) case-expr
                         (if (eq pat t)
                             `(t ,@body)
                             `((string-equal ,var ,pat)
                               ,@body))))
              cases))))

;;; From swank.lisp which is public domain.
(defmacro destructure-case (value &rest patterns)
  "Dispatch VALUE to one of PATTERNS.
A cross between `case' and `destructuring-bind'.
The pattern syntax is:
  ((HEAD . ARGS) . BODY)
The list of patterns is searched for a HEAD `eq' to the car of
VALUE. If one is found, the BODY is executed with ARGS bound to the
corresponding values in the CDR of VALUE."
  (let ((operator (gensym "op-"))
        (operands (gensym "rand-"))
        (tmp (gensym "tmp-")))
    `(let* ((,tmp ,value)
            (,operator (car ,tmp))
            (,operands (cdr ,tmp)))
       (case ,operator
         ,@(loop for (pattern . body) in patterns collect 
                 (if (eq pattern t)
                     `(t ,@body)
                     (destructuring-bind (op &rest rands) pattern
                       `(,op (destructuring-bind ,(ensure-list rands)
                                 ,operands 
                               ,@body)))))
         ,@(if (eq (caar (last patterns)) t)
               '()
               `((t (error "destructure-case failed: ~S" ,tmp))))))))

;;; Note: this implementation does not properly handle declarations.
(defmacro flet* (bindings &body body)
  "FLET* is to FLET what LET* is to LET."
  (labels ((gen (bindings body)
             (if (null bindings)
                 body
                 `((flet (,(first bindings))
                      ,@(gen (rest bindings) body))))))
    (car (gen bindings body))))

(defun body-context (forms)
  (let ((*print-length* 3)
        (*print-level* 3)
        (*print-pretty* t))
    (prin1-to-string forms)))

(defun %assert-no-returning (ctx thunk)
  (funcall thunk)
  (error "Asserted~%  ~@<~A~:>~%not to return. But we did return." ctx))

(defmacro assert-no-returning (&body body)
  `(%assert-no-returning ,(body-context body) #'(lambda () ,@body)))

(defun %assert-no-nlx (ctx thunk)
  (unwind-protect-case ()
      (funcall thunk)
    (:abort
     (error "Asserted~%  ~@<~A~:>~%not to perform a non-local ~
             exit. Caught attempt to do so." ctx))))

(defmacro assert-no-nlx (&body body)
  `(%assert-no-nlx ,(body-context body) #'(lambda () ,@body)))
