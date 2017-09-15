;; Intermediate values
;; Liam Healy Sat Apr  7 2001 - 14:42
;; Time-stamp: <2011-01-15 11:13:28EST intermediate.lisp>

;;; This facilitates optionally seeing intermediate values.
;;; These will go away, so don't use in new code.

(in-package :antik)

(export '(pedagogical see-intermediate-value see-values set-showing))

(defparameter *pedagogical* nil
  "A list of categories of intermediate values
   that should be printed during calculation.")

(defparameter *pedagogical-stream* t
  "Stream to output information.")

(defmacro pedagogical (keyword &body body)
  `(when (member ,keyword *pedagogical*)
     ,@body))

(defmacro see-intermediate-value (keyword label form)
  "If keyword is in *pedagogical*, then print returned value of form,
   and return it; otherwise, just evaluate the form and return."
  (alexandria:with-gensyms (answer)
    (let ((tex-label (if (listp label) (second label) label))
	  (screen-label (if (listp label) (first label) label)))
      `(let ((,answer ,form))
	 (when (member ,keyword *pedagogical*)
	   (if texstyle
	       (format t "~&~a~2t~a \\\\"
		       ;; Tie this in with tex-print-name etc.
		       (mkstr ,tex-label "=&")
		       (nf-string ,answer))
	       (format t "~&~a:~&~2t~a"
		       ,screen-label
		       (nf-string ,answer))))
	 ,answer))))

;;; Experimental, user can specify format string.  Used in gps.lisp.  
(defmacro see-values
    (keyword skipping-compute format-string &rest args)
  "If keyword is in *pedagogical*, then print returned value of form,
   and return it; otherwise, just evaluate the form and return.
   If skipping-compute is true, forms are evaluated left to right,
   whether or not the keyword is in *pedagogical*.
   If it is false, they are only evaluated if keyword is in *pedagogical*, 
   and the last one is returned from this form."
  (let ((formsyms (loop repeat (length args) collect (gensym))))
    ;; let is only used to insure left-to-right evaluation
    ;; file:/pop/usr/local/lisp/HyperSpec/Body/speope_letcm_letst.html:
    ;; "...evaluates the expressions init-form-1, init-form-2, and so on, in that order,..."
    `(,(if skipping-compute 'if 'when) (member ,keyword *pedagogical*)
	 (let ,(loop for s in formsyms for f in args collect (list s f))
	   (format *pedagogical-stream* ,format-string ,@formsyms)
	   ,(alexandria:lastcar formsyms))
       ,@(when skipping-compute `((progn ,@args))))))

(defun set-showing (keyword &optional (status :on))
  "Set the showing of intermediate values."
  (if (eq status :on)
      (pushnew keyword *pedagogical*)
    (setf *pedagogical* (remove keyword *pedagogical*))))
