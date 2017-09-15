;;;
;;; Copyright (c) 2009, Nikolaus Demmel <demmeln@cs.tum.edu>
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

(in-package :walker)

;;;; DISCLAIMER: This walker doesn't aim to actually expand code correctly
;;;;             in every possible way.
;;;; This walker also does not check for validity of passed lisp code.
;;;; It might do unexpected things, if the code passed is not ANSI CL.


;;; TODO: Maybe add the handling of some cpl internal macros.
;;;       Make them special forms for more performance.
;;; TODO: Introduce proper error signalling with own condition class
;;; TODO: Lots and lots of test cases



(defvar *current-path* nil
  "Current path during walking and creating the plan tree.")
(defvar *current-parent* nil
  "Contains the current parent when constructing the plan tree during code
   walking. If we call walk-form with this being nil, it doesnt build a tree,
   nor follow plan calls.")
(defvar *shadowed-functions* nil
  "This picks up the shaddowed function while walking, so we dont need to
   access the environment (which is not possible in a standardized way, not
   even with whats in CLTL2).")
(defvar *tag-walker-handlers* nil
  "When a (:tag name body) form is encounterd during code walking,
   all functions in the list that *tag-walker-handlers* refers to are called
   with the 2 parameters name and body.")

(defun tag-form-handler (form)
  (destructuring-bind (label name &rest body) form
    (declare (ignore label))
    (loop for handler in *tag-walker-handlers*
       do (funcall handler name body))))

(defun extend-path (type label current-path)
  (case type
    (:top-level-plan (assert (null current-path))
                     (cons (list 'cpl::top-level label) current-path))
    (:plan (cons (list label) current-path))
    (:tag (cons (list 'cpl::tagged label) current-path))
    (otherwise (error "Trying to extend path in an invalid way."))))

(defmacro with-extended-path (type label &body body)
  `(let ((*current-path* (extend-path ,type ,label *current-path*)))
     ,@body))

(defun register-child-plan (child)
  (setf (plan-tree-node-children *current-parent*)
        (append (plan-tree-node-children *current-parent*) (list child))))

(defun walk-plan-form (form env)
  "Handles a plan or tag form and creates a node in the plan tree."
  (assert *current-parent*)
  (let ((label (if (get (car form) 'cpl::plan-type)
                   (car form)    ; plan form
                   (cadr form))) ; tagged form
        (type (or (get (car form) 'cpl::plan-type) ; plan form
                  :tag))                           ; tagged form
        (sexp (or (get (car form) 'cpl::plan-sexp) ; plan form
                  (cddr form))))                   ; tagged form
    (assert (if (eq type :tag)
                (eq (car form) :tag)
                t))
    (when (not (eq :tag type))
      ;; If we encounter a plan call, walk the parameters first
      ;; (those might contain calls to plans aswell)
      (walk-funcall form env))
    (with-extended-path type label 
      (let ((node (make-plan-tree-node :sexp form
                                       :parent *current-parent*
                                       :path *current-path*)))
        (register-child-plan node)
        (let ((*current-parent* node))
          (if (eq :tag type)
              (walk-block form env)
              ;; If walking new plan, start with a clean slate
              ;; regarding env and *shadowed-funcitons*,
              (let ((*shadowed-functions* nil))
                (walk-form (cons 'locally sexp) nil))))))))

(defun walk-form (form env)
  "Takes a form and an environment and walkes the form if neccessary, after macroexpanding it fully and considering special cases."
  (flet ((handle-macro/funcall (form env)
           (multiple-value-bind (expansion expanded)
               (macroexpand-1 form env)
             (if expanded
                 (walk-form expansion env)
                 ;; Determin if form is a call to a plan, that is not shadowed
                 ;; and the we actually want to create a plan tree (indicated
                 ;; by *current-parent* not being nil.
                 (if (and *current-parent*
                          (get (car form) 'cpl::plan-type)
                          (not (member (car form) *shadowed-functions*)))
                     (progn (walk-plan-form form env) ;; call to plan 
                            form)
                     (walk-funcall form env)))))) ;; ordinary function
    (cond
      ((atom form)
       (multiple-value-bind (expansion expanded)
           (macroexpand-1 form env)
         (if expanded
             (walk-form expansion env)
             form)))
      ((not (symbolp (car form)))
       (if (not (and (consp (car form)) (eq (caar form) 'lambda)))
           (error "Invalid form ~a." form)
           `(,(walk-lambda (car form) env)
              ,(walk-list (cdr form) env))))
      ((get (car form) 'walk-fn)
       (funcall (get (car form) 'walk-fn) form env))
      (t
       (handle-macro/funcall form env)))))

(defun walk-cpl-tag (form env)
  (tag-form-handler form)
  (if *current-parent*
      (walk-plan-form form env)
      (walk-block form env)))


;;; TODO: Maybe refactor walker-fns using destructuring-bind
;;;       instead of car/cadr/... for more clarity.
;;;       Maybe not ;-)

(defun walk-list (list env)
  "Walk each element of the list."
  (mapcar #'(lambda (x) (walk-form x env))
          list))

(defun walk-funcall (form env)
  "Walk all but car."
  `(,(car form) ,@(walk-list (cdr form) env)))

(defun walk-quote (form env)
  "Don't walk anything"
  (declare (ignore env))
  form)

(defun walk-block (form env)
  "Walk all but car and cadr."
  `(,(car form) ,(cadr form) ,@(walk-list (cddr form) env)))

(defun walk-load-time-value (form env)
  "Walk the cadr in a nil lexical environment."
  (declare (ignore env))
  (assert (<= (length form) 3))
  `(,(car form) ,(walk-form (cadr form) nil) ,@(cddr form)))

(defun walk-let (form env &key (let*-style-bindings nil))
  "Walk list of bindings and body."
  (multiple-value-bind (bindings new-env)
      (walk-binding-list (cadr form) env let*-style-bindings)
    `(,(car form)
       ,bindings
       ,@(walk-list (cddr form) new-env))))

(defun walk-let* (form env)
  (walk-let form env :let*-style-bindings t))

(defun walk-binding-list (list env let*-style-bindings)
  "Walk list of variable bindings like found in let or let*. If
   let*-style-bindings is not nil, the bindings are expanded like in a let*."
  (let ((new-env env))
    (values 
     (mapcar (lambda (b)
               (prog1 
                   (walk-binding b (if let*-style-bindings new-env env))
                 (setf new-env (aug-env new-env :variable (list b)))))
             list)
     new-env)))

(defun walk-binding (b env)
  "Walk variable binding like found in let."
  (assert (or (atom b) (<= (length b) 3)))
  (if (and (consp b) (consp (rest b)))
      `(,(car b) ,(walk-form (cadr b) env))
      b))

(defun walk-function (form env)
  "Walk the cadr. Handle special case of cadr being (lambda () ...), or (setf
   ...)"
  (assert (= 2 (length form)))
  (let ((f (cadr form)))
    `(function
      ,(if (consp f)
           (cond 
             ((eq (car f) 'lambda)
              (walk-lambda f env))
             #+sbcl ((eq (car f) 'sb-int:named-lambda)
                     (walk-named-lambda f env))
             ((and (eq (car f)  'setf)
                   (cdr f)
                   (symbolp (cadr f))
                   (null (cddr f)))
              f)
             (t (error "Invalid function form with ~a as cadr." f)))
           (walk-form f env)))))        ; must be symbol 

(defun walk-lambda (form env)
  "Walk argument list and body."
  `(,(car form)
     ,(walk-ordinary-lambda-list (cadr form) env)
     ,@(walk-list (cddr form)
                  (aug-env env :variable (cadr form)))))

#+sbcl
(defun walk-named-lambda (form env)
  "Walk argument list and body."
  `(,(car form)
     ,(cadr form)
     ,(walk-ordinary-lambda-list (caddr form) env)
     ,@(walk-list (cdddr form)
                  (aug-env env :variable (caddr form)))))

(defun walk-ordinary-lambda-list (list env)
  "Walk lambda argument list (ordinary lambda list)."
  (mapcar (rcurry #'walk-argument env)
          list))

(defun walk-argument (arg env)
  "Walk lambda argument (part of an ordinary lambda list). If it is a list of
   at least two elements, walk the cadr. Walk nothing else."
  (if (and (consp arg) (consp (cdr arg)))
      `(,(car arg)
         ,(walk-form (cadr arg) env)
         ,@(cddr arg))
      arg))

(defun walk-macrolet (form env)
  "Add macro or symbol-macro definitions to the environment and proceed
   expanding the body (wrapped in a locally)."
  (walk-form `(locally ,@(cddr form))
             (aug-env env (car form) (cadr form))))

(defmacro with-shadowed-functions (binds &body body)
  `(let ((*shadowed-functions* (union (mapcar #'car ,binds)
                                      *shadowed-functions*)))
     ,@body))

(defun walk-flet (form env)
  "Walk the function definitions and the walk the body with the functions
   added to the environment."
  `(flet ,(walk-list-of-lambdas (cadr form) env)
     ,@(with-shadowed-functions (cadr form)
                                (walk-list (cddr form)
                                           ;; aug-env only needs the function
                                           ;; name, so no need to expand here
                                           (aug-env env 'flet (cadr form)))))) 


(defun walk-labels (form env)
  "Add the function definitions to the environment, then walk those
   definitions and the body."
  (let ((env (aug-env env 'labels (cadr form))))
    (with-shadowed-functions (cadr form)
      `(labels ,(walk-list-of-lambdas (cadr form) env)
         ,@(walk-list (cddr form) env)))))

(defun walk-list-of-lambdas (list env)
  "Walk each element like a lambda form."
  (mapcar (rcurry #'walk-lambda env)
          list))


;;; Add walker function to the plist of the symbols that need to handled as
;;; special forms
(mapc #'(lambda (x)
          (setf (get (car x) 'walk-fn)
                (symbol-function (cadr x))))
      '( ;; *** CL special forms
        (cl:block                walk-block)
        (cl:catch                walk-funcall)
        (cl:eval-when            walk-block)
        (cl:flet                 walk-flet)
        (cl:function             walk-function)
        (cl:go                   walk-quote)
        (cl:if                   walk-funcall)
        (cl:labels               walk-labels)
        (cl:let                  walk-let)
        (cl:let*                 walk-let*)
        (cl:load-time-value      walk-load-time-value)
        (cl:locally              walk-funcall)
        (cl:macrolet             walk-macrolet)
        (cl:multiple-value-call  walk-funcall)
        (cl:multiple-value-prog1 walk-funcall)
        (cl:progn                walk-funcall)
        (cl:progv                walk-funcall)
        (cl:quote                walk-quote)
        (cl:return-from          walk-block)
        (cl:setq                 walk-funcall)
        (cl:symbol-macrolet      walk-macrolet)
        (cl:tagbody              walk-funcall)
        (cl:the                  walk-block)
        (cl:throw                walk-funcall)
        (cl:unwind-protect       walk-funcall)

        ;; *** not secpial forms, but need to be handled seperatelly anyway
        (cl:declare              walk-quote) ; ingnore declarations for now

        ;; *** CRAM plan macros, that we treat specially
        (:tag                    walk-cpl-tag)
        
        ;; *** non standard support

        ;; compiler-let:
        ;; was in CL 1 but not in ANSI CL, so we don't process it for now.

        ;; internal-the (UNTESTED):
        ;; special lispworks form that we handle the same way as cl-walker.
        (internal-the            walk-block)
        ))
