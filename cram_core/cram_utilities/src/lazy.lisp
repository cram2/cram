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


(in-package :cut)

;;; This file provides the lazy data type delay.
;;; evaluation of expression is done when force is called
;;; on the object.

;;; Lazy lists are also provided. When generating a lazy list,
;;; an element generator must be passed. It gets the previously
;;; generated element as parameter (nil in first call) and
;;; returns t as second value when list is finished.
;;; When the list is fully expanded, it transforms into a normal
;;; list.

(defconstant +lazy-value-uninitialized+ '+lazy-value-uninitialized+)

;; Lazy container
(defstruct delay
  (value +lazy-value-uninitialized+)
  generator)

(defmacro delay (&body expr)
  `(make-delay :generator #'(lambda () ,@expr)))

(defun force (obj)
  (typecase obj
    (delay (when (eq (delay-value obj) +lazy-value-uninitialized+)
             (setf (delay-value obj) (funcall (delay-generator obj))))
      (delay-value obj))
    (t obj)))

;; Lazy list
(defstruct lazy-cons-elem
  generator)

(defvar *lazy-list-dynamic-environment*
  (lambda (body-function) (funcall body-function)))

(defmacro with-lazy-list-dynamic-environment (variable-definition-forms &body body)
  (let ((variable-names (loop for definition in variable-definition-forms
                              collecting (car definition))))
    `(flet ((run-in-environment (body-function)
              (let ,variable-definition-forms
                (declare (special ,@variable-names))
                (funcall body-function))))
       (let ((*lazy-list-dynamic-environment* #'run-in-environment))
         ,@body))))

(defmacro lazy-list ((&rest params) &body body)
  "Creates a new lazy list.
   To return a lazy list, two additional forms are lexically bound
   within the body:
   (cont value . params) to return an element and pass the parameters
                         for generating the next element
   (finish value)        to return the last element of the list 
   (next . params )      to recall generator with new parameters without
                         generating a value

   With these forms, it is possible to return one element of a lazy list
   and to pass arguments to the next call of the body to create the next
   element. The parameter specification of lazy-list is similar to the let
   form. 

   example:
   (lazy-list ((i 0)) (when (< i 10) (cont i (1+ i))))
   i is initialized with 0, the list from 0 to 9 is returned"
  (let ((generator (gensym "GENERATOR-"))
        (next-hook (gensym "NEXT-HOOK-"))
        (var-decls (mapcar (lambda (var)
                             (if (atom var)
                                 `(,var nil)
                                 var)) params))
        (next-hook-params (gensym "NEXT-HOOK-PARAMS-")))
    `(let ((call-in-environment-function *lazy-list-dynamic-environment*))
       (labels ((,generator ,(mapcar #'car var-decls)
                  (macrolet ((cont (value &rest params)
                               `(invoke-restart :cont ,value ,@params))
                             (next (&rest params)
                               `(invoke-restart :next ,@params))
                             (finish (value)
                               `(invoke-restart :finish ,value)))
                    (labels ((,next-hook (&rest ,next-hook-params)
                               ;; For some reason, tail call
                               ;; optimization doesn't work here for
                               ;; next. That's why we need to deal with
                               ;; next using a loop.
                               (flet ((next-hook-body ,(mapcar #'car var-decls)
                                        ,@body))
                                 (loop do 
                                   (restart-case
                                       (return-from ,next-hook
                                         (apply #'next-hook-body ,next-hook-params))
                                     (:cont (value &rest params)
                                       (return-from ,generator
                                         (cons value (make-lazy-cons-elem
                                                      :generator (lambda () (apply #',generator params))))))
                                     (:next (&rest params)
                                       (setf ,next-hook-params params))
                                     (:finish (&optional (value nil value-p))
                                       (if value-p
                                           (return-from ,generator (cons value nil))
                                           (return-from ,generator nil)))
                                     (:rest (rest)
                                       (return-from ,generator rest)))))))
                      (funcall
                       call-in-environment-function
                       (lambda ()
                         (,next-hook ,@(mapcar #'car var-decls))))))))
         (,generator ,@(mapcar #'cadr var-decls))
         ;; (list (make-lazy-cons-elem
         ;;        :generator #'(lambda () (,generator ,@(mapcar #'cadr var-decls)))))
         ))))


(defun lazy-list-p (lc)
  (when (consp lc)
    (lazy-cons-elem-p (cdr (last lc)))))

(defun lazy-car (ll)
  (when (and ll (lazy-cons-elem-p (car ll)))
    (let ((new-cons (funcall (lazy-cons-elem-generator (car ll)))))
      (setf (car ll) (car new-cons)
            (cdr ll) (cdr new-cons))))
  (car ll))

(defun lazy-cdr (ll)
  (when (lazy-cons-elem-p (car ll))
    (lazy-car ll))
  (if (lazy-cons-elem-p (cdr ll))
      (setf (cdr ll) (funcall (lazy-cons-elem-generator (cdr ll))))
      (cdr ll)))

(defun lazy-mapcar (fun list-1 &rest more-lists)
  (when list-1
    (lazy-list ((lists (lambda ()
                         (cons list-1 more-lists))))
      (let ((expanded-lists (funcall lists)))
        (unless (position nil expanded-lists)
          (cont (apply fun (mapcar #'lazy-car expanded-lists))
                (lambda ()
                  (mapcar #'lazy-cdr expanded-lists))))))))

(defun lazy-mapcan (fun list-1 &rest more-lists)
  (unless (position nil (cons list-1 more-lists))
    (lazy-list ((values-generator (lambda () (apply fun (mapcar #'lazy-car (cons list-1 more-lists)))))
                (lists (cons list-1 more-lists)))
      (let ((values (when values-generator (funcall values-generator))))
        (flet ((proceed (next-values lists)
                 (if next-values
                     (cont (lazy-car next-values) (lambda () (lazy-cdr next-values)) lists)
                     (next nil lists))))
          (cond (values (proceed values lists))
                (t
                 (let* ((cdrs (mapcar #'lazy-cdr lists))
                        (cars (mapcar #'lazy-car cdrs)))
                   (unless (position nil cdrs)
                     (let ((fun-res (apply fun cars)))                       
                       (assert (listp fun-res) () "Applied fun ~a to ~a but result ~a is not a list~%" fun cars fun-res)
                       (proceed fun-res cdrs)))))))))))

(defun lazy-elt (ll index)
  (if (eql index 0)
      (lazy-car ll)
      (lazy-elt (lazy-cdr ll) (1- index))))

(defun force-ll (ll)
  (labels ((worker (ll)
             (declare (optimize (speed 3) (debug 1)))
             (when (lazy-cdr ll)
               (worker (lazy-cdr ll)))))
    (worker ll)
    ll))

(defun copy-lazy-list (src)
  (if (consp (cdr src))
      (cons (car src) (copy-lazy-list (cdr src)))
      (cons (car src) (cdr src))))

(defun lazy-filter (pred ll)
  "Creates a new lazy list containing only elements for which
   pred doesn't return NIL. When pred has a second return value which
   is not nil, the current element is the last one."
  (lazy-list ((cur-ll ll)) 
    (labels ((find-next-elem (ll)
               (multiple-value-bind (keep? last?) 
                   (funcall pred (lazy-car ll))
                 (cond ((and keep? (or last? (not (lazy-cdr ll))))
                        (finish (lazy-car ll)) )
                       (keep? 
                        (cont (lazy-car ll) (lazy-cdr ll)) )
                       ((lazy-cdr ll)
                        (find-next-elem (lazy-cdr ll)) )))))
      (find-next-elem cur-ll))))

(defun lazy-fold (fun sequence &key (initial-value nil initial-value-p))
  "Folds a lazy list, i.e. reduce for lazy lists."
  (labels ((worker (seq acc)
             (if seq
                 (worker (lazy-cdr seq) (funcall fun acc (lazy-car seq)))
                 acc)))
    (if initial-value-p
        (worker sequence initial-value)
        (worker (lazy-cdr sequence) (lazy-car sequence)))))

(defun lazy-append (l-1 &rest further-lists)
  "Appends l-1 and l-2 and returns the resulting lazy list."
  (lazy-list ((l-1 l-1)
              (rest further-lists))
    (cond ((consp l-1)
           (cont (lazy-car l-1) (lazy-cdr l-1) rest))
          (rest
           (if (consp (car rest))
               (cont (lazy-car (car rest)) (lazy-cdr (car rest)) (cdr rest))
               (next nil (cdr rest)))))))

(defun lazy-take (n l)
  (lazy-list ((list-generator (lambda () l))
              (i n))
    (when (> i 0)
      (let ((list (funcall list-generator)))
        (when list
          (cont (lazy-car list)
                (lambda () (lazy-cdr list)) (- i 1)))))))

(defun lazy-skip (n l)
  (let ((result l))
    (dotimes (i n result)
      (setf result (lazy-cdr l)))))

(defun lazy-flatten (list)
  (lazy-mapcan (lambda (elem)
                 (if (listp elem)
                     (lazy-flatten elem)
                     (list elem)))
               list))

(defun lazy-rests (list)
  "Returns the list of all cdrs of `list'"
  (lazy-list ((list list))
    (when list
      (cont list (cdr list)))))

(defmacro lazy-dolist ((var list &optional result) &body body)
  (with-gensyms (list-var)
    `(let* ((,list-var ,list)
            (,var nil))
       (block nil
         (tagbody next-loop
            (setq ,var (lazy-car ,list-var))
            ,@body
            (setq ,list-var (lazy-cdr ,list-var))
            (when ,list-var
              (go next-loop)))
         ,result))))


