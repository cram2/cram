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

(in-package :crs-tests)

(def-suite unify :in reasoning)

(in-suite unify)

(defmacro unify-test-single (name match? lhs rhs
                             &optional (test-var-values nil) (muffle-warnings nil))
  (flet ((wrap (form)
           (if muffle-warnings
               `(handler-bind ((warning #'muffle-warning))
                  ,form)
               form)))
    `(test ,name
       (let ((result (multiple-value-list ,(wrap `(unify ',lhs ',rhs)))))
         (is (= 2 (length result)))
         ,(if match?
              `(progn
                 (is-true (cadr result))
                 ,@(mapcar (lambda (test)
                             `(progn
                                (is (equal
                                     ',(cdr test)
                                     (var-value ',(var-name (car test))
                                                (car result))))
                                (is-true 
                                 (or (not (is-segvar ',(car test)))
                                     (proper-list-p
                                      (var-value ',(var-name (car test))
                                                 (car result)))))))
                           test-var-values))
              `(is (equal '(nil nil) result)))))))

(defmacro unify-test (name match? lhs rhs
                      &optional (test-var-values nil) (muffle-warnings nil))
  (flet ((name (suf)
           (format-symbol t "~a~a" name suf)))
    `(progn (unify-test-single ,(name "-1") ,match? ,lhs ,rhs
                               ,test-var-values ,muffle-warnings)
            (unify-test-single ,(name "-2") ,match? ,rhs ,lhs
                               ,test-var-values ,muffle-warnings))))

(defmacro unify-test-segvar (name match? lhs rhs &optional (test-var-values nil))
  `(unify-test ,name ,match? ,lhs ,rhs ,test-var-values t))

(defmacro unify-test-perms (name match? lhs rhs &optional (test-var-values nil))
  (let ((count 0))
    (flet ((next-name ()
             (format-symbol t "~a-PERM-~a" name (incf count))))
      `(progn
         ,@(loop for perm in (permute-all (list lhs rhs))
              collecting `(unify-test ,(next-name) ,match?
                                      ,(car perm) ,(cadr perm) ,test-var-values))))))

(defmacro def-unify-test-group (name &body tests)
  (let ((count 0)
        (name (format-symbol t "UNIFY-~a" name)))
    (flet ((next-name ()
             (format-symbol t "~a-~a" name (incf count))))
      `(progn
         (def-suite ,name :in unify)
         (in-suite ,name)
         ,@(mapcar (lambda (form)
                     (let ((fun (case (car form)
                                  (:test-perms 'unify-test-perms)
                                  (:test 'unify-test)
                                  (:test-segvar 'unify-test-segvar)
                                  (otherwise (error "Invalid test form.")))))
                       `(,fun ,(next-name) ,@(cdr form))))
                   tests)
         (in-suite unify)))))

(def-unify-test-group constant
  (:test t x x)
  (:test-perms t (a b c) (a b c)))

(def-unify-test-group constant-fail
  (:test nil x y)
  (:test nil (a b) (a b c))
  (:test nil (a b) (a))
  (:test-perms nil (a b c) (a b foo)))

(def-unify-test-group like-patmatch
  (:test-perms t (?x b) (a b) ((?x . a)))
  (:test-perms t (?x c) ((a b) c) ((?x . (a b))))
  (:test t (?x ?x) (a a) ((?x . a))))

(test unify-binds-variables
  (is (eq 'foo (var-value '?x (unify '(a b ?x) '(a b foo))))))

(def-unify-test-group like-patmatch-fail
  (:test nil (?x) a)
  (:test-perms nil (?x ?x) (a b))
  (:test-perms nil (?x ?x) (nil foo)))

(test seg-form-throws-error
  (signals simple-error (unify '!?foo 'bar))
  (signals simple-error (unify 'bar '!?foo))
  (signals simple-error (unify '(a . !?x) '(a . b)))
  (signals simple-error (unify '(a . b) '(a . !?x))))

(def-unify-test-group seg-form-like-patmatch
  (:test-segvar t (a !?x d) (a b c d)
                ((!?x . (b c))))
  (:test-segvar t (foo !?x foo !?y !?z bar) (foo foo bar bar foo bar)
         ((!?x . nil)
          (!?y . nil)
          (!?z . (bar bar foo))))
  (:test-segvar t (!?x ?y) (a b c)
                ((!?x . (a b)) (?y . c)))
  (:test-segvar t (!?x ?x) (a b (a b))
                ((!?x . (a b)))))

(def-unify-test-group seg-form-like-patmatch-fail
  (:test-segvar nil (!?x d) (a b c))
  (:test-segvar nil (a (b !?x ?y) c) (a (b) c))
  (:test-segvar nil (!?x) a)
  (:test-segvar nil (a !?x) (a b . c)))

(def-unify-test-group seg-var-proper-list
  (:test-segvar nil (a !?x) (a b . c)))

(def-unify-test-group simple
  (:test-perms t (?x b) (a ?y)
               ((?x . a) (?y . b)))
  (:test t ?x ?y)
  (:test-perms t (?x ?y c) (a ?x c)
               ((?x . a) (?y . a))))

(def-unify-test-group unusual
  (:test-perms t (?x ?y ?x) ((?y) a (a)) ((?x . (a)) (?y . a)))
  (:test-perms t (a (b ?x) ?_) (a (?y c) ?x)
               ((?x . c) (?y . b)))
  (:test t (a (b ?x) . ?_) (a (?y c) ?x ?y ?z)
         ((?x . c) (?y . b)))
  (:test-segvar t (a (b ?x) . (!?_)) (a (?y c) . (?x ?y ?z))
         ((?x . c) (?y . b))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Unify and segvar issues
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; (unify '(?x !?x !y) '((b c d) b c d e f)) -> nil (should unify)
;; 
;; CRS-TESTS> (unify '(a !?x d) '(?y d))
;; ((?X) (?Y . A))
;; T
;; CRS-TESTS> (unify '(a !?x d) '(?y foo d))
;; ((?X FOO) (?Y . A))
;; T
;; CRS-TESTS> (unify '(a !?x d) '(?y ?z d))
;; ((?X ?Z) (?Y . A))
;; T
;; CRS-TESTS> (unify '(a !?x d ?z (e)) '(?y ?z d e ?x))
;; NIL
;; NIL