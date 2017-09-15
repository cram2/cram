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

(in-package :prolog-tests)

(define-test constant
  (multiple-value-bind (result matched?)
      (unify 'x 'x)
    (assert-eq result nil)
    (assert-true matched?)))

(define-test constant-fail
  (assert-false (nth-value 1 (unify 'x 'y)))
  (assert-false (nth-value 1 (unify '(a b) '(a b c))))
  (assert-false (nth-value 1 (unify '(a b) '(a))))
  (assert-false (nth-value 1 (unify '(a b c) '(a b foo)))))

(define-test like-patmatch
  (multiple-value-bind (result matched?)
      (unify '(?x b) '(a b))
    (assert-equality #'bindings-equal result '((?x . a)))
    (assert-true matched?)))

(define-test like-patmatch-fail
  (assert-false (nth-value 1 (unify '(?x) 'a)))
  (assert-false (nth-value 1 (unify '(?x ?x) '(a b))))
  (assert-false (nth-value 1 (unify '(?x ?x) '(nil foo)))))

(define-test seg-form-errors
  (assert-error 'simple-error (unify '!?foo 'bar))
  (assert-error 'simple-error (unify 'bar '!?foo))
  (assert-error 'simple-error (unify '(a . !?x) '(a . b)))
  (assert-error 'simple-error (unify '(a . b) '(a . !?x))))

(define-test seg-form-like-patmatch
  (handler-bind ((warning #'muffle-warning))
    (assert-equality
     #'bindings-equal '((?x b c)) (unify '(a !?x d) '(a b c d)))
    (assert-equality
     #'bindings-equal
     '((?x . nil) (?y . nil) (?z . (bar bar foo)))
     (unify '(foo !?x foo !?y !?z bar) '(foo foo bar bar foo bar)))
    (assert-equality
     #'bindings-equal
     '((?x . (a b)) (?y . c))
     (unify '(!?x ?y) '(a b c)))
    (assert-equality
     #'bindings-equal '((?x . (a b))) (unify '(!?x ?x) '(a b (a b))))))

(define-test seg-form-like-patmatch-fail
  (handler-bind ((warning #'muffle-warning))
    (assert-false (nth-value 1 (unify '(!?x d) '(a b c))))
    (assert-false (nth-value 1 (unify '(a (b !?x ?y) c) '(a (b) c))))
    (assert-false (nth-value 1 (unify '(!?x) 'a)))
    (assert-false (nth-value 1 (unify '(a !?x) '(a b . c))))))

(define-test seg-var-proper-list
  (handler-bind ((warning #'muffle-warning))
    (assert-false (nth-value 1 (unify '(a !?x) '(a b . c))))))

(define-test simple-unification
  (assert-equality #'bindings-equal
                   '((?x . a) (?y . b))
                   (unify '(?x b) '(a ?y)))
  (assert-equality #'bindings-equal
                   '((?x . ?y))
                   (unify '?x '?y))
  (assert-equality #'bindings-equal
                   '((?x . a) (?y . a))
                   (unify '(?x ?y c) '(a ?x c))))

(define-test unusual-unification
  (assert-equality
   #'bindings-equal
   '((?x . (?y)) (?y . a)) (unify '(?x ?y ?x) '((?y) a (a))))
  (assert-equality #'bindings-equal
                   '((?x . c) (?y . b))
                   (unify '(a (b ?x) ?_) '(a (?y c) ?x)))
  (assert-equality #'bindings-equal
                   '((?x . c) (?y . b))
                   (unify '(a (b ?x) . ?_) '(a (?y c) ?x ?y ?z)))
  (handler-bind ((warning #'muffle-warning))
    (assert-equality #'bindings-equal
                     '((?x . c) (?y . b))
                     (unify '(a (b ?x) . (!?_)) '(a (?y c) . (?x ?y ?z))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Unify and segvar issues
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; (unify '(?x !?x !y) '((b c d) b c d e f)) -> nil (should unify)
;; 
;; PROLOG-TESTS> (unify '(a !?x d) '(?y d))
;; ((?X) (?Y . A))
;; T
;; PROLOG-TESTS> (unify '(a !?x d) '(?y foo d))
;; ((?X FOO) (?Y . A))
;; T
;; PROLOG-TESTS> (unify '(a !?x d) '(?y ?z d))
;; ((?X ?Z) (?Y . A))
;; T
;; PROLOG-TESTS> (unify '(a !?x d ?z (e)) '(?y ?z d e ?x))
;; NIL
;; NIL
