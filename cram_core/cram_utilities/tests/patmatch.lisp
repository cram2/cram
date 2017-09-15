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

(in-package :cut-tests)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Auxiliary
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define-test valid-vars
  (assert-true (is-var '?x))
  (assert-true (is-var '?foo))
  (assert-true (is-var '?))
  (assert-true (is-var '??))
  (assert-true (is-var '|? |))
  (assert-true (is-var :?bar))
  (assert-true (is-var (gensym "?"))))

(define-test invalid-vars
  (assert-false (is-var '()))
  (assert-false (is-var '!))
  (assert-false (is-var '!?foo))
  (assert-false (is-var 'x))
  (assert-false (is-var 'b?))
  (assert-false (is-var '(a b c)))
  (assert-false (is-var '(?x))))

(define-test unnamed-var
  (assert-true (is-unnamed-var '?_))
  (assert-true (is-unnamed-var 'cut:?_))
  (assert-true (is-unnamed-var (var-name '!?_))))

(define-test not-unnamed-var
  (assert-false (is-unnamed-var '?x))
  (assert-false (is-unnamed-var '!?_)))

(define-test valid-segvars
  (assert-true (is-segvar '!?foo))
  (assert-true (is-segvar '!?)))

(define-test invalid-segvars
  (assert-false (is-segvar '?foo))
  (assert-false (is-segvar '!))
  (assert-false (is-segvar 'x?))
  (assert-false (is-segvar '(a b c))))

(define-test valid-segforms
  (assert-true (is-segform '(!?a b c)))
  (assert-true (is-segform '(!?foo))))

(define-test invalid-segforms
  (assert-false (is-segform '(?a b c)))
  (assert-false (is-segform '(a b c)))
  (assert-false (is-segform '()))
  (assert-false (is-segform '?x))
  (assert-false (is-segform '!?x)))

(define-test var-name-works
  (assert-eq '?foo (var-name '?foo))
  (assert-eq '?bar (var-name '!?bar))
  (assert-true (is-unnamed-var (var-name '!?_))))

(define-test var-name-signals-error
  (assert-error 'simple-error (var-name 'foo))
  (assert-error 'simple-error (var-name '(?foo))))

(define-test gen-var-is-var
  (assert-true (is-var (gen-var)))
  (assert-true (is-var (gen-var "?")))
  (assert-true (is-var (gen-var "?FOO"))))

(define-test gen-var-signals-error
  (assert-error 'simple-error (gen-var "FOO"))
  (assert-error 'simple-error (gen-var :?foo)))

(define-test gen-vars-unique
  (let ((vars (loop repeat 20 collecting (gen-var))))
    (dolist (var vars)
      (assert-eql 1 (count var vars)))))

(define-test is-genvar
  (assert-true (is-genvar (gen-var)))
  (assert-false (is-genvar '?foo))
  (assert-error 'simple-error (is-genvar 23)))

(define-test add-bdgs
  (assert-equal :foo (var-value '?x (add-bdg '?x :foo nil)))
  (assert-equal '(nil t) (multiple-value-list (add-bdg '?_ :foo nil)))
  (assert-equal '(nil nil) (multiple-value-list
                            (add-bdg '?x :foo
                                     (add-bdg '?x :bar nil))))
  (assert-true (nth-value 1 (add-bdg '?x '(foo bar) (add-bdg '?x '(foo bar) nil)))))

(define-test substitute-vars
  (let ((bdgs (add-bdg '?x '(a b ?y)
                       (add-bdg '?y 'foo nil))))
    (assert-equal '(a b foo) (substitute-vars '?x bdgs))
    (assert-equal '((a b foo) foo ?z) (substitute-vars '(?x ?y ?z) bdgs))
    (assert-equal 'foo (substitute-vars 'foo bdgs))))

#+nil
(test substitute-segvars
  (let ((bdgs (add-bdg '?x '(a b ?y)
                       (add-bdg '?y 'foo nil))))
    (is (equal '(a (a b foo) b)
               (substitute-vars '(a !?x b))))
    (is (equal 'nil
               (substitute-vars '(a !?y b))))))

;;; TODO:  handle (var-value '!?x bdg) when ?x is bound to non-list
(define-test var-value
  (let ((bdgs (add-bdg '?x '(a b ?y)
                       (add-bdg '?y 'foo nil))))
    (assert-equal '(a b c) (var-value '(a b c) bdgs))
    (assert-equal 'foo (var-value '?y bdgs))
    (assert-equal '?z (var-value '?z bdgs))
    (assert-equal '(?x ?y) (var-value '(?x ?y) bdgs))))

(define-test rename-vars
  (let* ((pat '(foo ?bar (baz . ?quux) ?_ :quax))
         (renamed (rename-vars pat))
         (sub (substitute-vars pat '((?bar . 1) (?quux . 2) (?_ . 3))))
         (binds (pat-match renamed sub)))
    (assert-eql 2 (length binds))
    (assert-true (every #'null (mapcar #'symbol-package (vars-in binds))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Patmatch
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define-test pat-match-constant
  (assert-equal '(nil t) (multiple-value-list (pat-match 'x 'x)))
  (assert-equal '(nil t) (multiple-value-list (pat-match '(a b c) '(a b c)))))

(define-test pat-match-constant-fail
  (assert-equal '(nil nil) (multiple-value-list (pat-match 'x 'y)))
  (assert-equal '(nil nil) (multiple-value-list (pat-match '(a b c) '(a b foo))))
  (assert-equal '(nil nil) (multiple-value-list (pat-match '(a b) '(a b c))))
  (assert-equal '(nil nil) (multiple-value-list (pat-match '(a b) '(a)))))

(define-test pat-match-simple
  (assert-equality #'bindings-equal
                   '((?x . a))
                   (pat-match '(?x b) '(a b)))
  (assert-equality #'bindings-equal
                   '((?x . (a b)))
                   (pat-match '(?x c) '((a b) c)))
  (assert-equality #'bindings-equal
                   '((?x . a))
                   (pat-match '(?x ?x) '(a a)))
  (assert-eq 'foo (var-value '?x (pat-match '(a b ?x) '(a b foo)))))

(define-test pat-match-simple-fail
  (assert-equal '(nil nil) (multiple-value-list (pat-match '(?x) 'a)))
  (assert-equal '(nil nil) (multiple-value-list (pat-match '(?x ?x) '(a b))))
  (assert-equal '(nil nil) (multiple-value-list (pat-match '(?x ?x) '(nil foo)))))

(define-test pat-match-no-vars-on-left-side
  (assert-error 'simple-error (pat-match '(a ?x c) '(a b ?y)))
  (assert-error 'simple-error (pat-match '(a b c) '(!?y))))

(define-test pat-match-add-value-bug
  (assert-equal '(nil nil) (multiple-value-list (pat-match '(?x ?x) '(nil foo)))))

(define-test pat-match-seg-form-invalid-place
  (assert-error 'simple-error (pat-match '!?foo 'bar))
  (assert-error 'simple-error (pat-match '(a . !?x) '(a . b))))

(define-test pat-match-seg-form
  (assert-equality #'bindings-equal
                   '((?x . (b c)))
                   (pat-match '(a !?x d) '(a b c d)))
  (assert-equality #'bindings-equal
                   '((?x . nil)
                     (?y . nil)
                     (?z . (bar bar foo)))
                   (pat-match '(foo !?x foo !?y !?z bar) '(foo foo bar bar foo bar)))
  (assert-equality #'bindings-equal
                   '((?x . (a b)) (?y . c))
                   (pat-match '(!?x ?y) '(a b c)))
  (assert-equality #'bindings-equal
                   '((?x . (a b)))
                   (pat-match '(!?x ?x) '(a b (a b)))))

(define-test pat-match-seg-form-fail
  (assert-false (pat-match '(!?x d) '(a b c)))
  (assert-false (pat-match '(a (b !?x ?y) c) '(a (b) c)))
  (assert-false (pat-match '(!?x) 'a))
  (assert-false (pat-match '(?x !?x) '(a a))))

(define-test pat-match-seg-var-proper-list
  (assert-false (pat-match '(a !?x) '(a b . c))))

(define-test vars-in
  (assert-equality #'set-equal '(?x) (vars-in '?x))
  (assert-equality #'set-equal '(?x) (vars-in '(a ?x b)))
  (assert-equality #'set-equal '(?x ?y) (vars-in '(?x (a ?y) b)))
  (assert-equality #'set-equal '(?x ?y) (vars-in '(?y (a ?x) b))))

(define-test vars-in-empty
  (assert-eq '() (vars-in 'a))
  (assert-eq '() (vars-in '(a b c))))

(define-test vars-in-duplicates-removed
  (assert-eql 1 (count '?x (vars-in '(?x ?x))))
  (let ((vars (vars-in '(a (b . ?y) . (?y ?x foo)))))
    (assert-eql 1 (count '?x vars))
    (assert-eql 1 (count '?y vars))))

(define-test with-vars-bound
  (with-vars-bound (?x ?y) (pat-match '(?x ?y) '(foo bar))
    (assert-eq 'foo ?x)
    (assert-eq 'bar ?y)))

(define-test with-pat-vars-bound
  (with-pat-vars-bound (?x b ?y) '(a b c)
    (assert-eq 'a ?x)
    (assert-eq 'c ?y))
  (assert-error 'simple-error (with-pat-vars-bound (?x b ?y) '(foo bar baz)
                                (declare (ignore ?x ?y))))
  (assert-error 'simple-warning (with-pat-vars-bound (a b) '(a b))))

(define-test is-bound-ground
  (let* ((bdgs1 (pat-match '(?x ?y ?z) '(a (b c) nil)))
         (bdgs2 (add-bdg '?bar '(?free)
                         (add-bdg '?foo '(?x ?y) bdgs1))))
    ;; bdgs1
    (assert-true (is-bound '?x bdgs1))
    (assert-true (is-ground '?x bdgs1))
    (assert-true (is-bound '?y bdgs1))
    (assert-true (is-ground '?y bdgs1))
    (assert-true (is-bound '?z bdgs1))
    (assert-true (is-ground '?z bdgs1))
    (assert-false (is-bound '?foo bdgs1))
    (assert-false (is-ground '?foo bdgs1))
    (assert-true (is-bound 'nil bdgs1))
    (assert-true (is-ground 'nil bdgs1))
    (assert-true (is-bound '(a b) bdgs1))
    (assert-true (is-ground '(a b) bdgs1))
    (assert-true (is-bound '(?x) bdgs1))
    (assert-true (is-ground '(?x) bdgs1))
    (assert-true (is-bound '(?foo) bdgs1))
    (assert-false (is-ground '(?foo) bdgs1))
    ;; bdgs2
    (assert-true (is-bound '(?foo) bdgs2))
    (assert-true (is-ground '(?foo) bdgs2))
    (assert-true (is-bound '(?bar) bdgs2))
    (assert-false (is-ground '(?bar) bdgs2))
    (assert-true (is-bound '(?foo ?bar) bdgs2))
    (assert-false (is-ground '(?foo ?bar) bdgs2))))
