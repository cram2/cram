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

(def-suite auxiliary :in utilities)

(in-suite auxiliary)

(test valid-vars
  (is-true (is-var '?x))
  (is-true (is-var '?foo))
  (is-true (is-var '?))
  (is-true (is-var '??))
  (is-true (is-var '|? |))
  (is-true (is-var :?bar))
  (is-true (is-var (gensym "?"))))

(test invalid-vars
  (is-false (is-var '()))
  (is-false (is-var '!))
  (is-false (is-var '!?foo))
  (is-false (is-var 'x))
  (is-false (is-var 'b?))
  (is-false (is-var '(a b c)))
  (is-false (is-var '(?x))))

(test unnamed-var
  (is-true (is-unnamed-var '?_))
  (is-true (is-unnamed-var 'cut:?_))
  (is-true (is-unnamed-var (var-name '!?_))))

(test not-unnamed-var
  (is-false (is-unnamed-var 'cram-test-utilities::?_))
  (is-false (is-unnamed-var '?x))
  (is-false (is-unnamed-var '!?_)))

(test valid-segvars
  (is-true (is-segvar '!?foo))
  (is-true (is-segvar '!?)))

(test invalid-segvars
  (is-false (is-segvar '?foo))
  (is-false (is-segvar '!))
  (is-false (is-segvar 'x?))
  (is-false (is-segvar '(a b c))))

(test valid-segforms
  (is-true (is-segform '(!?a b c)))
  (is-true (is-segform '(!?foo))))

(test invalid-segforms
  (is-false (is-segform '(?a b c)))
  (is-false (is-segform '(a b c)))
  (is-false (is-segform '()))
  (is-false (is-segform '?x))
  (is-false (is-segform '!?x)))

(test var-name-works
  (is (eq '?foo (var-name '?foo)))
  (is (eq '?bar (var-name '!?bar)))
  (is (eq 'cram-test-utilities::?baz
          (var-name 'cram-test-utilities::!?baz)))
  (is-true (is-unnamed-var (var-name '!?_))))

(test var-name-signals-error
  (signals simple-error (var-name 'foo))
  (signals simple-error (var-name '(?foo))))

(test gen-var-is-var
  (is-true (is-var (gen-var)))
  (is-true (is-var (gen-var "?")))
  (is-true (is-var (gen-var "?FOO"))))

(test gen-var-signals-error
  (signals simple-error (gen-var "FOO"))
  (signals simple-error (gen-var :?foo)))

(test gen-vars-unique
  (flet ((every-unique (l)
           (every #'(lambda (x)
                      (= 1 (count x l :test #'eq)))
                  l)))
    (is-true (every-unique (loop repeat 20 collecting (gen-var))))))

(test is-genvar
  (is-true (is-genvar (gen-var)))
  (is-false (is-genvar '?foo))
  (signals simple-error (is-genvar 23)))

(test add-bdgs
  (is (equal :foo (cdr (assoc '?x (add-bdg '?x :foo nil)))))
  (is (equal '(nil t)
             (multiple-value-list
              (add-bdg '?_ :foo nil))))
  (is (equal '(nil nil)
             (multiple-value-list
              (add-bdg '?x :foo
                       (add-bdg '?x :bar nil)))))
  (is-true (cadr (multiple-value-list
                  (add-bdg '?x '(foo bar)
                           (add-bdg '?x '(foo bar) nil))))))

(test substitute-vars
  (let ((bdgs (add-bdg '?x '(a b ?y)
                       (add-bdg '?y 'foo nil))))
    (is (equal '(a b foo)
               (substitute-vars '?x bdgs)))
    (is (equal '((a b foo) foo ?z)
               (substitute-vars '(?x ?y ?z) bdgs)))
    (is (equal 'foo
               (substitute-vars 'foo bdgs)))))

#+nil
(test substitute-segvars
  (let ((bdgs (add-bdg '?x '(a b ?y)
                       (add-bdg '?y 'foo nil))))
    (is (equal '(a (a b foo) b)
               (substitute-vars '(a !?x b))))
    (is (equal 'nil
               (substitute-vars '(a !?y b))))))

;;; TODO:  handle (var-value '!?x bdg) when ?x is bound to non-list
(test var-value
  (let ((bdgs (add-bdg '?x '(a b ?y)
                       (add-bdg '?y 'foo nil)))
        (pat '(a b c)))
    (is (equal pat (var-value pat bdgs)))
    (is (equal 'foo (var-value '?y bdgs)))
    (is (equal '?z (var-value '?z bdgs)))
    (is (equal '(?x ?y) (var-value '(?x ?y) bdgs)))))

(test rename-vars
  (let* ((pat '(foo ?bar (baz . ?quux) ?_ :quax))
         (renamed (rename-vars pat))
         (sub (substitute-vars pat '((?bar . 1) (?quux . 2) (?_ . 3))))
         (binds (pat-match renamed sub)))
    (is (= 2 (length binds)))
    (is-true (every #'null (mapcar #'symbol-package (vars-in binds))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Patmatch
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(def-suite patmatch :in utilities)

(in-suite patmatch)

(defmacro pat-match-test (match? pat seq &optional (test-binds nil))
  `(progn
     (,(if match? 'is-true 'is-false)
       (pat-match-p ',pat ',seq))
     (let ((result (multiple-value-list (pat-match ',pat ',seq))))
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
                         test-binds))
            `(is (equal '(nil nil) result))))))

(test pat-match-constant
  (pat-match-test t x x)
  (pat-match-test t (a b c) (a b c)))

(test pat-match-constant-fail
  (pat-match-test nil x y)
  (pat-match-test nil (a b c) (a b foo))
  (pat-match-test nil (a b) (a b c))
  (pat-match-test nil (a b) (a)))

(test pat-match-simple
  (pat-match-test t (?x b) (a b) ((?x . a)))
  (pat-match-test t (?x c) ((a b) c) ((?x . (a b))))
  (pat-match-test t (?x ?x) (a a) ((?x . a)))
  (is (eq 'foo (var-value '?x (pat-match '(a b ?x) '(a b foo))))))

(test pat-match-simple-fail
  (pat-match-test nil (?x) a)
  (pat-match-test nil (?x ?x) (a b))
  (pat-match-test nil (?x ?x) (nil foo)))

(test pat-match-no-vars-on-left-side
  (signals simple-error (pat-match '(a ?x c) '(a b ?y)))
  (signals simple-error (pat-match '(a b c) '(!?y))))

(test pat-match-add-value-bug
  ;; Used to fail. Now fixed.
  (pat-match-test nil (?x ?x) (nil foo)))

(test pat-match-seg-form-invalid-place
  (signals simple-error (pat-match '!?foo 'bar))
  (signals simple-error (pat-match '(a . !?x) '(a . b))))

(test pat-match-seg-form
  (pat-match-test t (a !?x d) (a b c d)
                  ((?x . (b c))))
  (pat-match-test t (foo !?x foo !?y !?z bar) (foo foo bar bar foo bar)
                  ((?x . nil)
                   (?y . nil)
                   (?z . (bar bar foo))))
  (pat-match-test t (!?x ?y) (a b c)
                  ((?x . (a b)) (?y . c)))
  (pat-match-test t (!?x ?x) (a b (a b))
                  ((?x . (a b)))))

(test pat-match-seg-form-fail
  (pat-match-test nil (!?x d) (a b c))
  (pat-match-test nil (a (b !?x ?y) c) (a (b) c))
  (pat-match-test nil (!?x) a)
  (pat-match-test nil (?x !?x) (a a)))

(test pat-match-seg-var-proper-list
  (pat-match-test nil (a !?x) (a b . c)))

(test vars-in
  (is (symbol-set-equal '(?x) (vars-in '?x)))
  (is (symbol-set-equal '(?x) (vars-in '(a ?x b))))
  (is (symbol-set-equal '(?x ?y) (vars-in '(?x (a ?y) b))))
  (is (symbol-set-equal '(?x ?y) (vars-in '(?y (a ?x) b)))))

(test vars-in-empty
  (is (symbol-set-equal '() (vars-in 'a)))
  (is (symbol-set-equal '() (vars-in '(a b c)))))

(test vars-in-duplicates-removed
  (is (symbol-set-equal '(?x) (vars-in '(?x ?x))))
  (is (symbol-set-equal '(?x ?y) (vars-in '(a (b . ?y) . (?y ?x foo))))))

(test with-vars-bound
  (with-vars-bound (?x ?y) (pat-match '(?x ?y) '(foo bar))
    (is (equal '(foo bar) (list ?x ?y)))))

(test with-pat-vars-bound
  (with-pat-vars-bound (?x b ?y) '(a b c)
    (is (equal '(a c) (list ?x ?y))))
  (signals simple-error (with-pat-vars-bound (?x b ?y) '(foo bar baz)
                          (declare (ignore ?x ?y))))
  (signals simple-warning (with-pat-vars-bound (a b) '(a b))))

(test is-bound-ground
  (let* ((bdgs1 (pat-match '(?x ?y ?z) '(a (b c) nil)))
         (bdgs2 (add-bdg '?bar '(?free)
                         (add-bdg '?foo '(?x ?y) bdgs1))))
    ;; bdgs1
    (is-true (is-bound '?x bdgs1))
    (is-true (is-ground '?x bdgs1))
    (is-true (is-bound '?y bdgs1))
    (is-true (is-ground '?y bdgs1))
    (is-true (is-bound '?z bdgs1))
    (is-true (is-ground '?z bdgs1))
    (is-false (is-bound '?foo bdgs1))
    (is-false (is-ground '?foo bdgs1))
    (is-true (is-bound 'nil bdgs1))
    (is-true (is-ground 'nil bdgs1))
    (is-true (is-bound '(a b) bdgs1))
    (is-true (is-ground '(a b) bdgs1))
    (is-true (is-bound '(?x) bdgs1))
    (is-true (is-ground '(?x) bdgs1))
    (is-true (is-bound '(?foo) bdgs1))
    (is-false (is-ground '(?foo) bdgs1))
    ;; bdgs2
    (is-true (is-bound '(?foo) bdgs2))
    (is-true (is-ground '(?foo) bdgs2))
    (is-true (is-bound '(?bar) bdgs2))
    (is-false (is-ground '(?bar) bdgs2))
    (is-true (is-bound '(?foo ?bar) bdgs2))
    (is-false (is-ground '(?foo ?bar) bdgs2))))
