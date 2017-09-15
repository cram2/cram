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

(in-package :prolog-tests)

(define-test and-handler
  (assert-equality #'solutions-equal
                   '(((?x . a) (?y . a)))
                   (force-ll
                    (prolog '(and (== ?x a) (== ?x a) (== ?x ?y)))))
  (assert-equal '(nil) (force-ll (prolog '(and (and) (and)))))
  (assert-false (prolog '(and (fail))))
  (assert-false (prolog '(and (and) (and) (fail))))
  (assert-equal '(nil) (force-ll (prolog '(and)))))

(define-test or-handler
  (assert-false (prolog '(or)))
  (assert-equal '(nil) (force-ll
                        (prolog '(or (== a a) (== b a) (== a b)))))
  (assert-equal '(nil) (force-ll
                        (prolog '(or (fail) (and)))))
  (assert-false (prolog '(or (fail) (fail))))
  (assert-equality #'solutions-equal
                   '(((?x . a))
                     ((?x . b)))
                   (force-ll (prolog `(or (== ?x a) (== ?x b))))))

(define-test not-handler
  (assert-equal '(nil) (force-ll (prolog '(not (fail)))))
  (assert-false (prolog '(not (and)))))

(define-test lisp-fun
  (assert-equality #'solutions-equal
                   '(((?x . (a b c))))
                   (force-ll
                    (prolog '(and
                              (== (a b c) ?x)
                              (lisp-fun append (a b) (c) ?x)))))
  (assert-equality #'solutions-equal
                   '(((?x . 3)))
                   (force-ll (prolog '(lisp-fun + 1 2 ?x)))))

(define-test lisp-pred
  (assert-equal '(nil) (force-ll (prolog '(lisp-pred symbolp a))))
  (assert-false (prolog '(lisp-pred atom (a b))))
  (assert-equality #'solutions-equal
                   '(((?x . (foo bar))))
                   (force-ll
                    (prolog '(and
                              (== ?x (foo bar))
                              (lisp-pred equal (foo bar) ?x))))))

(define-test bound-handler
  (assert-false (prolog '(bound ?x)))
  (assert-equal '(nil) (force-ll (prolog '(bound a))))
  (assert-equality #'solutions-equal
                   '(((?x . a)))
                   (force-ll
                    (prolog '(and
                              (== ?x a)
                              (bound ?x)))))
  (assert-false (prolog '(and (== ?x ?y) (bound ?x))))
  (assert-equality #'solutions-equal
                   '(((?x . (?y))))
                   (force-ll (prolog '(and (== ?x (?y)) (bound ?x))))))

(define-test ground-handler
  (assert-false (prolog '(ground ?x)))
  (assert-equal '(nil) (force-ll (prolog '(ground a))))
  (assert-equality #'solutions-equal
                   '(((?x . a))) (force-ll (prolog '(and (== ?x a) (ground ?x)))))
  (assert-false (prolog '(and (== ?x ?y) (ground ?x))))
  (assert-false (prolog '(and (== ?x (?y)) (ground ?x)))))

(define-test findall-handler
  (assert-equality (rcurry #'solutions-equal :test #'lazy-lists-equal)
                   '(((?y . (1 2 3))))
                   (force-ll (prolog '(findall ?x (member ?x (1 2 3)) ?y))))
  (assert-equality (rcurry #'solutions-equal :test #'lazy-lists-equal)
                   '(((?first . ((1 2 3) (1 11)))))
                   (force-ll (prolog '(and (== ?x (1 2 3))
                                       (findall (?x (?y ?z)) (and (member ?y ?x )
                                                              (lisp-fun + ?y 10 ?z))
                                        ?result)
                                       (== (?first . ?_) ?result)))))
  (assert-equality (rcurry #'solutions-equal :test #'lazy-lists-equal)
                   '(((?result . ())))
                   (force-ll (prolog '(findall () (fail) ?result)))))

(define-test forall-handler
  (assert-equal
   '(nil) (force-ll (prolog '(forall (member ?x (1 2 3)) (> ?x 0)))))
  (assert-equal
   '(nil) (force-ll (prolog '(forall (and (member ?x (1 2 3)) (> ?x 3)) (fail)))))
  (assert-equality #'solutions-equal
                   '(((?t . integer)))
                   (force-ll
                    (prolog '(and
                              (== ?t integer)
                              (forall (or (== ?x 42) (== ?x 23)) (lisp-pred typep ?x ?t)))))))

(define-test filter-bindings
  (assert-equality #'solutions-equal
                   '(((?a . 1) (?b . 2)))
                   (force-ll
                    (prolog '(filter-bindings (?a ?b)
                              (== (?a ?b ?c ?d) (1 2 3 4))))))
  (assert-equality #'solutions-equal
                   '(((?b . :new)))
                   (force-ll
                    (prolog '(and
                              (== ?b 42)
                              (filter-bindings ()
                               (> ?b 23))
                              (== ?b :new)))))
  (assert-false (prolog '(filter-bindings () (fail)))))

(define-test prolog-unification
  (assert-equality #'solutions-equal
                   '(((?x . a)))
                   (force-ll (prolog '(== ?x a))))
  (assert-equality #'solutions-equal
                   '(((?x . a) (?y . b)))
                   (force-ll (prolog '(== (?x b) (a ?y)))))
  (assert-false (prolog '(== (?x b) (a a)))))

(define-test prolog-binary-predicates
  (assert-equal '(nil) (force-ll (prolog '(< 1 2))))
  (assert-equal '(nil) (force-ll (prolog '(> 1 0))))
  (assert-equal '(nil) (force-ll (prolog '(>= 0 0))))
  (assert-equal '(nil) (force-ll (prolog '(<= 0 0))))
  (assert-false (prolog '(< 3 2)))
  (assert-false (prolog '(> 0 1)))
  (assert-false (prolog '(>= 0 1)))
  (assert-false (prolog '(<= 1 0))))

(define-test prolog-member
  (assert-equality #'solutions-equal
                   '(((?x . a)) ((?x . b)) ((?x . c)))
                   (force-ll (prolog '(member ?x (a b c)))))
  (assert-false (prolog '(member ?x nil))))

(defclass dummy ()
  ((a :initform :a)))

(define-test prolog-clos-utils
  (assert-false (prolog '(instance-of ?t ?o)))
  (assert-equality #'solutions-equal
                   '(((?a . :a)))
                   (force-ll
                    (prolog '(and
                              (instance-of dummy ?obj)
                              (get-slot-value ?obj a ?a)))))
  (assert-equality #'solutions-equal
                   '(((?a . :a)))
                   (force-ll
                    (prolog '(and
                              (instance-of dummy ?obj)
                              (instance-of dummy ?obj)
                              (== ?a :a)))))
  (assert-equality #'solutions-equal
                   '(((?type . dummy)))
                   (force-ll
                    (prolog '(and
                              (instance-of dummy ?obj)
                              (instance-of ?type ?obj)))))
  (assert-false (prolog '(and (instance-of dummy ?obj)
                          (get-slot-value ?obj a :b))))
  (assert-equality #'solutions-equal
                   '(((?b . :b)))
                   (force-ll
                    (prolog '(and
                              (instance-of dummy ?obj)
                              (slot-value ?obj a :b)
                              (slot-value ?obj a ?b))))))

;; Defining facts like this actually modifies the state of the software, since
;; it adds a fact group and fact. A test suite should not do that. However for
;; now this is the easiest solution and since we are in a seperate package,
;; there should be no name clashes.

(def-fact-group prolog-tests (fact-extenable)
  
  (<- (fact-1 ?x ?y)
    (== ?y (1 2))
    (member ?x ?y))

  (<- (fact-extendable ?x)
    (== ?x 1)))

(def-fact-group prolog-tests-2 (fact-extendable)

  (<- (fact-extendable ?y)
    (== ?y 2)))

(define-test prolog-fact-1
  (assert-equality #'solutions-equal
                   '(((?a . 1) (?b 1 2))
                     ((?a . 2) (?b 1 2)))
                   (force-ll (prolog '(fact-1 ?a ?b)))))

(define-test prolog-fact-extendable
  (assert-equality #'solutions-equal
                   '(((?foo . 1)) ;; this tests for correct order of answers as well
                     ((?foo . 2)))
                   (force-ll (prolog '(fact-extendable ?foo)))))

(define-test prolog-cut
  (assert-equality #'solutions-equal
                   '(((?x . 1)))
                   (force-ll (prolog '(and (member ?x (1 2 3)) (cut)))))
  (assert-equality #'solutions-equal
                   '(((?x . 1)) nil)
                   (force-ll (prolog '(or (== ?x 1) (cut)))))
  (assert-equality #'solutions-equal
                   '(((?x . 1)) ((?x . 1)))
                   (force-ll (prolog '(and (== ?x 1)
                                       (or (== ?x 1)
                                        (== ?x 2)
                                        (cut)
                                        (== ?y 1))))))  
  (assert-equality #'solutions-equal
                   '(((?x . 1))
                     ((?x . 2)))
                   (force-ll (prolog '(and (member ?x (1 2 3))
                                       (-> (== ?x 2)
                                        (cut)
                                        (true))))))
  (assert-equality #'solutions-equal
                   '(((?x . 1))
                     ((?x . 2)))
                   (force-ll (prolog '(or
                                       (and (member ?x (1 2 3))
                                        (-> (== ?x 2)
                                         (cut)
                                         (true)))
                                       (== ?z 1)))))
  (assert-equality #'solutions-equal
                   '(((?x . 1)) ((?x . 2)) ((?x . 3)) ((?y . 1)))
                   (force-ll (prolog '(or
                                       (member ?x (1 2 3))
                                       (and (member ?y (1 2 3)) (cut))
                                       (member ?x (1 2 3)))))))

(def-fact-group prolog-bug ()

  (<- (fact-rename ?f)
    (== ?x (1 2))
    (== ?x (?f . ?_)))
  
  (<- (bug-1-test ?foo) ; see explanation below in the test case
    (== ?foo 23)
    (and
     (== ?bar :a)
     (not (== ?foo :foo))
     (<= ?foo 42)))

  (<- (bug-2-test ?foo)
    (or (== ?foo 10) (== ?foo 20))))

(define-test rename-variables-bug
  (assert-equality #'solutions-equal
                   '(((?x . 1)))
                   (force-ll (prolog '(fact-rename ?x))))
  (assert-equality #'solutions-equal
                   '(((?y . 1)))
                   (force-ll (prolog '(fact-rename ?y)))))

;; The following tests for an annoying bug which was in the interaction of
;; AND, NOT, PROLOG, FILTER-BINDINGS, and ADD-BDG. The behaviour was, that in
;; the BUG-1 fact in the last goal (<= ?foo 42) an error would be raised,
;; because ?foo would be unbound.  This bug has been fixed 12.07.2010.
(define-test prolog-bug-1
  (assert-equality #'solutions-equal
                   '(((?foo . 23)))
                   (force-ll (prolog '(bug-1-test ?foo)))))

(define-test prolog-bug-2
  (assert-equality #'solutions-equal
                   '(((?x . 10)) ((?x . 20)))
                   (force-ll (prolog '(bug-2-test ?x)))))

(define-test correct-number-of-or-expansions
  ;; Tests if only one form is expanded if we request only one
  ;; solution.
  (let ((expansions nil))
    (declare (special expansions))
    (let ((solutions
            (prolog '(or
                      (lisp-fun set expansions first ?_)
                      (lisp-fun set expansions second ?_)))))
      (lazy-car solutions)
      (assert-eq 'first expansions)
      (lazy-cdr solutions)
      (assert-eq 'second expansions))))

(define-test correct-expansion-of-multiple-choicepoints
  (let ((expansions-1 nil)
        (expansions-2 nil))
    (declare (special expansions-1 expansions-2))
    (let ((solutions
            (prolog '(and
                      (member ?x (1 2))
                      (lisp-fun set expansions-1 ?x ?_)
                      (member ?y (3 4))
                      (lisp-fun set expansions-2 ?y ?_)))))
      (lazy-car solutions)
      (assert-eq 1 expansions-1)
      (assert-eq 3 expansions-2))))

(define-test forall-handler
  (assert-equality #'solutions-equal
                   '(nil)
                   (force-ll (prolog '(forall (member ?x (1 2 3)) (member ?x (1 2 3))))))
  (assert-false (force-ll (prolog '(forall (member ?x (1 2 3 4)) (member ?x (1 2 3))))))
  (assert-false (force-ll (prolog '(forall (member ?x (1 2 3)) (fail))))))

(define-test forall-handler-correct-expansions
  (let ((cond-expansions nil)
        (action-expansions nil))
    (declare (special cond-expansions action-expansions))
    ;; We need to expand all bindings of `cond' but we need only one
    ;; solution of `action'
    (let ((solutions
            (prolog '(forall
                      (and
                       (member ?x (1 2))
                       (lisp-fun set cond-expansions ?x ?_))
                      (and
                       (member ?y (a b))
                       (lisp-fun set action-expansions ?y ?_)
                       (true))))))
      (lazy-car solutions)
      (assert-eq 2 cond-expansions)
      (assert-eq 'a action-expansions))))
