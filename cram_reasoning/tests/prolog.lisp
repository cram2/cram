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

(def-suite prolog :in reasoning)

(defmacro test-prolog (name &body tests)
  (let ((count 0)
        (name (format-symbol t "PROLOG-~a" name)))
    (flet ((next-name ()
             (format-symbol t "~a-~a" name (incf count))))
      `(progn
         ,@(mapcar (lambda (test)
                     (destructuring-bind (test query result) test
                       (case test
                         (:prolog
                          `(test ,(next-name)
                             (is (equal ',result
                                        (force-ll (prolog ',query))))))
                         (:prolog-probe
                          `(test ,(next-name)
                             (let ((answers (force-ll (prolog ',query))))
                               (is-true (not (null answers)))
                               (dolist (bdg answers)
                                 (dolist (x ',result)
                                   (is (equal (cdr x) (var-value (car x) bdg))))))))
                         (t (error "Invalid test form.")))))
                   tests)))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Prolog handlers
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(def-suite prolog-handlers :in prolog)

(in-suite prolog-handlers)

(test-prolog and-handler
  (:prolog (and (== ?x a) (== ?x a) (== ?x ?y))
           (((?x . a) (?y . a))))
  (:prolog (and (and) (and))
           (nil))
  (:prolog (and (fail))
           nil)
  (:prolog (and (and) (and) (fail))
           nil)
  (:prolog (and)
           (nil)))

(test-prolog or-handler
  (:prolog (or)
           nil)
  (:prolog (or (== a a) (== b a) (== a b))
           (nil))
  (:prolog (or (fail) (and))
           (nil))
  (:prolog (or (fail) (fail))
           nil))

(test-prolog not-handler
  (:prolog (not (fail))
           (nil))
  (:prolog (not (and))
           nil))

(test-prolog lisp-fun
  (:prolog (and (== (a b c) ?x)
                (lisp-fun append (a b) (c) ?x))
           (((?x . (a b c)))))
  (:prolog (lisp-fun + 1 2 ?x)
           (((?x . 3)))))

(test-prolog lisp-pred
  (:prolog (lisp-pred symbolp a)
           (nil))
  (:prolog (lisp-pred atom (a b))
           nil)
  (:prolog (and (== ?x (foo bar))
                (lisp-pred equal (foo bar) ?x))
           (((?x . (foo bar))))))

(test-prolog bound-handler
  (:prolog (bound ?x)
           nil)
  (:prolog (bound a)
           (nil))
  (:prolog (and (== ?x a)
                (bound ?x))
           (((?x . a))))
  (:prolog (and (== ?x ?y)
                (bound ?x))
           nil)
  (:prolog (and (== ?x (?y))
                (bound ?x))
           (((?x . (?y))))))

(test-prolog ground-handler
  (:prolog (ground ?x)
           nil)
  (:prolog (ground a)
           (nil))
  (:prolog (and (== ?x a)
                (ground ?x))
           (((?x . a))))
  (:prolog (and (== ?x ?y)
                (bound ?x))
           nil)
  (:prolog (and (== ?x (?y))
                (ground ?x))
           nil))

(test-prolog findall-handler
  (:prolog-probe (findall ?x (member ?x (1 2 3)) ?y)
                 ((?y . (1 2 3))))
  (:prolog-probe (and (== ?x (1 2 3))
                      (findall (?x (?y ?z)) (and (member ?y ?x )
                                                      (lisp-fun + ?y 10 ?z))
                                    ?result)
                      (== (?first . ?_) ?result))
                 ((?first . ((1 2 3) (1 11)))))
  (:prolog-probe (findall () (fail) ?result)
                 ((?result . ()))))

(test-prolog forall-handler
  (:prolog (forall (member ?x (1 2 3)) (> ?x 0))
           (nil))
  (:prolog (forall (and (member ?x (1 2 3)) (> ?x 3)) (fail))
           (nil))
  (:prolog-probe (and (== ?t integer)
                      (forall (or (== ?x 42) (== ?x 23)) (lisp-pred typep ?x ?t)))
                 ((?t . integer))))

(test-prolog filter-bindings
  (:prolog (filter-bindings (?a ?b)
                            (== (?a ?b ?c ?d) (1 2 3 4)))
           (((?a . 1) (?b . 2))))
  (:prolog (and (== ?b 42)
                (filter-bindings ()
                                 (> ?b 23))
                (== ?b :new))           ; ?b is still bound to 42 here.
                                        ; filter-bindings cannot remove
                                        ; bindings like suggested here.
           nil)
  (:prolog (filter-bindings ()
                            (fail))
           nil))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Predefined prolog facts
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(def-suite prolog-facts :in prolog)

(in-suite prolog-facts)

(test-prolog unification
  (:prolog (== ?x a)
           (((?x . a)))))

(test-prolog bin-preds
  (:prolog (< 1 2) (nil))
  (:prolog (< 3 2) nil)
  (:prolog (> 1 0) (nil))
  (:prolog (> 0 1) nil)
  (:prolog (>= 0 0) (nil))
  (:prolog (>= 0 1) nil)
  (:prolog (<= 0 0) (nil))
  (:prolog (<= 1 0) nil))

(test-prolog member
  (:prolog (member ?x (a b c))
           (((?x . a)) ((?x . b)) ((?x . c))))
  (:prolog (member ?x ())
           nil))

(defclass dummy ()
  ((a :initform :a)))

(test-prolog clos-utils
  (:prolog (instance-of ?t ?o)
           ())
  (:prolog-probe (and (instance-of dummy ?obj)
                      (get-slot-value ?obj a ?a)) 
                 ((?a . :a)))
  (:prolog-probe (and (instance-of dummy ?obj)
                      (instance-of dummy ?obj)
                      (== ?a :a)) ;; test with ?obj bound
                 ((?a . :a)))
  (:prolog-probe (and (instance-of dummy ?obj)
                      (instance-of ?type ?obj)) 
                 ((?type . dummy)))
  (:prolog-probe (and (instance-of dummy ?obj)
                      (get-slot-value ?obj a :a)
                      (== ?a :a))
                 ((?a . :a)))
  (:prolog (and (instance-of dummy ?obj)
                (get-slot-value ?obj a :b))
           ())
  (:prolog-probe (and (instance-of dummy ?obj)
                      (slot-value ?obj a ?a))
                 ((?a . :a)))
  (:prolog-probe (and (instance-of dummy ?obj)
                      (slot-value ?obj a :a)
                      (== ?a :a))
                 ((?a . :a)))
  (:prolog-probe (and (instance-of dummy ?obj)
                      (slot-value ?obj a :b)
                      (slot-value ?obj a ?b))
                 ((?b . :b))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; General tests
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(in-suite prolog)

;; Defining facts like this actually modifies the state of the software, since
;; it adds a fact group and fact. A test suite should not do that. However for
;; now this is the easiest solution and since we are in a seperate package,
;; there should be no name clashes.

(def-fact-group prolog-tests (fact-extenable)
  
  (<- (fact1 ?x ?y)
    (== ?y (1 2))
    (member ?x ?y))

  (<- (fact-extendable ?x)
    (== ?x 1)))

(def-fact-group prolog-tests-2 (fact-extendable)

  (<- (fact-extendable ?y)
    (== ?y 2)))

(test-prolog 1
  (:prolog (fact1 ?a ?b)
           (((?A . 1) (?B 1 2))
            ((?A . 2) (?B 1 2)))))

(test-prolog extendable
  (:prolog (fact-extendable ?foo)
           (((?foo . 1)) ;; this tests for correct order of answers as well
            ((?foo . 2)))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Tests introduced for specific bugs that were found
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(def-suite prolog-bugs :in prolog)

(in-suite prolog-bugs)

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

(test-prolog rename-variables-bug
  (:prolog (fact-rename ?x) (((?x . 1)))) ;; used to fail while next one passed
  (:prolog (fact-rename ?y) (((?y . 1)))))

;; The following tests for an annoying bug which was in the interaction of
;; AND, NOT, PROLOG, FILTER-BINDINGS, and ADD-BDG. The behaviour was, that in
;; the BUG-1 fact in the last goal (<= ?foo 42) an error would be raised,
;; because ?foo would be unbound.  This bug has been fixed 12.07.2010.
(test-prolog bug-1
  (:prolog-probe (bug-1-test ?foo)
                 ((?foo . 23))))

(test-prolog bug-2
  (:prolog (bug-2-test ?x)
           (((?x . 10))
            ((?x . 20)))))
