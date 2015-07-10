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

(in-package :prolog-tests)

(def-production stacked
  (on ?a ?b)
  (on ?b ?c))

(def-production on
  (on ?a ?b))

(define-test default-handler
  (let ((triggered nil))
    (with-productions
        ((is-identity (?op ?a ?a)))
      (with-production-handlers
            ((is-identity (op &key ?a &allow-other-keys)
               (declare (ignore ?a))
               (push op triggered)))
          (with-facts-asserted
              ('(x 1 1) '(x 1 2)))))
    (assert-equality #'member :assert triggered)
    (assert-equality #'member :retract triggered)))

(define-test with-productions
  (let ((triggered nil))
    (with-productions
        ((is-identity (?op ?a ?a)))
      (with-production-handlers
            ((is-identity (op &key ?a &allow-other-keys)
               (declare (ignore ?a))
               (push op triggered)))
          (with-facts-asserted
              ('(x 1 1) '(x 1 2)))))
    (assert-equality #'member :assert triggered)
    (assert-equality #'member :retract triggered)))

(define-test rete-prove
  (let ((prolog::*alpha-network* (make-instance 'prolog::alpha-node :parent nil :key nil)))
    (rete-assert '(on a b))
    (rete-assert '(on b c))
    (rete-assert '(on b d))
    (assert-equality #'solutions-equal
                     '(((?x . b)))
                     (force-ll (rete-prove '((on a ?x) (on ?x c)))))
    (assert-equality #'solutions-equal
                     '(((?x . b)))
                     (force-ll (rete-prove '((on a ?x) (on ?x d)))))
    (assert-false (rete-prove '((on a ?x) (on ?x e))))
    (assert-equality #'solutions-equal
                     '(((?x . b) (?y . c))
                       ((?x . b) (?y . d)))
                     (force-ll (rete-prove '((on a ?x) (on ?x ?y)))))))
