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

(in-package :cram-test-utilities-tests)

(def-suite utils :in test-utilities)

(in-suite utils)

(test symbol-set-equal
  (is-true (symbol-set-equal '() '()))
  (is-true (symbol-set-equal '(a b) '(b a)))
  (is-false (symbol-set-equal '(a) '(b)))
  (is-false (symbol-set-equal '(a a) '(a)))
  (is-false (symbol-set-equal '(a a) '(a a))))

(defun every-correct (p orig)
  (every (lambda (perm)
           (and (= (length orig) (length perm))
                (every (rcurry #'member orig) perm)
                (every (rcurry #'member perm) orig)))
         p))

(defun every-correct-permute-all (p orig)
  (let ((orig-p (mapcar #'permute orig)))
    (every (lambda (perm)
             (and (= (length orig) (length perm))
                  (every (lambda (x y)
                           (member x y :test #'equal))
                         perm orig-p)))
           p)))

(defun every-unique (l)
  (every (lambda (x)
           (= 1 (count x l :test #'equal)))
         l))

(macrolet ((permute-test (l)
             `(test ,(intern (format nil "PERMUTE-~a" l))
                (let* ((l ',l)
                       (p (permute l)))
                  (is (= (factorial (length l)) (length p)))
                  (is-true (member l p :test #'equal))
                  (is-true (every-correct p l))
                  (is-true (every-unique p))))))
  (permute-test ())
  (permute-test (1))
  (permute-test (1 2))
  (permute-test (1 2 3))
  (permute-test (1 2 3 4))
  (permute-test (1 2 3 4 5))
  (permute-test ((1 2) (a b))))

(test permute-all-no-arguments
  (is (eq () (permute-all ()))))

(macrolet ((permute-all-test (l)
             `(test ,(intern (format nil "PERMUTE-ALL-~a" l))
                (let* ((l ',l)
                       (p (permute-all l)))
                  (is (= (factorial (length (car l))) (length p)))
                  (is-true (member l p :test #'equal))
                  (is-true (every-correct-permute-all p l))
                  (is-true (every-unique p))))))
  (permute-all-test (()))
  (permute-all-test ((a)))
  (permute-all-test ((a b)))
  (permute-all-test ((a b c)))
  (permute-all-test (() ()))
  (permute-all-test ((a) (d)))
  (permute-all-test ((a b) (d e)))
  (permute-all-test ((a b c) (d e f)))
  (permute-all-test (() () ()))
  (permute-all-test ((1) (4) (7)))
  (permute-all-test ((1 2) (4 5) (7 8)))
  (permute-all-test ((1 2 3) (4 5 6) (7 8 9)))
  (permute-all-test ((a (a b)) (:foo :bar))))

