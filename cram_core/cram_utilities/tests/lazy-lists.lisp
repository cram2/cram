;;; Copyright (c) 2011, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
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

(in-package :cut-tests)

(define-test delay-test
  (let* ((evaluations 0)
         (delayed-value (delay
                          (incf evaluations)
                          'value)))
    (assert-true (delay-p delayed-value))
    (assert-eql 0 evaluations)
    (assert-eq 'value (force delayed-value))
    (assert-eql 1 evaluations)
    (assert-eq 'value (force delayed-value))
    (assert-eql 1 evaluations)))

(define-test lazy-list
  (let* ((evaluations 0)
         (lazy-list (lazy-list ((i 0))
                      (when (< i 3)
                        (incf evaluations)
                        (cont i (1+ i))))))
    (assert-true (< evaluations 2))
    (assert-eql 0 (lazy-car lazy-list))
    (assert-eql 1 evaluations)
    (assert-eql 1 (lazy-car (lazy-cdr lazy-list)))
    (assert-eql 2 evaluations)
    (force-ll lazy-list)
    (assert-eql 3 evaluations)
    ;; After expanding the lazy list, it should have turned into a
    ;; proper list.
    (assert-equal '(0 1 2) lazy-list)))

(define-test lazy-mapcar
  (let* ((evaluations 0)
         (lazy-list (lazy-mapcar
                     (lambda (elem)
                       (incf evaluations)
                       elem)
                     '(1 2 3))))
    (assert-true (< evaluations 2))
    (lazy-car lazy-list)
    (assert-eql 1 evaluations)
    (lazy-car (lazy-cdr lazy-list))
    (assert-eql 2 evaluations)
    (force-ll lazy-list)
    (assert-eql 3 evaluations)
    (assert-equal '(1 2 3) lazy-list)))

(define-test lazy-mapcan-1
  (let* ((evaluations 0)
         (lazy-list (lazy-mapcan
                     (lambda (elem)
                       (incf evaluations)
                       (list elem))
                     '(1 2 3))))
    (assert-true (< evaluations 2))
    (lazy-car lazy-list)
    (assert-eql 1 evaluations)
    (lazy-car (lazy-cdr lazy-list))
    (assert-eql 2 evaluations)
    (force-ll lazy-list)
    (assert-eql 3 evaluations)
    (assert-equal '(1 2 3) lazy-list)))

(define-test lazy-mapcan-2
  (assert-equal '(1 2 3) (force-ll
                          (lazy-mapcan #'identity '((1) nil (2) nil (3)))))
  (let ((lazy-list (lazy-mapcan #'identity '((1) 2 3))))
    (assert-eql 1 (lazy-car lazy-list))
    ;; This needs to throw because the second element is not a list.
    (assert-error 'simple-error (lazy-cdr lazy-list))))

(define-test lazy-filter
  (let ((filtered-list (lazy-filter
                        #'evenp (loop for i from 1 below 8 collecting i))))
    (assert-true (lazy-list-p filtered-list))
    (force-ll filtered-list)
    (assert-equal '(2 4 6) filtered-list)))

(define-test lazy-filter-evaluations
  (let* ((evaluations 0)
         (filtered-list (lazy-filter (lambda (value)
                                       (declare (ignore value))
                                       (incf evaluations)
                                       t)
                                     '(1 2 3))))
    (assert-true (< evaluations 2))
    (lazy-car filtered-list)
    (assert-eql 1 evaluations)
    (lazy-car (lazy-cdr filtered-list))
    (assert-eql 2 evaluations)
    (force-ll filtered-list)
    (assert-eql 3 evaluations)))

(define-test lazy-take
  (let* ((evaluations 0)
         (list (lazy-take 3 (lazy-list ()
                              (incf evaluations)
                              (cont :elem)))))
    (assert-true (< evaluations 2))
    (lazy-car list)
    (assert-eq 1 evaluations)
    (lazy-cdr list)
    (assert-eq 2 evaluations)
    (force-ll list)
    (assert-eq 3 evaluations)
    (assert-equal '(:elem :elem :elem) list)))

(define-test lazy-dynamic-environment
  (let ((*foo* nil))
    (declare (special *foo*))
    (let ((lazy-list
              (with-lazy-list-dynamic-environment
                  ((*foo* :bar))
                (lazy-list ((i 2))
                  (when (> i 0)
                    (cont *foo* (1- i)))))))
      (assert-eq :bar (lazy-elt lazy-list 0))
      (assert-eq :bar (lazy-elt lazy-list 1)))))
