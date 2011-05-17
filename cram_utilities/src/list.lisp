;;;
;;; Copyright (c) 2011, Alexandra Kirsch
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
;;;

(in-package :cut)

; random-list
(defun make-random-list (count &key (lower 0) (upper 1000))
  "returns a list of random numbers in range [lower, limit[ of length count"
  (labels ( (random-list-tail (count lower upper result)
              (cond ( (zerop count)
                      result )
                    ( T
                      (random-list-tail (1- count) lower upper
                                        (cons (random-number upper :lower lower) result)) ))) )
    (random-list-tail count lower upper ())))

; rec-remove
(defun rec-remove (item sequence &key (test #'eql))
  (cond ( (null sequence)
          () )
        ( (funcall test (first sequence) item)
          (rec-remove item (rest sequence) :test test) )
        ( T
          (cons
            (if (atom (first sequence))
              (first sequence)
              (rec-remove item (first sequence) :test test))
            (rec-remove item (rest sequence) :test test)) )))

; find-and-remove
(declaim (inline find-and-remove))
(defun find-and-remove (item seq &rest key-args)
  (let ( (found-item (apply #'find item seq key-args)) )
    (values
      found-item
      (apply #'remove item seq key-args))))

; find-atom-rec-if
(defun find-atom-rec-if (predicate tree)
  (cond ( (null tree)
          nil )
        ( (atom tree)
          (when (funcall predicate tree)
            (list tree)) )
        ( T
          (remove-duplicates (mapcan #'(lambda (tt) (find-atom-rec-if predicate tt)) tree)) )))

; make number list
(defun make-number-list (start end &optional (delta 1) (acc ()))
  "returns list of numbers between start and end"
  (cond ( (or (and (>= delta 0) (> start end))
              (and (< delta 0) (< start end)))
          (reverse acc) )
        ( T
          (make-number-list (+ start delta) end delta (cons start acc)) )))

