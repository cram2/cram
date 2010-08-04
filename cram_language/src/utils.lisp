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


(in-package :cpl-impl)

(deftype thread ()
  'sb-thread:thread)

(deftype list-of (type)
  `(or null (cons ,type list)))

(defun format-gensym (format-string &rest format-args)
  (make-gensym (apply #'format-symbol nil format-string format-args)))

(defun partition (predicate list)
  "Equivalent to (VALUES (REMOVE-IF-NOT P L) (REMOVE-IF P L))."
  (declare (optimize speed))
  (declare (function predicate))
  (declare (list list))
  (let ((other-bucket '()))
    (values
     (remove-if-not #'(lambda (x)
                        (if (funcall predicate x)
                            t
                            (prog1 nil (push x other-bucket))))
                    list)
     (nreverse other-bucket))))

(defun get-alist (name alist &rest keys &key key test test-not)
  (declare (ignore key test test-not))
  (let ((result (apply #'assoc name alist keys)))
    (if result
        (values (cdr result) t)
        (values nil nil))))

;;; Kludge: This won't work on (let ((l nil)) (setf (get-alist :foo l ) 42) l).
(defun (setf get-alist) (new-value name alist &rest keys &key key test test-not)
  (declare (ignore key test test-not))
  (let ((result (apply #'assoc name alist keys)))
    (if result
        (setf (cdr result) new-value)
        (let ((last-elem (last alist)))
          (setf (cdr last-elem) (list (cons name new-value))))))
  new-value)
