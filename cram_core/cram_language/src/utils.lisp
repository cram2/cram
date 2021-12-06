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

;;; FIXME: Put these into CRAM-UTILITIES

(deftype list-of (type)
  `(or null (cons ,type list)))

(defun format-gensym (format-string &rest format-args)
  (make-gensym (apply #'format-symbol nil format-string format-args)))

(defun partition (predicate list &key key)
  "Equivalent to (VALUES (REMOVE-IF-NOT P L) (REMOVE-IF P L))."
  (declare (optimize speed))
  (declare (function predicate))
  (declare (list list))
  (let ((key (if key (ensure-function key) #'identity))
        (other-bucket '()))
    (values
     (remove-if-not #'(lambda (x)
                        (if (funcall predicate (funcall key x))
                            t
                            (prog1 nil (push x other-bucket))))
                    list)
     (nreverse other-bucket))))

;;; FIXME: get rid of these, there's ASSOC-VALUE, and (SETF ASSOC-VALUE)
;;;        in Alexandria nowadays.

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

;;; FIXME:

(declaim (inline float-/))
(defun float-/ (a b)
  (/ (float a 1.0d0) (float b 1.0d0)))

(defun sleep* (seconds)
  (let ((seconds (coerce seconds 'double-float)))
    (declare (double-float seconds))
    #+nil (declare (optimize speed))
    (prog ((deadline-seconds sb-impl::*deadline-seconds*)
           (stop-time
            (+ (sb-impl::seconds-to-internal-time seconds)
               (get-internal-real-time))))
     :retry
     (cond ((not deadline-seconds)
            (sleep seconds))
           ((> deadline-seconds seconds)
            (sleep seconds))
           (t
            (sleep deadline-seconds)
            (sb-sys:signal-deadline)
            (setq deadline-seconds sb-impl::*deadline-seconds*)
            (setf seconds (float-/ (- stop-time (get-internal-real-time))
                                   internal-time-units-per-second))
            (when (plusp seconds)
              (go :retry)))))))

(defmacro mapcar-clean (function list &rest more-lists)
  "Automatically removes all `NIL' entries from a generated list after
performing a `mapcar'."
  (if more-lists
      `(remove-if #'not (mapcar ,function ,list ,more-lists))
      `(remove-if #'not (mapcar ,function ,list))))
