;;;
;;; Copyright (c) 2009, Lorenz Moesenlechner <moesenle@cs.tum.edu>,
;;;                     Nikolaus Demmel <demmeln@cs.tum.edu>
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

(in-package :cram-test-utilities)

(defmacro hangs (&body body)
  "Generates a pass if `body' hangs, and a failure if it returns."
  `(handler-case
       (let ((result
              ;; Kludge: We must be pretty gratituous about this
              ;; timeout value because in (HANGS (TOP-LEVEL ...)) the
              ;; main thread will evaporate+wait until the toplevel
              ;; task and all its subtasks shut down---however, this
              ;; timeout could mistakenly interrupt that wait in case
              ;; the DEADLINE fired.
              (sb-ext:with-timeout 3.0
                ;; NB. As HANGS will be used in the main thread, only,
                ;; not in a task, this won't be mistaken for an
                ;; entrance into a task's event loop.
                (sb-sys:with-deadline (:seconds 0.5 :override t)
                  ,@body))))
         (5am:fail "~@<Expected ~S to hang, but it returned: ~S~@:>"
                   ',body result))
     (sb-ext:timeout ()
       (cpl-impl::log-event
         (:context "*** TIMEOUT ***")
         (:tags :finish))
       (pass))))

(defun symbol-set-equal (a b)
  "Checks if two sets are equal. Returns non-NIL if the `a' and `b'
   are sets (contain no duplicates using test EQ) and contain the same
   elements."
  (flet ((no-duplicates (x)
           (= (length x) (length (remove-duplicates x)))))
    (and (no-duplicates a)
         (no-duplicates b)
         (= 0
            (length (remove-if (rcurry #'member b) a))
            (length (remove-if (rcurry #'member a) b))))))

(defun permute (list)
  "Returns a list of all permutations of `list'."
  (cond
    ((endp list) (list nil))
    ((endp (cdr list)) (list list))
    (t (loop for subpermutation in (permute (cdr list)) nconc
            (loop for i from 0 to (length subpermutation)
               collecting (append (subseq subpermutation 0 i)
                                  (cons (car list) (subseq subpermutation i))))))))

(defun permute-all (l)
  "Permute a list of lists of same length. Generates permutations for the
   sublists simultationsly, so the corresponding elements are allways in
   corresponding positions. I.e. if you call it on ((a b) (c d)) it will
   generate the permutations ((a b) (c d)) and ((b a) (d c)) but not e.g. ((a
   b) (d c))."
  (flet ((car-all (l)
           (mapcar #'car l))
         (cdr-all (l)
           (mapcar #'cdr l))
         (endp-all (l)
           (every #'endp l))
         (cons-all (a l)
           (mapcar #'cons a l))
         (append-all (l &rest ls)
           (apply #'mapcar #'append l ls))
         (subseq-all (l start &optional end)
           (mapcar (rcurry #'subseq start end) l))
         (length-all (l)
           (length (car l))))
    (cond
      ((null l) ())
      ((endp-all l) (list l))
      ((endp-all (cdr-all l)) (list l))
      (t (loop for subperm in (permute-all (cdr-all l)) nconc
              (loop for i from 0 to (length-all subperm)
                 collecting (append-all (subseq-all subperm 0 i)
                                        (cons-all (car-all l) (subseq-all subperm i)))))))))

;;; FIXME:

(declaim (inline float-/))
(defun float-/ (a b)
  (/ (float a 1.0d0) (float b 1.0d0)))

(defun sleep* (seconds)
  (let ((seconds (coerce seconds 'double-float)))
    (declare (double-float seconds))
    #+nil (declare (optimize speed))
    (cpl-impl::log-event
      (:context "- SLEEP -")
      (:display "~S" seconds)
      (:tags :finish))
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
              (go :retry)))))
    (cpl-impl::log-event
      (:context "- WAKE -")
      (:display "~S" seconds)
      (:tags :finish))))

