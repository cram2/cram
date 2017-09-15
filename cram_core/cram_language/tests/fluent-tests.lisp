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

(in-package :cpl-tests)

(def-suite fluent-tests :in language)

(in-suite fluent-tests)

(def-cpl-parameter test-cpl-parameter 42)

(test cpl-parameter
  (is (= 42 test-cpl-parameter))
  (is-true (typep test-cpl-parameter 'number))
  (setf test-cpl-parameter ())
  (is (eq () test-cpl-parameter))
  (push 23 test-cpl-parameter)
  (is (equal '(23) test-cpl-parameter)))

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


(define-cram-test fl-funcall-vs-fl-apply
    "Basic workingness of FL-FUNCALL and FL-APPLY." ()
  (let ((knob (make-semaphore)))
    (with-task-hierarchy ((P -> Cs)
                          (Cs ->  ))
        ((:task  P    (wait-on-semaphore knob))
         (:tasks Cs 3 (wait-on-semaphore knob)))
      (let ((f1 (fl-funcall #'eq (status (first Cs)) :running)) ; good
            (f2 (fl-funcall #'every (curry #'eq :running)       ; wrong
                            (mapcar #'status Cs)))
            (f3 (fl-apply #'every (curry #'eq :running)         ; good
                          (mapcar #'status Cs))))
        (is (typep f1 'fluent))
        (is (not (typep f2 'fluent)))
        (is (typep f3 'fluent))
        (is (eq 't (value f1)))
        ;; This is NIL because in F2, EQ was called on fluent objects,
        ;; not their status values.
        (is (eq 'nil f2))
        (is (eq 't (value f3)))
        (evaporate-tasks-and-wait P Cs)
        (is (eq 'nil (value f1)))
        (is (eq 'nil (value f3)))))))

(define-cram-test wait-for.1
    "Make sure WAIT-FOR blocks if a fluent's value is NIL."
    ((:n-runs 1))
  (let ((fluent (make-fluent :name "test" :value nil)))
    (hangs (wait-for fluent))
    (hangs (wait-for (fl-value-changed fluent)))))

(define-cram-test wait-for.2
    "More WAIT-FOR & FL-VALUE-CHANGED."
    ((:n-runs 1))
  (let ((fluent (make-fluent :name "test" :value 'foo)))
    (finishes (wait-for fluent))
    (hangs    (wait-for (fl-value-changed fluent)))
    (hangs    (wait-for (fl-pulsed fluent)))))

(define-cram-test wait-for.3
    "Verify that WAIT-FOR blocks in case the fluent's value is NIL."
    ((:timeout 2))
  (let ((fluent (make-fluent :name "test" :value nil)))
    (with-task-hierarchy ((P -> ))
        ((:task P (sleep 0.05) (pulse fluent)))
      (hangs (wait-for fluent)))
    (sleep 1)))

(define-cram-test wait-for.4
    "Verify that FL-VALUE-CHANGED really means that the underlaying
     value changed." ()
  (let ((fluent (fluent 'foo)))
    (with-task-hierarchy ((P -> ))
        ((:task P (sleep 0.05) (pulse fluent)))
      (hangs (wait-for (fl-value-changed fluent))))))

(define-cram-test wait-for-wakeups-on-pulse
    "FIXME" ()
  (let ((fluent (make-fluent :name :test-fluent :value t)))
    (with-producer-consumer-threads (:n-consumers 10)
      (:producer #'(lambda () (sleep 0.05) (pulse fluent)))
      (:consumer #'(lambda () (wait-for (fl-pulsed fluent)))))
    (pass)))

(define-cram-test wait-for-with-timeout
    "FIXME" ()
  (wait-for (end-of-time) :timeout 0.05)
  (pass))

(define-cram-test wait-for-fluent-network-not-eq
    "FIXME" ()
  (let* ((fluent (make-fluent :name :test-fluent :value :wait-for-me))
         (fluent-net (fl-not (fl-eq fluent :wait-for-me))))
    (with-producer-consumer-threads (:n-consumers 10)
      (:producer #'(lambda ()
                     (sleep 0.05)
                     (setf (value fluent) :stop-waiting)))      
      (:consumer #'(lambda ()
                     (wait-for fluent-net))))
    (pass)))

(define-cram-test wait-for-fluent-network-fl-funcall
    "FIXME"
    ((:generators (a-value (gen-one-element :a :b :c :d))))
  (let* ((fluent (make-fluent :name :test-fluent :value :wait-for-me))
         (fluent-net (fl-funcall #'member fluent '(:a :b :c :d))))
    (with-producer-consumer-threads (:n-consumers 10)
      (:producer #'(lambda ()
                     (sleep 0.05)
                     (setf (value fluent) a-value)))
      (:consumer #'(lambda ()
                     (wait-for fluent-net))))
    (pass)))

(define-cram-test whenever.1
    "Verify that WHENEVER executes its body if the passed fluent comes
     with a non-NIL initial value." ((:n-runs 1))
  (is (= 42 (whenever ((make-fluent :name "foof" :value 1))
              (return 42)))))

(define-cram-test whenever.2
    "Verify that WHENEVER does -not- execute its body if the passed
     fluent comes with an initial value of NIL." ((:n-runs 1))
  (hangs
    (whenever ((make-fluent :name "foof"))
      (return))))

(define-cram-test whenever.3
    "Verify that WHENEVER's body is -not- executed on pulses when the
     fluent's value ist still NIL." ()
  (let ((fluent (make-fluent :name "foof" :value nil)))
    (with-task-hierarchy ((P -> ))
        ((:task P (sleep 0.05) (pulse fluent))))
    (hangs (whenever (fluent)
             (return)))
    (pass)))

(define-cram-test whenever.4
    "Verify that WHENEVER's body is executed on (SETF VALUE)." ()
  (let ((fluent (make-fluent :name "foof" :value nil)))
    (with-task-hierarchy ((P -> ))
        ((:task P (sleep 0.05) (setf (value fluent) t)))
      (whenever (fluent)
        (return))
      (pass))))

(define-cram-test whenever.5
    "Verify that WHENEVER's body is executed on every pulse if the
     fluent's value is non-NIL." ((:timeout 1))
  (let ((fluent (make-fluent :name "foof" :value nil)))
    (with-task-hierarchy ((P -> ))
        ((:task P
                (sleep 0.05)
                (setf (value fluent) t)
                (pulse fluent)
                (pulse fluent)
                (pulse fluent)
                (pulse fluent)))
      (let ((n-triggers 0))
        (hangs
          (whenever ((fl-pulsed fluent :handle-missed-pulses :always))
            (incf n-triggers)))
        (sleep 0.3)
        (is (= 5 n-triggers))))))

(define-cram-test fl-value-changed
    "Check that FL-VALUE-CHANGED becomes T exactly once when the value
    of the fluent changed." ()
  (let* ((fluent (make-fluent :name "val-changed-test" :value nil))
         (val-changed-fl (fl-value-changed fluent))
         val-1 val-2 val-3)
    (setf val-1 (value val-changed-fl))
    (setf (value fluent) 'foo)
    (setf val-2 (value val-changed-fl))
    (setf val-3 (value val-changed-fl))
    (is (eq val-1 nil))
    (is (eq val-2 t))
    (is (eq val-3 nil))))

(define-cram-test whenever-always-triggered
    "FIXME"
    ((:timeout 10.0)
     (:generators (n-consumers (gen-integer :min 1 :max 25))
                  (n-triggers  (gen-integer :min 1 :max 50))))
  (let ((fluent (make-fluent :name :test-fluent :value t)))
    (with-producer-consumer-threads (:n-producers 1
                                                  :n-consumers n-consumers)
      (:producer #'(lambda ()
                     ;; Sleeps are evil, but we need to let the
                     ;; consumers enter the whenever state. If we are
                     ;; finished producing before all consumers
                     ;; entered whenever, some block since all pulses
                     ;; are done already.
                     (sleep 0.5)
                     (loop repeat n-triggers
                           do (pulse fluent))))
      (:consumer #'(lambda ()
                     (let ((i 0))
                       (whenever ((fl-pulsed fluent :handle-missed-pulses :always))
                         (when (eql (incf i) n-triggers)
                           (return)))))))
    (pass)))

(define-cram-test whenever-once-triggered
    "FIXME"
    ((:timeout 5.0)
     (:generators (n-consumers (gen-integer :min 1 :max 50))
                  (n-triggers  (gen-integer :min 1 :max 100))))
  (let ((fluent (make-fluent :name :test-fluent :value 0)))
    (with-producer-consumer-threads (:n-consumers n-consumers)
      (:producer #'(lambda ()
                     (sleep 0.5)
                     (loop for i from 1 to n-triggers
                           do (setf (value fluent) i))))
      (:consumer #'(lambda ()
                     (whenever ((fl-pulsed fluent :handle-missed-pulses :once))
                       (when (eql (value fluent) n-triggers)
                         (return))))))
    (pass)))

(define-cram-test fluent-net-many-inputs
    "FIXME"
    ((:timeout 2.5))
  (let* ((fl-1 (make-fluent :name :fl-1 :value 0))
         (fl-2 (make-fluent :name :fl-2 :value 0))
         (fl-net (fl+ fl-1 fl-2)))
    (declare (ignore fl-net))
    (top-level
      (pursue
        (loop repeat 100 do
          (incf (value fl-1)))
        (loop repeat 100 do
          (incf (value fl-2)))))
    (pass)))