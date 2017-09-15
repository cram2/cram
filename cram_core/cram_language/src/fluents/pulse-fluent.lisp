;;;
;;; Copyright (c) 2009, Lorenz Moesenlechner <moesenle@cs.tum.edu>,
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

(defclass pulse-fluent (fluent)
  ((handle-missed-pulses :initarg :handle-missed-pulses)
   (processed-pulse-count :initform 0 :initarg :processed-pulse-count
                          :documentation "# pulses we have processed
                          so far. We don't want to use pulse-count
                          since it is used in wait-for to determine if
                          something changed. This slot is used to
                          compute the value.")
   (reference-pulse-count :initform 0 :initarg :reference-pulse-count
                          :documentation "The pulse count of the
                          fluent we are monitoring.")))

(defmethod value ((fluent pulse-fluent))
  (with-slots (pulse-count
               processed-pulse-count
               reference-pulse-count
               handle-missed-pulses)
      fluent
    (with-fluent-locked fluent
      (when (< processed-pulse-count reference-pulse-count)
        (unless *peek-value*
          (ecase handle-missed-pulses
            (:always
             ;; wait-for might ignore wake-ups because it listens for
             ;; pulse-count. So, increment it again here.
             (incf pulse-count)
             (incf processed-pulse-count))
            ((:once :never)
             (setf processed-pulse-count reference-pulse-count))))
        t))))

(defun fl-pulsed (fluent &key (handle-missed-pulses :once))
  "Returns a fluent whose value becomes T only and only if its parent
fluent has been pulsed. How long it stays T depends on the parameter
`handle-missed-pulses'. It can be either :NEVER :ONCE or :ALWAYS:

      - :NEVER means that missed pulses are ignored, i.e. the initial
        value is always NIL and becomes T only when `fluent' gets
        pulsed. It stays T only for exactly one call of VALUE, even if
        more than one pulse occured since the last call to VALUE.

      - :ONCE is similar to :NEVER but has an initial value of T if
        `fluent' has been pulsed at least once before.

      - :ALWAYS means that the VALUE method returns T for exactly the
        number of pulses of `fluent'.

Please note that the value of a PULSE-FLUENT changes from one call to
another. That means it is not re-entrant, and FL-PULSED should not be
bound to a variable or passed around. This could lead to unwanted
behavior if one thread catches pulses another one was to handle."
  (check-type fluent fluent)
  (flet ((make-fluent-callback (weak-result-fluent fl-name)
           (lambda (value)
             (declare (ignore value))
             (without-scheduling
               (let ((result-fluent (tg:weak-pointer-value weak-result-fluent)))
                 (cond (result-fluent
                        (with-fluent-locked result-fluent
                          (setf (slot-value result-fluent 'reference-pulse-count)
                                (with-fluent-locked fluent
                                  (slot-value fluent 'pulse-count))))
                        (pulse result-fluent))
                       (t
                        (remove-update-callback fluent fl-name))))))))
    (let* ((fl-name (gensym "FN-PULSE-FLUENT"))
           (result-fluent (make-fluent :class 'pulse-fluent :name fl-name
                                       :handle-missed-pulses handle-missed-pulses
                                       :processed-pulse-count (ecase handle-missed-pulses
                                                                (:never (slot-value fluent 'pulse-count))
                                                                ((:once :always) 0))
                                       :reference-pulse-count (slot-value fluent 'pulse-count)))
           (weak-result-fluent (tg:make-weak-pointer result-fluent)))
      (without-scheduling
        (register-update-callback
         fluent fl-name
         (make-fluent-callback weak-result-fluent fl-name)))
      result-fluent)))
