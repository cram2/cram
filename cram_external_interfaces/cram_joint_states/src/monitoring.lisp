;;;
;;; Copyright (c) 2020, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
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

(in-package :joints)

(defun monitor-joint-state (&key
                              joint-name joint-angle-threshold
                              comparison-function)
  (declare (type string joint-name)
           (type number joint-angle-threshold)
           (type function comparison-function))
  (cpl:wait-for
   (cpl:fl-funcall (lambda (joint-state-msg-fluent)
                     (funcall comparison-function
                              (car (joints:joint-positions
                                    (list joint-name)
                                    joint-state-msg-fluent))
                              joint-angle-threshold))
                   *robot-joint-states-msg*))
  (roslisp:ros-info (joints monitor)
                    "Joint ~a reached threshold ~a."
                    joint-name joint-angle-threshold))


(cpm:def-process-module joint-state-pm (motion-designator)
  (destructuring-bind (command argument-1 argument-2 argument-3)
      (desig:reference motion-designator)
    (ecase command
      (cram-common-designators:monitor-joint-state
       (monitor-joint-state
        :joint-name argument-1
        :joint-angle-threshold argument-2
        :comparison-function argument-3)))))


(prolog:def-fact-group joint-state-pm-facts (cpm:matching-process-module
                                             cpm:available-process-module)

  (prolog:<- (cpm:matching-process-module ?motion-designator joint-state-pm)
    (or (desig:desig-prop ?motion-designator (:type :monitoring-joint-state))))

  (prolog:<- (cpm:available-process-module joint-state-pm)
    (prolog:not (cpm:projection-running ?_))))
