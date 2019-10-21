;;;
;;; Copyright (c) 2019, Arthur Niedzwiecki <aniedz@cs.uni-bremen.de>
;;;                     
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

(in-package :plt-tests)

(setf lisp-unit:*print-failures* t)
(setf lisp-unit:*print-errors* t)
(setf lisp-unit:*print-summary* t)
(setf btr-belief:*spawn-debug-window* t)

(define-test apply-rules-both-hands-test
  "Executes the `both-hands-demo', transforms the task tree with `plt:apply-rules'
and executes the `both-hands-demo' again. Is successful when the transformation worked and
bowl and cup are on the kitchen island after both executions.
Since demos go wrong for no reason, the transformed demo is executed up to 5 times,
or until the demo was successful."
  (unless (eq (roslisp:node-status) :RUNNING)
    (roslisp-utilities:startup-ros))
  (execute-demo 'both-hands-demo '(:bowl-1 :cup-1) t)
  (let* ((executing-basic-demo-successful (demo-successful :bowl-1 :cup-1))
         (both-hands-rule-applied
           (when executing-basic-demo-successful (plt:apply-rules)))
         transformed-demo-results)
    (assert-true executing-basic-demo-successful)
    (assert-true both-hands-rule-applied)
    (when both-hands-rule-applied
      (btr-utils:kill-all-objects)
      (loop for try-count to 4
            until (demo-successful :bowl-1 :cup-1)
            do (execute-demo 'both-hands-demo '(:bowl-1 :cup-1))
               (push (demo-successful :bowl-1 :cup-1) transformed-demo-results)))
    ;; If transformed-demo-results contains at least one non-NIL value, this will yield T.
    (assert-true (remove nil transformed-demo-results))))

(define-test apply-rules-environment-test
  "Executes the `environment-demo' before and after transformation via `plt:apply-rules'.
Same structure as in `apply-rules-both-hands-test'."
  (unless (eq (roslisp:node-status) :RUNNING)
    (roslisp-utilities:startup-ros))
  (execute-demo 'environment-demo '(:spoon-1 :fork-1) t)
  (let* ((executing-basic-demo-successful (demo-successful :spoon-1 :fork-1))
         (environment-rule-applied
           (when executing-basic-demo-successful (plt:apply-rules)))
         transformed-demo-results)
    (assert-true executing-basic-demo-successful)
    (assert-true environment-rule-applied)
    (when environment-rule-applied
      (btr-utils:kill-all-objects)
      (loop for try-count to 4
            until (demo-successful :spoon-1 :fork-1)
            do (execute-demo 'environment-demo '(:spoon-1 :fork-1))
               (push (demo-successful :spoon-1 :fork-1) transformed-demo-results)))
    (assert-true (reduce (lambda (r1 r2) (or r1 r2)) transformed-demo-results))))

