;;;
;;; Copyright (c) 2020, Arthur Niedzwiecki <aniedz@cs.uni-bremen.de>
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
(setf btr-belief:*spawn-debug-window* nil)

(lisp-unit:define-test apply-rules-both-hands-test
  "Executes the `demos:household-demo', transforms the task tree with `plt:apply-rules'
and executes the `demos:household-demo', again."
  (unless (eq (roslisp:node-status) :running)
    (roslisp-utilities:startup-ros))
  ;; Bowl needs to be in the middle of the drawer for grasping it.
  (setf demos::*demo-object-spawning-poses*
        '((:bowl
           "sink_area_left_middle_drawer_main"
           ((0.10 -0.0 -0.062256) (0 0 -1 0)))
          (:breakfast-cereal
           "oven_area_area_right_drawer_board_3_link"
           ((0.123 -0.03 0.11) (0.0087786 0.005395 -0.838767 -0.544393)))))
  (handler-case
      (progn
        (plt:kill-task-tree)
        (demos:household-demo '(:bowl :breakfast-cereal))
        ;; save transformation response before test, for concurrency reasons
        (let ((rules-applied (plt:apply-rules)))
          (lisp-unit:assert-true rules-applied))
        (demos:household-demo '(:bowl :breakfast-cereal))
        (let ((rules-applied (plt:apply-rules)))
          (lisp-unit:assert-false rules-applied)))
    (cpl:simple-plan-failure (error)
      (format T "Plan failure occured during test: ~%<~a>~%~a"
              error
              "Searching errors are common, just try again while watching.")
      (prolog `(and (btr:bullet-world ?w)
                    (btr:debug-window ?w)))
      (lisp-unit:assert-true NIL))))
