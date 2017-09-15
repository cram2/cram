;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :plan-lib)

(def-goal (perform ?action-designator)
  "If there is a clash between a plan that implements a designator and
an available process module that implements the same designator,
the preference will always be given to the process module.
If the action designator has a GOAL key it will be checked if the goal holds.
TODO: it might be nice to have a clash check and let the user know.
For that one would need an assertion like matching-plan.
Then the check would be, if both matching-pm and matching-plan, issue a warning.
The trade-off is: do we want to define matching-plan for each plan to get this warning?
TODO: there might be multiple plans that can execute the same action designator.
In PMs the solution is: try-each-in-order.
In plans it would make more sense to explicitly specify the order.
For now we will only take the first action designator solution.
For future we can implement something like next-different-action-solution
similar to what we have for locations."
  (flet ((convert-desig-goal-to-occasion (keyword-expression)
           (destructuring-bind (occasion &rest params)
               keyword-expression
             (cons (intern (string-upcase occasion) :cram-plan-occasions-events) params)))
         (try-reference-action-designator (action-designator)
           (handler-case (reference ?action-designator)
             (designator-error (e)
               (declare (ignore e))
               (cpl:fail "Action designator ~a could not be resolved.~%Cannot perform action."
                         action-designator)))))

    (if (matching-available-process-modules ?action-designator)
        (pm-execute-matching ?action-designator)
        (destructuring-bind (command &rest arguments)
            (try-reference-action-designator ?action-designator)
          (if (fboundp command)
              (let ((desig-goal (desig-prop-value ?action-designator :goal)))
                (if desig-goal
                    (let ((occasion (convert-desig-goal-to-occasion desig-goal)))
                      (if (holds occasion)
                          (ros-info (achieve plan-lib)
                                    "Action goal `~a' already achieved."
                                    occasion)
                          (apply command arguments))
                      (unless (holds occasion)
                        (cpl:fail "Goal `~a' of action `~a' was not achieved."
                                  ?action-designator occasion)))
                    (apply command arguments)))
              (cpl:fail "Action designator `~a' resolved to cram function `~a',
but it isn't defined. Cannot perform action." ?action-designator command))))))

(def-goal (perform-on-process-module ?module ?action-designator)
  (pm-execute ?module ?action-designator))

(def-goal (monitor-action ?action-designator)
  (let ((matching-process-modules
          (matching-available-process-modules ?action-designator :fail-if-none t))) 
    (monitor-process-module (car matching-process-modules) :designators (list ?action-designator))
    ;; (par-loop (module matching-process-modules)
    ;;   (monitor-process-module module :designators (list ?action-designator)))
    ))
