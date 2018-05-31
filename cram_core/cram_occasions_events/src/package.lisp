;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :cl-user)

(defpackage cram-occasions-events
  (:use #:common-lisp
        #:cram-prolog
        #:cpl-impl
        #:alexandria
        #:desig
        #:cram-utilities
        ;; #:cram-execution-trace
        )
  (:nicknames #:occasions-events #:coe)
  (:shadowing-import-from #:cpl #:name)
  (:shadow event)
  (:import-from #:cpl-impl
                #:make-fluent
                #:value
                #:plan-failure
                #:common-lisp-error-envelope
                #:task-tree-node-children
                #:task-tree-node-status-fluent)
  (:shadowing-import-from #:cram-prolog
                          #:fail)
  (:import-from #:cram-execution-trace
                #:throughout)
  (:export #:clear-belief
           #:assert-occasion
           #:retract-occasion
           #:holds-occasion
           ;; prolog
           #:fluent
           #:fluent-value
           #:task-path
           #:top-level
           #:task #:goal-task
           #:task-status-fluent
           #:task-goal
           #:task-outcome
           #:task-result
           #:task-error
           #:error-type
           #:holds
           #:occurs
           #:task-status
           #:duration-includes
           #:throughout
           #:during
           #:task-started-at
           #:task-created-at
           #:task-ended-at
           #:subtask
           #:subtask+
           #:at
           #:event #:on-event
           #:event-timestamp))
