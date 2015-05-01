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

(in-package :cl-user)

(defpackage :cram-walker
  (:nicknames :walker)
  (:documentation
   "A fairly basic code walker for use in CPL for expanding plans.")
  (:use #:common-lisp)
  (:export
   ;; plan tree
   #:plan-tree-node
   #:plan-tree-node-sexp
   #:plan-tree-node-parent
   #:plan-tree-node-children
   #:plan-tree-node-path
   #:find-plan-node
   ;; interface-functions
   #:expand-plan
   #:walk-with-tag-handler)
  (:shadow ;; Do this in case some lisp implementation defines
   ;; these cltl2 functions in the common-lisp package
   #:augment-environment
   #:parse-macro
   #:enclose)
  (:import-from #:alexandria
                #:rcurry))

;;;; A few notes on the package setup.
;;;
;;; Cram, the language, is split up into two packages, CPL-IMPL and
;;; CPL. The former contains the language implementation whereas the
;;; latter is supposed to be :USEd.
;;;
;;; There are three notable differences:
;;;
;;;   i) CPL-IMPL only exports symbols which are specific to CRAM.
;;;
;;;      CPL additionally reexports all of CL. So users are supposed
;;;      to simply write (:USE :CPL), and to not include :CL there.
;;;
;;;  ii) The fluent operations are prefixed with "FL" in CPL-IMPL;
;;;      however, in CPL, these are exported as +,-,*,/,etc.
;;;
;;;      E.g. CPL:+ is actually CPL-IMPL:FL+.
;;;
;;; iii) CPL-IMPL also exports some bits used in its implementation
;;;      which may be good for other uses, too. In particular for the
;;;      test suite.
;;;
;;; Caveat:
;;;   CPL:EQL is not the same as CL:EQL. That means, if you want to
;;;   use, e.g., eql-specializers in a package which uses CPL, you
;;;   have to write CL:EQL explicitly.

#.(let ((cpl-symbols
         '(;; utils
           #:sleep*
           #:mapcar-clean
           ;; walker
           #:plan-tree-node #:plan-tree-node-sexp #:plan-tree-node-parent
           #:plan-tree-node-children #:plan-tree-node-path
           #:find-plan-node
           #:expand-plan
           ;; fluent.lisp
           #:fluent
           #:value-fluent
           #:value
           #:peek-value
           #:wait-for
           #:pulse
           #:whenever
           #:make-fluent
           #:on-make-fluent-hook
           #:register-update-callback
           #:remove-update-callback
           #:get-update-callback
           #:def-cpl-parameter
           ;; with-policy
           #:with-policy
           #:with-named-policy
           #:with-policies
           #:with-named-policies
           #:make-policy
           #:named-policy
           #:define-policy
           #:policy-not-found
           #:policy-condition
           #:policy-failure
           #:policy-init-failed
           #:policy-check-condition-met
           ;; policies
           #:timeout-policy
           #:parameters
           ;; task interface
           #:*current-task*
           #:*task-pprint-verbosity*
           #:*current-path*
           #:current-task
           #:define-task-variable
           #:task #:toplevel-task
           #:task-alive #:task-dead
           #:task-running-p
           #:task-dead-p
           #:task-done-p
           #:task-failed-p
           #:status #:result
           #:status-indicator
           #:+alive+
           #:+dead+
           #:+done+
           #:evaporate #:suspend #:wake-up #:join-task
           #:tv-closure
           #:task-path
           #:without-scheduling
           #:with-scheduling
           #:retry-after-suspension
           #:on-suspension
           #:*break-on-plan-failures*
           ;; failures.lisp
           #:fail #:on-fail
           #:simple-plan-failure
           #:plan-failure
           #:with-failure-handling #:retry
           #:with-retry-counters #:do-retry #:reset-counter #:get-counter
           #:common-lisp-error-envelope
           #:envelop-error
           #:*break-on-plan-failures*
           #:*debug-on-lisp-errors*
           ;; task-tree.lisp
           #:code
           #:code-parameters
           #:code-sexp
           #:code-function
           #:code-task
           #:task-tree-node
           #:task-tree-node-p
           #:task-tree-node-path
           #:task-tree-node-code
           #:task-tree-node-parent
           #:task-tree-node-children
           #:task-tree-node-effective-code
           #:with-task-tree-node
           #:make-task-tree-node
           #:replaceable-function
           #:make-task
           #:sub-task
           #:task
           #:clear-tasks
           #:*task-tree*
           #:stale-task-tree-node-p
           #:filter-task-tree
           #:flatten-task-tree
           #:task-tree-node-parameters
           #:task-tree-node-status-fluent
           #:task-tree-node-result
           #:goal-task-tree-node-p
           #:goal-task-tree-node-pattern
           #:goal-task-tree-node-parameter-bindings
           #:goal-task-tree-node-goal
           ;; base.lisp
           #:top-level #:seq #:par #:tag #:with-tags #:with-task-suspended
           #:par-loop
           #:pursue #:composite-failure #:composite-failures #:try-all
           #:try-in-order #:tagged
           #:try-each-in-order #:partial-order
           #:on-top-level-setup-hook
           #:on-top-level-cleanup-hook
           ;; plans.lisp
           #:on-def-top-level-plan-hook
           #:def-top-level-plan #:get-top-level-task-tree #:def-plan
           #:def-cram-function #:def-top-level-cram-function
           ;; goals.lisp
           #:declare-goal #:def-goal #:goal #:register-goal #:goal-context
           #:succeed #:describe-goal))
        (fluent-ops
         '(;; fluent-net.lisp
           #:fl< #:fl> #:fl=  #:fl+ #:fl- #:fl* #:fl/
           #:fl-eq #:fl-eql #:fl-not #:fl-and #:fl-or
           #:fl-pulsed #:fl-funcall #:fl-apply #:fl-value-changed))
        (cpl-impl-ext-symbols
         '(;; logging.lisp
           #:*log-output*
           #:*log-right-margin*
           #:log-event
           #:+log-all+
           #:+log-default+
           #:+log-verbose+
           #:+log-very-verbose+
           #:+log-language+
           #:logging-enabled-p
           #:list-available-log-tags
           #:list-active-log-tags
           #:log-enable
           #:log-disable
           #:log-set
           ;; tasks
           #:name
           #:*save-tasks*
           #:*tasks*
           #:list-saved-tasks))
        (cl-symbols
         (let (r) (do-external-symbols (s :cl r) (push s r)))))

    `(progn

       (defpackage :cram-language-implementation
         (:nicknames :cpl-impl)
         (:documentation "Internal implementation package of CPL.")
         (:use :common-lisp
               :walker
               :cram-utilities
               :trivial-garbage
               :alexandria)
         (:export ,@cpl-symbols ,@fluent-ops ,@cpl-impl-ext-symbols))

       (defpackage :cram-language
         (:nicknames :cpl)
         (:documentation
          "Main package of a new planning language similar to RPL.")
         (:use :common-lisp
               :cram-language-implementation)
         (:export ,@cl-symbols
                  ,@cpl-symbols
                  ;; Wrappers are defined in src/language.lisp.
                  #:< #:> #:+ #:- #:* #:/ #:= #:eq #:eql #:not
                  #:pulsed #:fl-and #:fl-or #:fl-funcall #:fl-value-changed
                  #:sleep)
         (:shadow
          #:< #:> #:+ #:- #:* #:/ #:= #:eq #:eql #:not #:sleep))))

(defpackage cram-user
  (:use #:cpl)
  (:nicknames #:cpl-user)
  (:export))
