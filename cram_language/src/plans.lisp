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

;;; TODO@demmeln: update following note.
;;; We need to manage the task trees somehow. The idea is to have a hash-table
;;; containing one task-tree for every top-level plan. Otherwise, the
;;; task-tree management would become really really messy.
;;; --> see episode knowledge.

;;; TODO@demmeln: Add this to the docstring!
;;; Note: Don't have (certain kinds of) surrounding FLET / LABLES / MACROLET /
;;; SYMBOL-MACROLET / LET / ...  when using DEF-TOP-LEVEL-PLAN or
;;; DEF-PLAN. They could mess with (WITH-TAGS ...) or shadow globally defined
;;; plans, which would not be picked up by WITH-TAGS / EXPAND-PLAN.  See the
;;; comment before the definition of WITH-TAGS for more details.

(defmacro def-top-level-plan (name args &body body)
  "Defines a top-level plan. Every top-level plan has its own
   episode-knowledge and task-tree."
  (with-gensyms (call-args)
    `(progn
       (eval-when (:load-toplevel)
         (setf (get ',name 'plan-type) :top-level-plan)
         (setf (get ',name 'plan-lambda-list) ',args)
         (setf (get ',name 'plan-sexp) ',body))
       (defun ,name (&rest ,call-args)
         (named-top-level (:name ,name)
           (replaceable-function ,name ,args ,call-args `(top-level ,',name)
             (with-tags
               ,@body)))))))

(defmacro def-plan (name lambda-list &rest body)
  "Defines a plan. All functions that should appear in the task-tree
   must be defined with def-plan."
  (with-gensyms (call-args)
    `(progn
       (eval-when (:load-toplevel)
         (setf (get ',name 'plan-type) :plan)
         (setf (get ',name 'plan-lambda-list) ',lambda-list)
         (setf (get ',name 'plan-sexp) ',body))
       (defun ,name (&rest ,call-args)
         (replaceable-function ,name ,lambda-list ,call-args (list ',name)
           (with-tags
             ,@body))))))
