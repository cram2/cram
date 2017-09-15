;;;
;;; Copyright (c) 2009, Nikolaus Demmel <demmeln@cs.tum.edu>
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

(in-package :walker)

;;;; FIXME: The walker currently has some bugs. Most noteably plan macros are
;;;; not walked correctely.

(defun expand-plan (form)
  "Expand-plan takes a sexp and macroexpands it. Whenever it encounters a call
   to a plan it continues by expanding the sexp that defined that plan and
   thus builds up a hierarchie of plans (plan-tree). Expand-plan returns two
   values: The resulting plan tree and the macroexpansion of form (with calls
   to plans still in place) of which the plan tree will be the most
   interesting.  The root of the plan-tree returned by expand-plan doesn't
   correspond to a plan call, but the form that was passed to expand-plan."
  (let* ((*shadowed-functions* nil)
         (*current-path* nil)
         (*current-parent* (make-plan-tree-node :sexp form))
         (expansion (walk-form form nil)))
    (values *current-parent*
            expansion)))

(defun walk-with-tag-handler (form tag-handler env)
  "Macroexpands form and calls tag-handler once for every (:tag name body)
   form it encounters. The two parameters passed to the tag-handler are name
   and body.  Returns the expansion of form."
  (let ((*current-parent* nil)
        (*tag-walker-handlers* (cons tag-handler *tag-walker-handlers*)))
    (walk-form form env)))