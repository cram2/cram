;;; Copyright (c) 2013, Georg Bartels <georg.bartels@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;; * Redistributions of source code must retain the above copyright
;;; notice, this list of conditions and the following disclaimer.
;;; * Redistributions in binary form must reproduce the above copyright
;;; notice, this list of conditions and the following disclaimer in the
;;; documentation and/or other materials provided with the distribution.
;;; * Neither the name of the Institute for Artificial Intelligence/
;;; Universitaet Bremen nor the names of its contributors may be used to 
;;; endorse or promote products derived from this software without specific 
;;; prior written permission.
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

(in-package :pr2-fccl-process-module)

;;; BASIC DEFINITION OF THE PROCESS MODULE AND ITS MECHANICS

(def-process-module pr2-fccl-process-module (desig)
  "Does three things: (a) Defines the PR2-FCCL-PROCESS-MODULE. (b) References the action designator to identify the correct action-handler. (c) Calls 'call-action' which calls the corresponding action-handler."
  (apply #'call-action (reference desig)))

(defgeneric call-action (action &rest params)
  (:documentation "Calls the appropriate 'action' action-handler and passes 'params' along."))

(defmethod call-action ((action-sym t) &rest params)
  ;; Fallback case of call-action in case non of the action-handlers matched the result of '(reference desig)'.
  (format t "[PR2-FCCL-PM PROCESS-MODULE] Unimplemented operation `~a' with parameters ~a. Doing nothing.~%"
          action-sym params)
  (sleep 0.5))

(defmethod call-action :around (action-sym &rest params)
  ;; Adds some printouts to the other actual call-action-method to inform the user.
  (format t "[PR2-FCCL-PM PROCESS-MODULE] Executing manipulation action ~a ~a.~%"
          action-sym params)
  (prog1 (call-next-method)
    (format t "[PR2-FCCL-PM PROCESS-MODULE] Manipulation action done.~%")))

(defmacro def-action-handler (name args &body body)
  "Generates a new 'call-action' method for every action-handler defined."
  (alexandria:with-gensyms (action-sym params)
    `(defmethod call-action ((,action-sym (eql ',name)) &rest ,params)
       (destructuring-bind ,args ,params ,@body))))