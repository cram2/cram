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
;;;

(in-package :pex)

(defvar *process-modules-thread* nil)
(defvar *process-modules-running* (make-fluent :name '*process-modules-running* :value nil))

(defvar *ever* (make-fluent :name '*ever* :value nil))

(cpm:process-module-alias :manipulation 'popcorn-manipulation-pm)
(cpm:process-module-alias :perception 'popcorn-perception-pm)

(def-plan run-process-modules ()
  (when (value *process-modules-running*)
    (error 'simple-error :format-control "Process modules are already running."))
  (unwind-protect
       (seq
         (setf (value *process-modules-running*) t)
         (pursue
           (pm-run :perception)
           (pm-run :manipulation)
           (wait-for (cpl:not *process-modules-running*))))
    (setf (value *process-modules-running*) nil)))

(defun start-process-modules ()
  (assert (not (value *process-modules-running*)) ()
          "Cannot start process modules when they are already running")
  (setf *process-modules-thread* (sb-thread:make-thread (lambda ()
                                                          (top-level
                                                            (run-process-modules))))))

(defun stop-process-modules ()
  (assert (and (value *process-modules-thread*)
               (sb-thread:thread-alive-p *process-modules-thread*)) ()
          "Process modules are not running.")
  (setf (value *process-modules-running*) nil))
