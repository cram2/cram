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

(in-package :pr2-ex)

(defmacro with-process-modules (&body body)
  `(cpm:with-process-modules-running
       ((:manipulation pr2-manip-pm:pr2-manipulation-process-module)
        (:navigation pr2-navigation-process-module:pr2-navigation-process-module)
        (:perception perception-pm:perception)
        (:ptu point-head-process-module:point-head-process-module))
     ,@body))

(defvar *process-modules-running* (make-fluent :name :process-modules-running :value nil))
(defvar *process-modules-thread* nil)

(defun start-process-modules ()
  (assert (or (not *process-modules-thread*)
              (not (sb-thread:thread-alive-p *process-modules-thread*)))
          () "Process modules already running.")
  (setf *process-modules-thread*
        (sb-thread:make-thread
         (lambda ()
           (unwind-protect
                (top-level
                  (setf (value *process-modules-running*) t)
                  (pursue
                    (par
                      (cpm:pm-run 'pr2-manip-pm:pr2-manipulation-process-module :manipulation)
                      (cpm:pm-run 'pr2-navigation-process-module:pr2-navigation-process-module
                                  :navigation)
                      (cpm:pm-run 'perception-pm:perception :perception)
                      (cpm:pm-run 'point-head-process-module:point-head-process-module :ptu))
                    (wait-for (not *process-modules-running*))))
             (setf (value *process-modules-running*) nil))))))

(defun stop-process-modules ()
  (assert (and *process-modules-thread*
               (sb-thread:thread-alive-p *process-modules-thread*))
          () "Process modules not running.")
  (setf (value *process-modules-running*) nil))
