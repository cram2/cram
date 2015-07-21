;;;
;;; Copyright (c) 2011, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :cram-execution-trace-server)

(defun json-prolog-init ()
  (json-prolog:start-prolog-server *ros-node-name* :package (find-package :vis-et)))

(roslisp-utilities:register-ros-init-function json-prolog-init)

(roslisp:def-service-callback cram_execution_trace_server-srv:ListExecutionTraces ()
  (handler-case
      (roslisp:make-response
       :filenames (map 'vector #'namestring
                       (directory (merge-pathnames
                                   (make-pathname :type "ek" :name :wild)
                                   (make-pathname
                                    :directory (roslisp:get-param "~execution_trace_dir"))))))
    (error (e)
      (roslisp:ros-warn (list-execution-traces cram-execution-trace-server)
                        "Service callback failed: ~a" e)
      (roslisp:make-response))))

(defun enable-episode-knowledge (filename)
  (setf cet:*episode-knowledge*
        (cet:load-episode-knowledge filename)))

(roslisp:def-service-callback cram_execution_trace_server-srv:SelectExecutionTrace (filename)
  (handler-case
      (progn
        (enable-episode-knowledge filename)
        (roslisp:make-response))
    (error (e)
      (roslisp:ros-warn (list-execution-traces cram-execution-trace-server)
                        "Service callback failed: ~a" e)
      (roslisp:make-response))))

(defun init-pick-and-place-info-srv ()
  (roslisp:register-service "~list_exectuon_traces"
                            cram_execution_trace_server-srv:ListExecutionTraces)
  (roslisp:register-service "~select_execution_trace"
                            cram_execution_trace_server-srv:SelectExecutionTrace))

(roslisp-utilities:register-ros-init-function init-pick-and-place-info-srv)

(defun run ()
  (unwind-protect
       (progn
         (roslisp-utilities:startup-ros :name "execution_trace" :anonymous nil)
         (spin-until nil 100))
    (roslisp-utilities:shutdown-ros)))
