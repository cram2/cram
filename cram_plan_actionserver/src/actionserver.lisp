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

(in-package :cram-actionserver)

(defvar *results* (make-hash-table))

(actionlib:def-exec-callback cram-actionserver-execute (plan parameters)
  (handler-case
      (let ((plan-symbol (assoc plan *plans* :test #'string-equal)))
        (unless plan-symbol
          (actionlib:abort-current result "Plan not found."))
        (pursue
          (apply (symbol-function plan-symbol)
                 (let ((*read-eval* nil)) (map 'list #'read-from-string parameters)))
          (loop-at-most-every 0.1
            (when (actionlib:cancel-request-received)
              (actionlib:abort-current)))))
    (error (e)
      (actionlib:abort-current result (format nil "~a" e)))))

(def-service-callback (plan-list cram_plan_actionserver-srv:planlist) ()
  (flet ((plan-description (plan-sym)
           (let ((doc-string (documentation (symbol-function plan-sym) 'function))
                 (lambda-list (get plan-sym 'cpl-impl::plan-lambda-list)))
             (format nil "~a~%Lambda list: ~a~%" doc-string lambda-list))))
    (make-response :plans (map 'vector #'car *plans*)
                   :descriptions (map 'vector (alexandria:compose #'plan-description #'cdr)
                                      *plans*))))

(defun server ()
  "Runs the actionserver and blocks."
  (unwind-protect
       (progn
         (startup-ros :name "cram_actionserver" :anonymous nil)
         (register-service-fn "~list_plans" #'plan-list 'cram_plan_actionserver-srv:planlist)
         (actionlib:start-action-server
          "~execute_plan" "cram_plan_actionserver/ExecutePlanAction"
          #'cram-actionserver-execute))
    (shutdown-ros)))
