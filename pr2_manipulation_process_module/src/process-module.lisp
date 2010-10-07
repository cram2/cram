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

(in-package :pr2-manip-pm)

(defvar *open-handle-action* nil)
(defvar *close-handle-action* nil)

(defun init-pr2-manipulation-process-module ()
  (setf *open-handle-action* (actionlib:make-action-client
                              "/foo/bar"
                              "ias_drawer_executive/OperateHandleAction"))
  (setf *close-handle-action* (actionlib:make-action-client
                               "/foo/baz"
                               "ias_drawer_executive/CloseHandleAction")))

(register-ros-init-function init-pr2-manipulation-process-module)

(defgeneric call-action (action goal params))

(defmethod call-action ((action-sym (eql 'object-opened)) goal params)
  (destructuring-bind (obj side) params
    (multiple-value-bind (result status)
        (actionlib:call-goal *open-handle-action* goal)
      (unless (eq status :succeeded)
        (cpl-impl:fail 'manipulation-failed))
      (roslisp:with-fields (trajectoryhandle) result
        (retract-occasion `(object-closed ?obj))
        (assert-occasion `(object-opened ,obj ,side))
        (assert-occasion `(object-open-handle ,obj ,trajectoryhandle))))))

(defmethod call-action ((action-sym (eql 'object-closed)) goal params)
  (destructuring-bind (obj side) params
    (multiple-value-bind (result status)
        (actionlib:call-goal *close-handle-action* goal)
      (declare (ignore result))
      (unless (eq status :succeeded)
        (cpl-impl:fail 'manipulation-failed))
      (retract-occasion `(object-opened ,obj ,side))
      (retract-occasion `(object-open-handle ,obj ?_))
      (assert-occasion `(object-closed ?obj)))))

(def-process-module pr2-manipulation-process-module (desig)
  (apply #'call-action (reference desig)))
