;;;
;;; Copyright (c) 2017-2022, Sebastian Koralewski <seba@cs.uni-bremen.de>
;;;
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
;;;     * Neither the name of the Institute for Artificial Intelligence/
;;;       Universitaet Bremen nor the names of its contributors may be used to
;;;       endorse or promote products derived from this software without
;;;       specific prior written permission.
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

(in-package :ccl)

(defparameter action-designator-parameter-logging-functions (make-hash-table))

(defun define-all-action-designator-parameter-logging-functions ()
  ;;(add-logging-function-for-action-designator-parameter 'send-effort-action-parameter :effort)
  ;;(add-logging-function-for-action-designator-parameter 'send-position-action-parameter :position)
  (add-logging-function-for-action-designator-parameter 'send-object-action-parameter :object)
  ;;(add-logging-function-for-action-designator-parameter 'send-arm-action-parameter :arm)
  ;;(add-logging-function-for-action-designator-parameter 'send-gripper-action-parameter :gripper)
  (add-logging-function-for-action-designator-parameter 'send-grasp-action-parameter :grasp)
  (add-logging-function-for-action-designator-parameter 'send-container-object-action-parameter :container-object)
  ;;(add-logging-function-for-action-designator-parameter 'send-left-pose-stamped-list-action-parameter :left-poses)
  ;;(add-logging-function-for-action-designator-parameter 'send-right-pose-stamped-list-action-parameter :right-poses)
  ;;(add-logging-function-for-action-designator-parameter 'send-location-action-parameter :location)
  ;;(add-logging-function-for-action-designator-parameter 'send-target-action-parameter :target))
  )

(defun add-logging-function-for-action-designator-parameter (logging-function action-designator-parameter)
  (setf
   (gethash action-designator-parameter action-designator-parameter-logging-functions)
   logging-function))

(defun get-logging-function-for-action-designator-parameter (action-designator-parameter)
  (gethash action-designator-parameter action-designator-parameter-logging-functions))
