;;;
;;; Copyright (c) 2016, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :pr2-pms)

(def-process-module pr2-ptu-pm (action-designator)
  (destructuring-bind (command goal-type goal) (reference action-designator)
    (ecase command
      (cram-common-designators:move-head
       (handler-case
           (ecase goal-type
             (:point (pr2-ll:call-ptu-action :point goal))
             (:frame (pr2-ll:call-ptu-action :frame goal)))
         ;; (cram-pr2:look-at-failed ()
         ;;   (cpl:fail 'cram-plan-failures:look-at-failed :action action-designator))
         )))))

;; Examples:
;;
;; (cram-process-modules:with-process-modules-running
;;     (pr2-pms::pr2-grippers-pm pr2-pms::pr2-ptu-pm)
;;   (cpl:top-level
;;     (cpm:pm-execute-matching
;;      (desig:an action
;;                (to look)
;;                (at (an object
;;                        (part-of robot)
;;                        (link end-effector-link)
;;                        (which-link right)))))))
;;
;; (cram-process-modules:with-process-modules-running
;;     (pr2-pms::pr2-grippers-pm pr2-pms::pr2-ptu-pm)
;;   (cpl:top-level
;;     (cpm:pm-execute-matching
;;      (desig:an action
;;                (to look)
;;                (at ((1 0 1) (0 0 0 1)))))))
