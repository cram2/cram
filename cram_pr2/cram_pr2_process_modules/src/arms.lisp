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

(defun fill-in-with-nils (some-list desired-length)
  (let ((current-length (length some-list)))
    (if (> desired-length current-length)
        (append some-list (make-list (- desired-length current-length)))
        some-list)))

(def-process-module pr2-arms-pm (action-designator)
  (destructuring-bind (command goal-left goal-right)
      (reference action-designator)
    (ecase command
      (cram-common-designators:move-tcp
       (handler-case
           (progn
             (unless (listp goal-left)
               (setf goal-left (list goal-left)))
             (unless (listp goal-right)
               (setf goal-right (list goal-right)))
             (let ((max-length (max (length goal-left) (length goal-right))))
               (mapc (lambda (single-pose-left single-pose-right)
                       ;; (pr2-ll:visualize-marker (list single-pose-left single-pose-right)
                       ;;                          :r-g-b-list '(1 0 1))
                       (pr2-ll:call-giskard-cartesian-action :left single-pose-left
                                                             :right single-pose-right))
                     (fill-in-with-nils goal-left max-length)
                     (fill-in-with-nils goal-right max-length))))
         (common-fail:manipulation-low-level-failure ()
           (cpl:fail 'common-fail:manipulation-low-level-failure :action action-designator))))
      (cram-common-designators:move-joints
       (handler-case
           (pr2-ll:call-giskard-joint-action :left goal-left
                                             :right goal-right)
         (common-fail:manipulation-low-level-failure ()
           (cpl:fail 'common-fail:manipulation-low-level-failure :action action-designator)))))))

;;; Examples:
;;
;; (cram-process-modules:with-process-modules-running
;;     (pr2-pms::pr2-arms-pm)
;;   (cpl:top-level
;;     (cpm:pm-execute-matching
;;      (desig:an action (to move-arm) (right ((0.5 0.5 1.5) (0 0 0 1)))))))
;;
;; (cram-process-modules:with-process-modules-running
;;     (pr2-pms::pr2-arms-pm)
;;   (cpl:top-level
;;     (cpm:pm-execute-matching
;;      (desig:an action
;;                (to move-arm)
;;                (right ((0.5 -0.5 1.5) (0 0 0 1)))
;;                (left ((0.5 0.5 1.5) (0 0 0 1)))))))
;;
;; (cram-process-modules:with-process-modules-running
;;     (pr2-pms::pr2-arms-pm)
;;   (cpl:top-level
;;     (cpm:pm-execute-matching
;;      (desig:an action (to move-arm) (right (((1 1 1) (0 0 0 1)) nil ((1 1 1) (0 0 0 1))))))))
