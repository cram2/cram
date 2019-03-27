;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :boxy-pm)

;;;;;;;;;;;;;;;;; TORSO ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; (cpm:def-process-module torso-pm (motion-designator)
;;   (destructuring-bind (command argument) (desig:reference motion-designator)
;;     (ecase command
;;       (cram-common-designators:move-torso
;;        (handler-case
;;            (boxy-ll:move-torso :goal-position argument))))))

;;;;;;;;;;;;;;;;;;;; NECK ;;;;;;;;;;;;;;;;;;;;;;;;

(cpm:def-process-module neck-pm (motion-designator)
  (destructuring-bind (command goal-pose goal-configuration)
      (desig:reference motion-designator)
    (declare (ignore goal-pose))
    (ecase command
      (cram-common-designators:move-head
       (boxy-ll:move-neck-joints :goal-configuration goal-configuration)))))

;;;;;;;;;;;;;;;;;;;; GRIPPERS ;;;;;;;;;;;;;;;;;;;;;;;;

(cpm:def-process-module grippers-pm (motion-designator)
  (destructuring-bind (command action-type-or-position which-gripper &optional effort)
      (desig:reference motion-designator)
    (ecase command
      (cram-common-designators:move-gripper-joint
       (boxy-ll:move-gripper-joint :action-type-or-position action-type-or-position
                                   :left-or-right which-gripper
                                   :effort effort)))))

;;;;;;;;;;;;;;;;;;;; BODY ;;;;;;;;;;;;;;;;;;;;;;;;

(cpm:def-process-module body-pm (motion-designator)
  (flet ((fill-in-with-nils (some-list desired-length)
           (let ((current-length (length some-list)))
             (if (> desired-length current-length)
                 (append some-list (make-list (- desired-length current-length)))
                 some-list))))
    (destructuring-bind (command argument-1 &rest rest-arguments)
        (desig:reference motion-designator)
      (ecase command
        (cram-common-designators:move-tcp
         (let ((goal-left argument-1)
               (goal-right (car rest-arguments)))
           (progn
             (unless (listp goal-left)
               (setf goal-left (list goal-left)))
             (unless (listp goal-right)
               (setf goal-right (list goal-right)))
             (let ((max-length (max (length goal-left) (length goal-right))))
               (mapc (lambda (single-pose-left single-pose-right)
                       (cram-tf:visualize-marker (list single-pose-left single-pose-right)
                                                  :r-g-b-list '(1 0 1))
                       (boxy-ll:move-arm-cartesian :goal-pose-stamped-left single-pose-left
                                                   :goal-pose-stamped-right single-pose-right))
                     (fill-in-with-nils goal-left max-length)
                     (fill-in-with-nils goal-right max-length))))))
        (cram-common-designators:move-joints
         (boxy-ll:move-arm-joints :goal-joint-states-left argument-1
                                  :goal-joint-states-right (car rest-arguments)))))))
