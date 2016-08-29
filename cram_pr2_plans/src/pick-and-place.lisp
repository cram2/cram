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

(in-package :pr2-plans)

;;;;;;;;;;;;;;;;;;;;;;;;;;;; actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun move-arms-in-sequence (?left-poses ?right-poses)
  (unless (listp ?left-poses)
    (setf ?left-poses (list ?left-poses)))
  (unless (listp ?right-poses)
    (setf ?right-poses (list ?right-poses)))
  (let ((max-length (max (length ?left-poses) (length ?right-poses))))
    (mapc (lambda (?left-pose ?right-pose)
            (cpl:with-failure-handling
                ((pr2-ll:pr2-low-level-failure (e)
                   (roslisp:ros-warn (top-level grasp) "~a" e)
                   ;; ignore failures
                   (return)))
              (cram-plan-library:perform
               (desig:an action
                         (to move-arm-motion)
                         (left ?left-pose)
                         (right ?right-pose)))))
          (pr2-pms::fill-in-with-nils ?left-poses max-length)
          (pr2-pms::fill-in-with-nils ?right-poses max-length))))

(defun open-gripper (?left-or-right)
  (cpl:with-failure-handling
      ((pr2-ll:pr2-low-level-failure (e)
         (roslisp:ros-warn (top-level open-gripper) "~a" e)
         ;; ignore failures
         (return)))
    (cram-plan-library:perform
     (desig:an action
               (to open-motion)
               (?left-or-right gripper)))))

(defun grasp (?left-grasp-poses ?right-grasp-poses &optional (retries 0))
  (unless (listp ?left-grasp-poses)
    (setf ?left-grasp-poses (list ?left-grasp-poses)))
  (unless (listp ?right-grasp-poses)
    (setf ?right-grasp-poses (list ?right-grasp-poses)))
  (let ((max-length (max (length ?left-grasp-poses) (length ?right-grasp-poses))))
    (mapc (lambda (?left-pregrasp-poses ?right-pregrasp-poses)
            (cpl:with-failure-handling
                ((pr2-ll:pr2-low-level-failure (e)
                   (roslisp:ros-warn (top-level grasp) "~a" e)
                   (return)))
              (cram-plan-library:perform
               (desig:an action
                         (to move-arm-motion)
                         (left ?left-pregrasp-poses)
                         (right ?right-pregrasp-poses)))))
          (pr2-pms::fill-in-with-nils (butlast ?left-grasp-poses) max-length)
          (pr2-pms::fill-in-with-nils (butlast ?right-grasp-poses) max-length)))
  (let ((?left-grasp-pose (last ?left-grasp-poses))
        (?right-grasp-pose (last ?right-grasp-poses)))
    (cpl:with-retry-counters ((approach-retries retries))
      (cpl:with-failure-handling
          ((pr2-ll:pr2-low-level-failure (e)
             (cpl:do-retry approach-retries
               (roslisp:ros-warn (top-level grasp) "~a" e)
               (cpl:retry))
             (return)))
        (cram-plan-library:perform
         (desig:an action
                   (to move-arm-motion)
                   (left ?left-grasp-pose)
                   (right ?right-grasp-pose)))))))

(defun grip (?left-or-right ?effort)
  (cpl:with-retry-counters ((grasping-retries 1))
    (cpl:with-failure-handling
        ((pr2-ll:pr2-low-level-failure (e)
           (cpl:do-retry grasping-retries
             (roslisp:ros-warn (top-level pick) "~a" e)
             (cpl:retry))
           (cpl:fail 'cram-plan-failures:gripping-failed)))
      (cram-plan-library:perform
       (desig:an action
                 (to grip-motion)
                 (with ?left-or-right)
                 (effort ?effort))))))

;;;;;;;;;;;;;;;;;;;;;;;; activities ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun perform-phases-in-sequence (action-designator)
  (declare (type desig:action-designator action-designator))
  (let ((phases (desig:desig-prop-value action-designator :phases)))
    (mapc (lambda (phase)
            (format t "Executing phase: ~%~a~%~%" phase)
            (cram-plan-library:perform phase))
          phases)))

(defun top-level-pick-up (?type &key (?arm '(:left :right)) (?color ""))
                                        ;with-pr2-process-modules
  (move-pr2-arms-out-of-sight)
  (let* ((?object-desig (desig:an object
                                  (type ?type)
                                  (color ?color)))
         (?updated-object-desig (cram-plan-library:perform
                                 (desig:an action
                                           (to detect-motion)
                                           (object ?object-desig)))))
    (plan-lib:perform (desig:an action
                                (to my-pick-up)
                                (arm ?arm)
                                (object ?updated-object-desig)))))

(defun top-level-place (&key ?type (?arm '(:left :right)) ?pose ?object)
                                        ;with-pr2-process-modules
  (move-pr2-arms-out-of-sight)
  (let ((?put-down-pose (or ?pose (get-cup-put-pose ?arm :front)))
        (?object-desig (or ?object (desig:an object (type ?type)))))
    (plan-lib:perform (desig:an action
                                (to my-place)
                                (arm ?arm)
                                (object ?object-desig)
                                (at ?put-down-pose)))))

(defun top-level-pick-and-place (?type &key (?arm '(:left :right)))
                                        ;with-pr2-process-modules
  (move-pr2-arms-out-of-sight)
  (let* ((?object-desig (desig:an object
                                  (type ?type)))
         (?updated-object-desig (cram-plan-library:perform
                                 (desig:an action
                                           (to detect-motion)
                                           (object ?object-desig)))))
    (plan-lib:perform (desig:an action
                                (to my-pick-up)
                                (arm ?arm)
                                (object ?updated-object-desig)))
    (plan-lib:perform (desig:an action
                                (to my-place)
                                (arm ?arm)
                                (object ?updated-object-desig)))))
