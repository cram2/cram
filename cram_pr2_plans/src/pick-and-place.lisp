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

(cpl:def-cram-function move-arms-in-sequence (?left-poses ?right-poses)
  (unless (listp ?left-poses)
    (setf ?left-poses (list ?left-poses)))
  (unless (listp ?right-poses)
    (setf ?right-poses (list ?right-poses)))
  (let ((max-length (max (length ?left-poses) (length ?right-poses))))
    (mapc (lambda (?left-pose ?right-pose)
            (cpl:with-failure-handling
                ((pr2-ll:pr2-low-level-failure (e)
                   (roslisp:ros-warn (pick-and-place grasp) "~a" e)
                   ;; ignore failures
                   (return)))
              (cram-plan-library:perform
               (desig:an action
                         (to move-arm-motion)
                         (left ?left-pose)
                         (right ?right-pose)))))
          (pr2-pms::fill-in-with-nils ?left-poses max-length)
          (pr2-pms::fill-in-with-nils ?right-poses max-length))))

(defmacro with-logging-reach ((?left-pose ?right-pose) &body body)
  `(progn
     (format t "poses: ~a and ~a~%" ,?left-pose ,?right-pose)
     (let ((log-id (let ((id (beliefstate:start-node "REACH" `() 2)))
                     (beliefstate:add-topic-image-to-active-node
                      cram-beliefstate::*kinect-topic-rgb*)
                     (beliefstate:add-designator-to-node
                      pr2-ll::*rs-result-designator*
                      id :annotation "object-acted-on")
                     (beliefstate:add-designator-to-node
                      (let ((?arm (if ,?left-pose
                                      (if ,?right-pose
                                          :both
                                          :left)
                                      :right)))
                        (desig:an action
                                  (to move-arm-motion)
                                  (arm ?arm)
                                  (left ,?left-pose)
                                  (right ,?right-pose)))
                      id :annotation "CRAMActionDesignator")
                     id))
           (success nil))
       (unwind-protect
            (progn
              ,@body
              (setf success t))
         (beliefstate:add-topic-image-to-active-node
          cram-beliefstate::*kinect-topic-rgb*)
         (beliefstate:stop-node log-id :success success)))))

(cpl:def-cram-function reach (?left-poses ?right-poses)
  (unless (listp ?left-poses)
    (setf ?left-poses (list ?left-poses)))
  (unless (listp ?right-poses)
    (setf ?right-poses (list ?right-poses)))
  (let ((max-length (max (length ?left-poses) (length ?right-poses))))
    (mapc (lambda (?left-pose ?right-pose)
            (cpl:with-failure-handling
                ((pr2-ll:pr2-low-level-failure (e)
                   (roslisp:ros-warn (pick-and-place reach) "~a" e)
                   ;; ignore failures
                   (return)))
              (with-logging-reach (?left-pose ?right-pose)
                (cram-plan-library:perform
                 (desig:an action
                           (to move-arm-motion)
                           (left ?left-pose)
                           (right ?right-pose))))))
          (pr2-pms::fill-in-with-nils ?left-poses max-length)
          (pr2-pms::fill-in-with-nils ?right-poses max-length))))

(cpl:def-cram-function open-gripper (?left-or-right)
  (cpl:with-failure-handling
      ((pr2-ll:pr2-low-level-failure (e)
         (roslisp:ros-warn (pick-and-place open-gripper) "~a" e)
         ;; ignore failures
         (return)))
    (cram-plan-library:perform
     (desig:an action
               (to open-motion)
               (?left-or-right gripper)))))

(defmacro with-logging-grasp ((?left-grasp-pose ?right-grasp-pose) &body body)
  `(let ((log-id (let ((id (beliefstate:start-node "GRASP" `() 2)))
                   (beliefstate:add-topic-image-to-active-node
                    cram-beliefstate::*kinect-topic-rgb*)
                   (beliefstate:add-designator-to-node
                    pr2-ll::*rs-result-designator*
                    id :annotation "object-acted-on")
                   (beliefstate:add-designator-to-node
                    (let ((?arm (if ,?left-grasp-pose
                                    (if ,?right-grasp-pose
                                        :both
                                        :left)
                                    :right)))
                      (desig:an action
                                (to move-arm-motion)
                                (arm ?arm)
                                (left ,?left-grasp-pose)
                                (right ,?right-grasp-pose)))
                    id :annotation "CRAMActionDesignator")
                   id))
         (success nil))
     (unwind-protect
          (progn
            ,@body
            (setf success t))
       (beliefstate:add-topic-image-to-active-node
        cram-beliefstate::*kinect-topic-rgb*)
       (beliefstate:stop-node log-id :success success))))

(cpl:def-cram-function grasp (?left-grasp-poses ?right-grasp-poses &optional (retries 0))
  (unless (listp ?left-grasp-poses)
    (setf ?left-grasp-poses (list ?left-grasp-poses)))
  (unless (listp ?right-grasp-poses)
    (setf ?right-grasp-poses (list ?right-grasp-poses)))

  (let ((max-length (max (length ?left-grasp-poses) (length ?right-grasp-poses))))
    (mapc (lambda (?left-pregrasp-poses ?right-pregrasp-poses)
            (cpl:with-failure-handling
                ((pr2-ll:pr2-low-level-failure (e)
                   (roslisp:ros-warn (pick-and-place grasp) "~a" e)
                   (return)))
              (cram-plan-library:perform
               (desig:an action
                         (to move-arm-motion)
                         (left ?left-pregrasp-poses)
                         (right ?right-pregrasp-poses)))))
          (pr2-pms::fill-in-with-nils (butlast ?left-grasp-poses) max-length)
          (pr2-pms::fill-in-with-nils (butlast ?right-grasp-poses) max-length)))

  (let ((?left-grasp-pose (car (last ?left-grasp-poses)))
        (?right-grasp-pose (car (last ?right-grasp-poses))))
    (cpl:with-retry-counters ((approach-retries retries))
      (cpl:with-failure-handling
          ((pr2-ll:pr2-low-level-failure (e)
             (cpl:do-retry approach-retries
               (roslisp:ros-warn (pick-and-place grasp) "~a" e)
               (cpl:retry))
             (return)))
        (with-logging-grasp (?left-grasp-pose ?right-grasp-pose)
          (cram-plan-library:perform
           (desig:an action
                     (to move-arm-motion)
                     (left ?left-grasp-pose)
                     (right ?right-grasp-pose))))))))

(defmacro with-logging-grip ((?left-or-right ?effort) &body body)
  `(let ((log-id (let ((id (beliefstate:start-node "GRIP" `() 2)))
                   (beliefstate:add-topic-image-to-active-node
                    cram-beliefstate::*kinect-topic-rgb*)
                   (beliefstate:add-designator-to-node
                    pr2-ll::*rs-result-designator*
                    id :annotation "object-acted-on")
                   (beliefstate:add-designator-to-node
                    (desig:an action
                              (to grip-motion)
                              (arm ,?left-or-right)
                              (effort ,?effort))
                    id :annotation "CRAMActionDesignator")
                   id))
         (success nil))
     (unwind-protect
          (progn
            ,@body
            (setf success t))
       (beliefstate:add-topic-image-to-active-node
        cram-beliefstate::*kinect-topic-rgb*)
       (beliefstate:stop-node log-id :success success))))

(cpl:def-cram-function grip (?left-or-right ?effort)
  (cpl:with-retry-counters ((grasping-retries 1))
    (cpl:with-failure-handling
        ((pr2-ll:pr2-low-level-failure (e)
           (cpl:do-retry grasping-retries
             (roslisp:ros-warn (pick-and-place grip) "~a" e)
             (cpl:retry))
           (cpl:fail 'cram-plan-failures:gripping-failed)))
      (with-logging-grip (?left-or-right ?effort)
        (cram-plan-library:perform
         (desig:an action
                   (to grip-motion)
                   (with ?left-or-right)
                   (effort ?effort)))))))

(cpl:def-cram-function logged-perceive (?object-designator)
  (let ((id (beliefstate:start-node "UIMA-PERCEIVE" nil)))
    (beliefstate:add-designator-to-node ?object-designator
                                        id :annotation "perception-request")
    (let ((resulting-designator (cram-plan-library:perform
                                 (desig:an action
                                           (to detect-motion)
                                           (object ?object-designator)))))
      (beliefstate:add-object-to-node
       resulting-designator id :annotation "perception-result")
      (beliefstate:add-topic-image-to-active-node cram-beliefstate::*kinect-topic-rgb*)
      (beliefstate:stop-node id :success (not (eql resulting-designator nil)))
      resulting-designator)))

;;;;;;;;;;;;;;;;;;;;;;;; activities ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun perform-phases-in-sequence (action-designator)
  (declare (type desig:action-designator action-designator))
  (let ((phases (desig:desig-prop-value action-designator :phases)))
    (mapc (lambda (phase)
            (format t "Executing phase: ~%~a~%~%" phase)
            (cram-plan-library:perform phase))
          phases)))

(defun pick-up-activity (action-designator object arm grasp)
  (perform-phases-in-sequence action-designator)
  (cram-occasions-events:on-event
   (make-instance 'object-gripped :object object :arm arm :grasp grasp)))

(defun pick-up-plan (?type &key (?arm '(:left :right)) (?color ""))
  (move-pr2-arms-out-of-sight)
  (let* ((?object-desig (desig:an object
                                  (type ?type)
                                  (color ?color)))
         (?updated-object-desig (cram-plan-library:perform
                                 (desig:an action
                                           (to detect-motion)
                                           (object ?object-desig)))))
    (plan-lib:perform (desig:an action
                                (to pick-up-activity)
                                (arm ?arm)
                                (object ?updated-object-desig)))))

(defun place-activity (action-designator arm)
  (perform-phases-in-sequence action-designator)
  (cram-occasions-events:on-event
   (make-instance 'object-released :arm arm)))

(defun place-plan (&key (?arm :right) ?type ?pose ?object)
  (let ((?put-down-pose (or ?pose (get-cup-put-pose ?arm :front)))
        (?object-desig (or ?object (desig:an object (type ?type)))))
    (plan-lib:perform (desig:an action
                                (to place-activity)
                                (arm ?arm)
                                (object ?object-desig)
                                (at ?put-down-pose)))))

(defun pick-and-place-plan (?type &key (?arm :right) ?cad-model)
  (move-pr2-arms-out-of-sight)
  (let* ((?object-desig (if ?cad-model
                            (desig:an object
                                      (type ?type)
                                      (cad-model ?cad-model))
                            (desig:an object
                                      (type ?type))))
         (?updated-object-desig (logged-perceive ?object-desig)))
    (plan-lib:perform (desig:an action
                                (to pick-up-activity)
                                (arm ?arm)
                                (object ?updated-object-desig)))
    (plan-lib:perform (desig:an action
                                (to place-activity)
                                (arm ?arm)))))
