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

;;;;;;;;;;;;;;;;;;;;;;;;;;;; overriding logging code: super ugly hack :) ;;;;;;;

(cram-beliefstate::def-logging-hook cram-language::on-preparing-performing-action-designator (designator matching-process-modules)
  (print "on-preparing-performing-action-designator")
  (let ((id (beliefstate:start-node
             "PERFORM-ACTION-DESIGNATOR"
             (list
              (list :description
                    nil;; (desig:description designator)
                    )
              (list :matching-process-modules
                    matching-process-modules))
             2)))
    (beliefstate:add-designator-to-node (make-designator :action ()) ;; designator
                                        id)
    id))

(defun cram-beliefstate::add-designator-to-node (designator node-id &key (annotation "") (relative-context-id))
  (let ((designator (if (typep designator 'list)
                        (car designator)
                        designator)))
    (let* ((type (etypecase designator
                   (cram-designators:action-designator "ACTION")
                   (cram-designators:location-designator "LOCATION")
                   (cram-designators:human-designator "HUMAN")
                   (cram-designators:object-designator "OBJECT")
                   (cram-designators:designator "DESIGNATOR")))
           (memory-address (write-to-string
                            (sb-kernel:get-lisp-obj-address designator)))
           (description (description designator))
           (result (cram-beliefstate::alter-node
                    (remove-if-not
                     #'identity
                     (list (list :command :add-designator)
                           (list :type type)
                           (list :annotation annotation)
                           (list :memory-address memory-address)
                           (list :description description)
                           (when relative-context-id
                             (list :_relative_context_id relative-context-id))))
                    :node-id node-id)))
      (when result
        (let* ((desig-id (desig-prop-value (first result) :id)))
          desig-id)))))


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
                                      :right))
                            (?left-pose (pr2-ll::ensure-pose-in-frame
                                         ?left-pose
                                         cram-tf:*fixed-frame*))
                            (?right-pose (pr2-ll::ensure-pose-in-frame
                                          ?right-pose
                                          cram-tf:*fixed-frame*)))
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
              (cram-plan-library:perform
               (desig:an action
                         (to move-arm-motion)
                         (left ?left-pose)
                         (right ?right-pose)))))
          (pr2-pms::fill-in-with-nils (butlast ?left-poses) max-length)
          (pr2-pms::fill-in-with-nils (butlast ?right-poses) max-length)))

  (let ((?left-pose (car (last ?left-poses)))
        (?right-pose (car (last ?right-poses))))
    (cpl:with-failure-handling
        ((pr2-ll:pr2-low-level-failure (e)
           (roslisp:ros-warn (pick-and-place reach) "~a" e)
           (return)))
      (with-logging-reach (?left-pose ?right-pose)
        (cram-plan-library:perform
         (desig:an action
                   (to move-arm-motion)
                   (left ?left-pose)
                   (right ?right-pose)))))))

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
                                    :right))
                            (?left-grasp-pose (pr2-ll::ensure-pose-in-frame
                                         ?left-grasp-pose
                                         cram-tf:*fixed-frame*))
                            (?right-grasp-pose (pr2-ll::ensure-pose-in-frame
                                          ?right-grasp-pose
                                          cram-tf:*fixed-frame*)))
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
           (cpl:fail 'pr2-ll:pr2-low-level-failure)))
      (with-logging-grip (?left-or-right ?effort)
        (cram-plan-library:perform
         (desig:an action
                   (to grip-motion)
                   (with ?left-or-right)
                   (effort ?effort)))))))

(defmacro with-logging-lift ((?left-pose ?right-pose) &body body)
  `(progn
     (let ((log-id (let ((id (beliefstate:start-node "LIFT" `() 2)))
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
                                      :right))
                            (?left-pose (pr2-ll::ensure-pose-in-frame
                                         ?left-pose
                                         cram-tf:*fixed-frame*))
                            (?right-pose (pr2-ll::ensure-pose-in-frame
                                          ?right-pose
                                          cram-tf:*fixed-frame*)))
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

(cpl:def-cram-function lift (?left-pose ?right-pose)
  (cpl:with-failure-handling
      ((pr2-ll:pr2-low-level-failure (e)
         (roslisp:ros-warn (pick-and-place lift) "~a" e)
         (return)))
    (with-logging-lift (?left-pose ?right-pose)
      (cram-plan-library:perform
       (desig:an action
                 (to move-arm-motion)
                 (left ?left-pose)
                 (right ?right-pose))))))

(defmacro with-logging-perceive ((input-desig) &body body)
  `(let ((id (beliefstate:start-node "UIMA-PERCEIVE" nil)))
     (beliefstate:add-designator-to-node
      ,input-desig id :annotation "perception-request")
     (let ((output-desig ,@body))
       (beliefstate:add-object-to-node
        output-desig id :annotation "perception-result")
       (beliefstate:add-topic-image-to-active-node
        cram-beliefstate::*kinect-topic-rgb*)
       (beliefstate:stop-node id :success (not (null output-desig)))
       output-desig)))

(cpl:def-cram-function perceive (?object-designator
                                 &key
                                 (quantifier :a)
                                 (object-chosing-function #'identity))
  (cpl:with-retry-counters ((perceive-retries 5))
    (cpl:with-failure-handling
        ((pr2-ll:pr2-low-level-failure (e)
           (cpl:do-retry perceive-retries
             (roslisp:ros-warn (pick-and-place perceive) "~a" e)
             (cpl:retry))
           (cpl:fail 'pr2-ll:pr2-low-level-failure :description "couldn't find object")))
      (with-logging-perceive (?object-designator)
        (let* ((resulting-designators
                 (case quantifier
                   (:all (cram-plan-library:perform
                          (desig:an action
                                    (to detect-motion)
                                    (objects ?object-designator))))
                   (t (cram-plan-library:perform
                       (desig:an action
                                 (to detect-motion)
                                 (object ?object-designator))))))
               (resulting-designator
                 (funcall object-chosing-function resulting-designators)))
          resulting-designator)))))

(defun look-at (object-designator)
  (let ((?pose (get-object-pose object-designator)))
    (cram-plan-library:perform
     (desig:an action
               (to look-motion)
               (at ?pose)))))

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

(defun drive-towards-object-plan (?object-designator &key (?arm :right))
  (let* ((object-pose-in-base (get-object-pose ?object-designator))
         (object-pose-in-map (cl-transforms-stamped:transform-pose-stamped
                              cram-tf:*transformer*
                              :timeout cram-tf:*tf-default-timeout*
                              :pose object-pose-in-base
                              :target-frame cram-tf:*fixed-frame*
                              :use-current-ros-time t))
         (?goal-for-base (pose-to-reach-object object-pose-in-map ?arm)))
    (plan-lib:perform (desig:an action
                                (to go-motion)
                                (to ?goal-for-base)))))

(defun drive-and-pick-up-plan (?object-designator &key (?arm :right))
  ;; navigate to a better pose
  (drive-towards-object-plan ?object-designator :?arm ?arm)
  (cpl:par
    (plan-lib:perform (desig:an action
                               (to look-at-action)
                               (object ?object-designator)))
    (plan-lib:perform (desig:an action
                                (to pick-up-activity)
                                (arm ?arm)
                                (object ?object-designator)))))

(defun perceive-and-drive-and-pick-up-plan (?type &key (?arm '(:left :right))
                                                    ?color ?cad-model)
  (if (member ?type '(:fork :knife :cutlery))
      (move-pr2-arms-out-of-sight :flipped t)
      (move-pr2-arms-out-of-sight))
  (let ((object-description `((:type ,?type))))
    (when ?color
      (push `(:color ,?color) object-description))
    (when ?cad-model
      (push `(:cad-model ,?cad-model) object-description))
    (let* ((?object-desig (desig:make-designator :object object-description))
           (?updated-object-desig (perceive ?object-desig)))
      (drive-and-pick-up-plan ?updated-object-desig :?arm ?arm))))

(defun place-activity (action-designator arm)
  (perform-phases-in-sequence action-designator)
  (cram-occasions-events:on-event
   (make-instance 'object-released :arm arm)))

(defun drive-and-place-plan (&key (?arm :right))
  (let ((?object-designator (get-object-in-hand ?arm)))
    (drive-towards-object-plan ?object-designator :?arm ?arm)
    (cpl:par
      (plan-lib:perform (desig:an action
                               (to look-at-action)
                               (object ?object-designator)))
      (plan-lib:perform (desig:an action
                                  (to place-activity)
                                  (arm ?arm)))))
  ;; (let ((?put-down-pose (or ?pose (get-cup-put-pose ?arm :front))))
  ;;   (plan-lib:perform (desig:an action
  ;;                               (to place-activity)
  ;;                               (arm ?arm)
  ;;                               (at ?put-down-pose))))
  )

(defun pick-and-place-plan (?type &key (?arm :right) ?color ?cad-model)
  (perceive-and-drive-and-pick-up-plan ?type :?arm ?arm :?color ?color :?cad-model ?cad-model)
  (plan-lib:perform (desig:an action
                              (to place-activity)
                              (arm ?arm))))
