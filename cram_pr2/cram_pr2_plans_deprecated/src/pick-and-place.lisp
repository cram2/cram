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

(defun fill-in-with-nils (some-list desired-length)
  (let ((current-length (length some-list)))
    (if (> desired-length current-length)
        (append some-list (make-list (- desired-length current-length)))
        some-list)))


(cpl:def-cram-function move-arms-in-sequence (?left-poses ?right-poses)
  (unless (listp ?left-poses)
    (setf ?left-poses (list ?left-poses)))
  (unless (listp ?right-poses)
    (setf ?right-poses (list ?right-poses)))
  (let ((max-length (max (length ?left-poses) (length ?right-poses))))

    (mapc (lambda (?left-pose ?right-pose)

            (cpl:with-failure-handling
                ((common-fail:low-level-failure (e) ; ignore failures
                   (roslisp:ros-warn (pick-and-place grasp) "~a" e)
                   (return)))

              (exe:perform
               (desig:a motion
                        (type moving-tcp)
                        (left-target (desig:a location (pose ?left-pose)))
                        (right-target (desig:a location (pose ?right-pose)))))))

          (fill-in-with-nils ?left-poses max-length)
          (fill-in-with-nils ?right-poses max-length))))


(defun create-moving-tcp-motion-designator (?left-pose ?right-pose)
  (declare (type (or null cl-transforms-stamped:pose-stamped) ?left-pose ?right-pose))
  (let ((?left-target-key-value
          (when ?left-pose
            `(:left-target ,(desig:a location (pose ?left-pose)))))
        (?right-target-key-value
          (when ?right-pose
            `(:right-target ,(desig:a location (pose ?right-pose))))))
    (desig:a motion
             (type moving-tcp)
             ?left-target-key-value
             ?right-target-key-value)))

(cpl:def-cram-function reach (left-poses right-poses)
  ;; Make `?left-poses' and `?right-poses' to lists if they are not already
  (unless (listp left-poses)
    (setf left-poses (list left-poses)))
  (unless (listp right-poses)
    (setf right-poses (list right-poses)))

  ;; Move arms through all but last poses of `?left-poses' and `?right-poses'
  ;; while ignoring failures: accuracy is not so important in pre-reach.
  (let ((max-length (max (length left-poses) (length right-poses))))

    (mapc (lambda (?left-pose ?right-pose)

            (cpl:with-failure-handling
                ((common-fail:low-level-failure (e) ; ignore failures
                   (roslisp:ros-warn (pick-and-place reach) "~a" e)
                   (return)))

              (exe:perform
               (create-moving-tcp-motion-designator ?left-pose ?right-pose))))

          (fill-in-with-nils (butlast left-poses) max-length)
          (fill-in-with-nils (butlast right-poses) max-length)))

  ;; Move arm to the last pose of `?left-poses' and `?right-poses'.
  ;; Ignore failures again for now. In future maybe do something smarter.
  (let ((?left-pose (car (last left-poses)))
        (?right-pose (car (last right-poses))))

    (cpl:with-failure-handling
        ((common-fail:low-level-failure (e) ; ignore failures
           (roslisp:ros-warn (pick-and-place reach) "~a" e)
           (return)))

      (exe:perform
       (create-moving-tcp-motion-designator ?left-pose ?right-pose)))))


(cpl:def-cram-function open-gripper (?left-or-right)
  (cpl:with-failure-handling
      ((common-fail:low-level-failure (e)
         (roslisp:ros-warn (pick-and-place open-gripper) "~a" e)
         ;; ignore failures
         (return)))
    (exe:perform
     (desig:a motion
              (type opening)
              (gripper ?left-or-right)))))

(cpl:def-cram-function grasp (?left-grasp-poses ?right-grasp-poses &optional (retries 0))
  (unless (listp ?left-grasp-poses)
    (setf ?left-grasp-poses (list ?left-grasp-poses)))
  (unless (listp ?right-grasp-poses)
    (setf ?right-grasp-poses (list ?right-grasp-poses)))

  (let ((max-length (max (length ?left-grasp-poses) (length ?right-grasp-poses))))
    (mapc (lambda (?left-pregrasp-poses ?right-pregrasp-poses)
            (cpl:with-failure-handling
                ((common-fail:low-level-failure (e)
                   (roslisp:ros-warn (pick-and-place grasp) "~a" e)
                   (return)))
              (exe:perform
               (desig:a motion
                        (type moving-tcp)
                        (left-target (desig:a location (pose ?left-pregrasp-poses)))
                        (right-target (desig:a location (pose ?right-pregrasp-poses)))))))
          (fill-in-with-nils (butlast ?left-grasp-poses) max-length)
          (fill-in-with-nils (butlast ?right-grasp-poses) max-length)))

  (let ((?left-grasp-pose (car (last ?left-grasp-poses)))
        (?right-grasp-pose (car (last ?right-grasp-poses))))
    (cpl:with-retry-counters ((approach-retries retries))
      (cpl:with-failure-handling
          ((common-fail:low-level-failure (e)
             (cpl:do-retry approach-retries
               (roslisp:ros-warn (pick-and-place grasp) "~a" e)
               (cpl:retry))
             (return)))
        (exe:perform
           (desig:a motion
                    (type moving-tcp)
                    (left-target (desig:a location (pose ?left-grasp-pose)))
                    (right-target (desig:a location (pose ?right-grasp-pose)))))))))

(cpl:def-cram-function grip (?left-or-right ?effort)
  (cpl:with-retry-counters ((grasping-retries 1))
    (cpl:with-failure-handling
        ((common-fail:low-level-failure (e)
           (cpl:do-retry grasping-retries
             (roslisp:ros-warn (pick-and-place grip) "~a" e)
             (cpl:retry))
           (cpl:fail 'common-fail:low-level-failure)))
      (exe:perform
         (desig:a motion
                  (type gripping)
                  (gripper ?left-or-right)
                  (effort ?effort))))))

(cpl:def-cram-function lift (?left-pose ?right-pose)
  (cpl:with-failure-handling
      ((common-fail:low-level-failure (e)
         (roslisp:ros-warn (pick-and-place lift) "~a" e)
         (return)))
    (exe:perform
       (desig:a motion
                (type moving-tcp)
                (left-target (desig:a location (pose ?left-pose)))
                (right-target (desig:a location (pose ?right-pose)))))))

(cpl:def-cram-function perceive (?object-designator
                                 &key
                                 (quantifier :a)
                                 (object-chosing-function #'identity))
  (cpl:with-retry-counters ((perceive-retries 5))
    (cpl:with-failure-handling
        ((common-fail:low-level-failure (e)
           (cpl:do-retry perceive-retries
             (roslisp:ros-warn (pick-and-place perceive) "~a" e)
             (cpl:retry))
           (cpl:fail 'common-fail:low-level-failure :description "couldn't find object")))
      (let* ((resulting-designators
               (case quantifier
                 (:all (exe:perform
                        (desig:a motion
                                 (type detecting)
                                 (objects ?object-designator))))
                 (t (exe:perform
                     (desig:a motion
                              (type detecting)
                              (object ?object-designator))))))
             (resulting-designator
               (funcall object-chosing-function resulting-designators)))
        ;; (format t "found object ~a~%" resulting-designator)
        resulting-designator))))

(defun look-at (object-designator)
  (let ((?pose (get-object-pose object-designator)))
    (exe:perform
     (desig:a motion
              (type looking)
              (target (desig:a location (pose ?pose)))))))

;;;;;;;;;;;;;;;;;;;;;;;; activities ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun perform-phases-in-sequence (action-designator)
  (declare (type desig:action-designator action-designator))
  (let ((phases (desig:desig-prop-value action-designator :phases)))
    (mapc (lambda (phase)
            (format t "Executing phase: ~%~a~%~%" phase)
            (exe:perform phase))
          phases)))

(defun pick-up (action-designator object arm grasp)
  (format t "PICK UP ACTION DESIG: ~a~%" action-designator)
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
    (exe:perform (desig:a motion
                          (type going)
                          (target (desig:a location (pose ?goal-for-base)))))))

(defun drive-and-pick-up-plan (?object-designator &key (?arm :right))
  ;; navigate to a better pose
  (drive-towards-object-plan ?object-designator :?arm ?arm)
  (cpl:par
    (exe:perform (desig:an action
                           (type looking)
                           (object ?object-designator)))
    (exe:perform (desig:an action
                           (type picking-up)
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
      (exe:perform (desig:an action
                             (type looking)
                             (object ?object-designator)))
      (exe:perform (desig:an action
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
  (exe:perform (desig:an action
                         (to place-activity)
                         (arm ?arm))))
