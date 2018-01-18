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

(defun append-pour-cartesian-action-designator (action-designator ?arm
                                                ?left-pour-poses ?right-pour-poses
                                                ?left-tilt-pose ?right-tilt-pose
                                                ?left-retract-poses ?right-retract-poses)
  (case ?arm
    (:left (setf ?right-pour-poses nil
                 ?right-retract-poses nil
                 ?right-tilt-pose nil))
    (:right (setf ?left-pour-poses nil
                  ?left-retract-poses nil
                  ?left-tilt-pose nil)))
  ;; (setf ?left-grasp-poses (reverse ?left-grasp-poses))
  ;; (setf ?right-grasp-poses (reverse ?right-grasp-poses))
  (let* (;; (?minus-angle (- ?angle))
         (phases (list
                  (an action
                      (to approach)
                      (left ?left-pour-poses)
                      (right ?right-pour-poses))
                  (an action
                      (to tilt-to)
                      (left ?left-tilt-pose)
                      (right ?right-tilt-pose))
                  (an action
                      (to tilt-to)
                      (left ?left-pour-poses)
                      (right ?right-pour-poses))
                  (an action
                      (to retract)
                      (left ?left-retract-poses)
                      (right ?right-retract-poses)))))
    (copy-designator action-designator :new-description `((:phases ,phases)))))

(defun append-pour-giskard-action-designator (action-designator arm
                                              source-type destination-type
                                              pour-volume liquid-in-source
                                              ?left-retract-poses ?right-retract-poses)
  (case arm
    (:left (setf ?right-retract-poses nil))
    (:right (setf ?left-retract-poses nil)))
  ;; (setf ?left-grasp-poses (reverse ?left-grasp-poses))
  ;; (setf ?right-grasp-poses (reverse ?right-grasp-poses))
  (cut:with-vars-bound (?left-tool-frame ?right-tool-frame)
      (cut:lazy-car
       (prolog:prolog
        `(and (cram-robot-interfaces:robot ?robot)
              (cram-robot-interfaces:robot-tool-frame ?robot :left ?left-tool-frame)
              (cram-robot-interfaces:robot-tool-frame ?robot :right ?right-tool-frame))))
    (if (cut:is-var ?left-tool-frame)
        (roslisp:ros-error (low-level giskard-init)
                           "?left-tool-frame is unknown. ~
                           Did you load a robot description package?")
        (if (cut:is-var ?right-tool-frame)
            (roslisp:ros-error (low-level giskard-init)
                               "?right-tool-frame is unknown. ~
                                Did you load a robot description package?")
            (let ((source-pose (cl-tf:transform-pose-stamped
                                cram-tf:*transformer*
                                :target-frame cram-tf:*robot-base-frame*
                                :pose (cl-tf:make-pose-stamped
                                       ?right-tool-frame
                                       0.0
                                       (cl-tf:make-identity-vector)
                                       (cl-tf:make-identity-rotation))
                                :timeout cram-tf:*tf-default-timeout*))
                  (destination-pose (cl-tf:transform-pose-stamped
                                     cram-tf:*transformer*
                                     :target-frame cram-tf:*robot-base-frame*
                                     :pose (cl-tf:make-pose-stamped
                                            ?left-tool-frame
                                            0.0
                                            (cl-tf:make-identity-vector)
                                            (cl-tf:make-identity-rotation))
                                     :timeout cram-tf:*tf-default-timeout*)))

              (let* ((pouring-constraints (call-learned-constraints-service
                                           source-type source-pose
                                           destination-type destination-pose
                                           pour-volume liquid-in-source))
                     (?approach-constraints (cadr (assoc :approach pouring-constraints)))
                     (?tilt-down-constraints (cadr (assoc :tilt-down pouring-constraints)))
                     (?tilt-back-constraints (cadr (assoc :tilt-back pouring-constraints)))
                     (phases (list
                              (an action
                                  (to approach)
                                  (constraints ?approach-constraints))
                              (an action
                                  (to tilt-down)
                                  (constraints ?tilt-down-constraints))
                              (an action
                                  (to wait)
                                  (duration 3.0))
                              (an action
                                  (to tilt-back)
                                  (constraints ?tilt-back-constraints))
                              (an action
                                  (to wait)
                                  (duration 3.0))
                              ;; (an action
                              ;;     (to retract)
                              ;;     (left ?left-retract-poses)
                              ;;     (right ?right-retract-poses))
                              )))
                (copy-designator action-designator :new-description `((:phases ,phases)))))))))

;; (declaim (inline car-last))
(defun car-last (some-list)
  (if (listp some-list)
      (car (last some-list))
      some-list))

(def-fact-group pr2-pouring-plans (action-grounding)

  (<- (action-grounding ?action-designator (perform-phases-in-sequence ?updated-action-designator))
    (or (desig-prop ?action-designator (:to :pour-activity)) ;; cartesian one-armed
        (desig-prop ?action-designator (:type :pouring-activity)))
    (desig-prop ?action-designator (:arm ?arm))
    (not (equal ?arm (:left :right)))
    ;; source
    (once (or (cpoe:object-in-hand ?source-designator ?arm)
              (desig-prop ?action-designator (:source ?source-designator))))
    (current-designator ?source-designator ?current-source-designator)
    (desig-prop ?current-source-designator (:type ?source-type))
    (lisp-fun get-object-pose ?current-source-designator ?source-pose)
    (object-type-grasp ?source-type ?grasp)
    ;; destination / target
    (desig-prop ?action-designator (:target ?destination-designator))
    (current-designator ?destination-designator ?current-destination-designator)
    (desig-prop ?current-destination-designator (:type ?destination-type))
    (lisp-fun get-object-pose ?current-destination-designator ?destination-pose)
    ;; so we have (an action (to pour) (destination (an object (pose ...) (type ...))))
    ;; now we need to add the phases with the corresponding via-points and angles
    ;; find the missing info
    ;; cartesian pouring:
    (lisp-fun get-object-type-pour-pose ?source-type ?current-destination-designator
              :left ?grasp ?left-pour-pose)
    (lisp-fun get-object-type-pour-pose ?source-type ?current-destination-designator
              :right ?grasp ?right-pour-pose)
    (lisp-fun cram-math:degrees->radians 120 ?angle)
    (lisp-fun get-tilted-pose ?left-pour-pose ?angle :left ?grasp ?left-tilt-pose)
    (lisp-fun get-tilted-pose ?right-pour-pose ?angle :right ?grasp ?right-tilt-pose)
    ;; retract phase:
    (lisp-fun get-object-type-grasp-pose ?source-type ?destination-pose :left ?grasp
              ?left-grasp-pose)
    (lisp-fun get-object-type-grasp-pose ?source-type ?destination-pose :right ?grasp
              ?right-grasp-pose)
    ;;
    (lisp-fun get-object-type-pregrasp-pose ?source-type ?left-grasp-pose :left ?grasp
              ?left-retract-pose)
    (lisp-fun get-object-type-pregrasp-pose ?source-type ?right-grasp-pose :right ?grasp
              ?right-retract-pose)
    ;; create new designator with updated appended action-description
    (lisp-fun append-pour-cartesian-action-designator ?action-designator ?arm
              ?left-pour-pose ?right-pour-pose
              ?left-tilt-pose ?right-tilt-pose
              ?left-retract-pose ?right-retract-pose
              ?updated-action-designator))


  (<- (action-grounding ?action-designator (pour-activity ?updated-action-designator))
    (or (desig-prop ?action-designator (:to :pour-activity)) ;; yaml two-arm
        (desig-prop ?action-designator (:type :pouring-activity)))
    (desig-prop ?action-designator (:arm ?arm))
    (equal ?arm (:left :right))
    ;; source
    (once (or (cpoe:object-in-hand ?source-designator :right)
              (desig-prop ?action-designator (:source ?source-designator))))
    (current-designator ?source-designator ?current-source-designator)
    (desig-prop ?current-source-designator (:type ?source-type))
    (lisp-fun get-object-pose ?current-source-designator ?source-pose)
    (object-type-grasp ?source-type ?source-grasp)
    ;; destination / target
    (once (or (cpoe:object-in-hand ?destination-designator :left)
              (desig-prop ?action-designator (:target ?destination-designator))))
    (current-designator ?destination-designator ?current-destination-designator)
    (desig-prop ?current-destination-designator (:type ?destination-type))
    (lisp-fun get-object-pose ?current-destination-designator ?destination-pose)
    (object-type-grasp ?destination-type ?destination-grasp)
    ;; volume
    (desig-prop ?action-designator (:pour-volume ?pour-volume))
    (equal ?liquid-in-source 0.00025)
    ;; so we have (an action (to pour) (destination (an object (pose ...) (type ...))))
    ;; now we need to add the phases with the corresponding via-points and angles
    ;; find the missing info
    ;; retract phase:
    (lisp-fun get-object-type-grasp-pose ?source-type ?source-pose
              :left ?source-grasp ?left-source-grasp-pose)
    (lisp-fun get-object-type-grasp-pose ?source-type ?source-pose
              :right ?source-grasp ?right-source-grasp-pose)
    (lisp-fun get-object-type-grasp-pose ?destination-type ?destination-pose
              :left ?destination-grasp ?left-destination-grasp-pose)
    (lisp-fun get-object-type-grasp-pose ?destination-type ?destination-pose
              :right ?destination-grasp ?right-destination-grasp-pose)
    ;;
    (lisp-fun get-object-type-pregrasp-pose ?source-type ?left-source-grasp-pose
              :left ?source-grasp ?left-source-retract-pose)
    (lisp-fun get-object-type-pregrasp-pose ?source-type ?right-source-grasp-pose
              :right ?source-grasp ?right-source-retract-pose)
    (lisp-fun get-object-type-pregrasp-pose ?destination-type ?left-destination-grasp-pose
              :left ?destination-grasp ?left-destination-retract-pose)
    (lisp-fun get-object-type-pregrasp-pose ?destination-type ?right-destination-grasp-pose
              :right ?destination-grasp ?right-destination-retract-pose)
    ;; create new designator with updated appended action-description
    (lisp-fun append-pour-giskard-action-designator ?action-designator ?arm
              ?source-type ?destination-type
              ?pour-volume ?liquid-in-source
              ?left-destination-retract-pose ?right-source-retract-pose
              ?updated-action-designator))


  (<- (action-grounding ?action-designator (move-arms-in-sequence ?left-poses ?right-poses))
    (desig-prop ?action-designator (:to :approach))
    (once (or (desig-prop ?action-designator (:left ?left-poses))
              (equal ?left-poses nil)))
    (once (or (desig-prop ?action-designator (:right ?right-poses))
              (equal ?right-poses nil))))

  ;; (<- (action-grounding ?action-designator (tilt ?left-goal-pose ?right-goal-pose))
  ;;   (desig-prop ?action-designator (:to :my-tilt-angle))
  ;;   (desig-prop ?action-designator (:left ?left-initial-poses))
  ;;   (desig-prop ?action-designator (:right ?right-initial-poses))
  ;;   (desig-prop ?action-designator (:angle ?angle))
  ;;   (lisp-fun get-tilted-pose ?left-initial-poses ?angle ...))

  (<- (action-grounding ?action-designator (move-arms-in-sequence ?left-last-pose ?right-last-pose))
    (desig-prop ?action-designator (:to :tilt-to))
    (desig-prop ?action-designator (:left ?left-poses))
    (desig-prop ?action-designator (:right ?right-poses))
    (lisp-fun car-last ?left-poses ?left-last-pose)
    (lisp-fun car-last ?right-poses ?right-last-pose))

  (<- (action-grounding ?action-designator (giskard-yaml ?phase ?constraints))
    (or (desig-prop ?action-designator (:to :approach))
        (desig-prop ?action-designator (:to :tilt-down))
        (desig-prop ?action-designator (:to :tilt-back)))
    (desig-prop ?action-designator (:to ?phase))
    (desig-prop ?action-designator (:constraints ?constraints)))

  (<- (action-grounding ?action-designator (wait ?duration))
    (desig-prop ?action-designator (:to :wait))
    (desig-prop ?action-designator (:duration ?duration))))
