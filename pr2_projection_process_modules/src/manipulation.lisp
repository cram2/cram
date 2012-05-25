;;; Copyright (c) 2011, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
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

(in-package :projection-process-modules)

(defun set-robot-reach-pose (side pose)
  (or
   (crs:prolog `(crs:once
                 (robot ?robot)
                 (%object ?_ ?robot ?robot-instance)
                 (crs:lisp-fun reach-pose-ik ?robot-instance ,pose :side ,side
                               ?ik-solutions)
                 (member ?ik-solution ?ik-solutions)
                 (assert (joint-state ?_ ?robot ?ik-solution))))
   (cpl-impl:fail 'cram-plan-failures:manipulation-pose-unreachable)))

(defun calculate-put-down-pose (object put-down-pose robot-pose)
  (let ((current-object (desig:newest-valid-designator object)))
    (desig:with-desig-props (desig-props:at) current-object
      (assert desig-props:at () "Object ~a needs to have an `at' property"
              object)
      (desig:with-desig-props (desig-props:in desig-props:pose desig-props:height)
          desig-props:at
        (assert (and (eq desig-props:in 'gripper)) ()
                "Object ~a needs to be in the gripper" object)
        (assert desig-props:pose () "Object ~a needs to have a `pose' property" object)
        (assert desig-props:height () "Object ~a needs to have a `height' property" object)
        (let* ((put-down-pose-in-robot (cl-transforms:transform*
                                        (cl-transforms:transform-inv
                                         (cl-transforms:reference-transform robot-pose))
                                        put-down-pose))
               (goal-in-robot (cl-transforms:transform*
                               (cl-transforms:make-transform
                                (cl-transforms:make-3d-vector 0 0 desig-props:height)
                                (cl-transforms:make-identity-rotation))
                               (cl-transforms:reference-transform
                                put-down-pose-in-robot)
                               (cl-transforms:transform-inv
                                (cl-transforms:reference-transform desig-props:pose)))))
          (cl-transforms:transform->pose
           (cl-transforms:transform*
            (cl-transforms:reference-transform robot-pose)
            goal-in-robot)))))))

(defun execute-container-opened (object side)
  (declare (ignore object side))
  nil)

(defun execute-container-closed (object side)
  (declare (ignore object side))
  nil)

(defun execute-park (side &optional object)
  (declare (ignore object))
  (let ((sides (ecase side
                 (:left '(:left))
                 (:right '(:right))
                 (:both '(:left :right)))))
    ;; TODO(moesenle): if parking with object, make sure to use the
    ;; correct end-effector orientation and use IK.
    (dolist (side sides)
      (crs:prolog `(and
                    (robot-arms-parking-joint-states ?joint-states ,side)
                    (assert (joint-states pr2 ?joint-states)))))))

(defun execute-lift (side object distance)
  (cut:with-vars-bound (?end-effector-pose)
      (cut:lazy-car
       (crs:prolog `(and
                     (end-effector-link ,side ?link-name)
                     (robot ?robot)
                     (link-pose ?_ ?robot ?link-name ?end-effector-pose))))
    (assert (not (cut:is-var ?end-effector-pose)))
    (let* ((current-object (desig:newest-valid-designator object))
           (object-pose (desig:designator-pose current-object))
           (lift-pose (cl-transforms:transform->pose
                       (cl-transforms:transform*
                        (cl-transforms:pose->transform object-pose)
                        (cl-transforms:make-transform
                         (cl-transforms:make-3d-vector 0 0 distance)
                         (cl-transforms:orientation ?end-effector-pose))))))
      (set-robot-reach-pose side lift-pose))))

(defun execute-grasp (object side)
  (let* ((current-object (desig:newest-valid-designator object))
         (object-name (desig:object-identifier (desig:reference current-object))))
    (cut:with-vars-bound (?gripper-link)
        (or
         (cram-utilities:lazy-car
          (crs:prolog `(crs:once
                        (grasp ?grasp)
                        (side ,side)
                        (robot ?robot)
                        (end-effector-link ,side ?gripper-link)
                        (%object ?_ ?robot ?robot-instance)
                        (%object ?_ ,object-name ?object-instance)
                        (crs:lisp-fun reach-object-ik ?robot-instance ?object-instance
                                      :side ,side :grasp ?grasp ?ik-solutions)
                        (member ?ik-solution ?ik-solutions)
                        (assert (joint-state ?_ ?robot ?ik-solution)))))
         (cpl-impl:fail 'cram-plan-failures:manipulation-pose-unreachable))
      (assert (not (cut:is-var ?gripper-link)))
      (cram-plan-knowledge:on-event
       (make-instance 'cram-plan-knowledge:object-attached
         :object current-object :link ?gripper-link)))))

(defun execute-put-down (object location side)
  (let* ((current-object (desig:newest-valid-designator object)))
    (cut:with-vars-bound (?robot-pose ?gripper-link)
        (cut:lazy-car
         (crs:prolog `(crs:once
                       (robot ?robot)
                       (end-effector-link ,side ?gripper-link)
                       (pose ?_ ?robot ?robot-pose))))
      (assert (not (cut:is-var ?robot-pose)))
      (assert (not (cut:is-var ?gripper-link)))
      (let ((put-down-pose (calculate-put-down-pose
                            current-object (desig:reference (desig:current-desig location))
                            ?robot-pose)))
        (set-robot-reach-pose side put-down-pose)
        (cram-plan-knowledge:on-event
         (make-instance 'cram-plan-knowledge:object-detached
           :object current-object :link ?gripper-link))))))

(def-process-module projection-manipulation (input)
  (let ((action (desig:reference input 'projection-designators:projection-role)))
    (apply (symbol-function (car action)) (cdr action))
    (cram-plan-knowledge:on-event
     (make-instance 'cram-plan-knowledge:robot-state-changed))))
