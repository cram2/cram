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

(defparameter *both-arms-carry-pose*
  (tf:make-pose-stamped
   "torso_lift_link" 0.0
   (cl-transforms:make-3d-vector 0.5 0.0 -0.1)
   (cl-transforms:make-quaternion 0 0 0 1)))

(defun set-robot-reach-pose (side pose &key tool-frame seed-state)
  (or
   (crs:prolog `(crs:once
                 (robot ?robot)
                 (%object ?_ ?robot ?robot-instance)
                 (crs:lisp-fun reach-pose-ik ?robot-instance ,pose
                               :side ,side :tool-frame ,tool-frame
                               :seed-state ,seed-state
                               ?ik-solutions)
                 (member ?ik-solution ?ik-solutions)
                 (assert (joint-state ?_ ?robot ?ik-solution))))
   (cpl-impl:fail 'cram-plan-failures:manipulation-pose-unreachable)))

(defun action-end-effector-links (action-designator)
  (cut:force-ll
   (cut:lazy-mapcar (lambda (solution)
                      (cut:with-vars-bound (?end-effector-link) solution
                        (unless (cut:is-var ?end-effector-link)
                          ?end-effector-link)))
                    (crs:prolog
                     `(and
                       (trajectory-point ,action-designator ?_ ?side)
                       (end-effector-link ?side ?end-effector-link))))))

(defun get-link-orientation-in-robot (link-name &key (base-link "base_footprint"))
  (cl-transforms:rotation
   (tf:lookup-transform
    cram-roslisp-common:*tf* :source-frame link-name :target-frame base-link)))

(defun execute-action-trajectory-points (action-designator &optional object-name)
  (cut:force-ll
   (cut:lazy-mapcar
    (lambda (solution)
      (cram-plan-knowledge:on-event
       (make-instance 'cram-plan-knowledge:robot-state-changed))
      solution)
    (crs:prolog `(and
                  (bullet-world ?world)
                  (robot ?robot)
                  (object-pose ?world ?robot ?robot-pose)
                  (trajectory-point ,action-designator ?robot-pose ?point ?side)
                  (crs:once
                   ,(if object-name
                        `(valid-grasp ?world ,object-name ?grasp ?sides)
                        `(grasp ?grasp))
                   (member ?side ?sides)
                   (%object ?world ?robot ?robot-instance)
                   (crs:-> (crs:lisp-type ?point cl-transforms:3d-vector)
                           (crs:lisp-fun reach-point-ik ?robot-instance ?point
                                         :side ?side :grasp ?grasp ?ik-solutions)
                           (crs:lisp-fun reach-pose-ik ?robot-instance ?point
                                         :side ?side ?ik-solutions))
                   (member ?ik-solution ?ik-solutions)
                   (assert (joint-state ?world ?robot ?ik-solution))))))))

(defun execute-container-opened (object sides)
  (declare (ignore object sides))
  nil)

(defun execute-container-closed (object sides)
  (declare (ignore object sides))
  nil)

(defun carry-with-both-hands (object)
  (declare (ignore object))
  (cut:with-vars-strictly-bound (?left-end-effector
                                 ?right-end-effector
                                 ?left-parking-joint-states
                                 ?right-parking-joint-states)
      (cut:lazy-car
       (crs:prolog `(and
                     (end-effector-link :left ?left-end-effector)
                     (end-effector-link :right ?right-end-effector)
                     (robot-arms-parking-joint-states
                      ?left-parking-joint-states :left)
                     (robot-arms-parking-joint-states
                      ?right-parking-joint-states :right))))
    (let* ((left-gripper-transform
             (tf:lookup-transform
              cram-roslisp-common:*tf*
              :source-frame ?left-end-effector :target-frame "base_footprint"))
           (right-gripper-transform
             (tf:lookup-transform
              cram-roslisp-common:*tf*
              :source-frame ?right-end-effector :target-frame "base_footprint"))
           (left->right-arm-vector
             (cl-transforms:v-
              (cl-transforms:translation left-gripper-transform)
              (cl-transforms:translation right-gripper-transform)))
           (arm-distance (cl-transforms:v-norm left->right-arm-vector))
           (carry-pose-in-base
             (tf:transform-pose
              cram-roslisp-common:*tf*
              :target-frame "base_footprint" :pose *both-arms-carry-pose*)))
      (set-robot-reach-pose
       :left carry-pose-in-base
       :tool-frame (cl-transforms:make-pose
                    (cl-transforms:make-3d-vector (/ arm-distance 2) 0 0)
                    (cl-transforms:q-inv
                     (cl-transforms:rotation left-gripper-transform)))
       :seed-state (make-joint-state-message ?left-parking-joint-states))
      (set-robot-reach-pose
       :right carry-pose-in-base
       :tool-frame (cl-transforms:make-pose
                    (cl-transforms:make-3d-vector (/ arm-distance 2) 0 0)
                    (cl-transforms:q-inv
                     (cl-transforms:rotation right-gripper-transform)))
       :seed-state (make-joint-state-message ?right-parking-joint-states)))))

(defun carry-with-one-hand (side object link)
  (declare (ignore object))
  (cut:with-vars-strictly-bound (?robot ?parking-pose ?joint-states)
      (cut:lazy-car
       (crs:prolog
        `(and (robot ?robot)
              (end-effector-parking-pose ?parking-pose ,side)
              (robot-arms-parking-joint-states ?joint-states ,side))))
    (let* ((robot-object (object *current-bullet-world* ?robot))
           (ik-solution
             (cut:lazy-car
              (bullet-reasoning:get-ik
               robot-object
               (tf:copy-pose-stamped
                ?parking-pose :orientation (get-link-orientation-in-robot link))
               :ik-namespace (side->ik-namespace side)
               :seed-state (make-joint-state-message ?joint-states)))))
      (unless ik-solution
        (cpl-impl:fail 'cram-plan-failures:manipulation-pose-unreachable))
      (set-robot-state-from-joints ik-solution robot-object))))

(defun park (side)
  (crs:prolog `(and
                (robot ?robot)
                (robot-arms-parking-joint-states ?joint-states ,side)
                (assert (joint-state ?_ ?robot ?joint-states)))))

(defun execute-park (sides objects-in-hand)
  (flet ((object-in-both-hands (objects-in-hand)
           (when objects-in-hand
             (eql (second (first objects-in-hand))
                  (second (second objects-in-hand))))))
    (cut:force-ll sides)
    (cut:force-ll objects-in-hand)
    (cond ((object-in-both-hands objects-in-hand)
           (carry-with-both-hands (second (first objects-in-hand))))
          (t
           (dolist (side sides)
             (let ((object-in-hand (assoc side objects-in-hand)))
               (if object-in-hand
                   (apply #'carry-with-one-hand object-in-hand)
                   (park side)))))))
  (break))

(defun execute-lift (designator)
  (or
   (execute-action-trajectory-points designator)
   (cpl-impl:fail 'cram-plan-failures:manipulation-pose-unreachable)))

(defun execute-grasp (designator object)
  (let* ((current-object (desig:newest-valid-designator object))
         (object-name (desig:object-identifier (desig:reference current-object))))
    (or
     (when (execute-action-trajectory-points designator object-name)
       (let ((gripper-links (action-end-effector-links designator)))
         (assert gripper-links)
         (dolist (gripper-link gripper-links t)
           (cram-plan-knowledge:on-event
            (make-instance 'cram-plan-knowledge:object-attached
              :object current-object :link gripper-link)))))
     (cpl-impl:fail 'cram-plan-failures:manipulation-pose-unreachable))))

(defun execute-put-down (designator object)
  (let* ((current-object (desig:newest-valid-designator object))
         (object-name (desig:object-identifier (desig:reference current-object))))
    (or
     (when (execute-action-trajectory-points designator object-name)
       (let ((gripper-links (action-end-effector-links designator)))
         (assert gripper-links)
         (dolist (gripper-link gripper-links t)
           (cram-plan-knowledge:on-event
            (make-instance 'cram-plan-knowledge:object-detached
              :object current-object :link gripper-link)))))
     (cpl-impl:fail 'cram-plan-failures:manipulation-pose-unreachable))))

(def-process-module projection-manipulation (input)
  (let ((action (desig:reference input 'projection-designators:projection-role)))
    (apply (symbol-function (car action)) (cdr action))))
