;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :pr2-manipulation-knowledge)

(defun find-designator-pose-in-link (gripper-link designator)
  (find-if (lambda (pose-frame-id)
             (equal (tf::ensure-fully-qualified-name gripper-link)
                    (tf::ensure-fully-qualified-name pose-frame-id)))
           (desig-prop-values designator 'pose)
           :key #'tf:frame-id))

(defun get-latest-detected-object-pose (object-designator)
  "Returns the pose where the object has been detected using
  perception the last time, i.e. the object pose contained in the
  latest valid object designator."
  (designator-pose (newest-valid-designator object-designator)))

(defun calculate-grasp-trajectory-point
    (robot-pose object-pose grasp side tool-length)
  (let ((grasp-orientation
          (car (get-grasps grasp (lambda (grasp-name)
                                   (or (eq grasp-name grasp)
                                       (eq grasp-name side)))))))
    (assert grasp-orientation)
    (cl-transforms:transform->pose
     (cl-transforms:transform*
      (cl-transforms:make-transform
       (cl-transforms:origin object-pose)
       (cl-transforms:orientation robot-pose))
      (cl-transforms:transform-inv
       (cl-transforms:make-transform
        (cl-transforms:v-
         (get-tool-vector)
         (cl-transforms:v* (get-tool-direction-vector) tool-length))
        grasp-orientation))))))

(defun calculate-put-down-hand-pose (gripper-link object-designator put-down-pose
                                     &key robot-pose)
  (declare (type tf:pose-stamped put-down-pose))
  (let ((current-object (desig:current-desig object-designator)))
    (desig:with-desig-props (desig-props:at) current-object
      (assert desig-props:at () "Object ~a needs to have an `at' property"
              current-object)
      (desig:with-desig-props (in z-offset) at
        (assert (eq in 'gripper) ()
                "Object ~a needs to be in the gripper" current-object)
        (assert z-offset () "Object ~a needs to have a `height' property" current-object)
        (let* ((pose-in-gripper (find-designator-pose-in-link gripper-link at))
               (put-down-pose-in-fixed-frame  (tf:transform-pose
                                              cram-roslisp-common:*tf*
                                             :target-frame designators-ros:*fixed-frame*
                                              :pose put-down-pose))
               (put-down-pose (if (not robot-pose)
                                  put-down-pose-in-fixed-frame
                                  (tf:copy-pose-stamped
                                   put-down-pose-in-fixed-frame
                                   :orientation (cl-transforms:q*
                                                 (tf:orientation
                                                  (find-designator-pose-in-link "base_footprint" at))
                                                 (tf:orientation robot-pose))))))
          (assert pose-in-gripper () "Object ~a needs to have a `pose' property" current-object)
          (cl-transforms:transform->pose
           (cl-transforms:transform*
            (cl-transforms:pose->transform put-down-pose)
            (cl-transforms:make-transform
             (cl-transforms:make-3d-vector 0 0 desig-props:z-offset)
             (cl-transforms:make-identity-rotation))
            (cl-transforms:transform-inv
             (cl-transforms:transform*
              (cl-transforms:make-transform
               (tf:v* (get-tool-vector) -1)
               (cl-transforms:make-identity-rotation))
              (cl-transforms:pose->transform pose-in-gripper))))))))))

(defun calculate-object-lift-pose (gripper-link object-designator lifting-height)
  (let* ((current-object-designator (current-desig object-designator))
         (object-location (desig-prop-value current-object-designator 'at))
         (lift-transform
           (cl-transforms:transform*
            (cl-transforms:pose->transform
             (get-latest-detected-object-pose object-designator))
            (cl-transforms:make-transform
             (cl-transforms:make-3d-vector 0 0 lifting-height)
             (cl-transforms:make-identity-rotation)))))
    ;; If the object is in the gripper, we actually know about the
    ;; grasp. In that case, we use that information and return a
    ;; pose. Otherwise, we just return a point that is
    ;; `lifting-height' above the object.
    (cond ((eql (desig-prop-value object-location 'in) 'gripper)
           (cl-transforms:transform->pose
            (cl-transforms:transform*
             lift-transform
             (cl-transforms:transform-inv
              (cl-transforms:pose->transform
               (find-designator-pose-in-link gripper-link object-location)))
             (cl-transforms:make-transform
              (get-tool-vector) (cl-transforms:make-identity-rotation)))))
          (t (cl-transforms:translation lift-transform)))))

(def-fact-group pick-and-place-manipulation (trajectory-point)

  (<- (trajectory-point ?designator ?point ?side)
    (trajectory-desig? ?designator)
    (desig-prop ?designator (to grasp))
    (desig-prop ?designator (obj ?object))
    (desig-location-prop ?object ?pose)
    (-> (desig-prop ?desig (side ?side))
        (available-arms ?object (?side))
        (and
         (once
          (available-arms ?object ?sides))
         (member ?side ?sides)))
    (lisp-fun cl-transforms:origin ?pose ?point))

  (<- (trajectory-point ?designator ?robot-reference-pose ?point ?side)
    (desig-prop ?designator (to grasp))
    (desig-prop ?designator (obj ?object))
    (trajectory-point ?designator ?point-in-object ?side)
    (object-designator-tool-length ?object ?grasp ?tool-length)
    (desig-location-prop ?object ?pose)
    (lisp-fun calculate-grasp-trajectory-point
              ?robot-reference-pose ?pose ?grasp ?side ?tool-length
              ?point))

  (<- (trajectory-point ?designator ?point ?side)
    (desig-prop ?designator (to put-down))
    (trajectory-point ?designator ?_ ?point ?side))

  (<- (trajectory-point ?designator ?robot-reference-pose ?point ?side)
    (trajectory-desig? ?designator)
    (desig-prop ?designator (to put-down))
    (desig-prop ?designator (obj ?object))
    (-> (desig-prop ?desig (side ?side)) (true) (true))
    (object-in-hand ?object ?side)
    (end-effector-link ?side ?link)
    (desig-prop ?designator (at ?location))
    (lisp-fun current-desig ?location ?current-location)
    (lisp-fun reference ?current-location ?put-down-pose)
    (-> (not (bound ?robot-reference-pose))
        (and (lisp-fun calculate-put-down-hand-pose ?link ?object ?put-down-pose
                       ?pose)
             (lisp-fun cl-transforms:origin ?pose ?point))
        (-> (orientation-matters ?object)
            (lisp-fun calculate-put-down-hand-pose
                      ?link ?object ?put-down-pose ?point)
            (and
             (lisp-fun calculate-put-down-hand-pose
                       ?link ?object ?put-down-pose
                       :robot-pose ?robot-reference-pose
                       ?point)))))

  (<- (trajectory-point ?designator ?point ?side)
    (trajectory-desig? ?designator)
    (desig-prop ?designator (to lift))
    (desig-prop ?designator (obj ?object))
    (-> (desig-prop ?desig (side ?side)) (true) (true))
    (object-in-hand ?object ?side)
    (end-effector-link ?side ?link)
    (once (or (desig-prop ?designator (distance ?lifting-height))
              (== ?lifting-height 0.10)))
    (lisp-fun calculate-object-lift-pose ?link ?object ?lifting-height
              ?point))

  (<- (trajectory-point ?designator ?robot-reference-pose ?point ?side)
    (desig-prop ?designator (to lift))
    (trajectory-point ?designator ?point ?side)))
