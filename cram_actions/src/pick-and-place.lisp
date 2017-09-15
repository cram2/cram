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

(in-package :cram-common-designators)

(defparameter *put-down-pose-z-offset* 0.01
  "Extra padding between the calculated put-down pose and the surface [m].")

(defun find-designator-pose-in-link (gripper-link designator)
  (find-if (lambda (pose-frame-id)
             (equal (cl-transforms-stamped:unslash-frame gripper-link)
                    (cl-transforms-stamped:unslash-frame pose-frame-id)))
           (desig-prop-values designator :pose)
           :key #'frame-id))

(defun get-latest-detected-object-pose (object-designator)
  "Returns the pose where the object has been detected using
  perception the last time, i.e. the object pose contained in the
  latest valid object designator."
  (designator-pose (newest-effective-designator object-designator)))

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
  (declare (type pose-stamped put-down-pose))
  (let ((current-object (desig:current-desig object-designator)))
    (desig:with-desig-props (at) current-object
      (assert at () "Object ~a needs to have an `at' property"
              current-object)
      (desig:with-desig-props (in z-offset) at
        (assert (eq in :gripper) ()
                "Object ~a needs to be in the gripper" current-object)
        (assert z-offset () "Object ~a needs to have a `z-offset' property" current-object)
        (let* ((pose-in-gripper (find-designator-pose-in-link gripper-link at))
               (put-down-pose-in-fixed-frame
                 (cl-transforms-stamped:transform-pose-stamped
                  *transformer*
                  :target-frame *fixed-frame*
                  :pose put-down-pose
                  :timeout *tf-default-timeout*))
               (put-down-pose
                 (if (not robot-pose)
                     put-down-pose-in-fixed-frame
                     (copy-pose-stamped
                      put-down-pose-in-fixed-frame
                      :orientation (cl-transforms:q*
                                    (cl-transforms:orientation
                                     (find-designator-pose-in-link "base_footprint" at))
                                    (cl-transforms:orientation robot-pose))))))
          (assert pose-in-gripper () "Object ~a needs to have a `pose' property" current-object)
          (cl-transforms:transform->pose
           (cl-transforms:transform*
            (cl-transforms:pose->transform put-down-pose)
            (cl-transforms:make-transform
             (cl-transforms:make-3d-vector 0 0 (+ z-offset *put-down-pose-z-offset*))
             (cl-transforms:make-identity-rotation))
            (cl-transforms:transform-inv
             (cl-transforms:transform*
              (cl-transforms:make-transform
               (cl-transforms:v* (get-tool-vector) -1)
               (cl-transforms:make-identity-rotation))
              (cl-transforms:pose->transform pose-in-gripper))))))))))

(defun calculate-pour-hand-pose (gripper-link object-designator pour-pose
                                 &key robot-pose)
  (declare (type pose-stamped pour-pose))
  (let ((current-object (desig:current-desig object-designator)))
    (desig:with-desig-props (at) current-object
      (assert at () "Object ~a needs to have an `at' property"
              current-object)
      (desig:with-desig-props (in z-offset) at
        (assert (eq in :gripper) ()
                "Object ~a needs to be in the gripper" current-object)
        (assert z-offset () "Object ~a needs to have a `z-offset' property" current-object)
        (let* ((pose-in-gripper (find-designator-pose-in-link gripper-link at))
               (put-down-pose-in-fixed-frame
                 (cl-transforms-stamped:transform-pose-stamped
                  *transformer*
                  :target-frame *fixed-frame*
                  :pose pour-pose
                  :timeout *tf-default-timeout*))
               (pour-pose
                 (if (not robot-pose)
                     put-down-pose-in-fixed-frame
                     (copy-pose-stamped
                      put-down-pose-in-fixed-frame
                      :orientation (cl-transforms:q*
                                    (cl-transforms:orientation
                                     (find-designator-pose-in-link "base_footprint" at))
                                    (cl-transforms:orientation robot-pose))))))
          (assert pose-in-gripper () "Object ~a needs to have a `pose' property" current-object)
          (cl-transforms:transform->pose
           (cl-transforms:transform*
            (cl-transforms:pose->transform pour-pose)
            (cl-transforms:make-transform
             (cl-transforms:make-3d-vector 0 0 (+ z-offset z-offset z-offset
                                                  *put-down-pose-z-offset*))
             ;; (cl-transforms:make-quaternion 0 0.75 0 0.25)
             (cl-transforms:make-identity-rotation))
            (cl-transforms:transform-inv
             (cl-transforms:transform*
              (cl-transforms:make-transform
               (cl-transforms:v* (get-tool-vector) -1)
               (cl-transforms:make-identity-rotation))
              (cl-transforms:pose->transform pose-in-gripper))))))))))

(defun calculate-object-lift-pose (gripper-link object-designator lifting-height)
  (let* ((current-object-designator (current-desig object-designator))
         (object-location (desig-prop-value current-object-designator :at))
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
    (cond ((eql (desig-prop-value object-location :in) :gripper)
           (cl-transforms:transform->pose
            (cl-transforms:transform*
             lift-transform
             (cl-transforms:transform-inv
              (cl-transforms:pose->transform
               (find-designator-pose-in-link gripper-link object-location)))
             (cl-transforms:make-transform
              (get-tool-vector) (cl-transforms:make-identity-rotation)))))
          (t (cl-transforms:translation lift-transform)))))

(defun object-type->tool-length (object-type)
  (let ((bounding-box (btr:item-dimensions object-type)))
    (cram-robot-interfaces:calculate-bounding-box-tool-length
     bounding-box)))

(def-fact-group pick-and-place-manipulation (trajectory-point)

  (<- (%object-type-tool-length ?object-type ?grasp ?tool-length)
    (object-type-grasp ?object-type ?grasp ?_)
    (lisp-fun object-type->tool-length ?object-type ?tool-length))

  (<- (object-type-tool-length ?object-type ?grasp ?tool-length)
    (once
     (or
      (%object-type-tool-length ?object-type ?grasp ?tool-length)
      (== ?tool-length 0.0)))
    (robot ?robot)
    (grasp ?robot ?grasp))

  (<- (object-designator-tool-length
       ?object-designator ?grasp ?tool-length)
    (lisp-fun desig:current-desig ?object-designator ?current-object-designator)
    (desig:desig-prop ?current-object-designator (:type ?object-type))
    (object-type-tool-length ?object-type ?grasp ?tool-length))

  (<- (trajectory-point ?designator ?point ?side)
    (trajectory-desig? ?designator)
    (desig-prop ?designator (:to :grasp))
    (or (desig-prop ?designator (:obj ?object))
        (desig-prop ?designator (:object ?object)))
    (newest-effective-designator ?object ?current-object)
    (desig-location-prop ?current-object ?pose)
    (-> (desig-prop ?desig (:side ?side))
        (available-arms ?current-object (?side))
        (and
         (once
          (available-arms ?current-object ?sides))
         (member ?side ?sides)))
    (lisp-fun cl-transforms:origin ?pose ?point))

  (<- (trajectory-point ?designator ?robot-reference-pose ?point ?side)
    (desig-prop ?designator (:to :grasp))
    (or (desig-prop ?designator (:obj ?object))
        (desig-prop ?designator (:object ?object)))
    (newest-effective-designator ?object ?current-object)
    (trajectory-point ?designator ?point-in-object ?side)
    (object-designator-tool-length ?current-object ?grasp ?tool-length)
    (desig-location-prop ?current-object ?pose)
    (lisp-fun calculate-grasp-trajectory-point
              ?robot-reference-pose ?pose ?grasp ?side ?tool-length
              ?point))

  (<- (trajectory-point ?designator ?point ?side)
    (desig-prop ?designator (:to :put-down))
    (trajectory-point ?designator ?_ ?point ?side))

  (<- (trajectory-point ?designator ?robot-reference-pose ?point ?side)
    (trajectory-desig? ?designator)
    (desig-prop ?designator (:to :put-down))
    (or (desig-prop ?designator (:obj ?object))
        (desig-prop ?designator (:object ?object)))
    (current-designator ?object ?current-object)
    (-> (desig-prop ?desig (:side ?side)) (true) (true))
    (object-in-hand ?current-object ?side)
    (robot ?robot)
    (end-effector-link ?robot ?side ?link)
    (desig-prop ?designator (:at ?location))
    (lisp-fun current-desig ?location ?current-location)
    (lisp-fun reference ?current-location ?put-down-pose)
    (-> (not (bound ?robot-reference-pose))
        (and (lisp-fun calculate-put-down-hand-pose ?link ?current-object ?put-down-pose
                       ?pose)
             (lisp-fun cl-transforms:origin ?pose ?point))
        (-> (cram-object-interfaces:orientation-matters ?current-object)
            (lisp-fun calculate-put-down-hand-pose
                      ?link ?current-object ?put-down-pose ?point)
            (and
             (lisp-fun calculate-put-down-hand-pose
                       ?link ?current-object ?put-down-pose
                       :robot-pose ?robot-reference-pose
                       ?point)))))

  (<- (trajectory-point ?designator ?pose ?side)
    (desig-prop ?designator (:to :pour))
    (trajectory-point ?designator ?_ ?pose ?side))

  (<- (trajectory-point ?designator ?robot-reference-pose ?pose ?side)
    (trajectory-desig? ?designator)
    (desig-prop ?designator (:to :pour))
    (desig-prop ?designator (:container ?object))
    (current-designator ?object ?current-object)
    (-> (desig-prop ?desig (:side ?side)) (true) (true))
    (object-in-hand ?current-object ?side)
    (robot ?robot)
    (end-effector-link ?robot ?side ?link)
    (desig-prop ?designator (:at ?location))
    (lisp-fun current-desig ?location ?current-location)
    (lisp-fun reference ?current-location ?put-down-pose)
    (-> (not (bound ?robot-reference-pose))
        (lisp-fun calculate-pour-hand-pose
                  ?link ?current-object ?put-down-pose
                  ?pose)
        (lisp-fun calculate-pour-hand-pose
                  ?link ?current-object ?put-down-pose
                  :robot-pose ?robot-reference-pose
                  ?pose)))

  (<- (trajectory-point ?designator ?point ?side)
    (trajectory-desig? ?designator)
    (desig-prop ?designator (:to :lift))
    (or (desig-prop ?designator (:obj ?object))
        (desig-prop ?designator (:object ?object)))
    (current-designator ?object ?current-object)
    (-> (desig-prop ?desig (:side ?side)) (true) (true))
    (object-in-hand ?current-object ?side)
    (robot ?robot)
    (end-effector-link ?robot ?side ?link)
    (once (or (desig-prop ?designator (:distance ?lifting-height))
              (== ?lifting-height 0.10)))
    (lisp-fun calculate-object-lift-pose ?link ?current-object ?lifting-height
              ?point))

  (<- (trajectory-point ?designator ?robot-reference-pose ?point ?side)
    (desig-prop ?designator (:to :lift))
    (trajectory-point ?designator ?point ?side)))
