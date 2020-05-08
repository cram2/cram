;;;
;;; Copyright (c) 2020, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;                     Vanessa Hassouna <hassouna@uni-bremen.de>
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

(in-package :cp-plans)

(defun translate-pose-in-base (bTg &key (x-offset 0.0) (y-offset 0.0) (z-offset 0.0))
  (cram-tf:translate-transform-stamped bTg
                                       :x-offset x-offset
                                       :y-offset y-offset
                                       :z-offset z-offset))

(defun calculate-init-slicing-pose (object arm bTg)
  (let* ((x-gripper-position-offset
           (/(cl-transforms:y
              (cl-bullet::bounding-box-dimensions
               (btr:aabb object)))
             2))
         (y-gripper-position-offset
           (/(cl-transforms:x
              (cl-bullet::bounding-box-dimensions
               (btr:aabb object)))
             2)))
    (translate-pose-in-base 
     bTg
     :x-offset (- x-gripper-position-offset)
     :y-offset (if (eq arm :right)
                   (- y-gripper-position-offset)
                   y-gripper-position-offset))))

(defun calculate-slicing-trajectory-in-map (object arm bTg)
  (let* ((mTb
           (cram-tf:pose->transform-stamped
            cram-tf:*fixed-frame*
            cram-tf:*robot-base-frame*
            0.0
            (btr:pose (btr:get-robot-object)))))
    (mapcar (lambda (bTg-pose)
               (cl-tf:ensure-pose-stamped
                (cram-tf:apply-transform 
                 mTb
                 bTg-pose)))
             (calculate-slicing-trajectory object arm bTg))))

(defun calculate-slicing-trajectory (object arm bTg
                                     &optional (slicing-thickness 0.02))

   (let* ((x-dim-object
            (cl-transforms:x
             (cl-bullet::bounding-box-dimensions
            (btr::aabb object))))
          (n-times-cut-value
            (round 
             (/ (* 2 x-dim-object)
                slicing-thickness)))
          (slice-poses
            `(,(calculate-init-slicing-pose object arm bTg))))

     (dotimes (n (- n-times-cut-value 3))
       (let ((slice-adjustment-pose
               (translate-pose-in-base (car (last slice-poses)) 
                                       :y-offset (if (eq arm :right)
                                                     slicing-thickness
                                                     (- slicing-thickness)))))
         (push slice-adjustment-pose
               (cdr (last slice-poses)))))
     slice-poses))



;;get sclicing trajectory has the poses:
;;reaching,graspinng,lifting,slice-up,slice down
(defmethod man-int:get-action-trajectory :heuristics 20 ((action-type (eql :slicing))
                                                          arm
                                                          grasp
                                                          objects-acted-on
                                                          &key )


  (let* ((object
           (car objects-acted-on))
         (object-name
           (desig:desig-prop-value object :name))
         (btr-object
           (btr:object btr:*current-bullet-world* object-name))
         (object-type
           (desig:desig-prop-value object :type))
         (bTo
           (man-int:get-object-old-transform object))
         (oTg-std 
           (man-int:get-object-type-to-gripper-transform
            object-type object-name arm grasp)))

    (flet ((get-base-to-gripper-transform-for-slicing (bTb-offset)
             (cl-tf:make-transform-stamped
              (cl-tf:frame-id bTo)
              (if (eql arm :right)
                  "r_gripper_tool_frame"
                  "l_gripper_tool_frame")
              0.0
              (cl-tf:v+
               (cl-tf:translation bTo)
               (cl-tf:translation bTb-offset))
              (cl-tf:rotation bTb-offset))))

      (mapcar (lambda (label transforms)
                (man-int:make-traj-segment
                 :label label
                 :poses 
                 (if (or (eq label :slice-up) 
                         (eq label :slice-down))
                     (calculate-slicing-trajectory-in-map btr-object arm
                                                          (get-base-to-gripper-transform-for-slicing 
                                                           (car transforms)))
                     (mapcar 
                      (alexandria:curry #'man-int:calculate-gripper-pose-in-map bTo arm)
                      transforms))))
              '(:reaching
                :grasping
                :lifting
                :slice-up
                :slice-down)
              `((,(man-int:get-object-type-to-gripper-pregrasp-transform
                   object-type object-name arm grasp oTg-std)
                 ,(man-int:get-object-type-to-gripper-2nd-pregrasp-transform
                   object-type object-name arm grasp oTg-std))
                (,oTg-std)
                (,(man-int:get-object-type-to-gripper-lift-transform
                   object-type object-name arm grasp oTg-std)
                 ,(man-int:get-object-type-to-gripper-2nd-lift-transform
                   object-type object-name arm grasp oTg-std))
                (,(man-int:get-object-type-robot-frame-slice-up-transform
                   object-type arm grasp))
                (,(man-int:get-object-type-robot-frame-slice-down-transform
                   object-type arm grasp)))))))

(defun get-tilting-poses (grasp approach-poses &optional (angle (cram-math:degrees->radians 100)))
  (mapcar (lambda (?approach-pose)
            ;;depending on the grasp the angle to tilt is different
            (case grasp
              (:front (rotate-once-pose ?approach-pose (- angle) :y))
              (:left-side (rotate-once-pose ?approach-pose (+ angle) :x))
              (:right-side (rotate-once-pose ?approach-pose (- angle) :x))
              (:back (rotate-once-pose ?approach-pose (+ angle) :y))
              (t (error "can only pour from :side, back or :front"))))
          approach-poses))

;;helper function for tilting
;;rotate the pose around the axis in an angle
(defun rotate-once-pose (pose angle axis)
  (cl-transforms-stamped:copy-pose-stamped
   pose
   :orientation (let ((pose-orientation (cl-transforms:orientation pose)))
                  (cl-tf:normalize
                   (cl-transforms:q*
                    (cl-transforms:axis-angle->quaternion
                     (case axis
                       (:x (cl-transforms:make-3d-vector 1 0 0))
                       (:y (cl-transforms:make-3d-vector 0 1 0))
                       (:z (cl-transforms:make-3d-vector 0 0 1))
                       (t (error "in ROTATE-ONCE-POSE forgot to specify axis properly: ~a" axis)))
                     angle)
                    pose-orientation)))))

;;get pouring trajectory workes like picking-up it will get the 
;;object-type-to-gripper-tilt-approch-transform und makes a traj-segment out of it
;;here we have only the approach pose, followed by that is the titing pose (above)
(defmethod man-int:get-action-trajectory :heuristics 20 ((action-type (eql :pouring))
                                                         arm
                                                         grasp
                                                         objects-acted-on
                                                         &key )
  (let* ((object
           (car objects-acted-on))
         (object-name
           (desig:desig-prop-value object :name))
         (object-type
           (desig:desig-prop-value object :type))
         (bTo
           (man-int:get-object-transform object))
         (bTb-offset
           (man-int::get-object-type-robot-frame-tilt-approach-transform
            object-type arm grasp))
         (oTg-std
           (man-int:get-object-type-to-gripper-transform
            object-type object-name arm grasp))
         (approach-pose
           (man-int:calculate-gripper-pose-in-base
            (cram-tf:apply-transform
             bTb-offset
             (cram-tf:copy-transform-stamped
              bTo
              :rotation (cl-tf:make-identity-rotation)))
            arm oTg-std))
         (tilting-poses
           (get-tilting-poses grasp (list approach-pose))))
    (mapcar (lambda (label poses-in-base)
              (man-int:make-traj-segment
               :label label
               :poses (mapcar 
                       (lambda (pose-in-base)
                         (let ((mTb (cram-tf:pose->transform-stamped
                                     cram-tf:*fixed-frame*
                                     cram-tf:*robot-base-frame*
                                     0.0
                                     (btr:pose (btr:get-robot-object))))
                               (bTg-std
                                 (cram-tf:pose-stamped->transform-stamped
                                  pose-in-base
                                  (cl-tf:child-frame-id bTo))))
                           (cl-tf:ensure-pose-stamped
                            (cram-tf:apply-transform mTb bTg-std))))
                       poses-in-base)))
            '(:approach
              :tilting)
            `((,approach-pose)
              ,tilting-poses))))
