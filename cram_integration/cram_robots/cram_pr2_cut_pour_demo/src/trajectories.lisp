;;;
;;; Copyright (c) 2018, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
;;;                     Christopher Pollok <cpollok@cs.uni-bremen.de>
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

(in-package :demo)

(defun translate-pose-in-base (bTg &key (x-offset 0.0) (y-offset 0.0) (z-offset 0.0))
  (cram-tf:translate-transform-stamped bTg
                                       :x-offset x-offset
                                       :y-offset y-offset
                                       :z-offset z-offset))

(defun calculate-slicing-trajectory (label object arm bTg mTb
                                     &optional (slicing-thickness 0.02))

   (let* ((x-dim-object
           (cl-transforms:x
            (cl-bullet::bounding-box-dimensions
             (btr::aabb object))))
          (n-times-cut-value
            (round 
             (/ (* 2 x-dim-object)
                slicing-thickness)))
          (n-gripper-position-offset
            (/(cl-transforms:y
               (cl-bullet::bounding-box-dimensions
                (btr:aabb object)))
              2))
          ;;- 0.01 since the other gripper is holding the object
          (length-one-cut
            (/ (* 2 x-dim-object) n-times-cut-value))
          (init-slice-pose
            (translate-pose-in-base 
             bTg
             :x-offset (- n-gripper-position-offset)))
          (slice-poses
            `(,init-slice-pose)))

     (dotimes (n (- n-times-cut-value 3))
       (let ((slice-adjustment-pose
               (translate-pose-in-base (car (last slice-poses)) 
                                       :y-offset (if (eq arm :right)
                                                     length-one-cut
                                                     (- length-one-cut)))))
         (print slice-adjustment-pose)
         ;;(break)
         (push slice-adjustment-pose
               (cdr (last slice-poses)))))
     (mapcar (lambda (bTg-pose)
               (cl-tf:ensure-pose-stamped
                (cram-tf:apply-transform 
                 mTb
                 bTg-pose)))
             slice-poses)))

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

;;get sclicing trajectory has the poses:
;;reaching,graspinng,lifting,slice-up,slice down
;;the rest will be calculated in the plan for sclicing
(defmethod man-int:get-action-trajectory :heuristics 20 ((action-type (eql :slicing))
                                                          arm
                                                          grasp
                                                          objects-acted-on
                                                          &key )
  (print (car objects-acted-on))
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
         (oTg-std ;; <- !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! THIS
           ;; IS ONLY FOR THE BREAD, FOR SLICING THIS SHOULD NOT BE
           ;; USED TO CALC BTG. INSTEAD TAKE BTO AND GET OTG-
           (man-int:get-object-type-to-gripper-transform
            object-type object-name arm grasp))
         (fake-oTg
           (cl-tf:make-transform-stamped
            (cl-tf:child-frame-id bTo)
            (if (eql arm :right)
                "r_gripper_tool_frame"
                "l_gripper_tool_frame")
            0.0
            (cl-tf:make-identity-vector)
            (cl-tf:make-identity-rotation)))
         (bTg 
           (cram-tf:apply-transform 
            bTo fake-oTg))
         (mTb
           (cram-tf:pose->transform-stamped
            cram-tf:*fixed-frame*
            cram-tf:*robot-base-frame*
            0.0
            (btr:pose (btr:get-robot-object))))) ;; mTb * bTb * bTo * oTg
    (print oTg-std)
    (print "oTg")
    (print bto)
    (print "bto")
    (print btg)
    (print "btg")
    (break)
    (mapcar (lambda (label transforms)
              (man-int:make-traj-segment
               :label label
               :poses ;; <- this
               ;; should be for slicing in frame object type
               ;; to gripper, but we use in map. calc gripper
               ;; pose like in lift-z-tf pr https://github.com/cram2/cram/pull/155/files, others should be
               ;; calc like this
               (if (or (eq label :slice-up) 
                       (eq label :slice-down))
                   (progn
                     (print (calculate-slicing-trajectory label btr-object arm
                                                          (cram-tf:apply-transform
                                                           (car
                                                            transforms) ;; bTb
                                                           bTg) ;; bTg
                                                          mTb))
                     (calculate-slicing-trajectory label btr-object arm
                                                   (cram-tf:apply-transform
                                                    (car transforms)
                                                    bTg) ;; bTg
                                                   mTb))
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
                 object-type arm grasp))))))

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
         (oTg-std
           (man-int:get-object-type-to-gripper-transform
            object-type object-name arm grasp))
         (approach-pose
           (let* ((tmp-pose-stmp
                    (cl-tf:pose->pose-stamped
                     cram-tf:*fixed-frame*
                     0.0
                     (man-int:calculate-gripper-pose-in-map 
                      bTo arm oTg-std)))
                  (transform (cram-tf:apply-transform
                              (man-int::get-object-type-fixed-frame-tilt-approach-transform
                               object-type arm grasp)
                              (cl-tf:make-transform-stamped
                               cram-tf:*fixed-frame*
                               cram-tf:*fixed-frame*
                               0.0
                               (cl-tf:origin tmp-pose-stmp)
                               (cl-tf:orientation tmp-pose-stmp)))))
             (print arm)
             (print grasp)
             (print oTg-std)
             (print bTo)
             (print tmp-pose-stmp)
             (cl-tf:make-pose-stamped 
               cram-tf:*fixed-frame*
               0.0
               (cl-tf:translation transform)
               (cl-tf:rotation transform))))
         (tilting-poses
           (get-tilting-poses grasp (list approach-pose))))
    (print approach-pose)
    (print (cl-tf:quaternion->euler (cl-tf:orientation (car (get-tilting-poses grasp (list approach-pose))))))
    ;;(break)
    (mapcar (lambda (label poses)
              (man-int:make-traj-segment
               :label label
               :poses poses))
            '(:approach
              :tilting)
            `((,approach-pose)
              ,tilting-poses))))
