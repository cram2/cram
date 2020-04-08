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


(defun translate-pose (pose &key (x-offset 0.0) (y-offset 0.0) (z-offset 0.0))
  (let* ((translate-frame cram-tf:*robot-base-frame*)
         (output-frame cram-tf:*fixed-frame*)
         (pose-in-base
           (cl-tf:transform-pose cram-tf:*transformer*
                                 :pose pose
                                 :target-frame translate-frame))
         (pose-in-base-with-offset
           (cl-transforms-stamped:copy-pose-stamped
            pose-in-base
            :origin (let ((pose-origin (cl-transforms:origin pose-in-base)))
                      (cl-transforms:copy-3d-vector
                       pose-origin
                       :x (let ((x-pose-origin (cl-transforms:x pose-origin)))
                            (+ x-pose-origin x-offset))
                       :y (let ((y-pose-origin (cl-transforms:y pose-origin)))
                            (+ y-pose-origin y-offset))
                       :z (let ((z-pose-origin (cl-transforms:z pose-origin)))
                            (+ z-pose-origin z-offset)))))))
    (cl-tf:transform-pose cram-tf:*transformer*
                          :pose pose-in-base-with-offset
                          :target-frame output-frame)))

(defun calculate-slicing-trajectory (label object arm slice-pose
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
            (translate-pose slice-pose :x-offset (- n-gripper-position-offset)))
          (slice-poses
            `(,init-slice-pose)))

     (dotimes (n (- n-times-cut-value 3))
       (let ((slice-adjustment-pose
               (translate-pose (car (last slice-poses)) 
                               :y-offset (if (eq arm :right)
                                             length-one-cut
                                             (- length-one-cut)))))

         (push slice-adjustment-pose
               (cdr (last slice-poses)))))
     slice-poses))

;;get sclicing trajectory has the poses:
;;reaching,graspinng,lifting,slice-up,slice down
;;the rest will be calculated in the plan for sclicing
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
           
    (mapcar (lambda (label transforms)
              (man-int:make-traj-segment
               :label label
               :poses (let ((poses (mapcar 
                                    (alexandria:curry #'man-int:calculate-gripper-pose-in-map bTo arm)
                                    transforms)))
                        (if (or (eq label :slice-up) 
                                (eq label :slice-down))
                            (calculate-slicing-trajectory label btr-object arm (car poses))
                            poses))))
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
              (,(man-int::get-object-type-to-gripper-slice-up-transform
                 object-type arm grasp))
              (,(man-int::get-object-type-to-gripper-slice-down-transform
                 object-type arm grasp))))))
