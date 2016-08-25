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

;;; THE ASSUMPTION IS that all objects are lying flat on the table
;;; The Z axis can point either upwards or down, that's not fixed.
;;; X is the main axis, the longer eigenvector of the cluster point distribution
;;; Tgrasp is in robot coordinate frame, i.e. transform from gripper to robot frame

(defparameter *kitchen-sink-block-z* 0.85 "in meters")

(defparameter *plate-diameter* 0.26 "in meters")
(defparameter *plate-pregrasp-y-offset* 0.2 "in meters")
(defparameter *plate-grasp-y-offset* (- (/ *plate-diameter* 2) 0.015;; 0.05
                                        ) "in meters")
(defparameter *plate-pregrasp-z-offset* 0.4 "in meters")
(defparameter *plate-2nd-pregrasp-z-offset* 0.035 "in meters") ; grippers can't go into table
(defparameter *plate-grasp-z-offset* 0.05 "in meters")
;; (defparameter *plate-grasp-z-offset* 0.06 "in meters") ; red stacked on blue plate

(defparameter *cutlery-pregrasp-z-offset* 0.4 "in meters")
(defparameter *cutlery-grasp-z-offset* 0.01 "in meters") ; 1 cm because TCP is not at the edge

(defparameter *cup-pregrasp-xy-offset* 0.1 "in meters")
(defparameter *cup-pregrasp-z-offset* 0.4 "in meters")
(defparameter *cup-grasp-z-offset* 0.09 "in meters")

(defparameter *bottle-pregrasp-xy-offset* 0.1 "in meters")
(defparameter *bottle-pregrasp-z-offset* 0.4 "in meters")
(defparameter *bottle-grasp-z-offset* 0.09 "in meters")

(defparameter *lift-z-offset* 0.4 "in meters")

(defparameter *pour-xy-offset* 0.12 "in meters")
(defparameter *pour-z-offset* -0.04 "in meters")

(defparameter *pr2-right-arm-joint-names-list*
  '("r_shoulder_pan_joint" "r_shoulder_lift_joint" "r_upper_arm_roll_joint"
    "r_elbow_flex_joint" "r_forearm_roll_joint" "r_wrist_flex_joint"
    "r_wrist_roll_joint")
  "right arm joint names list")
(defparameter *pr2-right-arm-out-of-sight-joint-positions*
  '(-1.019571035104263d0 -0.35240589506980174d0 -1.2175261637487018d0
    -1.2277736100819567d0 -0.5681516927854576d0 -0.801287536774071d0
    -4.735392876572174d0)
  "joint positions for right arm to move the arm away from sight")
(defparameter *pr2-right-arm-out-of-sight-gripper-pose*
  (cl-transforms-stamped:make-pose-stamped
   "base_footprint"
   0.0
   (cl-transforms:make-3d-vector 0.4 -0.3 1.55)
   (cl-transforms:make-quaternion 0.029319081708036543d0 -0.018714920400581137d0
                                  0.5257710356470319d0 0.8499146788218482d0)))
(defparameter *pr2-left-arm-out-of-sight-gripper-pose*
  (cl-transforms-stamped:make-pose-stamped
   "base_footprint"
   0.0
   (cl-transforms:make-3d-vector 0.4 0.3 1.55)
   (cl-transforms:make-quaternion 0.9215513103717499d0 -0.387996037470125d0
                                  -0.014188589447636247d0 -9.701489976338351d-4)))

(defmacro with-pr2-process-modules (&body body)
  `(cram-process-modules:with-process-modules-running
       (pr2-pms::pr2-perception-pm pr2-pms::pr2-base-pm pr2-pms::pr2-arms-pm
                                   pr2-pms::pr2-grippers-pm pr2-pms::pr2-ptu-pm)
     (cpl:top-level
       ,@body)))

(defun move-pr2-arms-out-of-sight (&key (arm :both))
  (cpl:with-failure-handling
      ((pr2-ll:pr2-low-level-failure (e)
         (declare (ignore e))
         (return)))
    (let (?left-pose-to-go ?right-pose-to-go)
      (case arm
        (:left (setf ?left-pose-to-go *pr2-left-arm-out-of-sight-gripper-pose*))
        (:right (setf ?right-pose-to-go *pr2-right-arm-out-of-sight-gripper-pose*))
        (:both (setf ?left-pose-to-go *pr2-left-arm-out-of-sight-gripper-pose*)
         (setf ?right-pose-to-go *pr2-right-arm-out-of-sight-gripper-pose*)))
      (cram-plan-library:perform
       (desig:an action
                 (to move-arm-motion)
                 (left ?left-pose-to-go)
                 (right ?right-pose-to-go))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; GRASP CONFIGURATIONS ;;;;;;;;;;;;;;;;;;;;;

(defun calculate-cutlery-grasp-pose (yaw x-obj y-obj z-support z-obj-offset)
  "same for both arms"
  (cl-transforms-stamped:pose->pose-stamped
   cram-tf:*robot-base-frame*
   0.0
   (cl-transforms:transform->pose
    (cl-transforms:matrix->transform
     (let ((sin-yaw (sin yaw))
           (cos-yaw (cos yaw)))
       (make-array '(4 4)
                   :initial-contents
                   `((0 ,(- sin-yaw) ,cos-yaw ,x-obj)
                     (0 ,cos-yaw ,sin-yaw ,y-obj)
                     (-1 0 0 ,(+ z-support z-obj-offset))
                     (0 0 0 1))))))))

(defun calculate-plate-grasp-pose (x-obj y-obj y-obj-offset
                                   z-support z-obj-offset &key arm (roll (/ pi 4)))
  (cl-transforms-stamped:pose->pose-stamped
   cram-tf:*robot-base-frame*
   0.0
   (cl-transforms:transform->pose
    (cl-transforms:matrix->transform
     (let ((sin-roll (sin roll))
           (cos-roll (cos roll)))
       (make-array '(4 4)
                   :initial-contents
                   (case arm
                    (:right `((0 0 -1 ,x-obj)
                              (,cos-roll ,(- sin-roll) 0 ,(- y-obj y-obj-offset))
                              (,(- sin-roll) ,(- cos-roll) 0 ,(+ z-support z-obj-offset))
                              (0 0 0 1)))
                    (:left `((0 0 -1 ,x-obj)
                             (,(- cos-roll) ,(- sin-roll) 0 ,(+ y-obj y-obj-offset))
                             (,(- sin-roll) ,cos-roll 0 ,(+ z-support z-obj-offset))
                             (0 0 0 1)))
                    (t (error "get only get grasp poses for :left or :right arms")))))))))

(defun calculate-cup-grasp-pose (x-obj y-obj z-support z-obj-offset &optional arm grasp) 
  "same for both arms"
  (cl-transforms-stamped:pose->pose-stamped
   cram-tf:*robot-base-frame*
   0.0
   (cl-transforms:transform->pose
    (cl-transforms:matrix->transform
     (make-array '(4 4)
                 :initial-contents
                 (case grasp
                   (:front `((1 0 0 ,(+ x-obj 0;; -0.04
                                        )) ;  hack to fix perception problem
                             (0 1 0 ,y-obj)
                             (0 0 1 ,(+ z-support z-obj-offset))
                             (0 0 0 1)))
                   (:side `((0 ,(case arm
                                  (:left 1)
                                  (:right -1)
                                  (t (error "arm can only be :left or :right"))) 0 ,x-obj)
                            (,(case arm
                                (:left -1)
                                (:right 1)
                                (t (error "arm can only be :left or :right"))) 0 0 ,y-obj)
                            (0 0 1 ,(+ z-support z-obj-offset))
                            (0 0 0 1)))
                   (t (error "grasp can only be :side or :front"))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; GENERAL USE FUNCTIONS ;;;;;;;;;;;;;;;;;;;;

(defun get-object-pose (object-designator)
  (cadar (desig:desig-prop-value object-designator :pose)))

(defun translate-pose (pose &key (x-offset 0.0) (y-offset 0.0) (z-offset 0.0))
  (cl-transforms-stamped:copy-pose-stamped
   pose
   :origin (let ((pose-origin (cl-transforms:origin pose)))
             (cl-transforms:copy-3d-vector
              pose-origin
              :x (let ((x-pose-origin (cl-transforms:x pose-origin)))
                   (+ x-pose-origin x-offset))
              :y (let ((y-pose-origin (cl-transforms:y pose-origin)))
                   (+ y-pose-origin y-offset))
              :z (let ((z-pose-origin (cl-transforms:z pose-origin)))
                   (+ z-pose-origin z-offset))))))

(defun rotate-once-pose (pose angle axis)
  (cl-transforms-stamped:copy-pose-stamped
   pose
   :orientation (let ((pose-orientation (cl-transforms:orientation pose)))
                  (cl-transforms:q*
                   (cl-transforms:axis-angle->quaternion
                    (case axis
                      (:x (cl-transforms:make-3d-vector 1 0 0))
                      (:y (cl-transforms:make-3d-vector 0 1 0))
                      (:z (cl-transforms:make-3d-vector 0 0 1))
                      (t (error "in ROTATE-ONCE-POSE forgot to specify axis properly: ~a" axis)))
                    angle)
                   pose-orientation))))

(defun get-object-type-grasp-pose (object-type object-pose arm grasp)
  (case object-type
    ((:fork :knife :cutlery)
     (if (eq grasp :top)
         (let* ((yaw (cl-transforms:get-yaw (cl-transforms:orientation object-pose)))
                (translation (cl-transforms:origin object-pose))
                (x-obj (cl-transforms:x translation))
                (y-obj (cl-transforms:y translation)))
           (calculate-cutlery-grasp-pose yaw x-obj y-obj
                                         *kitchen-sink-block-z*
                                         *cutlery-grasp-z-offset*))
         (error "can only grasp cutlery from top")))
    ((:plate)
     (if (eq grasp :side)
         (let* ((translation (cl-transforms:origin object-pose))
                (x-obj (cl-transforms:x translation))
                (y-obj (cl-transforms:y translation)))
           (calculate-plate-grasp-pose x-obj y-obj *plate-grasp-y-offset*
                                       *kitchen-sink-block-z*
                                       *plate-grasp-z-offset*
                                       :arm arm))
         (error "can only grasp plates from a side")))
    ((:bottle :drink)
     (if (or (eq grasp :side) (eq grasp :front))
         (let* ((translation (cl-transforms:origin object-pose))
                (x-obj (cl-transforms:x translation))
                (y-obj (cl-transforms:y translation)))
           (calculate-cup-grasp-pose x-obj y-obj
                                     *kitchen-sink-block-z*
                                     *bottle-grasp-z-offset*
                                     arm grasp))
         (error "can only grasp bottles from a side or front")))
    ((:cup)
     (if (or (eq grasp :side) (eq grasp :front))
         (let* ((translation (cl-transforms:origin object-pose))
                (x-obj (cl-transforms:x translation))
                (y-obj (cl-transforms:y translation)))
           (calculate-cup-grasp-pose x-obj y-obj
                                     *kitchen-sink-block-z*
                                     *cup-grasp-z-offset*
                                     arm grasp))
         (error "can only grasp cups from a side or front")))))

(defun get-object-grasp-pose (object-designator arm grasp)
  (get-object-type-grasp-pose
   (desig:desig-prop-value object-designator :type)
   (get-object-pose object-designator)
   arm grasp))

(defun get-object-type-lift-pose (object-type object-pose arm grasp)
  (translate-pose (get-object-type-grasp-pose object-type object-pose arm grasp)
                  :z-offset *lift-z-offset*))

(defun get-object-grasp-lift-pose (grasp-pose)
  (translate-pose grasp-pose :z-offset *lift-z-offset*))

(defun get-object-type-pregrasp-pose (object-type grasp-pose arm grasp)
  (case object-type
    ((:fork :knife :cutlery)
     (if (eq grasp :top)
         (translate-pose grasp-pose :z-offset *cutlery-pregrasp-z-offset*)
         (error "can only grasp cutlery from top")))
    ((:plate)
     (if (eq grasp :side)
         (translate-pose grasp-pose
                         :y-offset (case arm
                                     (:right (- *plate-pregrasp-y-offset*))
                                     (:left *plate-pregrasp-y-offset*)
                                     (t (error "arm can be :left or :right")))
                         :z-offset *plate-pregrasp-z-offset*)
         (error "can only grasp plates from a side")))
    ((:bottle :drink)
     (case grasp
       (:front (translate-pose grasp-pose :x-offset (- *bottle-pregrasp-xy-offset*)
                                          :z-offset *bottle-pregrasp-z-offset*))
       (:side (case arm
                (:left (translate-pose grasp-pose :y-offset *bottle-pregrasp-xy-offset*
                                                  :z-offset *bottle-pregrasp-z-offset*))
                (:right (translate-pose grasp-pose :y-offset (- *bottle-pregrasp-xy-offset*)
                                                   :z-offset *bottle-pregrasp-z-offset*))
                (t (error "arm can only be :left or :right"))))
       (t (error "grasp can only be :side or :front"))))
    ((:cup)
     (case grasp
       (:front (translate-pose grasp-pose :x-offset (- *cup-pregrasp-xy-offset*)
                                          :z-offset *cup-pregrasp-z-offset*))
       (:side (case arm
                (:left (translate-pose grasp-pose :y-offset *cup-pregrasp-xy-offset*
                                                  :z-offset *cup-pregrasp-z-offset*))
                (:right (translate-pose grasp-pose :y-offset (- *cup-pregrasp-xy-offset*)
                                                   :z-offset *cup-pregrasp-z-offset*))
                (t (error "arm can only be :left or :right"))))
       (t (error "grasp can only be :side or :front"))))))

(defun get-object-type-2nd-pregrasp-pose (object-type grasp-pose arm grasp)
  (case object-type
    ((:fork :knife :cutlery)
     ;; (if (eq grasp :top)
     ;;     (translate-pose grasp-pose :z-offset *cutlery-pregrasp-z-offset*)
     ;;     (error "can only grasp cutlery from top"))
     nil)
    ((:plate)
     (if (eq grasp :side)
         (translate-pose grasp-pose
                         :y-offset (case arm
                                     (:left *plate-pregrasp-y-offset*)
                                     (:right (- *plate-pregrasp-y-offset*))
                                     (t (error "arm can be :left or :right")))
                         :z-offset *plate-2nd-pregrasp-z-offset*)
         (error "can only grasp plates from a side")))
    ((:bottle :drink)
     (case grasp
       (:front (translate-pose grasp-pose :x-offset (- *bottle-pregrasp-xy-offset*)))
       (:side (case arm
                (:left (translate-pose grasp-pose :y-offset *bottle-pregrasp-xy-offset*))
                (:right (translate-pose grasp-pose :y-offset (- *bottle-pregrasp-xy-offset*)))
                (t (error "arm can only be :left or :right"))))
       (t (error "grasp can only be :side or :front"))))
    ((:cup)
     (case grasp
       (:front (translate-pose grasp-pose :x-offset (- *cup-pregrasp-xy-offset*)))
       (:side (case arm
                (:left (translate-pose grasp-pose :y-offset *cup-pregrasp-xy-offset*))
                (:right (translate-pose grasp-pose :y-offset (- *cup-pregrasp-xy-offset*)))
                (t (error "arm can only be :left or :right"))))
       (t (error "grasp can only be :side or :front"))))))

(defun get-object-type-pour-pose (object-type object-pose arm grasp)
  (let ((grasp-pose (get-object-type-grasp-pose object-type object-pose arm grasp)))
    (translate-pose grasp-pose
                    :x-offset (case grasp
                                (:front (- *pour-xy-offset*))
                                (:side 0.0)
                                (error "can only pour from :side or :front"))
                    :y-offset (case grasp
                                (:front 0.0)
                                (:side (case arm
                                         (:left *pour-xy-offset*)
                                         (:right (- *pour-xy-offset*))
                                         (t (error "arm can only be :left or :right"))))
                                (error "can only pour from :side or :front"))
                               :z-offset (+ *bottle-grasp-z-offset*
                                            *pour-z-offset*))))

(defun get-tilted-pose (initial-poses angle arm grasp)
  (let ((initial-pose (if (listp initial-poses)
                          (car (last initial-poses))
                          initial-poses)))
    (case grasp
      (:front (rotate-once-pose initial-pose angle :y))
      (:side (case arm
               (:left (rotate-once-pose initial-pose angle :x))
               (:right (rotate-once-pose initial-pose (- angle) :x))
               (t (error "arm can only be :left or :right"))))
      (t (error "can only pour from :side or :front")))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; RANDOM GARBAGE ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun get-cup-put-pose (arm grasp)
  (calculate-cup-grasp-pose 0.7 -0.2
                            *kitchen-sink-block-z* *cup-grasp-z-offset*
                            arm grasp))

(defun calculate-cutlery-destination-pose ()
  (cl-transforms:make-pose
   (cl-transforms:make-3d-vector (+ 0.7 (random 0.2))
                                 (- (random 0.4))
                                 *kitchen-sink-block-z*)
   (cl-transforms:make-identity-rotation)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; convergence testing ;;;;;;;;;;;;;;;;;;;;;;

(defparameter *velocities* '())
(defun bla (goal-pose)
  (setf *velocities* nil)
  (let ((old-dist (cl-transforms:v-norm (cl-transforms:origin (cl-transforms-stamped:transform-pose-stamped
                cram-tf:*transformer*
                :pose goal-pose
                :target-frame "r_gripper_tool_frame"
                :timeout cram-tf:*tf-default-timeout*
                :use-current-ros-time t)))))
    (roslisp:loop-at-most-every 0.5
      (let* ((goal-in-tcp-frame
               (cl-transforms-stamped:transform-pose-stamped
                cram-tf:*transformer*
                :pose goal-pose
                :target-frame "r_gripper_tool_frame"
                :timeout cram-tf:*tf-default-timeout*
                :use-current-ros-time t))
             (distance-xy (cl-transforms:v-norm (cl-transforms:origin goal-in-tcp-frame)))
             (distance-angle (cl-transforms:q-norm (cl-transforms:orientation goal-in-tcp-frame))))
        ;; (format t "xy: ~a~%angle: ~a~%~%" distance-xy distance-angle)
        ;; (roslisp:ros-info (bla bla) "~a" goal-in-tcp-frame)
        (roslisp:ros-info (bla velocity) "~a" (- old-dist distance-xy))
        (push (- old-dist distance-xy) *velocities* )
        (setf old-dist distance-xy)))))
