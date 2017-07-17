;;;
;;; Copyright (c) 2017, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :kr-belief)

(defparameter *default-z-offset* 0.2 "in meters")

(defgeneric get-object-type-grasp (object-type)
  (:documentation "Returns either of :top, :side, :front. Default is :top.")
  (:method (object-type) :top)
  (:method ((object-type (eql :porsche-body))) :top)
  (:method ((object-type (eql :camaro-body))) :top)
  (:method ((object-type (eql :chassis))) :side)
  (:method ((object-type (eql :axle))) :top)
  (:method ((object-type (eql :wheel))) :top))

(defgeneric get-object-type-gripping-effort (object-type)
  (:documentation "Returns effort in Nm, e.g. 50. Default is 35.")
  (:method (object-type) 35)
  (:method ((object-type (eql :porsche-body))) 35)
  (:method ((object-type (eql :camaro-body))) 35)
  (:method ((object-type (eql :chassis))) 50)
  (:method ((object-type (eql :axle))) 50)
  (:method ((object-type (eql :wheel))) 40))

(defgeneric get-object-type-gripper-opening (object-type)
  (:documentation "How wide to open the gripper before grasping, in m. Default is 0.10.")
  (:method (object-type) 0.10)
  (:method ((object-type (eql :axle))) 0.02))

(defgeneric get-object-to-gripper-transform (object-type arm grasp)
  (:documentation "Returns a pose stamped representing oTg -- transfrom from object to gripper.

oTg == oPg, i.e. the pose of gripper in the object coordinate frame."))

(defun get-object-type-grasp-pose (object-type arm grasp object-transform)
  (declare (type cl-transforms-stamped:transform-stamped object-transform))
  "Returns a pose stamped representing bTg -- transfrom from base to gripper.

Take object-transform, ensure it's from base frame  -- bTo.
Multiply from the right with the transform from object to gripper -- bTo * oTg == bTg."

  (unless (equal (cl-transforms-stamped:frame-id object-transform)
                   cram-tf:*robot-base-frame*)
      (error "In grasp calculations the OBJECT-TRANSFORM did not have ~
correct parent frame: ~a and ~a"
             (cl-transforms-stamped:frame-id object-transform)
             cram-tf:*robot-base-frame*))

  (let ((object-gripper-transform
          (get-object-to-gripper-transform object-type arm grasp)))
    (cl-transforms-stamped:pose->pose-stamped
     cram-tf:*robot-base-frame*
     0.0
     (cl-transforms:transform->pose
      (cl-transforms:transform* object-transform object-gripper-transform)))))

(defgeneric get-object-type-pregrasp-pose (object-type arm grasp grasp-pose)
  (:documentation "Returns a pose stamped"))

(defgeneric get-object-type-2nd-pregrasp-pose (object-type arm grasp grasp-pose)
  (:documentation "Returns a pose stamped. Default value is NIL.")
  (:method (object-type grasp-pose arm grasp) nil))

(defgeneric get-object-type-lift-pose (object-type arm grasp grasp-pose)
  (:documentation "Returns a pose stamped"))

(defun get-object-grasping-poses (object-type arm grasp object-transform)
  "Returns a list of (pregrasp-pose 2nd-pregrasp-pose grasp-pose lift-pose)"
  (let ((grasp-pose (get-object-type-grasp-pose object-type arm grasp object-transform)))
    (list (get-object-type-pregrasp-pose object-type arm grasp grasp-pose)
          (get-object-type-2nd-pregrasp-pose object-type arm grasp grasp-pose)
          grasp-pose
          (get-object-type-lift-pose object-type arm grasp grasp-pose))))

;;;;;;;;;;;;;;;;;;;;;;;;;; axle ;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod get-object-to-gripper-transform ((object-type (eql :axle))
                                            (arm (eql :left))
                                            (grasp (eql :top)))
  (cl-transforms-stamped:make-transform-stamped
   "object_of_type_Axle"
   "left_gripper_tool_frame"
   0.0
   (cl-transforms:make-3d-vector 0.0d0 0.0d0 0.0d0)
   (cl-transforms:matrix->quaternion
    #2A((-1 0 0)
        (0 1 0)
        (0 0 -1)))))
(defmethod get-object-type-pregrasp-pose ((object-type (eql :axle))
                                          (arm (eql :left))
                                          (grasp (eql :top))
                                          grasp-pose)
  (cram-tf:translate-pose grasp-pose :z-offset *default-z-offset*))
(defmethod get-object-type-lift-pose ((object-type (eql :axle))
                                      (arm (eql :left))
                                      (grasp (eql :top))
                                      grasp-pose)
  (cram-tf:translate-pose grasp-pose :z-offset *default-z-offset*))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; chassis ;;;;;;;;;;;;;;;;;;

(defmethod get-object-to-gripper-transform ((object-type (eql :chassis))
                                            (arm (eql :left))
                                            (grasp (eql :side)))
  (cl-transforms-stamped:make-transform-stamped
   "object_of_type_Chassis"
   "left_gripper_tool_frame"
   0.0
   (cl-transforms:make-3d-vector 0.0d0 0.0d0 0.0d0)
   (cl-transforms:matrix->quaternion
    #2A((0 -1 0)
        (0 0 -1)
        (1 0 0)))))
(defmethod get-object-type-pregrasp-pose ((object-type (eql :chassis))
                                          (arm (eql :left))
                                          (grasp (eql :side))
                                          grasp-pose)
  (cram-tf:translate-pose grasp-pose :y-offset *default-z-offset*))
(defmethod get-object-type-lift-pose ((object-type (eql :chassis))
                                      (arm (eql :left))
                                      (grasp (eql :side))
                                      grasp-pose)
  (cram-tf:translate-pose grasp-pose :z-offset *default-z-offset*))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; camaro-body ;;;;;;;;;;;;;;;;

(defmethod get-object-to-gripper-transform ((object-type (eql :camaro-body))
                                            (arm (eql :left))
                                            (grasp (eql :top)))
  (cl-transforms-stamped:make-transform-stamped
   "object_of_type_CamaroBody"
   "left_gripper_tool_frame"
   0.0
   (cl-transforms:make-3d-vector 0.0d0 0.0d0 0.0d0)
   (cl-transforms:matrix->quaternion
    #2A((0 1 0)
        (1 0 0)
        (0 0 -1)))))
(defmethod get-object-type-pregrasp-pose ((object-type (eql :camaro-body))
                                          (arm (eql :left))
                                          (grasp (eql :top))
                                          grasp-pose)
  (cram-tf:translate-pose grasp-pose :z-offset *default-z-offset*))
(defmethod get-object-type-lift-pose ((object-type (eql :camaro-body))
                                      (arm (eql :left))
                                      (grasp (eql :top))
                                      grasp-pose)
  (cram-tf:translate-pose grasp-pose :z-offset *default-z-offset*))
























#+asdfasdfadf
(
;;; THE ASSUMPTION IS that all objects are lying flat on the table
;;; The Z axis can point either upwards or down, that's not fixed.
;;; X is the main axis, the longer eigenvector of the cluster point distribution
;;; Tgrasp is in robot coordinate frame, i.e. transform from gripper to robot frame

(defparameter *kitchen-sink-block-z* 0.85 "in meters")
(defparameter *kitchen-meal-table-z* 0.76 "in meters")

(defparameter *plate-diameter* 0.26 "in meters")
(defparameter *plate-pregrasp-y-offset* 0.2 "in meters")
(defparameter *plate-grasp-y-offset* (- (/ *plate-diameter* 2) 0.015;; 0.05
                                        ) "in meters")
(defparameter *plate-pregrasp-z-offset* 0.4 "in meters")
(defparameter *plate-2nd-pregrasp-z-offset* 0.035 "in meters") ; grippers can't go into table
(defparameter *plate-grasp-z-offset* 0.05 "in meters")
;; (defparameter *plate-grasp-z-offset* 0.06 "in meters") ; red stacked on blue plate

(defparameter *cutlery-pregrasp-z-offset* 0.4 "in meters")
(defparameter *cutlery-grasp-z-offset* 0.02 "in meters") ; 1 cm because TCP is not at the edge

(defparameter *cup-pregrasp-xy-offset* 0.05 "in meters")
(defparameter *cup-pregrasp-z-offset* 0.4 "in meters")
(defparameter *cup-grasp-xy-offset* 0.01 "in meters")
(defparameter *cup-grasp-z-offset* 0.08 "in meters") ; 0.07?
(defparameter *cup-center-z* 0.044)

(defparameter *bottle-pregrasp-xy-offset* 0.05 "in meters")
(defparameter *bottle-pregrasp-z-offset* 0.4 "in meters")
(defparameter *bottle-grasp-xy-offset* 0.01 "in meters")
(defparameter *bottle-grasp-z-offset* 0.095 "in meters") ; 0.105?

(defparameter *lift-z-offset* 0.4 "in meters")

(defparameter *pour-xy-offset* 0.12 "in meters")
(defparameter *pour-z-offset* -0.04 "in meters")

(defparameter *pr2-right-arm-out-of-sight-joint-positions*
  '(-1.712587449591307d0 -0.2567290370386635d0 -1.4633501125737374d0
    -2.1221670650093913d0 1.7663253481913623d0 -0.07942669250968948d0
    0.05106258161229582d0)
  "joint positions for right arm to move the arm away from sight")
(defparameter *pr2-left-arm-out-of-sight-joint-positions*
  '(1.9652919379395388d0 -0.26499816732737785d0 1.3837617139225473d0
    -2.1224566064321584d0 16.99646118944817d0 -0.07350789589924167d0
    -50.282675816750015d0)
  "joint positions for right arm to move the arm away from sight")
(defparameter *pr2-right-arm-out-of-sight-flipped-joint-positions*
  '(-1.6863889585064997d0 -0.1975125908655453d0 -1.045465435053814d0
    -1.6784448346188787d0 -76.25678428315953d0 -0.4330236861590935d0
    -50.649577448004365d0)
  "joint positions for right arm to move the arm away from sight")
(defparameter *pr2-left-arm-out-of-sight-flipped-joint-positions*
  '(2.0658576647935627d0 -0.0465740758716759d0 0.64709164157929d0
    -1.8113443476689572d0 -11.712932369941111d0 -0.9136651430224094d0
    15.839744451087977d0)
  "joint positions for right arm to move the arm away from sight")

(defparameter *pr2-right-arm-pouring-joint-positions*
  '(-0.556869 -0.2 -1.57977 -1.02886 -94.3445 -1.18356 1.377))
(defparameter *pr2-left-arm-pouring-joint-positions*
  '(0.543886 0.156374 1.54255 -1.51628 0.237611 -1.32027 4.69513))

;; (defparameter *pr2-right-arm-pouring-joint-positions*
;;   '(-0.556869 -0.2 -1.57977 -1.02886 -94.3445 -1.18356 1.377))
;; (defparameter *pr2-left-arm-pouring-joint-positions*
;;   '(0.962039 0.150617 1.56769 -1.41351 -6.01118 -1.41351 4.70187))

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

(defparameter *meal-table-left-base-pose*
  (cl-transforms-stamped:make-pose-stamped
   "map"
   0.0
   (cl-transforms:make-3d-vector -1.12d0 -0.42d0 0.0)
   (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi -2))))
(defparameter *meal-table-right-base-pose*
  (cl-transforms-stamped:make-pose-stamped
   "map"
   0.0
   (cl-transforms:make-3d-vector -1.8547d0 -0.381d0 0.0d0)
   (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi -2))))
(defparameter *meal-table-left-base-look-down-pose*
  (cl-transforms-stamped:make-pose-stamped
   "base_footprint"
   0.0
   (cl-transforms:make-3d-vector 0.7d0 -0.12d0 0.7578d0)
   (cl-transforms:make-identity-rotation)))
(defparameter *meal-table-left-base-look-pose*
  (cl-transforms-stamped:make-pose-stamped
   "base_footprint"
   0.0
   (cl-transforms:make-3d-vector 0.75d0 -0.12d0 1.11d0)
   (cl-transforms:make-identity-rotation)))
(defparameter *meal-table-right-base-look-pose*
  (cl-transforms-stamped:make-pose-stamped
   "base_footprint"
   0.0
   (cl-transforms:make-3d-vector 0.65335d0 0.076d0 0.758d0)
   (cl-transforms:make-identity-rotation)))


(defun move-pr2-arms-out-of-sight (&key (arm '(:left :right)) flipped)
  (cpl:with-failure-handling
      ((common-fail:low-level-failure (e)
         (declare (ignore e))
         (return)))
    (unless (listp arm)
      (setf arm (list arm)))
    (let (?left-configuration-to-go ?right-configuration-to-go)
      (when (member :left arm)
        (setf ?left-configuration-to-go (if flipped
                                            *pr2-left-arm-out-of-sight-flipped-joint-positions*
                                            *pr2-left-arm-out-of-sight-joint-positions*)))
      (when (member :right arm)
        (setf ?right-configuration-to-go (if flipped
                                             *pr2-right-arm-out-of-sight-flipped-joint-positions*
                                             *pr2-right-arm-out-of-sight-joint-positions*)))
      (exe:perform
       (desig:a motion
                (type moving-joints)
                (left-configuration ?left-configuration-to-go)
                (right-configuration ?right-configuration-to-go))))))

;; For real robot experiments if CL-TF bugs out
;; (defun overwrite-transformer ()
;;   (setf cram-tf:*transformer* (make-instance 'cl-tf2:buffer-client)))
;; (roslisp-utilities:register-ros-init-function overwrite-transformer)

;;;;;;;;;;;;;;;;;;;;;;;;; GRASP CONFIGURATION CALCULATIONS ;;;;;;;;;;;;;;;;;;;;;

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

(defun calculate-cup-grasp-pose (x-obj y-obj z-support z-obj-offset
                                 &optional arm grasp)
  "same for both arms"
  (cl-transforms-stamped:pose->pose-stamped
   cram-tf:*robot-base-frame*
   0.0
   (cl-transforms:transform->pose
    (cl-transforms:matrix->transform
     (make-array '(4 4)
                 :initial-contents
                 (case grasp
                   (:front `((1 0 0 ,x-obj)
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

;;;;;;;;;;;;;;;;;;; MANIPULATION POSE CALCULATIONS: LIFT, PREGRASP ETC. ;;;;;;;

(defun get-object-type-grasp-pose (object-type object-pose arm grasp)
  (let ((object-pose (cram-tf:ensure-pose-in-frame
                      object-pose
                      cram-tf:*robot-base-frame*
                      :use-zero-time t)))
   (case object-type
     ((:fork :knife :cutlery)
      (if (eq grasp :top)
          (let* ((yaw (cl-transforms:get-yaw (cl-transforms:orientation object-pose)))
                 (translation (cl-transforms:origin object-pose))
                 (x-obj (cl-transforms:x translation))
                 (y-obj (cl-transforms:y translation)))
            (calculate-cutlery-grasp-pose yaw x-obj y-obj
                                          *kitchen-meal-table-z*
                                          *cutlery-grasp-z-offset*))
          (error "can only grasp cutlery from top")))
     ((:plate)
      (if (eq grasp :side)
          (let* ((translation (cl-transforms:origin object-pose))
                 (x-obj (cl-transforms:x translation))
                 (y-obj (cl-transforms:y translation)))
            (calculate-plate-grasp-pose x-obj y-obj *plate-grasp-y-offset*
                                        *kitchen-meal-table-z*
                                        *plate-grasp-z-offset*
                                        :arm arm))
          (error "can only grasp plates from a side")))
     ((:bottle :drink)
      (if (or (eq grasp :side) (eq grasp :front))
          (let* ((translation (cl-transforms:origin object-pose))
                 (x-obj (cl-transforms:x translation))
                 (y-obj (cl-transforms:y translation)))
            (calculate-cup-grasp-pose (case grasp
                                        (:side x-obj)
                                        (:front (+ x-obj *bottle-grasp-xy-offset*)))
                                      (case grasp
                                        (:side (case arm
                                                 (:left (- y-obj *bottle-grasp-xy-offset*))
                                                 (:right (+ y-obj *bottle-grasp-xy-offset*))
                                                 (error "arm can only be left or right")))
                                        (:front y-obj))
                                      *kitchen-meal-table-z*
                                      *bottle-grasp-z-offset*
                                      arm grasp))
          (error "can only grasp bottles from a side or front")))
     ((:cup)
      (if (or (eq grasp :side) (eq grasp :front))
          (let* ((translation (cl-transforms:origin object-pose))
                 (x-obj (cl-transforms:x translation))
                 (y-obj (cl-transforms:y translation)))
            (calculate-cup-grasp-pose (case grasp
                                        (:side x-obj)
                                        (:front (+ x-obj *cup-grasp-xy-offset*)))
                                      (case grasp
                                        (:side (case arm
                                                 (:left (- y-obj *cup-grasp-xy-offset*))
                                                 (:right (+ y-obj *cup-grasp-xy-offset*))
                                                 (error "arm can only be left or right")))
                                        (:front y-obj))
                                      *kitchen-meal-table-z*
                                      *cup-grasp-z-offset*
                                      arm grasp))
          (error "can only grasp cups from a side or front"))))))

(defun get-object-grasp-pose (object-designator arm grasp)
  (get-object-type-grasp-pose
   (desig:desig-prop-value object-designator :type)
   (get-object-pose object-designator)
   arm grasp))

(defun get-object-type-lift-pose (object-type object-pose arm grasp)
  (let ((object-pose (cram-tf:ensure-pose-in-frame
                      object-pose
                      cram-tf:*robot-base-frame*
                      :use-zero-time t)))
    (cram-tf:translate-pose (get-object-type-grasp-pose object-type object-pose arm grasp)
                            :z-offset *lift-z-offset*)))

(defun get-object-grasp-lift-pose (grasp-pose)
  (let ((grasp-pose (cram-tf:ensure-pose-in-frame
                     grasp-pose
                     cram-tf:*robot-base-frame*
                     :use-zero-time t)))
    (cram-tf:translate-pose grasp-pose :z-offset *lift-z-offset*)))

(defun get-object-type-pregrasp-pose (object-type grasp-pose arm grasp)
  (let ((grasp-pose (cram-tf:ensure-pose-in-frame
                     grasp-pose
                     cram-tf:*robot-base-frame*
                     :use-zero-time t)))
   (case object-type
     ((:fork :knife :cutlery)
      (if (eq grasp :top)
          (cram-tf:translate-pose grasp-pose :z-offset *cutlery-pregrasp-z-offset*)
          (error "can only grasp cutlery from top")))
     ((:plate)
      (if (eq grasp :side)
          (cram-tf:translate-pose grasp-pose
                                  :y-offset (case arm
                                              (:right (- *plate-pregrasp-y-offset*))
                                              (:left *plate-pregrasp-y-offset*)
                                              (t (error "arm can be :left or :right")))
                                  :z-offset *plate-pregrasp-z-offset*)
          (error "can only grasp plates from a side")))
     ((:bottle :drink)
      (case grasp
        (:front (cram-tf:translate-pose grasp-pose :x-offset (- *bottle-pregrasp-xy-offset*)
                                                   :z-offset *bottle-pregrasp-z-offset*))
        (:side (case arm
                 (:left (cram-tf:translate-pose grasp-pose :y-offset *bottle-pregrasp-xy-offset*
                                                           :z-offset *bottle-pregrasp-z-offset*))
                 (:right (cram-tf:translate-pose grasp-pose
                                                 :y-offset (- *bottle-pregrasp-xy-offset*)
                                                 :z-offset *bottle-pregrasp-z-offset*))
                 (t (error "arm can only be :left or :right"))))
        (t (error "grasp can only be :side or :front"))))
     ((:cup)
      (case grasp
        (:front (cram-tf:translate-pose grasp-pose :x-offset (- *cup-pregrasp-xy-offset*)
                                                   :z-offset *cup-pregrasp-z-offset*))
        (:side (case arm
                 (:left (cram-tf:translate-pose grasp-pose :y-offset *cup-pregrasp-xy-offset*
                                                           :z-offset *cup-pregrasp-z-offset*))
                 (:right (cram-tf:translate-pose grasp-pose :y-offset (- *cup-pregrasp-xy-offset*)
                                                            :z-offset *cup-pregrasp-z-offset*))
                 (t (error "arm can only be :left or :right"))))
        (t (error "grasp can only be :side or :front")))))))

(defun get-object-type-2nd-pregrasp-pose (object-type grasp-pose arm grasp)
  (let ((grasp-pose (cram-tf:ensure-pose-in-frame
                     grasp-pose
                     cram-tf:*robot-base-frame*
                     :use-zero-time t)))
   (case object-type
     ((:fork :knife :cutlery)
      ;; (if (eq grasp :top)
      ;;     (translate-pose grasp-pose :z-offset *cutlery-pregrasp-z-offset*)
      ;;     (error "can only grasp cutlery from top"))
      nil)
     ((:plate)
      (if (eq grasp :side)
          (cram-tf:translate-pose grasp-pose
                                  :y-offset (case arm
                                              (:left *plate-pregrasp-y-offset*)
                                              (:right (- *plate-pregrasp-y-offset*))
                                              (t (error "arm can be :left or :right")))
                                  :z-offset *plate-2nd-pregrasp-z-offset*)
          (error "can only grasp plates from a side")))
     ((:bottle :drink)
      (case grasp
        (:front (cram-tf:translate-pose grasp-pose :x-offset (- *bottle-pregrasp-xy-offset*)))
        (:side (case arm
                 (:left (cram-tf:translate-pose grasp-pose
                                                :y-offset *bottle-pregrasp-xy-offset*))
                 (:right (cram-tf:translate-pose grasp-pose
                                                 :y-offset (- *bottle-pregrasp-xy-offset*)))
                 (t (error "arm can only be :left or :right"))))
        (t (error "grasp can only be :side or :front"))))
     ((:cup)
      (case grasp
        (:front (cram-tf:translate-pose grasp-pose
                                        :x-offset (- *cup-pregrasp-xy-offset*)))
        (:side (case arm
                 (:left (cram-tf:translate-pose grasp-pose
                                                :y-offset *cup-pregrasp-xy-offset*))
                 (:right (cram-tf:translate-pose grasp-pose
                                                 :y-offset (- *cup-pregrasp-xy-offset*)))
                 (t (error "arm can only be :left or :right"))))
        (t (error "grasp can only be :side or :front")))))))
)
