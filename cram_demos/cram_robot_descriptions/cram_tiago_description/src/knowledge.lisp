;;;
;;; Copyright (c) 2022, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :tiago-descr)

(defparameter *forward-looking-position-in-base-frame*
  (cl-transforms:make-3d-vector 10.0 0.0 0.9))

;; TODO: this can be different for different hands
#+use-this-function-to-calculate-tcp-in-ee-pose
(defun calculate-tcp-in-ee-pose-from-bullet (robot-name arm-alias)
  (let* ((map-P-tcp
           (btr:link-pose
            (btr:get-robot-object)
            (cut:var-value
             '?link
             (cut:lazy-car
              (prolog:prolog
               `(rob-int:robot-tool-frame ,robot-name ,arm-alias ?link))))))
         (map-P-ee
           (btr:link-pose
            (btr:get-robot-object)
            (cut:var-value
             '?link
             (cut:lazy-car
              (prolog:prolog
               `(rob-int:end-effector-link ,robot-name ,arm-alias ?link))))))
         (ee-T-map
           (cl-transforms:transform-inv
            (cl-transforms:pose->transform map-P-ee))))
    (cl-transforms:transform ee-T-map map-P-tcp)))
(defparameter *tcp-in-ee-pose*
  (cl-transforms:make-pose
   ;; The X should've been 0.150575d0, but the grasping frame is defined
   ;; on Tiago in such a shitty way that have to use a custom offset :/
   (cl-transforms:make-3d-vector 0.24d0 0 0)
   (cl-transforms-stamped:make-quaternion 0.7071054825100438d0
                                          -1.2986782349788673d-6
                                          1.298673464711353d-6
                                          0.707108079858281d0)))

;; This is std-gripper-T-gripper
;; standard Z - towards object
;; standard X - from one finger to another
;; standard Y - right hand rule
#+visualize-the-gripper-frame-with-the-following-function
(defun visualize-gripper-frame-in-bullet (robot-name arm-alias)
  (btr:add-vis-axis-object
   (btr:link-pose
    (btr:get-robot-object)
    (cut:var-value
     '?link
     (cut:lazy-car
      (prolog:prolog
       `(rob-int:robot-tool-frame ,robot-name ,arm-alias ?link)))))))
(defparameter *standard-to-tiago-gripper-transform*
  (cl-transforms:make-transform
   (cl-transforms:make-identity-vector)
   (cl-transforms:matrix->quaternion
    #2A((0 1 0)
        (0 0 1)
        (1 0 0)))))

(def-fact-group tiago-metadata (robot-odom-frame
                                robot-base-frame robot-base-link
                                robot-torso-link-joint)
  (<- (robot-odom-frame :tiago-dual "odom"))
  (<- (robot-base-frame :tiago-dual "base_footprint"))
  (<- (robot-base-link :tiago-dual "base_link"))
  (<- (robot-torso-link-joint :tiago-dual "torso_lift_link" "torso_lift_joint")))

(def-fact-group tiago-camera-data (camera-frame
                                   camera-minimal-height
                                   camera-maximal-height
                                   camera-horizontal-angle
                                   camera-vertical-angle)

  (<- (camera-frame :tiago-dual "xtion_optical_frame"))
  (<- (camera-frame :tiago-dual "xtion_rgb_optical_frame"))
  (<- (camera-frame :tiago-dual "xtion_depth_frame"))

  #+calculate-with-the-following-function-in-bullet
  (defun calculate-cam-min-and-max-height-from-bullet (robot-name)
    (let* ((torso-joint-name
             (cut:var-value
              '?joint
              (cut:lazy-car
               (prolog:prolog
                `(rob-int:robot-torso-link-joint ,robot-name ?link ?joint)))))
           (torso-low-limit
             (rob-int:get-joint-lower-limit torso-joint-name))
           (torso-high-limit
             (rob-int:get-joint-upper-limit torso-joint-name))
           (camera-link-name
             (cut:var-value
              '?link
              (cut:lazy-car
               (prolog:prolog
                `(rob-int:camera-frame ,robot-name ?link)))))
           camera-low-z camera-high-z)
      (setf (btr:joint-state (btr:get-robot-object) torso-joint-name)
            torso-low-limit)
      (setf camera-low-z
            (cl-transforms:z
             (cl-transforms:origin
              (btr:link-pose (btr:get-robot-object) camera-link-name))))
      (setf (btr:joint-state (btr:get-robot-object) torso-joint-name)
            torso-high-limit)
      (setf camera-high-z
            (cl-transforms:z
             (cl-transforms:origin
              (btr:link-pose (btr:get-robot-object) camera-link-name))))
      (list camera-low-z camera-high-z)))
  (<- (camera-minimal-height :tiago-dual 1.0665))
  (<- (camera-maximal-height :tiago-dual 1.4165))

  ;; These are values taken from the Kinect's wikipedia page
  ;; for the 360 variant of a Kinect.
  ;; Tiago has an xtion but it should be more or less similar.
  (<- (camera-horizontal-angle :tiago-dual 0.99483)) ;  ca 57 degrees
  (<- (camera-vertical-angle :tiago-dual 0.75049))   ; ca 43 degrees
  )

(def-fact-group tiago-neck-data (robot-neck-links
                                 robot-neck-joints
                                 robot-neck-pan-joint-forward-facing-axis-sign
                                 robot-neck-tilt-joint-forward-facing-axis-sign
                                 robot-joint-states
                                 robot-pose)

  (<- (robot-neck-links :tiago-dual "head_1_link" "head_2_link"))
  (<- (robot-neck-joints :tiago-dual "head_1_joint" "head_2_joint"))

  (<- (robot-neck-pan-joint-forward-facing-axis-sign :tiago-dual
                                                     cl-transforms:x +1))
  (<- (robot-neck-tilt-joint-forward-facing-axis-sign :tiago-dual
                                                      cl-transforms:x +1))

  (<- (robot-joint-states :tiago-dual :neck ?_ :forward
                          ((?pan_joint 0.0) (?tilt_joint 0.0)))
    (robot-neck-joints :tiago-dual ?pan_joint ?tilt_joint))

  (<- (robot-pose :tiago-dual :neck ?_ :forward ?pose-stamped)
    (robot-base-frame :tiago-dual ?base-frame)
    (lisp-fun cl-transforms:make-identity-rotation ?identity-quaternion)
    (symbol-value *forward-looking-position-in-base-frame* ?forward-point)
    (lisp-fun cl-transforms-stamped:make-pose-stamped
              ?base-frame 0.0 ?forward-point ?identity-quaternion
              ?pose-stamped)))


(def-fact-group tiago-arm-facts (arm
                                 arm-links arm-joints
                                 hand-links hand-link hand-finger-link
                                 gripper-joint
                                 gripper-meter-to-joint-multiplier
                                 gripper-minimal-position
                                 gripper-convergence-delta
                                 standard<-particular-gripper-transform
                                 end-effector-link robot-tool-frame
                                 tcp-in-ee-pose
                                 robot-joint-states)

  (<- (arm :tiago-dual :right))
  (<- (arm :tiago-dual :left))

  (<- (arm-links :tiago-dual :left
                 "arm_left_1_link"
                 "arm_left_2_link"
                 "arm_left_3_link"
                 "arm_left_4_link"
                 "arm_left_5_link"
                 "arm_left_6_link"
                 "arm_left_7_link"
                 "arm_left_tool_link"
                 "wrist_left_ft_link"
                 "wrist_left_ft_tool_link"
                 "gripper_left_link"
                 "gripper_left_left_finger_link"
                 "gripper_left_right_finger_link"
                 "gripper_left_tool_link"))
  (<- (arm-links :tiago-dual :right
                 "arm_right_1_link"
                 "arm_right_2_link"
                 "arm_right_3_link"
                 "arm_right_4_link"
                 "arm_right_5_link"
                 "arm_right_6_link"
                 "arm_right_7_link"
                 "arm_right_tool_link"
                 "wrist_right_ft_link"
                 "wrist_right_ft_tool_link"
                 "gripper_right_link"
                 "gripper_right_left_finger_link"
                 "gripper_right_right_finger_link"
                 "gripper_right_tool_link"))

  (<- (arm-joints :tiago-dual :left ("arm_left_1_joint"
                                     "arm_left_2_joint"
                                     "arm_left_3_joint"
                                     "arm_left_4_joint"
                                     "arm_left_5_joint"
                                     "arm_left_6_joint"
                                     "arm_left_7_joint")))
  (<- (arm-joints :tiago-dual :right ("arm_right_1_joint"
                                      "arm_right_2_joint"
                                      "arm_right_3_joint"
                                      "arm_right_4_joint"
                                      "arm_right_5_joint"
                                      "arm_right_6_joint"
                                      "arm_right_7_joint")))

  (<- (hand-links :tiago-dual :left ("gripper_left_link"
                                     "gripper_left_left_finger_link"
                                     "gripper_left_right_finger_link"
                                     "gripper_left_tool_link")))
  (<- (hand-links :tiago-dual :right ("gripper_right_link"
                                      "gripper_right_left_finger_link"
                                      "gripper_right_right_finger_link"
                                      "gripper_right_tool_link")))

  (<- (hand-link :tiago-dual :left ?link)
    (bound ?link)
    (lisp-fun search "gripper_left" ?link ?pos)
    (lisp-pred identity ?pos))
  (<- (hand-link :tiago-dual :right ?link)
    (bound ?link)
    (lisp-fun search "gripper_right" ?link ?pos)
    (lisp-pred identity ?pos))

  (<- (hand-finger-link :tiago-dual ?arm ?link)
    (bound ?link)
    (hand-link :tiago-dual ?arm ?link)
    (lisp-fun search "finger" ?link ?pos)
    (lisp-pred identity ?pos))

  (<- (gripper-joint :tiago-dual :left "gripper_left_left_finger_joint"))
  (<- (gripper-joint :tiago-dual :left "gripper_left_right_finger_joint"))
  (<- (gripper-joint :tiago-dual :right "gripper_right_left_finger_joint"))
  (<- (gripper-joint :tiago-dual :right "gripper_right_right_finger_joint"))

  (<- (gripper-meter-to-joint-multiplier :tiago-dual 0.5))
  (<- (gripper-minimal-position :tiago-dual ?_ 0.0))
  (<- (gripper-convergence-delta :tiago-dual ?_ 0.005))

  (<- (standard<-particular-gripper-transform :tiago-dual ?transform)
    (symbol-value *standard-to-tiago-gripper-transform* ?transform))

  (<- (end-effector-link :tiago-dual :left "arm_left_tool_link"))
  (<- (end-effector-link :tiago-dual :right "arm_right_tool_link"))

  (<- (robot-tool-frame :tiago-dual :left "gripper_left_grasping_frame"))
  (<- (robot-tool-frame :tiago-dual :right "gripper_right_grasping_frame"))

  (<- (tcp-in-ee-pose :tiago-dual ?pose)
    (symbol-value *tcp-in-ee-pose* ?pose))

  (<- (robot-joint-states :tiago-dual :arm :left :park ?joint-states
                          ;; (("arm_left_1_joint" -0.5)
                          ;;  ("arm_left_2_joint" 1.3)
                          ;;  ("arm_left_3_joint" 2.1)
                          ;;  ("arm_left_4_joint" 2.0)
                          ;;  ("arm_left_5_joint" -1.95)
                          ;;  ("arm_left_6_joint" 1.0)
                          ;;  ("arm_left_7_joint" 0.0))
                          )
    (robot-joint-states :tiago-dual :arm :left :carry-top ?joint-states))
  (<- (robot-joint-states :tiago-dual :arm :left :carry ?joint-states
                          ;; (("arm_left_1_joint" -1.0)
                          ;;  ("arm_left_2_joint" 0.0)
                          ;;  ("arm_left_3_joint" 1.5)
                          ;;  ("arm_left_4_joint" 2.2)
                          ;;  ("arm_left_5_joint" -1.5)
                          ;;  ("arm_left_6_joint" 0.5)
                          ;;  ("arm_left_7_joint" 0.0))
                          )
    (robot-joint-states :tiago-dual :arm :left :carry-top ?joint-states))
  (<- (robot-joint-states :tiago-dual :arm :left :carry-top (("arm_left_1_joint" 0.27)
                                                             ("arm_left_2_joint" -1.07)
                                                             ("arm_left_3_joint" 1.5)
                                                             ("arm_left_4_joint" 1.96)
                                                             ("arm_left_5_joint" -2.0)
                                                             ("arm_left_6_joint" 1.2)
                                                             ("arm_left_7_joint" 0.5))))
  (<- (robot-joint-states :tiago-dual :arm :left :hand-over (("arm_left_1_joint" 1.4)
                                                             ("arm_left_2_joint" -0.45)
                                                             ("arm_left_3_joint" 1.7)
                                                             ("arm_left_4_joint" 1.8)
                                                             ("arm_left_5_joint" 0.4)
                                                             ("arm_left_6_joint" -1.4)
                                                             ("arm_left_7_joint" 1.5))
                          ;; (("arm_left_1_joint" 1.4)
                          ;;  ("arm_left_2_joint" -1.07)
                          ;;  ("arm_left_3_joint" 2.0)
                          ;;  ("arm_left_4_joint" 1.7)
                          ;;  ("arm_left_5_joint" -2.0)
                          ;;  ("arm_left_6_joint" 1.2)
                          ;;  ("arm_left_7_joint" 1.0))
                          ))
  (<- (robot-joint-states :tiago-dual :arm :left :carry-top-basket ?joint-states)
    (robot-joint-states :tiago-dual :arm :left :carry-top ?joint-states))

  (<- (robot-joint-states :tiago-dual :arm :right :park ?joint-states
                          ;; (("arm_right_1_joint" -0.5)
                          ;;  ("arm_right_2_joint" 1.3)
                          ;;  ("arm_right_3_joint" 2.1)
                          ;;  ("arm_right_4_joint" 2.0)
                          ;;  ("arm_right_5_joint" -1.95)
                          ;;  ("arm_right_6_joint" 1.0)
                          ;;  ("arm_right_7_joint" 0.0))
                          )
    (robot-joint-states :tiago-dual :arm :right :carry-top ?joint-states))
  (<- (robot-joint-states :tiago-dual :arm :right :carry ?joint-states
                          ;; (("arm_right_1_joint" -1.0)
                          ;;  ("arm_right_2_joint" 0.0)
                          ;;  ("arm_right_3_joint" 1.5)
                          ;;  ("arm_right_4_joint" 2.2)
                          ;;  ("arm_right_5_joint" -1.5)
                          ;;  ("arm_right_6_joint" 0.5)
                          ;;  ("arm_right_7_joint" 0.0))
                          )
    (robot-joint-states :tiago-dual :arm :right :carry-top ?joint-states))
  (<- (robot-joint-states :tiago-dual :arm :right :carry-top (("arm_right_1_joint" 0.27)
                                                              ("arm_right_2_joint" -1.07)
                                                              ("arm_right_3_joint" 1.5)
                                                              ("arm_right_4_joint" 2.0)
                                                              ("arm_right_5_joint" -2.0)
                                                              ("arm_right_6_joint" 1.2)
                                                              ("arm_right_7_joint" 0.5))))
  (<- (robot-joint-states :tiago-dual :arm :right :hand-over (("arm_right_1_joint" 1.4)
                                                              ("arm_right_2_joint" -0.45)
                                                              ("arm_right_3_joint" 1.7)
                                                              ("arm_right_4_joint" 1.8)
                                                              ("arm_right_5_joint" 0.4)
                                                              ("arm_right_6_joint" -1.4)
                                                              ("arm_right_7_joint" 1.5))))
  (<- (robot-joint-states :tiago-dual :arm :right :carry-top-basket ?joint-states)
    (robot-joint-states :tiago-dual :arm :right :carry-top ?joint-states)))


(def-fact-group tiago-cm-metadata (costmap:costmap-padding
                                   costmap:costmap-manipulation-padding
                                   costmap:costmap-in-reach-distance
                                   costmap:costmap-reach-minimal-distance
                                   costmap:orientation-samples
                                   costmap:orientation-sample-step
                                   costmap:visibility-costmap-size)
  ;; This is the radius of the base link,
  #+calculate-aabb-of-the-base-link-with-the-following-function
  (defun calculate-radius-x-and-y-of-base-link-from-bullet (robot-name)
    (let* ((base-link
             (cut:var-value
              '?link
              (cut:lazy-car
               (prolog:prolog
                `(rob-int:robot-base-link ,robot-name ?link)))))
           (base-link-aabb
             (btr:aabb
              (btr:rigid-body (btr:get-robot-object)
                              (intern (format nil "~a.~a"
                                              robot-name
                                              base-link)
                                      :keyword))))
           (aabb-dims
             (cl-bullet:bounding-box-dimensions base-link-aabb))
           (aabb-dims-x
             (cl-transforms:x aabb-dims))
           (aabb-dims-y
             (cl-transforms:y aabb-dims))
           (radius-x
             (/ aabb-dims-x 2.0))
           (radius-y
             (/ aabb-dims-y 2.0)))
      (list radius-x radius-y)))
  (<- (costmap:costmap-padding :tiago-dual 0.27))
  (<- (costmap:costmap-manipulation-padding :tiago-dual 0.3))
  ;; This is the length of the outstretched arm from base center to fingertip
  ;; But shorten it slightly (10-20cm):
  ;; the robot will rarely reach with an outstretched arm
  #+calculate-arm-length-from-bullet-with-this-function
  (defun calculate-arm-length-from-bullet (robot-name arm-name)
  (let* ((base-frame
           (cut:var-value
            '?link
            (cut:lazy-car
             (prolog:prolog
              `(rob-int:robot-base-link ,robot-name ?link)))))
         (tcp-frame
           (cut:var-value
            '?link
            (cut:lazy-car
             (prolog:prolog
              `(rob-int:robot-tool-frame ,robot-name ,arm-name ?link)))))
         (base-P-tcp
           (cl-transforms:transform
            (cl-transforms:transform-inv
             (cl-transforms:pose->transform
              (btr:link-pose (btr:get-robot-object) base-frame)))
            (btr:link-pose (btr:get-robot-object) tcp-frame)))
         (base-P-tcp-origin
           (cl-transforms:origin base-P-tcp))
         (projected-on-the-floor
           (cl-transforms:copy-3d-vector base-P-tcp-origin :z 0.0))
         (length (cl-transforms:v-norm projected-on-the-floor)))
    length))
  (<- (costmap:costmap-in-reach-distance :tiago-dual 0.93))
  (<- (costmap:costmap-reach-minimal-distance :tiago-dual 0.2))
  (<- (costmap:orientation-samples :tiago-dual 1))
  (<- (costmap:orientation-sample-step :tiago-dual 0.3))
  (<- (costmap:visibility-costmap-size :tiago-dual 2)))
