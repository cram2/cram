;;;
;;; Copyright (c) 2023, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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

(in-package :armar-descr)

(def-fact-group armar-metadata (robot-odom-frame
                                robot-base-frame robot-base-link
                                robot-torso-link-joint)
  (<- (robot-odom-frame :armar6 "odom"))
  (<- (robot-base-frame :armar6 "world"))
  (<- (robot-base-link :armar6 "platform"))
  (<- (robot-torso-link-joint :armar6 "torso" "torso_joint")))

(def-fact-group armar-camera-data (camera-frame
                                   camera-minimal-height
                                   camera-maximal-height
                                   camera-horizontal-angle
                                   camera-vertical-angle)

  (<- (camera-frame :armar6 "DepthCamera"))
  (<- (camera-frame :armar6 "FleaCamerasCenter"))
  (<- (camera-frame :armar6 "Roboception"))

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
  (<- (camera-minimal-height :armar6 1.4463))
  (<- (camera-maximal-height :armar6 1.8113))

  ;; These are values taken from the Kinect's wikipedia page
  ;; for the 360 variant of a Kinect.
  ;; Armar has an xtion but it should be more or less similar.
  (<- (camera-horizontal-angle :armar6 0.99483)) ;  ca 57 degrees
  (<- (camera-vertical-angle :armar6 0.75049))   ; ca 43 degrees
  )


(defparameter *forward-looking-position-in-base-frame*
  (cl-transforms:make-3d-vector 10.0 0.0 0.9))

(def-fact-group armar-neck-data (robot-neck-links
                                 robot-neck-joints
                                 robot-neck-pan-joint-forward-facing-axis-sign
                                 robot-neck-tilt-joint-forward-facing-axis-sign
                                 robot-joint-states
                                 robot-pose)

  (<- (robot-neck-links :armar6 "middle_neck" "upper_neck"))
  (<- (robot-neck-joints :armar6 "neck_1_yaw" "neck_2_pitch"))

  ;; It's actually the forward facing axis of the link not the joint.
  (<- (robot-neck-pan-joint-forward-facing-axis-sign :armar6
                                                     cl-transforms:x +1))
  (<- (robot-neck-tilt-joint-forward-facing-axis-sign :armar6
                                                      cl-transforms:z +1))

  (<- (robot-joint-states :armar6 :neck ?_ :forward
                          ((?pan_joint 0.0) (?tilt_joint 0.0)))
    (robot-neck-joints :armar6 ?pan_joint ?tilt_joint))

  (<- (robot-pose :armar6 :neck ?_ :forward ?pose-stamped)
    (robot-base-frame :armar6 ?base-frame)
    (lisp-fun cl-transforms:make-identity-rotation ?identity-quaternion)
    (symbol-value *forward-looking-position-in-base-frame* ?forward-point)
    (lisp-fun cl-transforms-stamped:make-pose-stamped
              ?base-frame 0.0 ?forward-point ?identity-quaternion
              ?pose-stamped)))


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
(defparameter *standard-to-armar-gripper-transform*
  (cl-transforms:make-transform
   (cl-transforms:make-identity-vector)
   (cl-transforms:matrix->quaternion
    #2A((1 0 0)
        (0 1 0)
        (0 0 1)))))

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
(defparameter *tcp-in-ee-pose-left*
  (cl-transforms:make-pose
   (cl-transforms:make-3d-vector -0.08
                                 0.01
                                 0.29)
   (cl-transforms-stamped:make-quaternion -3.702278117475241d-4
                                          -0.3826825663634556d0
                                          -1.533283476191083d-4
                                          0.9238797619694639d0)))
(defparameter *tcp-in-ee-pose-right*
  (cl-transforms:make-pose
   (cl-transforms:make-3d-vector 0.08
                                 0.01
                                 0.29)
   (cl-transforms-stamped:make-quaternion 0.38268251896193783d0
                                          -3.688499091666769d-4
                                          0.9238797712152454d0
                                          1.4996594144343206d-4)))

(def-fact-group armar-arm-facts (arm
                                 arm-links arm-joints
                                 hand-links hand-link hand-finger-link
                                 gripper-joint
                                 gripper-meter-to-joint-multiplier
                                 gripper-joint-min-limit-is-open-state
                                 gripper-minimal-position
                                 gripper-convergence-delta
                                 standard<-particular-gripper-transform
                                 end-effector-link robot-tool-frame
                                 tcp-in-ee-pose
                                 robot-joint-states)

  (<- (arm :armar6 :right))
  (<- (arm :armar6 :left))

  (<- (arm-links :armar6 :left
                 "arm_cla_r0"
                 "arm_t12_r0"
                 "arm_t23_r0"
                 "arm_t34_r0"
                 "arm_t45_r0"
                 "arm_t56_r0"
                 "arm_t67_r0"
                 "arm_t78_r0"
                 "arm_t8_r0"
                 "Hand L Palm"
                 "Index L 1"
                 "Index L 2"
                 "Index L 3"
                 "Middle L 1"
                 "Middle L 2"
                 "Middle L 3"
                 "Ring L 1"
                 "Ring L 2"
                 "Ring L 3"
                 "Pinky L 1"
                 "Pinky L 2"
                 "Pinky L 3"
                 "Thumb L 1"
                 "Thumb L 2"))
  (<- (arm-links :armar6 :right
                 "arm_cla_r1"
                 "arm_t12_r1"
                 "arm_t23_r1"
                 "arm_t34_r1"
                 "arm_t45_r1"
                 "arm_t56_r1"
                 "arm_t67_r1"
                 "arm_t78_r1"
                 "arm_t8_r1"
                 "Hand R Palm"
                 "Index R 1"
                 "Index R 2"
                 "Index R 3"
                 "Middle R 1"
                 "Middle R 2"
                 "Middle R 3"
                 "Ring R 1"
                 "Ring R 2"
                 "Ring R 3"
                 "Pinky R 1"
                 "Pinky R 2"
                 "Pinky R 3"
                 "Thumb R 1"
                 "Thumb R 2"))

  (<- (arm-joints :armar6 :left ("arm_t12_joint_r0"
                                 "arm_t23_joint_r0"
                                 "arm_t34_joint_r0"
                                 "arm_t45_joint_r0"
                                 "arm_t56_joint_r0"
                                 "arm_t67_joint_r0"
                                 "arm_t78_joint_r0"
                                 "arm_t8_joint_r0")))
  (<- (arm-joints :armar6 :right ("arm_t12_joint_r1"
                                  "arm_t23_joint_r1"
                                  "arm_t34_joint_r1"
                                  "arm_t45_joint_r1"
                                  "arm_t56_joint_r1"
                                  "arm_t67_joint_r1"
                                  "arm_t78_joint_r1"
                                  "arm_t8_joint_r1")))

  (<- (hand-links :armar6 :left ("Hand L Palm"
                                 "Index L 1"
                                 "Index L 2"
                                 "Index L 3"
                                 "Middle L 1"
                                 "Middle L 2"
                                 "Middle L 3"
                                 "Ring L 1"
                                 "Ring L 2"
                                 "Ring L 3"
                                 "Pinky L 1"
                                 "Pinky L 2"
                                 "Pinky L 3"
                                 "Thumb L 1"
                                 "Thumb L 2")))
  (<- (hand-links :armar6 :right ("Hand R Palm"
                                  "Index R 1"
                                  "Index R 2"
                                  "Index R 3"
                                  "Middle R 1"
                                  "Middle R 2"
                                  "Middle R 3"
                                  "Ring R 1"
                                  "Ring R 2"
                                  "Ring R 3"
                                  "Pinky R 1"
                                  "Pinky R 2"
                                  "Pinky R 3"
                                  "Thumb R 1"
                                  "Thumb R 2")))

  (<- (hand-link :armar6 ?arm ?link)
    (bound ?link)
    (hand-links :armar6 ?arm ?hand-links)
    (member ?link ?hand-links))

  (<- (hand-finger-link :armar6 ?arm ?link)
    (bound ?link)
    (lisp-fun search "Hand" ?link ?pos)
    (not (lisp-pred identity ?pos))
    (hand-links :armar6 ?arm ?hand-links)
    (member ?link ?hand-links))

  (<- (gripper-joint :armar6 :left ?joint)
    (member ?joint ("Thumb L 1 Joint"
                    "Thumb L 2 Joint"
                    "Index L 1 Joint" "Index L 2 Joint" "Index L 3 Joint"
                    "Middle L 1 Joint" "Middle L 2 Joint" "Middle L 3 Joint"
                    "Ring L 1 Joint" "Ring L 2 Joint" "Ring L 3 Joint"
                    "Pinky L 1 Joint" "Pinky L 2 Joint" "Pinky L 3 Joint")))
  (<- (gripper-joint :armar6 :right ?joint)
    (member ?joint ("Thumb R 1 Joint"
                    "Thumb R 2 Joint"
                    "Index R 1 Joint" "Index R 2 Joint" "Index R 3 Joint"
                    "Middle R 1 Joint" "Middle R 2 Joint" "Middle R 3 Joint"
                    "Ring R 1 Joint" "Ring R 2 Joint" "Ring R 3 Joint"
                    "Pinky R 1 Joint" "Pinky R 2 Joint" "Pinky R 3 Joint")))

  (<- (gripper-meter-to-joint-multiplier :armar6 16.0))
  (<- (gripper-joint-min-limit-is-open-state :armar6))
  (<- (gripper-minimal-position :armar6 ?_ 1.57))
  (<- (gripper-convergence-delta :armar6 ?_ 0.005))

  (<- (standard<-particular-gripper-transform :armar6 ?transform)
    (symbol-value *standard-to-armar-gripper-transform* ?transform))

  (<- (end-effector-link :armar6 :left "arm_t8_r0"))
  (<- (end-effector-link :armar6 :right "arm_t8_r1"))

  (<- (robot-tool-frame :armar6 :left "left_tool_frame"))
  (<- (robot-tool-frame :armar6 :right "right_tool_frame"))

  (<- (tcp-in-ee-pose :armar6 :left ?pose)
    (symbol-value *tcp-in-ee-pose-left* ?pose))
  (<- (tcp-in-ee-pose :armar6 :right ?pose)
    (symbol-value *tcp-in-ee-pose-right* ?pose))

  (<- (robot-joint-states :armar6 :arm :left :park (("arm_t12_joint_r0" 0)
                                                    ("arm_t23_joint_r0" 0)
                                                    ("arm_t34_joint_r0" 1.5)
                                                    ("arm_t45_joint_r0" 0.5)
                                                    ("arm_t56_joint_r0" 2.0)
                                                    ("arm_t67_joint_r0" 1.5)
                                                    ("arm_t78_joint_r0" 0)
                                                    ("arm_t8_joint_r0" 0.0))))
  (<- (robot-joint-states :armar6 :arm :left ?config-name ?joint-states)
    (member ?config-name (:carry :carry-top :hand-over :carry-top-basket))
    (robot-joint-states :armar6 :arm :left :park ?joint-states))

  (<- (robot-joint-states :armar6 :arm :right :park (("arm_t12_joint_r1" 0)
                                                     ("arm_t23_joint_r1" 0)
                                                     ("arm_t34_joint_r1" 1.5)
                                                     ("arm_t45_joint_r1" 2.64)
                                                     ("arm_t56_joint_r1" 2.0)
                                                     ("arm_t67_joint_r1" 1.6415)
                                                     ("arm_t78_joint_r1" 0)
                                                     ("arm_t8_joint_r1" -0.0))))
  (<- (robot-joint-states :armar6 :arm :right ?config-name ?joint-states)
    (member ?config-name (:carry :carry-top :hand-over :carry-top-basket))
    (robot-joint-states :armar6 :arm :right :park ?joint-states)))


(def-fact-group armar-cm-metadata (costmap:costmap-padding
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
  (<- (costmap:costmap-padding :armar6 0.4))
  (<- (costmap:costmap-manipulation-padding :armar6 0.4))
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
  (<- (costmap:costmap-in-reach-distance :armar6 1.3))
  (<- (costmap:costmap-reach-minimal-distance :armar6 0.4))
  (<- (costmap:orientation-samples :armar6 1))
  (<- (costmap:orientation-sample-step :armar6 0.3))
  (<- (costmap:visibility-costmap-size :armar6 2)))
