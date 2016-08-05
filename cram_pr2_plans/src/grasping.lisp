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
;;; X is the main axis, the longer eigenvector or the cluster point distribution

;;; Rgrasp is in robot coordinate frame, i.e. transform from gripper to robot frame
;;; tgrasp is in object frame

(defparameter *kitchen-sink-block-z* 0.85 "in meters")

(defparameter *plate-diameter* 0.26 "in meters")
(defparameter *plate-grasp-y-offset* (- (/ *plate-diameter* 2) 0.05) "in meters")
;; (defparameter *plate-grasp-z-offset* 0.015 "in meters")
(defparameter *plate-grasp-z-offset* 0.06 "in meters") ; red stacked on blue plate
(defparameter *plate-pregrasp-y-offset* 0.2 "in meters")
(defparameter *plate-pregrasp-z-offset* 0.4 "in meters")
(defparameter *plate-2nd-pregrasp-z-offset* 0.035 "in meters") ; grippers can't go into table

(defparameter *cutlery-pregrasp-z-offset* 0.4 "in meters")
(defparameter *cutlery-grasp-z-offset* 0.002 "in meters")

(defparameter *cup-grasp-z-offset* 0.03 "in meters")
(defparameter *cup-pregrasp-xy-offset* 0.2 "in meters")
(defparameter *cup-pregrasp-z-offset* 0.4 "in meters")

(defparameter *bottle-grasp-z-offset* 0.09 "in meters")
(defparameter *bottle-pregrasp-xy-offset* 0.2 "in meters")
(defparameter *bottle-pregrasp-z-offset* 0.4 "in meters")


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
   (cl-transforms:make-3d-vector 0.4 -0.3 1.4)
   (cl-transforms:make-quaternion 0.029319081708036543d0 -0.018714920400581137d0
                                  0.5257710356470319d0 0.8499146788218482d0)))
(defparameter *pr2-left-arm-out-of-sight-gripper-pose*
  (cl-transforms-stamped:make-pose-stamped
   "base_footprint"
   0.0
   (cl-transforms:make-3d-vector 0.4 0.3 1.4)
   (cl-transforms:make-quaternion 0.9215513103717499d0 -0.387996037470125d0
                                  -0.014188589447636247d0 -9.701489976338351d-4)))

(setf cram-tf:*tf-default-timeout* 20)

(defmacro with-pr2-process-modules (&body body)
  `(cram-process-modules:with-process-modules-running
       (pr2-pms::pr2-perception-pm pr2-pms::pr2-base-pm pr2-pms::pr2-arms-pm
                                   pr2-pms::pr2-grippers-pm pr2-pms::pr2-ptu-pm)
     (cpl:top-level
       ,@body)))

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


(defun move-pr2-right-arm-out-of-sight ()
  ;; (pr2-ll:call-joint-angle-action
  ;;  :right
  ;;  *pr2-right-arm-out-of-sight-joint-positions*)
  (cpl:with-failure-handling
      ((pr2-ll:pr2-low-level-failure (e)
         (declare (ignore e))
         (return)))
    (let ((?cartesian-pose-to-go *pr2-right-arm-out-of-sight-gripper-pose*))
      (cram-plan-library:perform
       (desig:an action
                 (to move)
                 (right arm)
                 (to ?cartesian-pose-to-go))))))

(defun move-pr2-left-arm-out-of-sight ()
  (cpl:with-failure-handling
      ((pr2-ll:pr2-low-level-failure (e)
         (declare (ignore e))
         (return)))
    (let ((?cartesian-pose-to-go *pr2-left-arm-out-of-sight-gripper-pose*))
      (cram-plan-library:perform
       (desig:an action
                 (to move)
                 (left arm)
                 (to ?cartesian-pose-to-go))))))

(defun move-pr2-arms-out-of-sight ()
  (cpl:with-failure-handling
      ((pr2-ll:pr2-low-level-failure (e)
         (declare (ignore e))
         (return)))
    (let ((?left-pose-to-go *pr2-left-arm-out-of-sight-gripper-pose*)
          (?right-pose-to-go *pr2-right-arm-out-of-sight-gripper-pose*))
      (cram-plan-library:perform
       (desig:an action
                 (to move)
                 (both arms)
                 (left ?left-pose-to-go)
                 (right ?right-pose-to-go))))))


(defun calculate-cutlery-grasp-pose (yaw x-obj y-obj z-support z-obj-offset &optional arm)
  (declare (ignore arm))
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

(defun get-cutlery-grasp-pose (object-designator)
  (let* ((object-pose (desig:desig-prop-value object-designator :pose))
         (theta (cl-transforms:get-yaw (cl-transforms:orientation object-pose)))
         (translation (cl-transforms:origin object-pose))
         (x-obj (cl-transforms:x translation))
         (y-obj (cl-transforms:y translation)))
    (calculate-cutlery-grasp-pose theta x-obj y-obj
                                  *kitchen-sink-block-z*
                                  *cutlery-grasp-z-offset*)))

(defun get-cutlery-put-pose ()
  ;; (calculate-cutlery-grasp-pose 0.0 (+ 0.7 (random 0.2)) (- (random 0.4))
  ;;                               *kitchen-sink-block-z*
  ;;                               *cutlery-grasp-z-offset*)
  (calculate-cutlery-grasp-pose 0.0 0.7 -0.7
                                *kitchen-sink-block-z*
                                *cutlery-grasp-z-offset*))

(defun get-cutlery-pregrasp-pose (grasp-pose)
  (translate-pose grasp-pose :z-offset *cutlery-pregrasp-z-offset*))

(defun get-cutlery-lift-pose (grasp-pose)
  (get-cutlery-pregrasp-pose grasp-pose))

(defun top-level-pick-cutlery (&optional (?color ""))
  (with-pr2-process-modules
    (move-pr2-right-arm-out-of-sight)
    (let* ((?object-desig (desig:an object
                                    (type "Cutlery")
                                    (color ?color)))
           (?updated-object-desig (cram-plan-library:perform
                                   (desig:an action
                                             (to detect)
                                             (object ?object-desig))))
           (?cutlery-grasp-pose (get-cutlery-grasp-pose ?updated-object-desig))
           (?cutlery-pregrasp-pose (get-cutlery-pregrasp-pose ?cutlery-grasp-pose))
           (?cutlery-lift-pose (get-cutlery-lift-pose ?cutlery-grasp-pose)))
      (pr2-ll:visualize-marker ?cutlery-grasp-pose)
      (cpl:with-failure-handling
          ((pr2-ll:pr2-low-level-failure (e)
             (roslisp:ros-warn (top-level pick) "~a" e)
             (return)))
        (cram-plan-library:perform
         (desig:an action
                   (to move)
                   (right arm)
                   (to ?cutlery-pregrasp-pose))))
      (print "pregrasped")
      (cpl:with-failure-handling
          ((pr2-ll:pr2-low-level-failure (e)
             (roslisp:ros-warn (top-level pick) "~a" e)
             (return)))
        (cram-plan-library:perform
         (desig:an action
                   (to open)
                   (right gripper))))
      (print "opened gripper")
      (cpl:with-retry-counters ((approach-retries 1))
        (cpl:with-failure-handling
            ((pr2-ll:pr2-low-level-failure (e)
               (cpl:do-retry approach-retries
                 (roslisp:ros-warn (top-level pick) "~a" e)
                 (cpl:retry))
               (return)))
          (cram-plan-library:perform
           (desig:an action
                     (to move)
                     (right arm)
                     (to ?cutlery-grasp-pose)))))
      (print "at grasp pose")
      (cpl:with-retry-counters ((grasping-retries 1))
        (cpl:with-failure-handling
            ((pr2-ll:pr2-low-level-failure (e)
               (cpl:do-retry grasping-retries
                 (roslisp:ros-warn (top-level pick) "~a" e)
                 (cpl:retry))
               (cpl:fail 'cram-plan-failures:gripping-failed)))
          (cram-plan-library:perform
           (desig:an action
                     (to grip)
                     (object (desig:an object (type cutlery)))
                     (with right)))))
      (print "grasped")
      (cpl:with-failure-handling
          ((pr2-ll:pr2-low-level-failure (e)
             (roslisp:ros-warn (top-level pick) "~a" e)
             (return)))
        (cram-plan-library:perform
         (desig:an action
                   (to move)
                   (right arm)
                   (to ?cutlery-lift-pose))))
      (move-pr2-right-arm-out-of-sight))))

(defun top-level-place-cutlery ()
  (with-pr2-process-modules
    (move-pr2-right-arm-out-of-sight)
    (let* ((?cutlery-grasp-pose (get-cutlery-put-pose))
           (?cutlery-pregrasp-pose (get-cutlery-pregrasp-pose ?cutlery-grasp-pose))
           (?cutlery-lift-pose (get-cutlery-lift-pose ?cutlery-grasp-pose)))
      (pr2-ll:visualize-marker ?cutlery-grasp-pose)
      (cpl:with-failure-handling
          ((pr2-ll:pr2-low-level-failure (e)
             (roslisp:ros-warn (top-level pick) "~a" e)
             (return)))
        (cram-plan-library:perform
         (desig:an action
                   (to move)
                   (right arm)
                   (to ?cutlery-lift-pose))))
      (print "lifted")
      (cpl:with-retry-counters ((approach-retries 1))
        (cpl:with-failure-handling
            ((pr2-ll:pr2-low-level-failure (e)
               (cpl:do-retry approach-retries
                 (roslisp:ros-warn (top-level pick) "~a" e)
                 (cpl:retry))
               (return)))
          (cram-plan-library:perform
           (desig:an action
                     (to move)
                     (right arm)
                     (to ?cutlery-grasp-pose)))))
      (print "at grasp pose")
      (cpl:with-failure-handling
          ((pr2-ll:pr2-low-level-failure (e)
             (declare (ignore e))
             (return)))
        (cram-plan-library:perform
         (desig:an action
                   (to open)
                   (right gripper))))
      (print "released grasp")
      (cpl:with-failure-handling
          ((pr2-ll:pr2-low-level-failure (e)
             (roslisp:ros-warn (top-level pick) "~a" e)
             (return)))
        (cram-plan-library:perform
         (desig:an action
                   (to move)
                   (right arm)
                   (to ?cutlery-pregrasp-pose))))
      (print "pregrasped")
      (move-pr2-right-arm-out-of-sight))))


;;;;;;;;;;;;;;;;;;;;;;;; PLATE ;;;;;;;;;;;;;;;;;;;;;;;;

(defun calculate-plate-grasp-pose (x-obj y-obj y-obj-offset
                                   z-support z-obj-offset &key arm (roll (/ pi 3)))
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

(defun get-plate-grasp-pose (object-designator arm)
  (let* ((object-pose (desig:desig-prop-value object-designator :pose))
         (translation (cl-transforms:origin object-pose))
         (x-obj (cl-transforms:x translation))
         (y-obj (cl-transforms:y translation)))
    (calculate-plate-grasp-pose x-obj y-obj *plate-grasp-y-offset*
                                *kitchen-sink-block-z* *plate-grasp-z-offset*
                                :arm arm)))

(defun get-plate-pregrasp-pose (grasp-pose arm)
  (translate-pose grasp-pose
                  :y-offset (case arm
                              (:right (- *plate-pregrasp-y-offset*))
                              (:left *plate-pregrasp-y-offset*)
                              (t (error "arm can be :left or :right")))
                  :z-offset *plate-pregrasp-z-offset*))

(defun get-plate-second-pregrasp-pose (grasp-pose arm)
  (translate-pose grasp-pose
                  :y-offset (case arm
                              (:right (- *plate-pregrasp-y-offset*))
                              (:left *plate-pregrasp-y-offset*)
                              (t (error "arm can be :left or :right")))
                  :z-offset *plate-2nd-pregrasp-z-offset*))

(defun get-plate-lift-pose (grasp-pose arm)
  (declare (ignore arm))
  (translate-pose grasp-pose
                  :z-offset *plate-pregrasp-y-offset*))

(defun top-level-pick-plate (&optional (?color ""))
  (with-pr2-process-modules
    (move-pr2-arms-out-of-sight)
    (let* ((?object-desig (desig:an object
                                    (type "plate")
                                    (color ?color)))
           (?updated-object-desig (cram-plan-library:perform
                                   (desig:an action
                                             (to detect)
                                             (object ?object-desig))))
           (?left-grasp-pose (get-plate-grasp-pose ?updated-object-desig :left))
           (?left-pregrasp-pose (get-plate-pregrasp-pose ?left-grasp-pose :left))
           (?left-2nd-pregrasp-pose (get-plate-second-pregrasp-pose ?left-grasp-pose :left))
           (?left-lift-pose (get-plate-lift-pose ?left-grasp-pose :left))
           (?right-grasp-pose (get-plate-grasp-pose ?updated-object-desig :right))
           (?right-pregrasp-pose (get-plate-pregrasp-pose ?right-grasp-pose :right))
           (?right-2nd-pregrasp-pose (get-plate-second-pregrasp-pose ?right-grasp-pose :right))
           (?right-lift-pose (get-plate-lift-pose ?right-grasp-pose :right)))
      (pr2-ll:visualize-marker ?left-grasp-pose :id 1 :r-g-b-list '(1 0 1))
      (pr2-ll:visualize-marker ?right-grasp-pose :id 2 :r-g-b-list '(1 0 1))
      (cpl:with-failure-handling
          ((pr2-ll:pr2-low-level-failure (e)
             (roslisp:ros-warn (top-level pick) "~a" e)
             (return)))
        (cram-plan-library:perform
         (desig:an action
                   (to move)
                   (both arms)
                   (left ?left-pregrasp-pose)
                   (right ?right-pregrasp-pose))))
      (print "pregrasped")
      (cpl:with-failure-handling
          ((pr2-ll:pr2-low-level-failure (e)
             (roslisp:ros-warn (top-level pick) "~a" e)
             (return)))
        (cram-plan-library:perform
         (desig:an action
                   (to open)
                   (right gripper))))
      (print "opened right gripper")
      (cpl:with-failure-handling
          ((pr2-ll:pr2-low-level-failure (e)
             (roslisp:ros-warn (top-level pick) "~a" e)
             (return)))
        (cram-plan-library:perform
         (desig:an action
                   (to open)
                   (left gripper))))
      (print "opened left gripper")
      (cpl:with-failure-handling
          ((pr2-ll:pr2-low-level-failure (e)
             (roslisp:ros-warn (top-level pick) "~a" e)
             (return)))
        (cram-plan-library:perform
         (desig:an action
                   (to move)
                   (both arms)
                   (left ?left-2nd-pregrasp-pose)
                   (right ?right-2nd-pregrasp-pose))))
      (print "second pregrasped")
      (cpl:with-retry-counters ((approach-retries 1))
        (cpl:with-failure-handling
            ((pr2-ll:pr2-low-level-failure (e)
               (cpl:do-retry approach-retries
                 (roslisp:ros-warn (top-level pick) "~a" e)
                 (cpl:retry))
               (return)))
          (cram-plan-library:perform
           (desig:an action
                     (to move)
                     (both arms)
                     (left ?left-grasp-pose)
                     (right ?right-grasp-pose)))))
      (print "at grasp pose")
      (cpl:with-retry-counters ((grasping-retries 1))
        (cpl:with-failure-handling
            ((pr2-ll:pr2-low-level-failure (e)
               (cpl:do-retry grasping-retries
                 (roslisp:ros-warn (top-level pick) "~a" e)
                 (cpl:retry))
               (cpl:fail 'cram-plan-failures:gripping-failed)))
          (cram-plan-library:perform
           (desig:an action
                     (to grip)
                     (object (desig:an object (type cutlery)))
                     (with left)))))
      (cpl:with-retry-counters ((grasping-retries 1))
        (cpl:with-failure-handling
            ((pr2-ll:pr2-low-level-failure (e)
               (cpl:do-retry grasping-retries
                 (roslisp:ros-warn (top-level pick) "~a" e)
                 (cpl:retry))
               (cpl:fail 'cram-plan-failures:gripping-failed)))
          (cram-plan-library:perform
           (desig:an action
                     (to grip)
                     (object (desig:an object (type cutlery)))
                     (with right)))))
      (print "grasped")
      (cpl:with-failure-handling
          ((pr2-ll:pr2-low-level-failure (e)
             (roslisp:ros-warn (top-level pick) "~a" e)
             (return)))
        (cram-plan-library:perform
         (desig:an action
                   (to move)
                   (both arms)
                   (left ?left-lift-pose)
                   (right ?right-lift-pose)))))))




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; CUPS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

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

(defun get-cup-grasp-pose (object-designator arm grasp)
  (let* ((object-pose (desig:desig-prop-value object-designator :pose))
         (translation (cl-transforms:origin object-pose))
         (x-obj (cl-transforms:x translation))
         (y-obj (cl-transforms:y translation)))
    (calculate-cup-grasp-pose x-obj y-obj
                              *kitchen-sink-block-z* *cup-grasp-z-offset*
                              arm grasp)))

(defun get-cup-put-pose (arm grasp)
  (calculate-cup-grasp-pose 0.8 -0.6
                            *kitchen-sink-block-z* *cup-grasp-z-offset*
                            arm grasp))

(defun get-cup-pregrasp-pose (grasp-pose arm grasp)
  (case grasp
    (:front (translate-pose grasp-pose :x-offset (- *cup-pregrasp-xy-offset*)
                                       :z-offset *cup-pregrasp-z-offset*))
    (:side (case arm
             (:left (translate-pose grasp-pose :y-offset *cup-pregrasp-xy-offset*
                                               :z-offset *cup-pregrasp-z-offset*))
             (:right (translate-pose grasp-pose :y-offset (- *cup-pregrasp-xy-offset*)
                                                :z-offset *cup-pregrasp-z-offset*))
             (t (error "arm can only be :left or :right"))))
    (t (error "grasp can only be :side or :front"))))

(defun get-cup-second-pregrasp-pose (grasp-pose arm grasp)
   (case grasp
    (:front (translate-pose grasp-pose :x-offset (- *cup-pregrasp-xy-offset*)))
    (:side (case arm
             (:left (translate-pose grasp-pose :y-offset *cup-pregrasp-xy-offset*))
             (:right (translate-pose grasp-pose :y-offset (- *cup-pregrasp-xy-offset*)))
             (t (error "arm can only be :left or :right"))))
    (t (error "grasp can only be :side or :front"))))

(defun get-cup-lift-pose (grasp-pose)
  (translate-pose grasp-pose :z-offset *cup-pregrasp-z-offset*))

(defun top-level-pick-cup (&key (?color "") (?arm :right) (?grasp :side))
  (with-pr2-process-modules
    (move-pr2-arms-out-of-sight)
    (let* ((?object-desig (desig:an object
                                    (type "Cup")
                                    (color ?color)))
           (?updated-object-desig (cram-plan-library:perform
                                   (desig:an action
                                             (to detect)
                                             (object ?object-desig))))
           (?cup-grasp-pose (get-cup-grasp-pose ?updated-object-desig ?arm ?grasp))
           (?cup-pregrasp-pose (get-cup-pregrasp-pose ?cup-grasp-pose ?arm ?grasp))
           (?cup-lift-pose (get-cup-lift-pose ?cup-grasp-pose))
           (?cup-2nd-pregrasp-pose (get-cup-second-pregrasp-pose ?cup-grasp-pose ?arm ?grasp)))
      (pr2-ll:visualize-marker ?cup-grasp-pose)
      (cpl:with-failure-handling
          ((pr2-ll:pr2-low-level-failure (e)
             (roslisp:ros-warn (top-level pick) "~a" e)
             (return)))
        (cram-plan-library:perform
         (desig:an action
                   (to move)
                   (?arm arm)
                   (to ?cup-pregrasp-pose))))
      (print "pregrasped")
      (cpl:with-failure-handling
          ((pr2-ll:pr2-low-level-failure (e)
             (roslisp:ros-warn (top-level pick) "~a" e)
             (return)))
        (cram-plan-library:perform
         (desig:an action
                   (to open)
                   (?arm gripper))))
      (print "opened gripper")
      (cpl:with-failure-handling
          ((pr2-ll:pr2-low-level-failure (e)
             (roslisp:ros-warn (top-level pick) "~a" e)
             (return)))
        (cram-plan-library:perform
         (desig:an action
                   (to move)
                   (?arm arm)
                   (to ?cup-2nd-pregrasp-pose))))
      (print "second pregrasped")
      (cpl:with-retry-counters ((approach-retries 1))
        (cpl:with-failure-handling
            ((pr2-ll:pr2-low-level-failure (e)
               (cpl:do-retry approach-retries
                 (roslisp:ros-warn (top-level pick) "~a" e)
                 (cpl:retry))
               (return)))
          (cram-plan-library:perform
           (desig:an action
                     (to move)
                     (?arm arm)
                     (to ?cup-grasp-pose)))))
      (print "at grasp pose")
      (cpl:with-retry-counters ((grasping-retries 1))
        (cpl:with-failure-handling
            ((pr2-ll:pr2-low-level-failure (e)
               (cpl:do-retry grasping-retries
                 (roslisp:ros-warn (top-level pick) "~a" e)
                 (cpl:retry))
               (cpl:fail 'cram-plan-failures:gripping-failed)))
          (cram-plan-library:perform
           (desig:an action
                     (to grip)
                     (object (desig:an object (type mug)))
                     (with ?arm)))))
      (print "grasped")
      (cpl:with-failure-handling
          ((pr2-ll:pr2-low-level-failure (e)
             (roslisp:ros-warn (top-level pick) "~a" e)
             (return)))
        (cram-plan-library:perform
         (desig:an action
                   (to move)
                   (?arm arm)
                   (to ?cup-lift-pose))))
      (move-pr2-arms-out-of-sight))))

(defun top-level-place-cup (&key (?arm :right) (?grasp :side))
  (with-pr2-process-modules
    (move-pr2-arms-out-of-sight)
    (let* ((?cup-grasp-pose (get-cup-put-pose ?arm ?grasp))
           (?cup-pregrasp-pose (get-cup-pregrasp-pose ?cup-grasp-pose ?arm ?grasp))
           (?cup-lift-pose (get-cup-lift-pose ?cup-grasp-pose)))
      (pr2-ll:visualize-marker ?cup-grasp-pose)
      (cpl:with-failure-handling
          ((pr2-ll:pr2-low-level-failure (e)
             (roslisp:ros-warn (top-level pick) "~a" e)
             (return)))
        (cram-plan-library:perform
         (desig:an action
                   (to move)
                   (?arm arm)
                   (to ?cup-lift-pose))))
      (print "lifted")
      (cpl:with-retry-counters ((approach-retries 1))
        (cpl:with-failure-handling
            ((pr2-ll:pr2-low-level-failure (e)
               (cpl:do-retry approach-retries
                 (roslisp:ros-warn (top-level pick) "~a" e)
                 (cpl:retry))
               (return)))
          (cram-plan-library:perform
           (desig:an action
                     (to move)
                     (?arm arm)
                     (to ?cup-grasp-pose)))))
      (print "at grasp pose")
      (cpl:with-failure-handling
          ((pr2-ll:pr2-low-level-failure (e)
             (declare (ignore e))
             (return)))
        (cram-plan-library:perform
         (desig:an action
                   (to open)
                   (?arm gripper))))
      (print "released grasp")
      (cpl:with-failure-handling
          ((pr2-ll:pr2-low-level-failure (e)
             (roslisp:ros-warn (top-level pick) "~a" e)
             (return)))
        (cram-plan-library:perform
         (desig:an action
                   (to move)
                   (?arm arm)
                   (to ?cup-pregrasp-pose))))
      (print "pregrasped")
      (move-pr2-arms-out-of-sight))))


(defun pick-and-place-stuff ()
  (top-level-pick-cutlery "blue")
  (top-level-place-cutlery)
  (top-level-pick-cup :?color "yellow")
  (top-level-place-cup :?grasp :front)
  (top-level-pick-plate "red"))



;; (an action
;;     (to pick-up)
;;     (object (an object (type cutlery) ...))
;;     (phases (a motion
;;                (type reaching)
;;                (pregasp-poses ...))
;;             (a motion (type grasping))
;;             (a motion (type lifting) (object #object))))

;; (an action
;;     (to pour)
;;     (destination (an object))
;;     (source (an object))
;;     (theme (some stuff))
;;     (goal drink-poured)
;;     (phases (a motion (type approach-destination))
;;             (a motion
;;                (type tilting)
;;                (angle )
;;                ())
;;             (a motion (type untilting))
;;             (a motion (type moving-out-of-way))))

;; (an aciton
;;     (to put-down))
