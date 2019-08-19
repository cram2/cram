(in-package :pp-tut)


(defparameter *final-object-destination*
  (make-pose "map" '((-0.8 2 0.9) (0 0 0 1))))

(defparameter *base-pose-near-table*
  (make-pose "map" '((-1.447d0 -0.150d0 0.0d0) (0.0d0 0.0d0 -0.7071067811865475d0 0.7071067811865476d0))))
;; The orientation in this pose corresponds to a rotation of -(pi/2) around the z-axis.
;; To check it for yourself or to create other orientations you can use the following functions
;; (cl-transforms:axis-angle->quaternion (cl-transforms:make-3d-vector 0 0 1) (/ pi -2))
;; or
;; (cl-transforms:euler->quaternion :az (/ pi -2))
 
(defparameter *downward-look-coordinate*
  (make-pose "base_footprint" '((0.65335d0 0.076d0 0.758d0) (0 0 0 1))))
 
(defparameter *base-pose-near-counter*
  (make-pose "base_footprint" '((-0.150d0 2.0d0 0.0d0) (0.0d0 0.0d0 -1.0d0 0.0d0))))

(defparameter *left-downward-look-coordinate*
  (make-pose "base_footprint" '((0.65335d0 0.76d0 0.758d0) (0 0 0 1))))

(defparameter *right-downward-look-coordinate*
  (make-pose "base_footprint" '((0.65335d0 -0.76d0 0.758d0) (0 0 0 1))))

(defun move-bottle (bottle-spawn-pose)
  (spawn-object bottle-spawn-pose)
  (pr2-proj:with-simulated-robot
    (let ((?navigation-goal *base-pose-near-table*))
      (cpl:par
        (exe:perform (desig:a motion 
                              (type moving-torso)
                              (joint-angle 0.3)))
        (park-arms)
        ;; Moving the robot near the table.
        (exe:perform (desig:a motion
                              (type going)
                              (target (desig:a location 
                                               (pose ?navigation-goal)))))))
    ;; Looking towards the bottle before perceiving.
    (let ((?looking-direction *downward-look-coordinate*))
      (exe:perform (desig:a motion 
                            (type looking)
                            (target (desig:a location 
                                             (pose ?looking-direction))))))
    ;; Detect the bottle on the table.
    (let ((?grasping-arm :right)
          (?perceived-bottle (exe:perform (desig:a motion
                                                   (type detecting)
                                                   (object (desig:an object 
                                                                     (type :bottle)))))))
      ;; Pick up the bottle
      (exe:perform (desig:an action
                             (type picking-up)
                             (arm ?grasping-arm)
                             (grasp left-side)
                             (object ?perceived-bottle)))
      (park-arm ?grasping-arm)
      ;; Moving the robot near the counter.
      (let ((?nav-goal *base-pose-near-counter*))
        (exe:perform (desig:a motion
                              (type going)
                              (target (desig:a location 
                                               (pose ?nav-goal))))))
      (coe:on-event (make-instance 'cpoe:robot-state-changed))
      ;; Setting the bottle down on the counter
      (let ((?drop-pose *final-object-destination*))
        (exe:perform (desig:an action
                               (type placing)
                               (arm ?grasping-arm)
                               (object ?perceived-bottle)
                               (target (desig:a location 
                                                (pose ?drop-pose))))))
      (park-arm ?grasping-arm))))
