(in-package :tut-tests)
 
;;; Turtle world is like this:
;;;  Y
;;;  ^
;;;  |
;;;  |
;;;  o-----> X
;;; Identity orientation is looking right in the direction of X axis.
 
(define-test relative-angle-to-dont-turn
  ;; turtle is in origin with identity orientation, wants to look at (3 0 0)
  (let* ((goal (cl-transforms:make-3d-vector 3 0 0))
         (pose-msg (roslisp:make-message 'turtlesim-msg:pose))
         (angle (tut::relative-angle-to goal pose-msg)))
    (assert-number-equal angle 0.0))
 
  ;; turtle is in (3 0 0) with identity orientation, wants to look at (4 0 0)
  (let* ((goal (cl-transforms:make-3d-vector 4 0 0))
         (pose-msg (roslisp:make-message 'turtlesim-msg:pose
                                         :x 3.0 :y 0.0 :theta 0.0))
         (angle (tut::relative-angle-to goal pose-msg)))
    (assert-number-equal angle 0.0))
 
  ;; turtle is in (0 0 0) looking up in direction of Y, wants to look at (0 1 0)
  (let* ((goal (cl-transforms:make-3d-vector 0 1 0))
         (pose-msg (roslisp:make-message 'turtlesim-msg:pose
                                         :x 0.0 :y 0.0 :theta (/ pi 2)))
         (angle (tut::relative-angle-to goal pose-msg)))
    (assert-number-equal angle 0.0))
 
  ;; turtle is in (1 1 0) looking towards the origin, wants to look at (0 0 0)
  (let* ((goal (cl-transforms:make-3d-vector 0 0 0))
         (pose-msg (roslisp:make-message 'turtlesim-msg:pose
                                         :x 1.0 :y 1.0 :theta (+ pi (/ pi 4))))
         (angle (tut::relative-angle-to goal pose-msg)))
    (assert-number-equal angle 0.0)))
 
(define-test relative-angle-to-look-back
  ;; turtle is in origin, wants to look at (-1 0 0)
  (let* ((goal (cl-transforms:make-3d-vector -1 0 0))
         (pose-msg (roslisp:make-message 'turtlesim-msg:pose))
         (angle (tut::relative-angle-to goal pose-msg)))
    (assert-number-equal angle pi))
 
  ;; turtle is in (1 0 0), wants to look at origin
  (let* ((goal (cl-transforms:make-3d-vector 0 0 0))
         (pose-msg (roslisp:make-message 'turtlesim-msg:pose
                                         :x 1.0 :y 0.0 :theta 0.0))
         (angle (tut::relative-angle-to goal pose-msg)))
    (assert-number-equal angle pi))
 
  ;; turtle is in (-1 -1 0) looking towards origin, wants to look away from origin
  (let* ((goal (cl-transforms:make-3d-vector -2 -2 0))
         (pose-msg (roslisp:make-message 'turtlesim-msg:pose
                                         :x -1.0 :y -1.0 :theta (/ pi 4)))
         (angle (tut::relative-angle-to goal pose-msg)))
      (assert-number-equal angle pi)))
 
(define-test relative-angle-to-type-error
  (assert-error 'type-error 
                (tut::relative-angle-to 
                 (cl-transforms:make-identity-pose)
                 (roslisp:make-message 'turtlesim-msg:pose))))
