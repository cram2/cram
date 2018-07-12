;;;
;;; Copyright (c) 2018, Gayane Kazhoyan <kazhoyan@cs.uni-bremen.de>
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
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors
;;;       may be used to endorse or promote products derived from this software
;;;       without specific prior written permission.
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

(defparameter *plate-x* -1.108)
(defparameter *plate-y* 1.6)
(defparameter *plate-z* 0.8626)

(defparameter *rotation-around-x+90* '(0.7071067811865475d0 0.0d0 0.0d0 0.7071067811865476d0))
(defparameter *rotation-around-y-90* '(0.0d0 -0.7071067811865475d0 0.0d0 0.7071067811865476d0))
(defparameter *rotation-around-z-90* '(0.0d0 0.0d0 -0.7071067811865475d0 0.7071067811865476d0))
(defparameter *rotation-around-z+90* '(0.0d0 0.0d0 0.7071067811865475d0 0.7071067811865476d0))
(defparameter *rotation-around-z-y* '(-0.7071067690849304d0 0.7071067690849304d0 0.0d0 0.0d0))
(defparameter *rotation-around-z-x* '(0.7071067690849304d0 0.7071067690849304d0 0.0d0 0.0d0))
(defparameter *rotation-around-x-y* '(0.0d0 0.7071067811865475d0 0.7071067811865475d0 0.0d0))
(defparameter *rotation-around-x-y-2* '(0.5d0 0.5d0 0.5d0 0.5d0))

(defparameter *plate-rad-x* 0.36)
(defparameter *plate-rad-y* 0.60)
(defparameter *plate-rad-z* 0.008)
(defparameter *holder-bolt-rad-x* 0.05)
(defparameter *holder-bolt-rad-y* 0.025)
(defparameter *holder-bolt-rad-z* 0.0125)
(defparameter *bolt-rad-x* 0.01)
(defparameter *bolt-rad-y* 0.01)
(defparameter *bolt-rad-z* 0.0225)
(defparameter *holder-upperbody-rad-x* 0.085)
(defparameter *holder-upperbody-rad-y* 0.025)
(defparameter *holder-upperbody-rad-z* 0.021)
(defparameter *holder-bottom-wing-rad-x* 0.035)
(defparameter *holder-bottom-wing-rad-y* 0.045)
(defparameter *holder-bottom-wing-rad-z* 0.045)
(defparameter *holder-underbody-rad-x* 0.0925)
(defparameter *holder-underbody-rad-y* 0.025)
(defparameter *holder-underbody-rad-z* 0.024)
(defparameter *holder-plane-horizontal-rad-x* 0.1015)
(defparameter *holder-plane-horizontal-rad-y* 0.05)
(defparameter *holder-plane-horizontal-rad-z* 0.0335)
(defparameter *holder-window-rad-x* 0.026)
(defparameter *holder-window-rad-y* 0.025)
(defparameter *holder-window-rad-z* 0.017)
(defparameter *front-wheel-rad-z* 0.015)
(defparameter *nut-rad-z* 0.0075)
(defparameter *chassis-rad-z* 0.0515)
(defparameter *holder-plane-vertical-rad-x* 0.065)
(defparameter *holder-plane-vertical-rad-y* 0.06)
(defparameter *holder-plane-vertical-rad-z* 0.0175)
(defparameter *holder-top-wing-rad-x* 0.035)
(defparameter *holder-top-wing-rad-y* 0.0415)
(defparameter *holder-top-wing-rad-z* 0.045)

(defparameter *yellow-plastic* '(0.949 0.807 0.082))
(defparameter *gray-plastic* '(0.568 0.592 0.584))
(defparameter *red-plane* '(0.878 0.376 0.243))
(defparameter *cyan-plane* '(0.078 0.513 0.541))
(defparameter *yellow-plane* '(0.964 0.847 0.584))
(defparameter *transparent-plane* '(1 1 1))
(defparameter *black-plane* '(0.184 0.152 0.121))
(defparameter *gray-plane* '(0.482 0.537 0.549))

(defparameter *object-spawning-data*
  `((chassis :chassis ,*yellow-plane*
             ((0.2 0.9 ,*chassis-rad-z*) ,*rotation-around-z-90*))
    (big-wooden-plate :big-wooden-plate (0.823 0.698 0.513)
                      ((,*plate-rad-x* ,*plate-rad-y* ,(- *plate-rad-z*)) (0 0 0 1)))
    (holder-bolt :holder-bolt ,*yellow-plastic*
                 ((,*holder-bolt-rad-x* ,*holder-bolt-rad-y* ,*holder-bolt-rad-z*) (0 0 0 1)))
    (bolt-1 :bolt ,*gray-plane*
            ((0.015 0.0125 ,*bolt-rad-z*) (0 0 0 1)))
    (bolt-2 :bolt ,*gray-plane*
            ((0.0325 0.0375 ,*bolt-rad-z*) (0 0 0 1)))
    (bolt-3 :bolt ,*gray-plane*
            ((0.05 0.0125 ,*bolt-rad-z*) (0 0 0 1)))
    (bolt-4 :bolt ,*gray-plane*
            ((0.0675 0.0375 ,*bolt-rad-z*) (0 0 0 1)))
    (bolt-5 :bolt ,*gray-plane*
            ((0.085 0.0125 ,*bolt-rad-z*) (0 0 0 1)))
    (holder-upper-body :holder-upper-body ,*yellow-plastic*
                       ((,(+ 0.05 *holder-upperbody-rad-x*) 0.10 ,*holder-upperbody-rad-z*)
                        (0 0 0 1)))
    (upper-body :upper-body ,*red-plane*
               ((0.119 0.1003 0.0482) (0 0 0 1)))
    (holder-bottom-wing :holder-bottom-wing ,*gray-plastic*
                        ((,(+ 0.1 *holder-bottom-wing-rad-x*)
                          ,(- 0.3 *holder-bottom-wing-rad-y*)
                          ,*holder-bottom-wing-rad-z*)
                         ,*rotation-around-z-90*))
    (bottom-wing :bottom-wing ,*cyan-plane*
                 ((0.134 0.25 0.093) (0 0 0 1)))
    (holder-underbody :holder-underbody ,*yellow-plastic*
                     ((,(+ 0.05 *holder-underbody-rad-x*) 0.4 ,*holder-underbody-rad-z*)
                      (0 0 0 1)))
    (underbody :underbody ,*red-plane*
               ((0.145 0.399 0.024) (0 0 0 1)))
    (holder-plane-horizontal :holder-plane-horizontal ,*yellow-plastic*
                             ((,(+ 0.05 *holder-plane-horizontal-rad-x*)
                               0.6
                               ,*holder-plane-horizontal-rad-z*)
                              (0 0 0 1)))
    (rear-wing :rear-wing ,*yellow-plane*
               ((0.079 0.599 0.056)
                ,*rotation-around-z+90*))
    (holder-window :holder-window ,*gray-plastic*
                   ((,*holder-window-rad-x*
                    ,(+ 0.75 *holder-window-rad-y*)
                    ,*holder-window-rad-z*)
                   (0 0 0 1)))
    (window :window ,*transparent-plane*
            ((0.024 0.775 0.01962) (0 0 0 1)))
    (front-wheel-1 :front-wheel ,*black-plane*
                   ((0.15 0.775 ,*front-wheel-rad-z*) (0 0 0 1)))
    (front-wheel-2 :front-wheel ,*black-plane*
                 ((0.215 0.775 ,*front-wheel-rad-z*) (0 0 0 1)))
    (nut-1 :nut ,*gray-plane*
           ((0.15 0.725 ,*nut-rad-z*) (0 0 0 1)))
    (nut-2 :nut ,*gray-plane*
           ((0.215 0.725 ,*nut-rad-z*) (0 0 0 1)))
    (holder-plane-vertical :holder-plane-vertical ,*yellow-plastic*
                              ((,*holder-plane-vertical-rad-x*
                                ,(- 1.0 *holder-plane-vertical-rad-y*)
                                ,*holder-plane-vertical-rad-z*)
                               (0 0 0 1)))
    (propeller :propeller ,*yellow-plane*
               ((0.075 1.10 0) (0 0 0 1)))
    (holder-top-wing :holder-top-wing ,*yellow-plastic*
                     ((,(+ 0.15 *holder-top-wing-rad-x*)
                       ,(- 1.15 *holder-top-wing-rad-y*)
                       ,*holder-top-wing-rad-z*)
                      ,*rotation-around-z-90*))
    (top-wing :top-wing ,*cyan-plane*
              ((0.18522 1.11423 0.08852) (0 0 0 1)))))

(defun spawn-objects-on-plate (&optional (spawning-poses *object-spawning-data*))
  (btr-utils:kill-all-objects)
  (btr:detach-all-objects (btr:get-robot-object))
  ;; let ((object-types '(:breakfast-cereal :cup :bowl :spoon :milk)))
  ;; spawn objects at default poses
  (let ((objects (mapcar (lambda (object-name-type-pose-list)
                           (destructuring-bind (object-name object-type object-color
                                                object-pose-list)
                               object-name-type-pose-list
                             (let ((object-relative-pose
                                     (cram-tf:list->pose object-pose-list)))
                               (btr-utils:spawn-object
                                object-name
                                object-type
                                :mass 0.0
                                :color object-color
                                :pose (cram-tf:pose->list
                                       (cl-tf:make-pose
                                        (cl-transforms:v+
                                         (cl-transforms:make-3d-vector
                                          (- *plate-x* *plate-rad-x*)
                                          (- *plate-y* *plate-rad-y*)
                                          (+ *plate-z* *plate-rad-z*))
                                         (cl-transforms:origin
                                          object-relative-pose))
                                        (cl-transforms:orientation
                                         object-relative-pose)))))))
                         spawning-poses)))
    objects))


#+everything-below-is-pr2-s-stuff-so-need-new-things-for-boxy
(
(defparameter *sink-nav-goal*
  (cl-transforms-stamped:make-pose-stamped
   "map"
   0.0
   (cl-transforms:make-3d-vector 0.75d0 0.70d0 0.0)
   (cl-transforms:make-identity-rotation)))
(defparameter *island-nav-goal*
  (cl-transforms-stamped:make-pose-stamped
   "map"
   0.0
   (cl-transforms:make-3d-vector -0.2d0 1.5d0 0.0)
   (cl-transforms:make-quaternion 0 0 1 0)))
(defparameter *look-goal*
  (cl-transforms-stamped:make-pose-stamped
   "base_footprint"
   0.0
   (cl-transforms:make-3d-vector 0.5d0 0.0d0 1.0d0)
   (cl-transforms:make-identity-rotation)))

(defparameter *object-grasping-arms*
  '((:breakfast-cereal . :right)
    (:cup . :left)
    (:bowl . :right)
    (:spoon . :right)
    (:milk . :right)))

(defparameter *object-placing-poses*
  '((:breakfast-cereal . ((-0.78 0.9 0.95) (0 0 1 0)))
    (:cup . ((-0.79 1.35 0.9) (0 0 0.7071 0.7071)))
    (:bowl . ((-0.76 1.19 0.88) (0 0 0.7071 0.7071)))
    (:spoon . ((-0.78 1.5 0.86) (0 0 0 1)))
    (:milk . ((-0.75 1.7 0.95) (0 0 0.7071 0.7071)))))

(defun go-to-sink-or-island (&optional (sink-or-island :sink))
  (let ((?navigation-goal (ecase sink-or-island
                            (:sink *sink-nav-goal*)
                            (:island *island-nav-goal*)))
        (?ptu-goal *look-goal*))
    (cpl:par
      (pp-plans::park-arms)
      (exe:perform (desig:a motion
                            (type going)
                            (target (desig:a location (pose ?navigation-goal))))))
    (exe:perform (desig:a motion
                          (type looking)
                          (target (desig:a location (pose ?ptu-goal)))))))

(defun pick-object (&optional (?object-type :breakfast-cereal) (?arm :right))
  (pp-plans:park-arms)
  (go-to-sink-or-island :sink)
  (let* ((?object-desig
           (desig:an object (type ?object-type)))
         (?perceived-object-desig
           (exe:perform (desig:an action
                                  (type detecting)
                                  (object ?object-desig)))))
    (cpl:par
      (exe:perform (desig:an action
                             (type looking)
                             (object ?perceived-object-desig)))
      (exe:perform (desig:an action
                             (type picking-up)
                             (arm ?arm)
                             (object ?perceived-object-desig))))))

(defun place-object (?target-pose &optional (?arm :right))
  (pp-plans:park-arms)
  (go-to-sink-or-island :island)
  (cpl:par
    (exe:perform (desig:a motion
                          (type looking)
                          (target (desig:a location
                                           (pose ?target-pose)))))
    (exe:perform (desig:an action
                           (type placing)
                           (arm ?arm)
                           (target (desig:a location
                                            (pose ?target-pose)))))))


;;; ASSEMBLY STEPS:
;;; * put chassis on holder (bump inwards)
;;; * put bottom wing on chassis
;;; * put underbody on bottom wing
;;; * put upperbody on underbody
;;; * screw rear hole
;;; * put top wing on body
;;; * screw top wing
;;; * put window on body
;;; * screw window
;;; * put plane on vertical holder
;;; * put propeller on grill
;;; * screw propeller
;;; * put wheel on
;;; * screw nut onto wheel
;;; * put other wheel on
;;; * screw nut onto wheel
;;; * screw bottom body
;;;
(defun demo-hard-coded ()
  (spawn-objects-on-plate)

  (boxy-proj:with-simulated-robot

    (dolist (object-type '(:breakfast-cereal :cup :bowl :spoon :milk))

      (let ((placing-target
              (cl-transforms-stamped:pose->pose-stamped
               "map" 0.0
               (cram-bullet-reasoning:ensure-pose
                (cdr (assoc object-type *object-placing-poses*)))))
            (arm-to-use
              (cdr (assoc object-type *object-grasping-arms*))))

        (pick-object object-type arm-to-use)
        (place-object placing-target arm-to-use)))))
)
