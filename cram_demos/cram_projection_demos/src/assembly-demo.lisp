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

(in-package :demos)

(defparameter *plate-x* -1.1115)
(defparameter *plate-y* 1.6)
(defparameter *plate-z* 0.8626)
(defparameter *original-plate-z* 0.8626)

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
  `((:big-wooden-plate
     :big-wooden-plate
     (0.823 0.698 0.513)
     ((,*plate-rad-x* ,*plate-rad-y* ,(- *plate-rad-z*)) (0 0 0 1)))
    (:holder-bolt
     :holder-bolt
     ,*yellow-plastic*
     ((,*holder-bolt-rad-x* ,*holder-bolt-rad-y* ,*holder-bolt-rad-z*) (0 0 0 1)))
    (:holder-upper-body
     :holder-upper-body
     ,*yellow-plastic*
     ((,(+ 0.05 *holder-upperbody-rad-x*) 0.10 ,*holder-upperbody-rad-z*) (0 0 0 1)))
    (:holder-bottom-wing
     :holder-bottom-wing
     ,*gray-plastic*
     ((,(+ 0.1 *holder-bottom-wing-rad-x*)
       ,(- 0.3 *holder-bottom-wing-rad-y*)
       ,*holder-bottom-wing-rad-z*)
      ,man-int:*rotation-around-z-90-list*))
    (:holder-underbody
     :holder-underbody
     ,*yellow-plastic*
     ((,(+ 0.05 *holder-underbody-rad-x*) 0.4 ,*holder-underbody-rad-z*) (0 0 0 1)))
    (:holder-plane-horizontal
     :holder-plane-horizontal
     ,*yellow-plastic*
     ((,(+ 0.02 *holder-plane-horizontal-rad-x*) 0.6 ,*holder-plane-horizontal-rad-z*)
      (0 0 0.7071d0 0.7071d0)))
    (:holder-window
     :holder-window
     ,*gray-plastic*
     ((,*holder-window-rad-x* ,(+ 0.75 *holder-window-rad-y*) ,*holder-window-rad-z*)
      (0 0 0 1)))
    (:holder-plane-vertical
     :holder-plane-vertical
     ,*yellow-plastic*
     ((,*holder-plane-vertical-rad-x*
       ,(- 1.0 *holder-plane-vertical-rad-y*)
       ,*holder-plane-vertical-rad-z*)
      (0 0 1 0)))
    (:holder-top-wing
     :holder-top-wing
     ,*yellow-plastic*
     ((,(+ 0.15 *holder-top-wing-rad-x*)
       ,(- 1.15 *holder-top-wing-rad-y*)
       ,*holder-top-wing-rad-z*)
      ,man-int:*rotation-around-z-90-list*))

    ;; rear wing is already well positioned
    (:rear-wing
     :rear-wing
     ,*yellow-plane*
     ((0.121 0.528 0.055) ,man-int:*rotation-around-z+180-list*))

    ;; bolts are used intermediately
    (:bolt-1
     :bolt
     ,*gray-plane*
     ((0.015 0.0125 ,*bolt-rad-z*) (0 0 0 1)))
    (:bolt-2
     :bolt
     ,*gray-plane*
     ((0.0325 0.0375 ,*bolt-rad-z*) (0 0 0 1)))
    (:bolt-3
     :bolt
     ,*gray-plane*
     ((0.05 0.0125 ,*bolt-rad-z*) (0 0 0 1)))
    (:bolt-4
     :bolt
     ,*gray-plane*
     ((0.0675 0.0375 ,*bolt-rad-z*) (0 0 0 1)))
    (:bolt-5
     :bolt
     ,*gray-plane*
     ((0.085 0.0125 ,*bolt-rad-z*) (0 0 0 1)))

    ;; first part of scenario on horizontal holder
    (:chassis
     :chassis
     ,*yellow-plane*
     ((0.2 0.95 ,*chassis-rad-z*) ,man-int:*rotation-around-z-90-list*))
    (:bottom-wing
     :bottom-wing
     ,*cyan-plane*
     ((0.134 0.25 0.093) (0 0 0 1)))
    (:underbody
     :underbody
     ,*red-plane*
     ((0.145 0.399 0.024) (0 0 0 1)))
    (:motor-grill
     :motor-grill
     ,*black-plane*
     ((0.238 0.399 0.039) ,man-int:*rotation-around-y+90-list*))
    (:upper-body
     :upper-body
     ,*red-plane*
     ((0.119 0.1003 0.0482) (0 0 0 1)))
    (:top-wing
     :top-wing
     ,*cyan-plane*
     ((0.18522 1.11423 0.08852) (0 0 0 1)))
    (:window
     :window
     ,*transparent-plane*
     ((0.024 0.775 0.01962) (0 0 0 1)))

    ;; second part of scenario on vertical holder
    (:propeller
     :propeller
     ,*yellow-plane*
     ((0.075 1.13 0) ,man-int:*rotation-around-z-90-list*))
    (:front-wheel-1
     :front-wheel
     ,*black-plane*
      ((0.15 0.825 ,*front-wheel-rad-z*) (0 0 0 1)))
    (:front-wheel-2
     :front-wheel
     ,*black-plane*
     ((0.215 0.825 ,*front-wheel-rad-z*) (0 0 0 1)))
    (:nut-1
     :nut
     ,*gray-plane*
     ((0.15 0.775 ,*nut-rad-z*) (0 0 0 1)))
    (:nut-2
     :nut
     ,*gray-plane*
     ((0.215 0.775 ,*nut-rad-z*) (0 0 0 1)))))


(defun spawn-assembly-objects (&optional (spawning-data *object-spawning-data*))
  (kill-and-detach-all)
  ;; spawn objects at default poses
  (let ((objects
          (mapcar (lambda (object-name-type-pose-list)
                    (destructuring-bind (object-name object-type object-color
                                         object-pose-list)
                        object-name-type-pose-list
                      (let ((object-relative-pose
                              (cram-tf:list->pose object-pose-list)))
                        (unless (btr:object btr:*current-bullet-world* object-name)
                          (roslisp:ros-info (assembly) "Spawning ~a" object-name)
                          (btr:add-object
                           btr:*current-bullet-world*
                           :mesh
                           object-name
                           (cl-transforms:make-identity-pose)
                           :mesh object-type
                           :mass 0.0
                           :color object-color))
                        (setf (btr:pose
                               (btr:object btr:*current-bullet-world*
                                           object-name))
                              (cl-transforms:make-pose
                               (cl-transforms:v+
                                (cl-transforms:make-3d-vector
                                 (- *plate-x* *plate-rad-x*)
                                 (- *plate-y* *plate-rad-y*)
                                 (+ *plate-z* *plate-rad-z*))
                                (cl-transforms:origin
                                 object-relative-pose))
                               (cl-transforms:orientation
                                object-relative-pose))))))
                  spawning-data)))

    (btr:attach-object :underbody :motor-grill)

    objects))


(defun transport (?object-type ?object-location-property
                  ?other-object-type ?other-object-location-property
                  ?attachment-type
                  ?wooden-plate)
  (let* ((?env-name
           (rob-int:get-environment-name))
         (?object-location
           (if (eq ?object-location-property :bolt)
               (desig:a location
                        (on (desig:an object
                                      (type counter-top)
                                      (urdf-name kitchen-island-surface)
                                      (part-of ?env-name)))
                        (range-invert 0.9)
                        (side right)
                        (side front))
               (desig:a location
                        (on ?wooden-plate)
                        (side front)
                        ?object-location-property)))
         (?other-object-location
           (desig:a location
                    (on ?wooden-plate)
                    (side front)
                    ?other-object-location-property))
         (?object
           (desig:an object
                     (type ?object-type)
                     (location ?object-location)))
         (?other-object
           (desig:an object
                     (type ?other-object-type)
                     (location ?other-object-location))))
    (exe:perform
     (desig:an action
               (type transporting)
               (object ?object)
               (target (desig:a location
                                (on ?other-object)
                                (for ?object)
                                (attachments (?attachment-type))))))))

;;; ASSEMBLY STEPS:
;;; (1)  put chassis on holder (bump inwards)
;;; (2)  put bottom wing on chassis
;;; *    maybe: dont hit the top-wing with the arm
;;; (3)  put underbody on bottom wing
;;; (4)  put upperbody on underbody
;;; (5)  screw rear hole
;;; (6)  put top wing on body
;;; (7)  screw top wing
;;; (8)  put window on body
;;; (9)  screw window
;;; (10) put plane on vertical holder
;;; (11) put propeller on grill
;;; (12) screw propeller
;;; * put wheel on
;;; * screw nut onto wheel
;;; * put other wheel on
;;; * screw nut onto wheel
;;; * screw bottom body
(defun assembly-demo ()
  (when (eq (rob-int:get-robot-name) :tiago-dual)
    (let ((kitchen-island-offset -0.2))
      (setf *plate-z* (+ *plate-z* kitchen-island-offset))
      (btr-belief:vary-kitchen-urdf `(("kitchen_island_footprint_joint"
                                       ((-1.365d0 0.59d0 ,kitchen-island-offset) (0 0 0 1)))))
      (setf btr:*current-bullet-world* (make-instance 'btr:bt-reasoning-world))
      (btr-belief:spawn-world)))

  (urdf-proj:with-projected-robot
    ;;(setf cram-robosherlock::*no-robosherlock-mode* t)
    (spawn-assembly-objects)
    (let ((old-visibility btr:*visibility-threshold*))
      (setf btr:*visibility-threshold*
            (case (rob-int:get-robot-name)
              (:iai-donbot 0.1) ; perceiving with an object in hand is hard
              (t 0.4)))
      (unwind-protect
           (let* ((?env-name
                    (rob-int:get-environment-name))
                  (wooden-plate
                    (desig:an object
                              (type big-wooden-plate)
                              (location (desig:a location
                                                 (on (desig:an object
                                                               (type counter-top)
                                                               (urdf-name
                                                                kitchen-island-surface)
                                                               (part-of ?env-name)))
                                                 ;; (side back)
                                                 (side front)
                                                 (range 0.3))))))
             ;; 1
             (transport :chassis '(:side :left) :holder-plane-horizontal '(:range 0.3)
                        :horizontal-attachment
                        wooden-plate)
             ;; 2
             (transport :bottom-wing '(:side :right) :chassis '(:range 0.3)
                        :wing-attachment
                        wooden-plate)
             ;; 3
             (transport :underbody '(:side :right) :bottom-wing '(:range 0.3)
                        :body-attachment
                        wooden-plate)

             ;; we put the underbody on the bottom-wing but by doing that
             ;; we also put it on the rear-wing.
             ;; as there is no explicit placing action,
             ;; the attachment that we get is loose,
             ;; so we have to attach them manually unfortunately.
             ;; this is required for later moving the whole plane onto another holder
             (btr:attach-object :underbody :rear-wing)

             ;; 4
             (transport :upper-body '(:side :right) :underbody '(:range 0.3)
                        :body-on-body
                        wooden-plate)
             ;; 5
             (transport :bolt '(:side :right) :upper-body '(:range 0.3)
                        :rear-thread
                        wooden-plate)
             ;; 6
             (transport :top-wing '(:side :left) :upper-body '(:range 0.3)
                        :wing-attachment
                        wooden-plate)
             ;; 7
             (transport :bolt :bolt :top-wing '(:range 0.3)
                        :middle-thread
                        wooden-plate)
             ;; 8
             (transport :window '(:side :left) :top-wing '(:range 0.3)
                        :window-attachment
                        wooden-plate)
             ;; 9
             (transport :bolt :bolt :window '(:range 0.3)
                        :window-thread
                        wooden-plate)

             ;; 10
             (transport :top-wing  '(:range 0.3) :holder-plane-vertical '(:side :left)
                        :vertical-attachment
                        wooden-plate)

             ;; 11
             (transport :propeller '(:side :left) :motor-grill '(:side :left)
                        :propeller-attachment
                        wooden-plate)

             ;; 12
             (transport :bolt :bolt :propeller '(:side :left)
                        :propeller-thread
                        wooden-plate))
        (setf *plate-z* *original-plate-z*)
        (setf btr:*visibility-threshold* old-visibility)))))


#+boxy-action-examples
(
 (urdf-proj:with-projected-robot
   (cram-executive:perform
    (desig:an action
              (type looking)
              (direction down))))

 (urdf-proj:with-projected-robot
   (cram-executive:perform
    (desig:an action
              (type detecting)
              (object (desig:an object (type chassis))))))

 (urdf-proj:with-simulated-robot
   (exe:perform
    (desig:an action
              (type opening-gripper)
              (gripper left))))

 (urdf-proj:with-projected-robot
   (cram-executive:perform
    (desig:an action
              (type placing)
              (arm left))))
 )
