;;; Copyright (c) 2014, Gayane Kazhoyan <kazhoyan@in.tum.de>
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

(in-package :spatial-relations-demo)

(disable-location-validation-function 'btr-desig::validate-designator-solution)
(disable-location-validation-function 'btr-desig::check-ik-solution)

;; roslaunch spatial_relations_demo demo.launch
(defvar *bdgs* nil)
(defun start-ros-and-bullet ()
  (setf *bdgs* nil)
  (roslisp-utilities:startup-ros :anonymous nil)
  (let ((urdf (cl-urdf:parse-urdf (roslisp:get-param "robot_description_lowres")))
        (kitchen-urdf (cl-urdf:parse-urdf (roslisp:get-param "kitchen_description")))
        (pi-rotation '(0 0 1 0)))
    (setf *bdgs*
          (car
           (force-ll
            (prolog
             `(and
               (clear-bullet-world)
               (bullet-world ?w)
               (debug-window ?w)
               (robot ?robot)
               (assert (object ?w btr::static-plane floor ((0 0 0.001) (0 0 0 1))
                               :normal (0 0 1) :constant 0 :disable-collisions-with (?robot)))
               (assert (object ?w btr::semantic-map my-kitchen ((-3.45 -4.35 0) ,pi-rotation)
                               :urdf ,kitchen-urdf))
               (assert (object ?w urdf ?robot ((0 0 0) (0 0 0 1)) :urdf ,urdf))
               (robot-arms-parking-joint-states ?joint-states)
               (assert (joint-state ?w ?robot ?joint-states))
               (assert (joint-state ?w ?robot (("torso_lift_joint" 0.33)))))))))))

;; (setf sem-map (var-value '?sem-map (lazy-car (prolog `(%object ?w my-kitchen ?sem-map) *bdgs*))))

(defun move-object (object-name new-pose)
  (prolog `(and
            (bullet-world ?w)
            (assert (object-pose ?w ,object-name ,new-pose)))))


;;;;;;;;;;;;;;;;;;;; PROLOG ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(def-fact-group spatial-relations-demo-utilities ()
  (<- (assign-object-pos ?obj-name ?desig)
    (once
     (bound ?obj-name)
     (bound ?desig)
     (bullet-world ?w)
     (desig-solutions ?desig ?solutions)
     (take 1 ?solutions ?8-solutions)
     (btr::generate ?poses-on (btr::obj-poses-on ?obj-name ?8-solutions ?w))
     (member ?solution ?poses-on)
     (assert (object-pose ?w ?obj-name ?solution))))

  (<- (assign-object-pos-on ?obj-name ?desig)
    (once
     (bound ?obj-name)
     (bound ?desig)
     (bullet-world ?w)
     (desig-solutions ?desig ?solutions)
     (take 8 ?solutions ?8-solutions)
     (member ?solution ?8-solutions)
     (assert (btr::object-pose-on ?w ?obj-name ?solution)))))


;;;;;;;;;;;;;;;;;;;; CRAM FUNCTIONS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(cpl-impl:def-cram-function find-object-on-counter (object-type counter-name)
  "Returns an object designator."
  (cram-language-designator-support:with-designators
      ((on-counter (desig-props:location `((desig-props:on "Cupboard")
                                           (desig-props:name ,counter-name))))
       (the-object (desig-props:object `((desig-props:type ,object-type)
                                         (desig-props:at ,on-counter)))))
    (reference on-counter)
    (format t "trying to perceive an object ~a~%" the-object)
    (plan-lib:perceive-object 'cram-plan-library:a the-object)))
