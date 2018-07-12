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

(in-package :kr-assembly)

;;; object-in-hand assert and retract

(defmethod cram-occasions-events:on-event object-in-hand ((event cpoe:object-attached))
  (let* (;; (arm (cpoe:event-arm event))
         ;; (grasp (cpoe:event-grasp event))
         (object-name (cpoe:event-object-name event))
         (gripper-id "left_gripper")
         (kr-grasp-class (car (get-possible-object-grasps object-name gripper-id))))
    (assert-object-grasped gripper-id object-name "boxy" kr-grasp-class)))

(defmethod cram-occasions-events:on-event object-in-hand ((event cpoe:object-detached))
  (let* (;; (arm (cpoe:event-arm event))
         (object-designator (cpoe:event-object event))
         (object-name (desig:desig-prop-value object-designator :name))
         (kr-gripper-id "left_gripper"))
    (retract-object-grasped object-name kr-gripper-id)))


;;; assemblage connections assert and retract

(defclass object-connection-event (cram-occasions-events:event)
  ((object
    :initarg :object :reader event-object
    :type desig:object-designator
    :initform (error
               'simple-error
               :format-control "OBJECT-CONNECTION-EVENT requires an object."))
   (with-object
    :initarg :with-object :reader event-with-object
    :type desig:object-designator
    :initform (error
               'simple-error
               :format-control "OBJECT-CONNECTION-EVENT requires a with-object."))
   ;; (affordance
   ;;  :initarg :arm :reader event-arm
   ;;  :type keyword
   ;;  :initform (error
   ;;             'simple-error
   ;;             :format-control "OBJECT-CONNECTION-EVENT requires an affordance."))
   )
  (:documentation "Base class for all events that indicate that a
  physical connection between an object and the with-object changed."))

(defclass object-attached (object-connection-event) ())

(defclass object-detached (object-connection-event) ())

(defmethod cram-occasions-events:on-event objects-connected ((event object-attached))
  (let* ((object-designator (event-object event))
         (with-object-designator (event-with-object event))
         (object-name (desig:desig-prop-value object-designator :name))
         (with-object-name (desig:desig-prop-value with-object-designator :name)))
    (ecase object-name
      (:axle1
       (assert-assemblage :chassis-with-front-axle :axle-snap-in-front
                          object-name with-object-name))
      (:axle2
       (assert-assemblage :chassis-with-axles :axle-snap-in-back
                          object-name with-object-name))
      (:camaro-body1
       (assert-assemblage :body-on-chassis :chassis-snap-in-connection
                          object-name with-object-name))
      (:seat1
       (assert-assemblage :chassis-with-axles-and-front-seat :seat-snap-in-front
                          object-name with-object-name))
      (:seat2
       (assert-assemblage :chassis-with-axles-and-seats :seat-snap-in-back
                          object-name with-object-name)))))

(defmethod cram-occasions-events:on-event objects-connected ((event object-detached))
  (format t "Detached was no implemented for this scenario yet. ~
             The prolog query has to change to accept assemblage description and not ID."))
