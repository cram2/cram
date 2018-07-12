;;; Copyright (c) 2011, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :cram-plan-occasions-events)

;;; Note that we do not provide an event for object change. The reason
;;; is that object changes are essentially object perception events.
(defclass object-perceived-event (event)
  ((object-designator
    :initarg :object-designator :reader event-object-designator
    :initform (error
               'simple-error
               :format-control "OBJECT-PERCEIVED-EVENT requires an object."))
   (perception-source
    :initarg :perception-source :reader perception-source
    :initform (error
               'simple-error
               :format-control "OBJECT-PERCEIVED-EVENT requires a perception source.")))
  (:documentation "Event that is generated whenever an object is
  perceived. The slot `object-designator' contains a reference to the
  designator describing the perceived object and the slot `source'
  contains a symbol indicating the sensor that produces the
  perception."))

;; NOTE(winkler): This contradicts the above notice; will be resolved
;; later after everything got cleaned up, as this is a conceptual
;; issue, not a code-one.
(defclass object-updated-event (object-perceived-event) ())

(defclass object-removed-event (event)
  ((object-name :initarg :object-name :reader event-object-name
                :initform :object-name)))

(defclass robot-state-changed (event)
  ()
  (:documentation "Event that is generated whenever the robot state
  changes, i.e. whenever the robot has moved."))

(defclass object-connection-event (event)
  ((object-name
    :initarg :object-name :reader event-object-name
    :initform (error
               'simple-error
               :format-control "OBJECT-CONNECTION-EVENT requires an object."))
   (arm
    :initarg :arm :reader event-arm
    :initform (error
               'simple-error
               :format-control "OBJECT-CONNECTION-EVENT requires an arm.")))
  (:documentation "Base class for all events that indicate that a
  physical connection between an object and the robot changed."))

;; (defclass object-connection-event (event)
;;   ((object
;;     :initarg :object :reader event-object
;;     :initform (error
;;                'simple-error
;;                :format-control "OBJECT-CONNECTION-EVENT requires an object."))
;;    (link
;;     :initarg :link :reader event-link
;;     :initform (error
;;                'simple-error
;;                :format-control "OBJECT-CONNECTION-EVENT requires a link."))
;;    (side
;;     :initarg :side :reader event-side
;;     :initform nil))
;;   (:documentation "Base class for all events that indicate that a
;;   physical connection between an object and the robot changed."))

(defclass object-attached (object-connection-event) ())

(defclass object-detached (object-connection-event) ())

(defclass object-articulation-event (event)
  ((object-designator
    :initarg :object-designator :reader event-object-designator
    :initform (error
               'simple-error
               :format-control "OBJECT-ARTICULATION-EVENT requires an object."))
   (opening-distance
    :initarg :opening-distance :reader opening-distance
    :initform  (error
                'simple-error
                :format-control "OBJECT-ARTICULATION-EVENT requires an opening distance."))))

(defclass environment-manipulation-event (event)
  ((joint-name
    :documentation "The name of the joint that is being manipulated."
    :initarg :joint-name :reader environment-event-joint-name
    :initform (error
               'simple-error
               :format-control "ENVIRONMENT_MANIPULATION_EVENT requires a joint-name."))
   (arm
    :documentation "Which arm is used to manipulate."
    :initarg :side :reader environment-event-arm
    :initform (error
               'simple-error
               :format-control "ENVIRONMENT_MANIPULATION_EVENT requires an arm."))
   (object
    :documentation "The btr:object in which the joint can be found with the joint-name."
    :initarg :environment :reader environment-event-object
    :initform (error
               'simple-error
               :format-control "ENVIRONMENT_MANIPULATION_EVENT requires an object."))
   (distance
    :documentation "Joint angle distance to open or close container."
    :initarg :distance :reader environment-event-distance
    :initform (error
               'simple-error
               :format-control "ENVIRONMENT_MANIPULATION_EVENT requires a distance."))))

;; (defclass container-handle-grasping-event (environment-manipulation-event) ())

(defclass container-opening-event (environment-manipulation-event) ())

(defclass container-closing-event (environment-manipulation-event) ())

;; (defclass object-gripped (cram-occasions-events:event)
;;   ((arm :initarg :arm
;;         :reader event-arm
;;         :initform (error 'simple-error
;;                          :format-control "OBJECT-GRIPPED event requires ARM."))
;;    (object :initarg :object
;;            :reader event-object
;;            :initform (error 'simple-error
;;                             :format-control "OBJECT-GRIPPED event requires OBJECT."))
;;    (grasp :initarg :grasp
;;           :reader event-grasp
;;           :initform (error 'simple-error
;;                            :format-control "OBJECT-GRIPPED event requires GRASP.")))
;;   (:documentation "Event that is generated whenever the robot successfully
;; closed a gripper around an object."))

;; (defclass object-released (cram-occasions-events:event)
;;   ((arm :initarg :arm
;;         :reader event-arm
;;         :initform (error 'simple-error
;;                          :format-control "OBJECT-GRIPPED event requires OBJECT."))
;;    (object :initarg :object ; maybe some epic robots can release only one of two objects they hold
;;            :reader event-object
;;            :initform (error 'simple-error
;;                             :format-control "OBJECT-GRIPPED event requires OBJECT."))))
