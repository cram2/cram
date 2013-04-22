;;; Copyright (c) 2013, Georg Bartels <georg.bartels@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;; * Redistributions of source code must retain the above copyright
;;; notice, this list of conditions and the following disclaimer.
;;; * Redistributions in binary form must reproduce the above copyright
;;; notice, this list of conditions and the following disclaimer in the
;;; documentation and/or other materials provided with the distribution.
;;; * Neither the name of the Institute for Artificial Intelligence/
;;; Universitaet Bremen nor the names of its contributors may be used to 
;;; endorse or promote products derived from this software without specific 
;;; prior written permission.
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

(in-package :pr2-itasc-pm)

(defvar *itasc-objects* () "List of itasc objects used in the demo")

(defvar *itasc-robots* () "List of itasc objects that might be used in a demo")

(defvar *itasc-tasks* () "List of tasks that are used in our demonstration.")

;; Definitions of classes that hold the relevant information for our demo
(defclass itasc-object-information ()
  ((object-name :reader object-name :initarg :object-name)
   (object-type :reader object-type :initarg :object-type)
   (attached-frames :reader attached-frames :initarg :attached-frames)))

(defclass itasc-robot ()
  ((robot-name :reader robot-name :initarg :robot-name)
   (robot-type :reader robot-type :initarg :robot-type)
   (kinematic-chains :reader kinematic-chains :initarg :kinematic-chains)
   (attached-frames :reader attached-frames :initarg :attached-frames)))

(defclass itasc-robot-kinematic-chain ()
  ((chain-name :reader chain-name :initarg :chain-name)
   (robot-joint-weights :reader robot-joint-weights :initarg :robot-joint-weights)))

(defclass itasc-robot-joint-weight ()
  ((joint-name :reader joint-name :initarg :joint-name)
   (weight :reader weight :initarg :weight)))

(defclass itasc-task ()
  ((name :reader name :initarg :name)
   (output-type :reader output-type :initarg :output-type)
   (vkc :reader vkc :initarg :vkc)
   (joint-constraints :reader joint-constraints :initarg :joint-constraints)
   (feature-constraints :reader feature-constraints :initarg :feature-constraints)
   (priority :reader priority :initarg :priority)))

(defclass itasc-virtual-kinematic-chain ()
  ((chain-joints :reader chain-joints :initarg :chain-joints)
   (object1 :reader object1 :initarg :object1)
   (object2 :reader object2 :initarg :object2)))

(defclass itasc-chain-joint ()
  ((joint-name :reader joint-name :initarg :joint-name)
   (joint-type :reader joint-type :initarg :joint-type)))

(defclass itasc-object-frame ()
  ((object-name :reader object-name :initarg :object-name)
   (frame-name :reader frame-name :initarg :frame-name)))

(defclass itasc-constraint ()
  ((constraint-type :reader constraint-type :initarg :constraint-type)
   (constraint-name :reader constraint-name :initarg :constraint-name)
   (referred-joint :reader referred-joint :initarg :referred-joint)
   (operator :reader operator :initarg :operator)
   (value :reader value :initarg :value)
   (lower-boundary :reader lower-boundary :initarg :lower-boundary)
   (upper-boundary :reader upper-boundary :initarg :upper-boundary)
   (weight :reader weight :initarg :weight)
   (controller :reader controller :initarg :controller)
   (trajectory-type :reader trajectory-type :initarg :trajectory-type)
   (trajectory-duration :reader trajectory-duration :initarg :trajectory-duration)
   (trajectory-velocity :reader trajectory-velocity :initarg :trajectory-velocity)))

;;; object database convenience functions...
(defun clear-itasc-object-list ()
  (setf *itasc-objects* ()))

(defun add-itasc-object (&key object-name object-type frames)
  (setf *itasc-objects*
        (append *itasc-objects*
                (list (make-instance 'itasc-object-information
                                     :object-name object-name
                                     :object-type object-type
                                     :attached-frames frames)))))

(defun find-itasc-object (object-name)
  (find object-name *itasc-objects* :key #'object-name :test #'string=))

(defun object-frame-valid-p (object-name frame-name)
  (or (object-frame-on-object-p object-name frame-name)
      (object-frame-on-robot-p object-name frame-name)))

(defun object-frame-on-object-p (object-name frame-name)
  (when (and object-name frame-name)
    (when (find-itasc-object object-name)
      (when (some (lambda (frame)
                    (string= frame frame-name))
                  (attached-frames (find-itasc-object object-name)))
        t))))

;;; robot description convenience functions
(defun clear-itasc-robot-list ()
  (setf *itasc-robots* ()))

(defun add-itasc-robot (&key robot-name robot-type kinematic-chains frames)
  (setf *itasc-robots*
        (append *itasc-robots*
                (list (make-instance 'itasc-robot
                                     :robot-name robot-name
                                     :robot-type robot-type
                                     :kinematic-chains kinematic-chains
                                     :attached-frames frames)))))

(defun find-itasc-robot (robot-name)
  (find robot-name *itasc-robots* :key #'robot-name :test #'string=))

(defun make-robot-kinematic-chain (&key chain-name robot-joint-weights)
  (make-instance 'itasc-robot-kinematic-chain
                 :chain-name chain-name
                 :robot-joint-weights robot-joint-weights))

(defun assemble-robot-joint-weights (&key robot-name chain-names)
  (apply #'append
         (mapcar (lambda (chain-name)
                   (let ((chain (find-kinematic-chain robot-name chain-name)))
                     (cond (chain
                            (robot-joint-weights chain))
                           (t
                            (cpl-impl:fail 'manipulation-failed
                                           :format-control 
                                           (concatenate 'string
                                                        "Asked to retrieve joint weights of non-existent kinematic chain. Chain name: "
                                                        chain-name
                                                        " Robot name: "
                                                        robot-name))))))
                 chain-names)))

(defun find-kinematic-chain (robot-name chain-name)
  (let ((robot (find-itasc-robot robot-name)))
    (when robot
      (find chain-name (kinematic-chains robot) :key #'chain-name :test #'string=))))

(defun make-robot-joint-weight-list (&key joint-names weights)
  (when (and joint-names weights)
    (cond ((eql (length joint-names)
                (length weights))
           (mapcar (lambda (name weight)
                     (make-robot-weight
                      :joint-name name
                      :weight weight))
                   joint-names weights))
          (t (cpl-impl:fail 'manipulation-failed
                            :format-control 
                            "Provided joint-names and weights lists that are of different length")))))

(defun make-robot-joint-weight-standard-list (&key joint-names)
  ;;uses standard joint weights of 1.0
  (when (listp joint-names)
    (let ((result (mapcar (lambda (name)
                            (make-robot-weight
                             :joint-name name))
                          joint-names)))
      result)))

(defun make-robot-weight (&key joint-name (weight 1.0))
  (make-instance 'itasc-robot-joint-weight
                 :joint-name joint-name
                 :weight weight))

(defun object-frame-on-robot-p (robot-name frame-name)
  (when (and robot-name frame-name)
    (when (find-itasc-robot robot-name)
      (when (some (lambda (frame)
                    (string= frame frame-name))
                  (attached-frames (find-itasc-robot robot-name)))
        t))))

;;; itasc task convenience functions...
(defun clear-itasc-tasks ()
  (setf *itasc-tasks* ()))

(defun add-itasc-task (task)
  (setf *itasc-tasks*
        (append *itasc-tasks* (list task))))

(defun find-itasc-task (task-name)
  (find task-name *itasc-tasks* :key #'name :test #'string=))

(defun make-vkc (&key chain-joints object1 object2)
  (make-instance 'itasc-virtual-kinematic-chain
                 :chain-joints chain-joints
                 :object1 object1
                 :object2 object2))

(defun make-object-frame (&key object-name frame-name)
  (cond ((object-frame-valid-p object-name frame-name)
         (make-instance 'itasc-object-frame
                        :object-name object-name
                        :frame-name frame-name))
         (t (cpl-impl:fail 'manipulation-failed
                           :format-control (concatenate 'string
                                                        "Asked to make invalid object-frame. Object: "
                                                        object-name
                                                        " Frame: "
                                                        frame-name)))))

(defun make-chain-joint-list (&key joint-names joint-types)
  (when (and joint-names joint-types)
    (cond ((eql (length joint-names)
                (length joint-types))
           (mapcar (lambda (name type)
                     (make-chain-joint
                      :joint-name name
                      :joint-type type))
                   joint-names joint-types))
          (t (cpl-impl:fail 'manipulation-failed
                            :format-control 
                            "Provided joint-names and joint-types lists that are of different length")))))

(defun make-chain-joint (&key joint-name joint-type)
  (make-instance 'itasc-chain-joint
                 :joint-name joint-name
                 :joint-type joint-type))

(defun make-constraint (&key constraint-name referred-joint
                          value (constraint-type :constrained)
                          (operator :equality_operator) (weight 1.0)
                          (lower-boundary 0) (upper-boundary 0)
                          (controller :position) 
                          (trajectory-type :trapezoidal_velocity_profile)
                          (trajectory-duration 0) (trajectory-velocity 0))
  (make-instance 'itasc-constraint
                 :constraint-type constraint-type
                 :constraint-name constraint-name
                 :referred-joint referred-joint
                 :operator operator
                 :value value
                 :weight weight
                 :lower-boundary lower-boundary
                 :upper-boundary upper-boundary
                 :controller controller
                 :trajectory-type trajectory-type
                 :trajectory-duration trajectory-duration
                 :trajectory-velocity trajectory-velocity))
