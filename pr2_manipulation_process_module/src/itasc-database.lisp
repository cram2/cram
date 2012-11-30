;;; Copyright (c) 2012, Georg Bartels <georg.bartels@cs.uni-bremen.de>
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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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

(in-package :pr2-manip-pm)

(defvar *itasc-objects* () "List of itasc objects used in the demo")

(defvar *robot-joint-weights* () "List of joint weights for the PR2.")

(defvar *itasc-tasks* () "List of tasks that are used in our demonstration.")

;; Definitions of classes that hold the relevant information for our demo
(defclass itasc-object-information ()
  ((object-name :reader object-name :initarg :object-name)
   (object-type :reader object-type :initarg :object-type)
   (attached-frames :reader attached-frames :initarg :attached-frames)
   (external-location :reader external-location :initarg :external-location)))

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
   (trajectory-generator :reader trajectory-generator :initarg :trajectory-generator)))

;;; object database convenience functions...
(defun clear-itasc-object-list ()
  (setf *itasc-objects* ()))

(defun add-itasc-object (&key object-name object-type frames external-location)
  (setf *itasc-objects*
        (append *itasc-objects*
                (list (make-instance 'itasc-object-information
                                     :object-name object-name
                                     :object-type object-type
                                     :attached-frames frames
                                     :external-location external-location)))))

(defun find-itasc-object (object-name)
  (find object-name *itasc-objects* :key #'object-name :test #'string=))

(defun object-frame-valid-p (object-name frame-name)
  (when (and object-name frame-name)
    (when (find-itasc-object object-name)
      (when (some (lambda (frame)
                    (string= frame frame-name))
                  (attached-frames (find-itasc-object object-name)))
        t))))

;;; robot joint weight convenience functions...
(defun clear-robot-joint-weights ()
  (setf *robot-joint-weights* ()))

(defun add-robot-joint-weight (&key joint-name weight)
  (setf *robot-joint-weights*
        (append *robot-joint-weights*
                (list (make-instance 'itasc-robot-joint-weight
                                     :joint-name joint-name
                                     :weight weight)))))

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
                           :format-control "Asked to make invalid object-frame."))))

(defun make-chain-joint (&key joint-name joint-type)
  (make-instance 'itasc-chain-joint
                 :joint-name joint-name
                 :joint-type joint-type))

(defun make-constraint (&key constraint-type constraint-name referred-joint
                             operator value (weight 1)
                             (lower-boundary 0) (upper-boundary 0)
                             (controller 1) (trajectory-generator "trapezoidal-trajectory"))
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
                 :trajectory-generator trajectory-generator))
