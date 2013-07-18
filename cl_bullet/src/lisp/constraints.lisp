;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;

(in-package :bullet)

(defclass constraint (foreign-class)
  ((body-1 :initarg :body-1 :reader body-1)
   (body-2 :initarg :body-2 :reader body-2)))

(defmethod foreign-class-free-fun ((constraint foreign-class))
  #'delete-constraint)

;; Motor protocol

(defgeneric limit (motor type)
  (:documentation "Returns the limit of `type'. `type' is a
  symbol. Supported types depend on the motor. They might
  include :lower :upper and :softness."))
(defgeneric (setf limit) (new-value motor type))
(defgeneric enabled (motor))
(defgeneric (setf enabled) (enabled motor))
(defgeneric max-impulse (motor))
(defgeneric (setf max-impulse) (new-value motor))
(defgeneric target-velocity (motor))
(defgeneric (setf target-velocity) (new-value motor))
(defgeneric set-target (motor target dt)
  (:documentation "Calculates and sets the velocity to reach `target'
  in time `dt'. Please note that this method has to be called
  continously since it doesn't start a controller but just sets the
  velocity command."))
(defgeneric motor-position (motor)
  (:documentation "Returns the current position of the motor"))
