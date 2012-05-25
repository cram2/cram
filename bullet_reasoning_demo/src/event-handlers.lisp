;;; Copyright (c) 2012, Lorenz Moesenlechner <moesenle@in.tum.de>
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

(in-package :btr)

(defun get-robot-object ()
  (with-vars-bound (?robot-name)
      (lazy-car (prolog `(robot ?robot-name)))
    (unless (is-var ?robot-name)
      (object *current-bullet-world* ?robot-name))))

(defun get-designator-object (object-designator)
  (let ((object-designator (desig:newest-valid-designator object-designator)))
    (when object-designator
      (object
       *current-bullet-world*
       (desig:object-identifier (desig:reference object-designator))))))

(defmethod plan-knowledge:on-event attach-objects
    ((event plan-knowledge:object-attached))
  (let ((robot (get-robot-object))
        (object (get-designator-object (plan-knowledge:event-object event))))
    (when object
      (attach-object robot object (plan-knowledge:event-link event)))))

(defmethod plan-knowledge:on-event detach-objects
    ((event plan-knowledge:object-detached))
  (let ((robot (get-robot-object))
        (object (get-designator-object (plan-knowledge:event-object event))))
    (when object
      (detach-object robot object (plan-knowledge:event-link event)))))
