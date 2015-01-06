;;; Copyright (c) 2014, Jan Winkler <winkler@cs.uni-bremen.de>
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

(in-package :continuous-interaction)

(defun lookup-object-location (object-name object-type)
  (declare (ignore object-name object-type))
  (make-designator 'location `((desig-props:in Cupboard)
                               (name "kitchen_island"))))

(defun lookup-object-type (object-name)
  (case object-name
    (desig-props::pancakemix
     'desig-props::pancakemix)
    (desig-props::pancake
     'desig-props::pancake)))

(defun lookup-object-handles (object-type)
  (declare (ignore object-type))
  (make-handles 0.04))

(defun lookup-object-details (object-name object-type)
  (declare (ignore object-name object-type)))

(defun lookup-object-description (name type)
  (let* ((handles (lookup-object-handles type))
         (details (lookup-object-details name type))
         (location (lookup-object-location name type)))
    (append `((desig-props:name ,name)
              (desig-props:at ,location))
            (when type
              `((desig-props:type ,type)))
            (when handles
              (mapcar (lambda (handle)
                        `(desig-props:handle ,handle))
                      handles))
            details)))

(defmacro with-known-object-types (objects &body body)
  `(with-designators (,@(mapcar (lambda (object)
                                  (destructuring-bind (object-variable object-type object-name)
                                      object
                                    `(,object-variable
                                      (object (lookup-object-description
                                               ,object-name ,object-type)))))
                                objects))
     ,@body))

(defun on-surface-p (object)
  (let* ((at (desig-prop-value object 'desig-props:at))
         (on (desig-prop-value at 'desig-props:on)))
    (when on t)))

(defun in-container-p (object)
  (let* ((at (desig-prop-value object 'desig-props:at))
         (in (desig-prop-value at 'desig-props:in)))
    (when in t)))

(defun make-handles (distance-from-center
                     &key
                       (segments 1)
                       (ax 0.0) (ay 0.0) (az 0.0)
                       (offset-angle 0.0)
                       grasp-type
                       (center-offset
                        (tf:make-identity-vector)))
  (loop for i from 0 below segments
        as current-angle = (+ (* 2 pi (float (/ i segments)))
                              offset-angle)
        as handle-pose = (tf:make-pose
                          (tf:make-3d-vector
                           (+ (* distance-from-center (cos current-angle))
                              (tf:x center-offset))
                           (+ (* distance-from-center (sin current-angle))
                              (tf:y center-offset))
                           (+ 0.0
                              (tf:z center-offset)))
                          (tf:euler->quaternion
                           :ax ax :ay ay :az (+ az current-angle)))
        as handle-object = (make-designator
                            'cram-designators:object
                            (append
                             `((desig-props:type desig-props:handle)
                               (desig-props:at
                                ,(a location `((desig-props:pose
                                                ,handle-pose)))))
                             (when grasp-type
                               `((desig-props:grasp-type ,grasp-type)))))
        collect handle-object))
