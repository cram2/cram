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
;;;

(in-package :designators-ros)

(defmethod location-proxy-solution->pose (desig (solution cl-transforms:pose))
  (cl-tf:make-pose-stamped "/map" (roslisp:ros-time)
                           (cl-transforms:origin solution)
                           (cl-transforms:orientation solution)))
  
(defmethod location-proxy-solution->pose (desig (solution cl-transforms:transform))
  (cl-tf:make-pose-stamped "/map" (roslisp:ros-time)
                           (cl-transforms:translation solution)
                           (cl-transforms:rotation solution)))

(defmethod location-proxy-solution->pose (desig (solution cl-tf:pose-stamped))
  solution)

(defclass pose-location-proxy ()
  ((pose :accessor location-proxy-current-solution :initarg :pose))
  (:documentation "Proxy class for designators that contain poses."))

(defclass point-location-proxy ()
  ((point :accessor location-proxy-current-solution :initarg :pose))
  (:documentation "Proxy class for designators that contain poses."))

(defmethod make-location-proxy ((type (eql 'point)) (value cl-transforms:3d-vector))
  (make-instance 'pose-location-proxy
                 :pose (cl-transforms:make-pose
                        value (cl-transforms:make-quaternion 0 0 0 1))))

(defmethod make-location-proxy ((type (eql 'pose)) (value cl-transforms:transform))
  (make-instance 'pose-location-proxy
                 :pose (cl-transforms:make-pose (cl-transforms:translation value)
                                                (cl-transforms:rotation value))))

(defmethod make-location-proxy ((type (eql 'pose)) (value cl-transforms:pose))
  (make-instance 'pose-location-proxy
                 :pose value))

(defmethod make-location-proxy ((type (eql 'pose)) (value cl-tf:pose-stamped))
  (make-instance 'pose-location-proxy :pose value))

(defmethod make-location-proxy ((type (eql 'pose)) (value cl-tf:stamped-transform))
  (make-instance 'pose-location-proxy
                 :pose (cl-tf:make-pose-stamped
                        (cl-tf:frame-id value) (cl-tf:stamp value)
                        (cl-transforms:translation value)
                        (cl-transforms:rotation value))))

(defmethod location-proxy-precedence-value ((type (eql 'pose)))
  1)

(defmethod location-proxy-next-solution ((proxy pose-location-proxy))
  nil)

(defmethod location-proxy-solution->pose (desig (solution cl-transforms:3d-vector))
  (with-vars-bound (?o)
      (lazy-car (prolog `(desig-orientation ,desig ,solution ?o)))
    (with-vars-bound (?z)
        (lazy-car (prolog `(desig-z-value ,desig ,solution ?z)))
      (cl-tf:make-pose-stamped
       "/map" (roslisp:ros-time)
       (cl-transforms:make-3d-vector
        (cl-transforms:x solution)
        (cl-transforms:y solution)
        (if (is-var ?z) 0.0d0 ?z))
       (cond ((and *tf* (is-var ?o) (tf:can-transform
                                     *tf*
                                     :target-frame "/map"
                                     :source-frame "/base_link"))
              (cl-transforms:rotation (tf:lookup-transform
                                       *tf*
                                       :target-frame "/map"
                                       :source-frame "/base_link")))
             ((is-var ?o)
              (cl-transforms:make-quaternion 0 0 0 1))
             (t ?o))))))
