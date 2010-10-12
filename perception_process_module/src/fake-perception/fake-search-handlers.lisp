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

(in-package :perception-pm)

(defclass fake-perceived-object ()
  ((pose :initarg :pose :reader object-pose)
   (desig :initarg :desig :accessor object-desig :initform nil)
   (probability :initform 0.0 :accessor perceived-object-probability)
   (timestamp :initarg :timestamp :reader object-timestamp)
   (properties :initarg :properties :reader object-properties)))

(defmethod object-search-function ((type (cl:eql 'round-plate)) desig &optional perceived-object)
  (let ((perceived-object (or (when perceived-object
                                (make-instance 'fake-perceived-object
                                               :pose (object-pose perceived-object)
                                               :desig (object-desig perceived-object)
                                               :timestamp (ros-time)
                                               :properties (object-properties perceived-object)))
                              (make-instance 'fake-perceived-object
                                             :pose (tf:make-pose-stamped
                                                    "/map" (ros-time)
                                                    (cl-transforms:make-3d-vector
                                                     0.44 1.1 0.7)
                                                    (cl-transforms:make-quaternion
                                                     0 0 0 1))
                                             :timestamp (ros-time)
                                             :properties (description desig)))))
    (assert-perceived-object perceived-object (description desig))))

(defmethod object-search-function ((type (cl:eql 'bottle)) desig &optional perceived-object)
  (let ((perceived-object (or (when perceived-object
                                (make-instance 'fake-perceived-object
                                               :pose (object-pose perceived-object)
                                               :timestamp (ros-time)
                                               :properties (object-properties perceived-object)))
                              (make-instance 'fake-perceived-object
                                             :pose (tf:make-pose-stamped
                                                    "/map" (ros-time)
                                                    (cl-transforms:make-3d-vector
                                                     1.243111 -0.728864 0.9)
                                                    (cl-transforms:make-quaternion
                                                     0 0 0 1))
                                             :timestamp (ros-time)
                                             :properties (description desig)))))
    (assert-perceived-object perceived-object (description desig))))

(defmethod object-search-function ((type (cl:eql 'silverware)) desig &optional perceived-object)
  (let ((perceived-object (or (when perceived-object
                                (make-instance 'fake-perceived-object
                                               :pose (object-pose perceived-object)
                                               :timestamp (ros-time)
                                               :properties (object-properties perceived-object)))
                              (make-instance 'fake-perceived-object
                                             :pose (tf:make-pose-stamped
                                                    "/map" (ros-time)
                                                    (cl-transforms:make-3d-vector
                                                     0.912 1.17 0.850)
                                                    (cl-transforms:make-quaternion
                                                     0 0 0 1))
                                             :timestamp (ros-time)
                                             :properties (description desig)))))
    (assert-perceived-object perceived-object (description desig))))

(defmethod make-new-desig-description ((old-desig object-designator)
                                       (po fake-perceived-object))
  (let ((obj-loc-desig (make-designator 'location `((pose ,(object-pose po))))))
    (cons `(at ,obj-loc-desig)
          (remove 'at (description old-desig) :key #'car))))
