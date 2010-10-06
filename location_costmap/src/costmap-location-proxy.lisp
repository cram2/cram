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

(in-package :location-costmap)

(defconstant +costmap-n-samples+ 5)

(defclass costmap-location-proxy (point-location-proxy)
  ((next-solutions :accessor :next-solutions :initform nil
                   :documentation "List of the next solution. We want
                   to minimize driving distances, so we always
                   generate a bunch of solutions, order them by
                   distance to the robot and always chose the closest
                   one when generating a new solution.")
   (costmap :initarg :costmap :reader costmap))
  (:documentation "Proxy class to generate designator solutions from a
  costmap."))

(defmethod make-location-proxy ((type (eql 'costmap)) (val location-costmap))
  (make-instance 'costmap-location-proxy :costmap val))

(defmethod location-proxy-precedence-value ((type (eql 'costmap)))
  0)

(defmethod initialize-instance :after ((proxy costmap-location-proxy) &key)
  (location-proxy-next-solution proxy)
  (publish-location-costmap (costmap proxy)))

(defmethod location-proxy-next-solution ((proxy costmap-location-proxy))
  (flet ((take-closest-point (points)
           (let ((closest (car points))
                 (dist (cl-transforms:v-dist (cl-transforms:translation
                                              (cl-tf:lookup-transform
                                               *tf*
                                               :target-frame "/map"
                                               :source-frame "/base_link"))
                                             (car points))))
             (dolist (p (cdr points) closest)
               (let ((new-dist (cl-transforms:v-dist (cl-transforms:translation
                                                      (cl-tf:lookup-transform
                                                       *tf*
                                                       :target-frame "/map"
                                                       :source-frame "/base_link"))
                                                     p)))
                 (when (< new-dist dist)
                   (setf dist new-dist)
                   (setf closest p)))))))
    (with-slots (point next-solutions costmap)
        proxy
      (let ((solutions (or next-solutions
                           (loop repeat +costmap-n-samples+
                                 collecting (generate-point costmap)))))
        (prog1 (setf point (take-closest-point solutions))
          (publish-point point)
          (setf next-solutions (delete point solutions)))))))
