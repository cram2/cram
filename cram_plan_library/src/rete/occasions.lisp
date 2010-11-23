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

(in-package :plan-lib)

(crs:def-production object-picked-up
  (object-in-hand ?obj ?side))

(crs:def-production object-in-hand-failure
  (object-in-hand-failure ?f ?obj ?side))

(crs:def-production object-placed-at
  (object-placed-at ?obj ?loc))

(defun on-obj-in-hand-retractions (op &key ?obj ?side)
  (declare (ignore ?side))
  (when (eql op :assert)
    (retract-occasion `(object-placed-at ,?obj))))

(defun on-obj-put-down (op &key ?obj ?loc)
  (when (eql op :assert)
    (let ((side (var-value '?side (car (holds `(object-in-hand ?obj ?side))))))
      (retract-occasion `(object-in-hand ?obj ,side))
      (make-designator
       'object
       (append (remove 'at (description ?obj) :key #'car)
               `((at ,?loc)))
       ?obj))))

(crs:register-production-handler 'object-picked-up #'on-obj-in-hand-retractions)
(crs:register-production-handler 'object-placed-at #'on-obj-put-down)
