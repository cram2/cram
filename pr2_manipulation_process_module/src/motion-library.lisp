;;; Copyright (c) 2012, Jan Winkler <winkler@informatik.uni-bremen.de>
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
;;;       Technische Universitaet Bremen nor the names of its contributors 
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

(in-package :pr2-manip-pm)

(defparameter *carry-pose-right* (tf:make-pose-stamped
                                  "/base_footprint" 0.0
                                  (cl-transforms:make-3d-vector
                                   0.2 -0.55 1.00)
                                  (cl-transforms:euler->quaternion :ay (/ pi 2))))
(defparameter *carry-pose-left* (tf:make-pose-stamped
                                 "/base_footprint" 0.0
                                 (cl-transforms:make-3d-vector
                                  0.2 0.55 1.00)
                                 (cl-transforms:euler->quaternion :ay (/ pi 2))))

(defparameter *top-grasp* (cl-transforms:euler->quaternion :ay (/ pi 2)))
(defparameter *front-grasp* (cl-transforms:make-identity-rotation))
(defparameter *left-grasp* (cl-transforms:euler->quaternion :az (/ pi -2)))
(defparameter *right-grasp* (cl-transforms:euler->quaternion :az (/ pi 2)))

(defparameter *pot-relative-right-handle-transform*
  (cl-transforms:make-transform
   (cl-transforms:make-3d-vector
    0.135 -0.012 0.08)
   (cl-transforms:make-quaternion
    -0.17903158312000483d0 -0.6841142429564597d0
    0.6850543286185364d0 -0.17503131625700874d0)))

(defparameter *pot-relative-left-handle-transform*
  (cl-transforms:make-transform
   (cl-transforms:make-3d-vector
    -0.135 -0.008 0.08)
   (cl-transforms:make-quaternion
    -0.6770480569314481d0 0.11600823330627819d0
    0.16801192666809586d0 0.7070502180947129d0)))
